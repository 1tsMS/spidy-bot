// Spidy firmware v2: "dumb" servo driver. ALL calibration lives on the PC (config/calibration.json).
//
// Line protocol over USB serial (115200) or WiFi TCP port 5000 (host spidy.local):
//   P16,<16 ticks>   set all 16 PCA9685 channels (0 = channel off). Streamed ~50 Hz, no reply.
//   M,<ch>,<ticks>   set one physical channel                         -> M_OK,ch,ticks
//   OFF  (or ESTOP)  all channels off                                 -> OFF_OK
//   PING,<n>         latency check                                    -> PONG,<n>
//   IMU              read the MPU6050 (m/s^2, rad/s)                  -> IMU,ax,ay,az,gx,gy,gz | IMU_ERR
//   STATE            last ticks per channel                           -> STATE,<16 ticks>
//   BOOT,<16 ticks>  store the power-on pose in flash                 -> BOOT_OK
//   BOOT?            read it back                                     -> BOOT,<16 ticks> | BOOT,NONE
//   I2C?             bus scan + PCA write errors  -> I2C,devices=0x40 0x68,pca_begin=1,pca_write_errors=0
//   PCA?             PCA9685 registers read back   -> PCA,mode1=..,mode2=..,prescale=121,ch=<16 values|OFF|ERR>
//   HELLO            identify          -> SPIDY,fw=2,ip=<ip>,imu=<0|1>,boot=<reset reason>,up=<s>
//
// Failsafe: if a P16 stream stops for STALE_MS the servos HOLD their last pulse
// (the PCA9685 keeps outputting it on its own) and STALE is reported once.
// Power-on: the stored BOOT pose is applied. With no stored pose all outputs stay OFF.
//
// Ticks: PCA9685 at 50 Hz, 4096 ticks per 20 ms (1 tick = 4.88 us). Same
// oscillator settings as the old esp32_receiver, so old pulses mean the same thing.

#include <Wire.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <esp_system.h>
#include "secrets.h"   // WIFI_SSID / WIFI_PASS. Copy secrets.example.h -> secrets.h

static const uint8_t  CHANNELS   = 16;
static const uint16_t TICK_MIN   = 100;    // hard limits, same as the PC's HARD_MIN/HARD_MAX
static const uint16_t TICK_MAX   = 700;
static const uint32_t STALE_MS   = 300;
static const uint16_t TCP_PORT   = 5000;
static const size_t   MAX_LINE   = 200;

Adafruit_PWMServoDriver pwm(0x40);
Adafruit_MPU6050 mpu;
Preferences prefs;
WiFiServer server(TCP_PORT);
WiFiClient client;

uint16_t ticks[CHANNELS];      // what each channel outputs now, 0 = off
bool imuOk = false;
bool pcaOk = false;            // pwm.begin() found the PCA9685
uint32_t pcaWriteErrors = 0;   // setPWM() calls the chip did not acknowledge
bool streaming = false;
bool staleReported = false;
uint32_t lastP16 = 0;
String serialBuf, tcpBuf;

// Why did the ESP32 last start? Sent with READY/HELLO so the PC can tell a brownout
// (servo current spike pulled the supply down) from a crash or a normal power-on.
const char *resetReason() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:   return "poweron";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_PANIC:     return "CRASH";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:       return "WATCHDOG";
    case ESP_RST_SW:        return "software";
    case ESP_RST_EXT:       return "reset-pin";
    default:                return "other";
  }
}

// ------------------------------------------------------------------ output
void reply(const String &msg) {
  Serial.println(msg);
  if (client && client.connected()) client.println(msg);
}

void writeChannel(uint8_t ch, uint16_t t) {
  if (ch >= CHANNELS) return;
  if (t == 0) {                       // off: no pulse at all, servo goes limp
    if (pwm.setPWM(ch, 0, 4096)) pcaWriteErrors++;   // 4096 = the PCA9685 "full off" bit
    ticks[ch] = 0;
    return;
  }
  t = constrain(t, TICK_MIN, TICK_MAX);
  if (ticks[ch] == t) return;         // unchanged: skip the I2C write
  if (pwm.setPWM(ch, 0, t)) pcaWriteErrors++;      // non-zero return = I2C write failed
  ticks[ch] = t;
}

void allOff() {
  for (uint8_t ch = 0; ch < CHANNELS; ch++) writeChannel(ch, 0);
  streaming = false;
}

String ticksCsv(const uint16_t *v) {
  String s;
  for (uint8_t i = 0; i < CHANNELS; i++) {
    if (i) s += ',';
    s += String(v[i]);
  }
  return s;
}

// Parse "a,b,c,..." starting at index `from` into out[]. Returns how many numbers were read.
int parseInts(const String &line, int from, long *out, int maxN) {
  int n = 0;
  while (from <= (int)line.length() && n < maxN) {
    int comma = line.indexOf(',', from);
    if (comma < 0) comma = line.length();
    String tok = line.substring(from, comma);
    tok.trim();
    if (tok.length() == 0) break;
    out[n++] = tok.toInt();
    from = comma + 1;
  }
  return n;
}

// ------------------------------------------------------------------ boot pose (flash)
bool loadBoot(uint16_t *out) {
  prefs.begin("spidy", true);
  size_t n = prefs.getBytes("boot", out, sizeof(uint16_t) * CHANNELS);
  prefs.end();
  return n == sizeof(uint16_t) * CHANNELS;
}

void saveBoot(const uint16_t *v) {
  prefs.begin("spidy", false);
  prefs.putBytes("boot", v, sizeof(uint16_t) * CHANNELS);
  prefs.end();
}

// ------------------------------------------------------------------ commands
void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line.startsWith("P16,")) {
    long v[CHANNELS];
    if (parseInts(line, 4, v, CHANNELS) != CHANNELS) { reply("ERR,P16 needs 16 values"); return; }
    for (uint8_t ch = 0; ch < CHANNELS; ch++) writeChannel(ch, (uint16_t)constrain(v[ch], 0, 4095));
    lastP16 = millis();
    streaming = true;
    staleReported = false;
    return;                                         // no reply: this is the 50 Hz stream
  }
  if (line.startsWith("M,")) {
    long v[2];
    if (parseInts(line, 2, v, 2) != 2 || v[0] < 0 || v[0] >= CHANNELS) { reply("ERR,M"); return; }
    writeChannel((uint8_t)v[0], (uint16_t)constrain(v[1], 0, 4095));
    reply("M_OK," + String(v[0]) + "," + String(ticks[v[0]]));
    return;
  }
  if (line == "OFF" || line == "ESTOP") { allOff(); reply("OFF_OK"); return; }
  if (line.startsWith("PING,")) { reply("PONG," + line.substring(5)); return; }
  if (line == "STATE") { reply("STATE," + ticksCsv(ticks)); return; }
  if (line == "IMU") {
    if (!imuOk) { reply("IMU_ERR"); return; }
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    reply("IMU," + String(a.acceleration.x, 3) + "," + String(a.acceleration.y, 3) + "," +
          String(a.acceleration.z, 3) + "," + String(g.gyro.x, 4) + "," + String(g.gyro.y, 4) + "," +
          String(g.gyro.z, 4));
    return;
  }
  if (line == "I2C?") {                             // which chips answer on the bus?
    String found;
    for (uint8_t a = 1; a < 127; a++) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission() == 0) {
        if (found.length()) found += ' ';
        found += "0x" + String(a, HEX);
      }
    }
    reply("I2C,devices=" + found + ",pca_begin=" + String(pcaOk ? 1 : 0) +
          ",pca_write_errors=" + String(pcaWriteErrors));
    return;
  }
  if (line == "PCA?") {                             // read the PCA9685's own registers back
    auto rd = [](uint8_t reg) -> int {
      Wire.beginTransmission(0x40);
      Wire.write(reg);
      if (Wire.endTransmission(false) != 0) return -1;
      if (Wire.requestFrom((uint8_t)0x40, (uint8_t)1) != 1) return -1;
      return Wire.read();
    };
    int mode1 = rd(0x00), mode2 = rd(0x01), pre = rd(0xFE);
    String msg = "PCA,mode1=0x" + String(mode1, HEX) + (mode1 >= 0 && (mode1 & 0x10) ? "(SLEEP)" : "") +
                 ",mode2=0x" + String(mode2, HEX) + ",prescale=" + String(pre) + ",ch=";
    for (uint8_t ch = 0; ch < CHANNELS; ch++) {
      uint8_t base = 0x06 + 4 * ch;               // LEDn_ON_L, ON_H, OFF_L, OFF_H
      int offL = rd(base + 2), offH = rd(base + 3);
      if (ch) msg += ' ';
      if (offL < 0 || offH < 0) msg += "ERR";
      else if (offH & 0x10) msg += "OFF";
      else msg += String(((offH & 0x0F) << 8) | offL);
    }
    reply(msg);
    return;
  }
  if (line == "BOOT?") {
    uint16_t b[CHANNELS];
    reply(loadBoot(b) ? "BOOT," + ticksCsv(b) : String("BOOT,NONE"));
    return;
  }
  if (line.startsWith("BOOT,")) {
    long v[CHANNELS];
    if (parseInts(line, 5, v, CHANNELS) != CHANNELS) { reply("ERR,BOOT needs 16 values"); return; }
    uint16_t b[CHANNELS];
    for (uint8_t i = 0; i < CHANNELS; i++)
      b[i] = v[i] == 0 ? 0 : (uint16_t)constrain(v[i], TICK_MIN, TICK_MAX);
    saveBoot(b);
    reply("BOOT_OK");
    return;
  }
  if (line == "HELLO") {
    String ip = WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : String("none");
    reply("SPIDY,fw=2,ip=" + ip + ",imu=" + String(imuOk ? 1 : 0) + ",pca=" + String(pcaOk ? 1 : 0) +
          ",pca_err=" + String(pcaWriteErrors) + ",boot=" + resetReason() +
          ",up=" + String(millis() / 1000));
    return;
  }
  reply("ERR,unknown: " + line);
}

// Collect characters into lines; guard against garbage growing the buffer forever.
void feed(String &buf, char c) {
  if (c == '\n' || c == '\r') {
    if (buf.length()) handleLine(buf);
    buf = "";
  } else if (buf.length() < MAX_LINE) {
    buf += c;
  } else {
    buf = "";                         // overlong line: drop it
  }
}

// ------------------------------------------------------------------ setup / loop
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setTimeOut(20);                // ms. A supply dip can wedge the bus: fail fast, never hang
  pcaOk = pwm.begin();
  pwm.setPWMFreq(50);
  for (uint8_t ch = 0; ch < CHANNELS; ch++) ticks[ch] = 1;   // force the first write
  allOff();

  uint16_t boot[CHANNELS];
  if (loadBoot(boot)) {
    for (uint8_t ch = 0; ch < CHANNELS; ch++) writeChannel(ch, boot[ch]);
    Serial.println("BOOT_POSE_APPLIED");
  } else {
    Serial.println("BOOT,NONE (outputs off until the PC sends something)");
  }

  imuOk = mpu.begin();                // 0x68 on the same I2C bus as the PCA9685
  if (imuOk) {
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  if (WIFI_SSID != nullptr && WIFI_SSID[0] != '\0') {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);             // modem power-save stalls the link 100-300 ms: bad for a 50 Hz stream
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) delay(200);
    if (WiFi.status() == WL_CONNECTED) {
      server.begin();
      server.setNoDelay(true);
      if (MDNS.begin("spidy")) MDNS.addService("spidy", "tcp", TCP_PORT);
      Serial.println("WIFI_OK," + WiFi.localIP().toString() + ",spidy.local,PORT," + String(TCP_PORT));
    } else {
      Serial.println("WIFI_ERR");
    }
  }
  reply(String("READY,imu=") + (imuOk ? "1" : "0") + ",pca=" + (pcaOk ? "1" : "0") + ",boot=" + resetReason());
}

void loop() {
  while (Serial.available()) feed(serialBuf, (char)Serial.read());

  if (server.hasClient()) {           // a new PC connection replaces the old one
    if (client) client.stop();
    client = server.available();
    client.setNoDelay(true);
    tcpBuf = "";
    client.println(String("READY,boot=") + resetReason() + ",up=" + String(millis() / 1000));
  }
  // At most ~4 stream lines per pass, so a burst of buffered Wi-Fi data can't keep loop()
  // busy long enough to starve the Wi-Fi stack / trip the watchdog.
  for (int n = 0; n < 512 && client && client.connected() && client.available(); n++)
    feed(tcpBuf, (char)client.read());

  if (streaming && !staleReported && millis() - lastP16 > STALE_MS) {
    reply("STALE");                   // stream stopped: servos HOLD the last pose
    staleReported = true;
  }
  delay(1);                           // let the Wi-Fi / idle tasks run every pass
}

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>

// Serial protocol (lines):
//   M,<id>,<pulse>   -> move servo id (0-15) to raw PCA9685 tick pulse
//   IDX,<id>,<idx>   -> map logical id to physical channel
//   POSE,<a0>...<a11>[,<duration_ms>] -> smooth 12-joint logical-angle motion
//   DUR,<ms>         -> set default POSE motion duration
//   SPD,<percent>    -> set motion speed percentage (1-100)
//   ESTOP            -> disable all outputs (sets all channels off)
// Same line protocol is accepted via WiFi TCP on WIFI_PORT.

// WiFi credentials live in secrets.h (gitignored). Copy secrets.example.h -> secrets.h.
#include "secrets.h"
static const uint16_t WIFI_PORT = 5000;

WiFiServer tcpServer(WIFI_PORT);
WiFiClient tcpClient;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

static const uint16_t SERVO_MIN = 100;
static const uint16_t SERVO_MAX = 700;
static const uint16_t SERVO_FREQ = 50;
static const uint8_t ANGLE_MIN = 0;
static const uint8_t ANGLE_MAX = 180;
static const uint8_t LOGICAL_JOINT_COUNT = 12;
static const uint16_t DEFAULT_MOTION_DURATION_MS = 800;
static const uint16_t MIN_MOTION_DURATION_MS = 50;
static const uint16_t MAX_MOTION_DURATION_MS = 10000;

static const uint8_t SERVO_COUNT = 16;
static uint16_t lastPulses[SERVO_COUNT];
static uint8_t motorMap[SERVO_COUNT];

static float currentAngle[LOGICAL_JOINT_COUNT];
static float startAngle[LOGICAL_JOINT_COUNT];
static float targetAngle[LOGICAL_JOINT_COUNT];
static bool isMoving = false;
static uint32_t motionStartMs = 0;
static uint16_t motionDurationMs = DEFAULT_MOTION_DURATION_MS;
static uint16_t defaultMotionDurationMs = DEFAULT_MOTION_DURATION_MS;
static uint8_t speedPercent = 100;

// Default startup profile from spidy_calibration3.txt + Sit pose from spidy_poses.txt
static const int8_t CALIB_DIR[SERVO_COUNT] = {
  -1, +1, -1, -1, -1, +1, +1, +1,
  -1, +1, -1, +1, +1, +1, +1, +1,
};

static const int8_t CALIB_OFFSET[SERVO_COUNT] = {
  +3, -3, -2, -6, +0, -4, +1, +0,
  -1, +4, +0, +6, +0, +0, +0, +0,
};

static const uint8_t CALIB_MIN[SERVO_COUNT] = {
  28, 22, 41, 57, 24, 44, 51, 17,
  49, 4, 26, 44, 0, 0, 0, 0,
};

static const uint8_t CALIB_MAX[SERVO_COUNT] = {
  129, 153, 180, 174, 180, 158, 149, 158,
  180, 131, 180, 150, 180, 180, 180, 180,
};

static const uint8_t CALIB_INDEX[SERVO_COUNT] = {
  0, 1, 2, 3, 4, 5, 11, 10,
  9, 15, 14, 13, 6, 7, 8, 12,
};

static const uint8_t STARTUP_SIT_ANGLES[SERVO_COUNT] = {
  90, 120, 100, 90, 120, 100, 90, 120,
  100, 90, 120, 100, 90, 90, 90, 90,
};

String lineBuffer;
String wifiLineBuffer;

void sendReply(const String &msg) {
  Serial.println(msg);
  if (tcpClient && tcpClient.connected()) {
    tcpClient.println(msg);
  }
}

void writeServo(uint8_t channel, uint16_t pulse) {
  if (channel >= SERVO_COUNT) return;
  pulse = constrain(pulse, SERVO_MIN, SERVO_MAX);
  lastPulses[channel] = pulse;
  pwm.setPWM(channel, 0, pulse);
}

void writeMappedServo(uint8_t logicalId, uint16_t pulse) {
  if (logicalId >= SERVO_COUNT) return;
  uint8_t channel = motorMap[logicalId];
  if (channel >= SERVO_COUNT) return;
  writeServo(channel, pulse);
}

uint8_t mappedChannel(uint8_t logicalId) {
  if (logicalId >= SERVO_COUNT) return 255;
  uint8_t channel = motorMap[logicalId];
  if (channel >= SERVO_COUNT) return 255;
  return channel;
}

void emergencyStop() {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    pwm.setPWM(i, 0, 0);
  }
}

uint16_t computePulseFromLogicalAngle(uint8_t logicalId, uint8_t logicalAngle) {
  if (logicalId >= SERVO_COUNT) return SERVO_MIN;

  int corrected = constrain((int)logicalAngle, (int)ANGLE_MIN, (int)ANGLE_MAX);
  if (CALIB_DIR[logicalId] < 0) {
    corrected = 180 - corrected;
  }

  int offset = (int)CALIB_OFFSET[logicalId];
  if (CALIB_DIR[logicalId] < 0) {
    offset = -offset;
  }
  corrected += offset;

  corrected = constrain(corrected, (int)CALIB_MIN[logicalId], (int)CALIB_MAX[logicalId]);
  int pulse = (int)SERVO_MIN + (corrected * ((int)SERVO_MAX - (int)SERVO_MIN) + 90) / 180;
  return (uint16_t)constrain(pulse, (int)SERVO_MIN, (int)SERVO_MAX);
}

void applyLogicalAngleToServo(uint8_t logicalId, float logicalAngle) {
  if (logicalId >= LOGICAL_JOINT_COUNT) return;
  int constrainedAngle = constrain((int)(logicalAngle + 0.5f), (int)ANGLE_MIN, (int)ANGLE_MAX);
  uint16_t pulse = computePulseFromLogicalAngle(logicalId, (uint8_t)constrainedAngle);
  writeMappedServo(logicalId, pulse);
}

void applyAllCurrentAngles() {
  for (uint8_t i = 0; i < LOGICAL_JOINT_COUNT; i++) {
    applyLogicalAngleToServo(i, currentAngle[i]);
  }
}

void startPoseMotion(const float newTarget[LOGICAL_JOINT_COUNT], uint16_t durationMs) {
  for (uint8_t i = 0; i < LOGICAL_JOINT_COUNT; i++) {
    startAngle[i] = currentAngle[i];
    targetAngle[i] = (float)constrain((int)(newTarget[i] + 0.5f), (int)ANGLE_MIN, (int)ANGLE_MAX);
  }

  motionDurationMs = constrain(durationMs, MIN_MOTION_DURATION_MS, MAX_MOTION_DURATION_MS);
  motionStartMs = millis();
  isMoving = true;
}

void updateMotion() {
  if (!isMoving) return;

  uint32_t now = millis();
  uint32_t elapsed = now - motionStartMs;
  float progress = (motionDurationMs > 0) ? ((float)elapsed / (float)motionDurationMs) : 1.0f;
  if (progress >= 1.0f) {
    progress = 1.0f;
  }
  if (progress < 0.0f) {
    progress = 0.0f;
  }

  float eased = progress * progress * (3.0f - 2.0f * progress);
  for (uint8_t i = 0; i < LOGICAL_JOINT_COUNT; i++) {
    currentAngle[i] = startAngle[i] + (targetAngle[i] - startAngle[i]) * eased;
  }

  applyAllCurrentAngles();

  if (progress >= 1.0f) {
    for (uint8_t i = 0; i < LOGICAL_JOINT_COUNT; i++) {
      currentAngle[i] = targetAngle[i];
    }
    isMoving = false;
  }
}

void applyStartupSitPose() {
  for (uint8_t logicalId = 0; logicalId < SERVO_COUNT; logicalId++) {
    motorMap[logicalId] = CALIB_INDEX[logicalId];
  }

  for (uint8_t logicalId = 0; logicalId < SERVO_COUNT; logicalId++) {
    uint16_t pulse = computePulseFromLogicalAngle(logicalId, STARTUP_SIT_ANGLES[logicalId]);
    writeMappedServo(logicalId, pulse);
  }

  for (uint8_t i = 0; i < LOGICAL_JOINT_COUNT; i++) {
    float a = (float)STARTUP_SIT_ANGLES[i];
    currentAngle[i] = a;
    startAngle[i] = a;
    targetAngle[i] = a;
  }
  isMoving = false;
}

bool parsePoseCommand(const String &line, float outAngles[LOGICAL_JOINT_COUNT], uint16_t &outDurationMs) {
  if (!line.startsWith("POSE,")) return false;

  uint8_t valueCount = 0;
  int start = 5;
  int len = line.length();
  outDurationMs = defaultMotionDurationMs;

  while (start <= len) {
    int comma = line.indexOf(',', start);
    if (comma < 0) comma = len;

    String token = line.substring(start, comma);
    token.trim();
    if (token.length() > 0) {
      if (valueCount < LOGICAL_JOINT_COUNT) {
        outAngles[valueCount] = token.toFloat();
      } else if (valueCount == LOGICAL_JOINT_COUNT) {
        int d = token.toInt();
        outDurationMs = (uint16_t)constrain(d, (int)MIN_MOTION_DURATION_MS, (int)MAX_MOTION_DURATION_MS);
      }
      valueCount++;
    }

    if (comma >= len) break;
    start = comma + 1;
  }

  return valueCount >= LOGICAL_JOINT_COUNT;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  uint16_t zeroPulse = (SERVO_MIN + SERVO_MAX) / 2;
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    lastPulses[i] = zeroPulse;
    motorMap[i] = i;
    writeServo(i, zeroPulse);
  }
  applyStartupSitPose();

  if (WIFI_SSID != nullptr && WIFI_SSID[0] != '\0') {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < 10000) {
      delay(200);
      Serial.print('.');
    }
    if (WiFi.status() == WL_CONNECTED) {
      tcpServer.begin();
      Serial.print("WIFI_OK,");
      Serial.print(WiFi.localIP());
      Serial.print(",PORT,");
      Serial.println(WIFI_PORT);
    } else {
      Serial.println("WIFI_ERR");
    }
  }

  sendReply("READY");
}

void handleLine(const String &line) {
  if (line == "ESTOP") {
    isMoving = false;
    emergencyStop();
    sendReply("ESTOP_OK");
    return;
  }

  if (line.startsWith("POSE,")) {
    float poseAngles[LOGICAL_JOINT_COUNT];
    uint16_t duration = defaultMotionDurationMs;
    if (!parsePoseCommand(line, poseAngles, duration)) {
      sendReply("POSE_ERR");
      return;
    }
    startPoseMotion(poseAngles, duration);
    sendReply(String("POSE_OK,") + String(duration));
    return;
  }

  if (line.startsWith("DUR,")) {
    int first = line.indexOf(',');
    if (first < 0) return;
    int value = line.substring(first + 1).toInt();
    defaultMotionDurationMs = (uint16_t)constrain(value, (int)MIN_MOTION_DURATION_MS, (int)MAX_MOTION_DURATION_MS);
    sendReply(String("DUR_OK,") + String(defaultMotionDurationMs));
    return;
  }

  if (line.startsWith("M,")) {
    int first = line.indexOf(',');
    int second = line.indexOf(',', first + 1);
    if (second < 0) return;
    uint8_t id = (uint8_t)line.substring(first + 1, second).toInt();
    uint16_t pulse = (uint16_t)line.substring(second + 1).toInt();
    writeMappedServo(id, pulse);
    uint8_t channel = mappedChannel(id);
    uint16_t constrainedPulse = constrain(pulse, SERVO_MIN, SERVO_MAX);
    String msg = String("M_OK,") + String(id) + "," + String(constrainedPulse) + ",CH,";
    if (channel < SERVO_COUNT) {
      msg += String(channel);
    } else {
      msg += "NA";
    }
    sendReply(msg);
    return;
  }

  if (line.startsWith("IDX,")) {
    int first = line.indexOf(',');
    int second = line.indexOf(',', first + 1);
    if (second < 0) return;
    uint8_t id = (uint8_t)line.substring(first + 1, second).toInt();
    uint8_t idx = (uint8_t)line.substring(second + 1).toInt();
    if (id >= SERVO_COUNT || idx >= SERVO_COUNT) return;
    motorMap[id] = idx;
    sendReply(String("IDX_OK,") + String(id) + "," + String(idx));
    return;
  }

  if (line.startsWith("SPD,")) {
    int first = line.indexOf(',');
    if (first < 0) return;
    int value = line.substring(first + 1).toInt();
    speedPercent = (uint8_t)constrain(value, 1, 100);
    defaultMotionDurationMs = (uint16_t)constrain((int)(DEFAULT_MOTION_DURATION_MS * (100.0f / speedPercent)), (int)MIN_MOTION_DURATION_MS, (int)MAX_MOTION_DURATION_MS);
    sendReply(String("SPD_OK,") + String(speedPercent));
    return;
  }
}

void consumeChars(Stream &stream, String &buffer) {
  while (stream.available() > 0) {
    char c = (char)stream.read();
    if (c == '\n' || c == '\r') {
      if (buffer.length() > 0) {
        handleLine(buffer);
        buffer = "";
      }
    } else {
      if (buffer.length() < 96) {
        buffer += c;
      } else {
        buffer = "";
      }
    }
  }
}

void pollWifiClient() {
  if (!tcpServer) return;

  if (!tcpClient || !tcpClient.connected()) {
    WiFiClient incoming = tcpServer.available();
    if (incoming) {
      if (tcpClient) {
        tcpClient.stop();
      }
      tcpClient = incoming;
      wifiLineBuffer = "";
      sendReply("WIFI_CLIENT_OK");
    }
  }

  if (tcpClient && tcpClient.connected()) {
    consumeChars(tcpClient, wifiLineBuffer);
  }
}

void loop() {
  consumeChars(Serial, lineBuffer);
  pollWifiClient();

  updateMotion();
}

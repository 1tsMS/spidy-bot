#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ================= CONFIG =================

#define PCA_ADDRESS 0x40
#define PCA_FREQ    50

#define SERVO_MIN   100
#define SERVO_MAX   700

#define SERVO_COUNT 12

Adafruit_PWMServoDriver pca(PCA_ADDRESS);

// ================= SERVO REMAP =================
// Logical index -> PCA9685 channel
uint8_t servoMap[SERVO_COUNT] = {
  0,  1,  2,
  3,  4,  5,
  6,  7,  8,
  9, 10, 11
};

// ================= CALIBRATION DATA =================

float servoOffset[SERVO_COUNT] = {
  0, 0, 0,
  0, 0, 0,
  0, 0, 0,
  0, 0, 0
};

float servoMinAngle[SERVO_COUNT] = {
  30, 30, 30,
  30, 30, 30,
  30, 30, 30,
  30, 30, 30
};

float servoMaxAngle[SERVO_COUNT] = {
  150, 150, 150,
  150, 150, 150,
  150, 150, 150,
  150, 150, 150
};

// ================= STATE =================

float currentAngle[SERVO_COUNT];

// ================= CORE FUNCTIONS =================

uint16_t angleToPulse(float angle) {
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}

void writeServoRaw(uint8_t logicalIndex, float angle) {
  if (logicalIndex >= SERVO_COUNT) return;

  angle = constrain(angle,
                    servoMinAngle[logicalIndex],
                    servoMaxAngle[logicalIndex]);

  uint8_t channel = servoMap[logicalIndex];
  pca.setPWM(channel, 0, angleToPulse(angle));

  currentAngle[logicalIndex] = angle;
}

void writeServo(uint8_t logicalIndex, float angle) {
  writeServoRaw(logicalIndex, angle + servoOffset[logicalIndex]);
}

// ================= SERIAL PARSER =================

void handleCommand(String cmd) {
  cmd.trim();

  // Expected: SET,index,angle
  if (cmd.startsWith("SET")) {

    int firstComma  = cmd.indexOf(',');
    int secondComma = cmd.indexOf(',', firstComma + 1);

    if (firstComma == -1 || secondComma == -1) {
      Serial.println("ERR:FORMAT");
      return;
    }

    int index = cmd.substring(firstComma + 1, secondComma).toInt();
    float angle = cmd.substring(secondComma + 1).toFloat();

    if (index < 0 || index >= SERVO_COUNT) {
      Serial.println("ERR:INDEX");
      return;
    }

    writeServo(index, angle);

    Serial.print("OK,");
    Serial.print(index);
    Serial.print(",");
    Serial.println(currentAngle[index]);
  }
}

// ================= SETUP =================

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(21, 22);
  pca.begin();
  pca.setPWMFreq(PCA_FREQ);

  delay(500);

  Serial.println("Spidy Bot Ready");
}

// ================= LOOP =================

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    handleCommand(cmd);
  }
}

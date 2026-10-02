#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// -----------------------
// Hardcoded Walk Test (No IK)
// -----------------------
// Uses only direct logical servo angles.
// Calibration arrays below apply direction + offset + min/max + channel mapping.

#define SERVO_MIN 100
#define SERVO_MAX 700
#define SERVO_FREQ 50
#define SERVO_COUNT 16
#define JOINT_COUNT 12

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Calibration (from spidy_calibration3)
static const int8_t CALIB_DIR[16] = {
  -1, +1, -1, -1, -1, +1, +1, +1,
  -1, +1, -1, +1, +1, +1, +1, +1
};

static const int8_t CALIB_OFFSET[16] = {
  +3, -3, -2, -6, +0, -4, +1, +0,
  -1, +4, +0, +6, +0, +0, +0, +0
};

static const uint8_t CALIB_MIN[16] = {
  28, 22, 41, 57, 24, 44, 51, 17,
  49, 4, 26, 44, 0, 0, 0, 0
};

static const uint8_t CALIB_MAX[16] = {
  129, 153, 180, 174, 180, 158, 149, 158,
  180, 131, 180, 150, 180, 180, 180, 180
};

static const uint8_t CALIB_INDEX[16] = {
  0, 1, 2, 3, 4, 5, 11, 10,
  9, 15, 14, 13, 6, 7, 8, 12
};

// Leg layout:
// BL: 0,1,2  FL: 3,4,5  FR: 6,7,8  BR: 9,10,11

static const float POSE_STAND[JOINT_COUNT] = {
  90, 105, 75,
  90, 105, 75,
  90, 105, 75,
  90, 105, 75
};

// Built from spidy_poses.txt Pose 10-13:
// - BL leg motion copied from Pose 10->13
// - FR mirrors BL hip around 90 (mirror), while keeping same knee/ankle profile
// - Adjacent hips (FL and BR) are turned oppositely for balance
static const float WALK_P10[JOINT_COUNT] = {
  90, 115, 75,    // BL from Pose 10
  80, 105, 75,    // FL adjacent hip turned (from Pose 10)
  90, 115, 75,    // FR mirror of BL
  100, 105, 75    // BR mirror of adjacent hip
};

static const float WALK_P11[JOINT_COUNT] = {
  100, 105, 75,   // BL from Pose 11
  80,  105, 75,   // FL adjacent hip turned
  80,  105, 75,   // FR mirrored hip of BL (100 -> 80)
  100, 105, 75    // BR mirror of adjacent hip
};

static const float WALK_P12[JOINT_COUNT] = {
  100, 105, 75,   // BL from Pose 12
  90,  105, 75,   // FL returns toward stand
  80,  105, 75,   // FR mirrored hip of BL
  90,  105, 75    // BR returns toward stand
};

static const float WALK_P13[JOINT_COUNT] = {
  90, 105, 75,
  90, 105, 75,
  90, 105, 75,
  90, 105, 75
};

float currentPose[JOINT_COUNT] = {
  90, 105, 75,
  90, 105, 75,
  90, 105, 75,
  90, 105, 75
};

void setServo(uint8_t logicalId, float logicalAngle) {
  if (logicalId >= SERVO_COUNT) return;

  int value = constrain((int)(logicalAngle + 0.5f), 0, 180);

  if (CALIB_DIR[logicalId] < 0) {
    value = 180 - value;
  }

  int offset = CALIB_OFFSET[logicalId];
  if (CALIB_DIR[logicalId] < 0) {
    offset = -offset;
  }
  value += offset;

  value = constrain(value, (int)CALIB_MIN[logicalId], (int)CALIB_MAX[logicalId]);

  int pulse = map(value, 0, 180, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(CALIB_INDEX[logicalId], 0, pulse);
}

void applyPose(const float pose[JOINT_COUNT]) {
  for (uint8_t i = 0; i < JOINT_COUNT; i++) {
    setServo(i, pose[i]);
    currentPose[i] = pose[i];
  }
}

void movePoseSmooth(const float target[JOINT_COUNT], uint16_t durationMs) {
  const uint8_t steps = 25;
  const uint16_t stepDelay = durationMs / steps;

  float start[JOINT_COUNT];
  for (uint8_t i = 0; i < JOINT_COUNT; i++) {
    start[i] = currentPose[i];
  }

  for (uint8_t s = 1; s <= steps; s++) {
    float t = (float)s / (float)steps;
    for (uint8_t i = 0; i < JOINT_COUNT; i++) {
      float value = start[i] + (target[i] - start[i]) * t;
      setServo(i, value);
    }
    delay(stepDelay);
  }

  for (uint8_t i = 0; i < JOINT_COUNT; i++) {
    currentPose[i] = target[i];
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(100);

  Serial.println("Hardcoded walk mode (no IK)");
  applyPose(POSE_STAND);
  delay(800);
}

void loop() {
  movePoseSmooth(WALK_P10, 300);
  delay(50);
  movePoseSmooth(WALK_P11, 300);
  delay(50);
  movePoseSmooth(WALK_P12, 300);
  delay(50);
  movePoseSmooth(WALK_P13, 300);
  delay(50);
}


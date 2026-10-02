#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

// ---- Servo calibration (adjust if needed) ----
#define SERVO_MIN   100
#define SERVO_MAX   700
#define SERVO_CENTER ((SERVO_MIN + SERVO_MAX) / 2)
#define SERVO_30_DEG ((SERVO_MAX - SERVO_MIN) / 6)

#define SERVO_60   (SERVO_CENTER - SERVO_30_DEG)
#define SERVO_120  (SERVO_CENTER + SERVO_30_DEG)

// ---- Speed control ----
#define STEP_DELAY  2   // ms per pulse step

// ---- SINGLE SERVO FUNCTION ----
void moveServoSmooth(uint8_t index, int startPulse, int endPulse) {
  int step = (endPulse > startPulse) ? 1 : -1;

  for (int pulse = startPulse; pulse != endPulse; pulse += step) {
    pca.setPWM(index, 0, pulse);
    delay(STEP_DELAY);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);
  pca.begin();
  pca.setPWMFreq(50);

  // Center all servos initially
  for (int i = 0; i < 16; i++) {
    pca.setPWM(i, 0, SERVO_CENTER);
  }

  Serial.println("Servo test ready");
}

void loop() {

  // -------- MOVE TO 60° --------
  Serial.println("State: 60 degrees");
  for (int i = 0; i < 16; i++) {
    moveServoSmooth(i, SERVO_CENTER, SERVO_60);
  }

  delay(500);

  // -------- MOVE TO 120° --------
  Serial.println("State: 120 degrees");
  for (int i = 0; i < 16; i++) {
    moveServoSmooth(i, SERVO_60, SERVO_120);
  }

  delay(500);

  // -------- MOVE BACK TO 90° --------
  Serial.println("State: 90 degrees (center)");
  for (int i = 0; i < 16; i++) {
    moveServoSmooth(i, SERVO_120, SERVO_CENTER);
  }

  delay(500);
}

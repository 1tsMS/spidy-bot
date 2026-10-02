#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// -------- PCA9685 --------
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

// -------- MPU --------
Adafruit_MPU6050 mpu;

// Servo pulse calibration (adjust if needed)
#define SERVO_MIN  100
#define SERVO_MAX  700

// Derived values
#define SERVO_CENTER ((SERVO_MIN + SERVO_MAX) / 2)
#define SERVO_30_DEG ((SERVO_MAX - SERVO_MIN) / 6)

#define SERVO_LOW  (SERVO_CENTER - SERVO_30_DEG)
#define SERVO_HIGH (SERVO_CENTER + SERVO_30_DEG)

// Speed → delay mapping
int speedToDelay(uint8_t speedPercent) {
  speedPercent = constrain(speedPercent, 1, 100);
  return map(speedPercent, 1, 100, 20, 1); // ms per step
}

// ---- Move all 16 servos together, smoothly ----
void moveAllServosSmooth(int startPulse, int endPulse, uint8_t speedPercent) {
  int step = (endPulse > startPulse) ? 1 : -1;
  int stepDelay = speedToDelay(speedPercent);

  for (int pulse = startPulse; pulse != endPulse; pulse += step) {

    // Move all 16 channels together
    for (int i = 0; i < 16; i++) {
      pca.setPWM(i, 0, pulse);
    }

    // ----- MPU READ -----
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    Serial.print("ACC X:");
    Serial.print(a.acceleration.x);
    Serial.print(" Y:");
    Serial.print(a.acceleration.y);
    Serial.print(" Z:");
    Serial.print(a.acceleration.z);

    Serial.print(" | GYRO X:");
    Serial.print(g.gyro.x);
    Serial.print(" Y:");
    Serial.print(g.gyro.y);
    Serial.print(" Z:");
    Serial.println(g.gyro.z);

    delay(stepDelay);
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin(21, 22);

  // ---- PCA INIT ----
  pca.begin();
  pca.setPWMFreq(50);

  // ---- MPU INIT ----
  if (mpu.begin()) {
    Serial.println("MPU6050 OK");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  } else {
    Serial.println("MPU6050 NOT FOUND");
  }

  // Center all servos
  for (int i = 0; i < 16; i++) {
    pca.setPWM(i, 0, SERVO_CENTER);
  }

  Serial.println("System ready (±30° test)");
}

void loop() {
  // Move +30°
  moveAllServosSmooth(SERVO_CENTER, SERVO_HIGH, 40);

  // Move -30°
  moveAllServosSmooth(SERVO_HIGH, SERVO_LOW, 40);

  // Return to center
  moveAllServosSmooth(SERVO_LOW, SERVO_CENTER, 50);
}

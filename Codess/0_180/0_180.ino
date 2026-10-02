#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

#define SERVO_MIN  100
#define SERVO_MAX  700

void setup() {
  Wire.begin(21, 22);
  pca.begin();
  pca.setPWMFreq(50);
}

void loop() {
  // Sweep slowly to avoid gear damage
  for (int pulse = SERVO_MIN; pulse <= SERVO_MAX; pulse++) {
    for (int i = 0; i < 16; i++) {
      pca.setPWM(i, 0, pulse);
    }
    delay(1);
  }

  for (int pulse = SERVO_MAX; pulse >= SERVO_MIN; pulse--) {
    for (int i = 0; i < 16; i++) {
      pca.setPWM(i, 0, pulse);
    }
    delay(1);
  }
}

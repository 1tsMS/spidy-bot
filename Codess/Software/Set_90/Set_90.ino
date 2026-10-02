#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

// Use your calibrated values
#define SERVO_MIN  100
#define SERVO_MAX  700

void setup() {
  Wire.begin(21, 22);
  pca.begin();
  pca.setPWMFreq(50);

  // Mechanical ZERO = MID position
  int zeroPulse = (SERVO_MIN + SERVO_MAX) / 2;

  for (int i = 0; i < 16; i++) {
    pca.setPWM(i, 0, zeroPulse);
  }

  // Hold position forever
  while (1) {
    delay(1000);
  }
}

void loop() {}

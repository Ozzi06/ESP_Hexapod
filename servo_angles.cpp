#include "servo_angles.h"

Adafruit_PWMServoDriver pwm0 = Adafruit_PWMServoDriver(0x40, Wire);
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x41, Wire);

void setupPwm(){
  Wire.begin(I2C_SDA, I2C_SCL);

  if (!pwm0.begin()) {
    Serial.println("PCA9685 0 not found! Servo control will not work.");
  }
  if (!pwm1.begin()) {
    Serial.println("PCA9685 1 not found! Servo control will not work.");
  }
  
  pwm0.setOscillatorFrequency(27000000);
  pwm1.setOscillatorFrequency(27000000);
  pwm0.setPWMFreq(SERVO_FREQ);
  pwm1.setPWMFreq(SERVO_FREQ);
  delay(10);
}
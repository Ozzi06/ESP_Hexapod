#ifndef UTILS_H
#define UTILS_H

// Define I2C pins for Xiao ESP32-S3
#define I2C_SDA 5
#define I2C_SCL 6

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm0 = Adafruit_PWMServoDriver(0x40, Wire);
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x41, Wire);

struct Vec3{
  float x;
  float y;
  float z;

  Vec3 operator*(float other){
    return Vec3{other * x, other * y, other * z};
  }
};

void setupPwm(){
  Wire.begin(I2C_SDA, I2C_SCL);

  if (!pwm0.begin()) {
    while (1) Serial.println("PCA9685 0 not found!");
  }
  if (!pwm1.begin()) {
    while (1) Serial.println("PCA9685 1 not found!");
  }
  
  pwm0.setOscillatorFrequency(27000000);
  pwm1.setOscillatorFrequency(27000000);
  pwm0.setPWMFreq(SERVO_FREQ);
  pwm1.setPWMFreq(SERVO_FREQ);
  delay(10);
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float clampf(float val, float min, float max){
  if (val < min) val = min;
  if (val > max) val = max;
  return val;
}

float clampmapf(float x, float in_min, float in_max, float out_min, float out_max){
  float val = mapf(x, in_min, in_max, out_min, out_max);
  if (val > out_max) val = out_max;
  if (val < out_min) val = out_min;
  return val;
}

uint16_t get_pulse_from_angle_degrees(float angle){
  return(clampmapf(angle, -76.0, 76.0, SERVOMIN, SERVOMAX));
}
uint16_t get_pulse_from_angle_radians(float angle){
  return(clampmapf(angle, -3.14159 * (76.0/180.0),3.14159 * (76.0/180.0), SERVOMIN, SERVOMAX));
}

void setAnglePulse(uint8_t servo_idx, uint16_t pulse){
  if(!(servo_idx / 9)){
    pwm0.setPWM(servo_idx, 0, pulse);
  }
  else{
    pwm1.setPWM(servo_idx - 9, 0, pulse);
  }
}

void setAngleDegrees(uint8_t servo_idx, float angle){
  setAnglePulse(servo_idx, get_pulse_from_angle_degrees(angle));
}

void setAngleRadians(uint8_t servo_idx, float angle){
  setAnglePulse(servo_idx, get_pulse_from_angle_radians(angle));
}

#endif
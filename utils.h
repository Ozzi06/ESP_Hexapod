#ifndef UTILS_H
#define UTILS_H

// Define I2C pins for Xiao ESP32-S3
#define I2C_SDA 5
#define I2C_SCL 6

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm0 = Adafruit_PWMServoDriver(0x40, Wire);
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x41, Wire);

//#include "Vec3.h" //not needed du eto quat including it
#include "quat.h"
#include "robot_spec.h"
#include <math.h>

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

uint16_t get_pulse_from_angle_degrees(uint8_t servo_idx, float angle){
  return(clampmapf(angle - servo_center_angle[servo_idx % 3] * 180.0f / M_PI, -76.0f, 76.0f, SERVOMIN, SERVOMAX));
}
uint16_t get_pulse_from_angle_radians(uint8_t servo_idx, float angle){
  return(clampmapf(angle - servo_center_angle[servo_idx % 3], -M_PI * (76.0/180.0), M_PI * (76.0/180.0), SERVOMIN, SERVOMAX));
}

void setAnglePulse(uint8_t servo_idx, uint16_t pulse){
  #ifdef OSSIAN_HEMMA
  latestServoAngles[servo_idx] = (((float)(pulse - SERVOMIN) * (2.0f * (76.0f * M_PI / 180.0f)) / (float)(SERVOMAX - SERVOMIN)) - (76.0f * M_PI / 180.0f))
                                  + servo_center_angle[servo_idx % 3];
  #endif
  #ifndef OSSIAN_HEMMA
  if(!(servo_idx / 9)){
    pwm0.setPWM(servo_idx, 0, pulse);
  }
  else{
    pwm1.setPWM(servo_idx - 9, 0, pulse);
  }
  #endif
}

void setAngleDegrees(uint8_t servo_idx, float angle){
  setAnglePulse(servo_idx, get_pulse_from_angle_degrees(servo_idx, angle));
}

void setAngleRadians(uint8_t servo_idx, float angle){
  setAnglePulse(servo_idx, get_pulse_from_angle_radians(servo_idx, angle));
}

#endif
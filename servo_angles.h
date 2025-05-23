#ifndef SERVO_ANGLES_H
#define SERVO_ANGLES_H

// Define I2C pins for Xiao ESP32-S3
#define I2C_SDA 5
#define I2C_SCL 6

#define SERVOMIN  175 //-71 degrees (assume -76 degrees)
#define SERVOMAX  535 //81 degrees (Assume 76 degrees)
#define SERVOMIDDLE 355
#define SERVO_FREQ 50

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
extern Adafruit_PWMServoDriver pwm0;
extern Adafruit_PWMServoDriver pwm1;

#include "robot_spec.h"
#include <math.h>

void setupPwm();

inline uint8_t get_servo_type(uint8_t servo_channel) {
  for (uint8_t leg = 0; leg < 6; leg++) {
    for (uint8_t joint = 0; joint < 3; joint++) {
      if (LEG_SERVOS[leg][joint] == servo_channel) {
        return joint;
      }
    }
  }
  Serial.printf("[WARN] get_servo_type: servo_channel %d not found.\n", servo_channel); // Optional warning
  return 255; // or some other error code
}

inline uint16_t get_pulse_from_angle_degrees(uint8_t servo_channel, float angle){
  return(clampmapf(angle - servo_center_angle[get_servo_type(servo_channel)] * 180.0f / M_PI, -76.0f, 76.0f, SERVOMIN, SERVOMAX));
}
inline uint16_t get_pulse_from_angle_radians(uint8_t servo_channel, float angle){
  return(clampmapf(angle - servo_center_angle[get_servo_type(servo_channel)], -M_PI * (76.0/180.0), M_PI * (76.0/180.0), SERVOMIN, SERVOMAX));
}

//used to get a compact joint order for latestServoAngles
inline uint8_t get_joint_idx_from_servo_channel(uint8_t servo_channel) {
    for (uint8_t leg_idx = 0; leg_idx < LEG_COUNT; ++leg_idx) {
        for (uint8_t joint_type_in_leg = 0; joint_type_in_leg < 3; ++joint_type_in_leg) {
            // joint_type_in_leg here is 0 for coxa, 1 for femur, 2 for tibia for that leg
            if (LEG_SERVOS[leg_idx][joint_type_in_leg] == servo_channel) {
                return (leg_idx * 3) + joint_type_in_leg; // This calculates the 0-17 index
            }
        }
    }
    Serial.printf("[WARN] get_joint_idx_from_servo_channel: servo_channel %d not found.\n", servo_channel);
    return 255; // Error: servo_channel not found in LEG_SERVOS mapping
}

inline void setAnglePulse(uint8_t servo_channel, uint16_t pulse){
  uint8_t joint_idx = get_joint_idx_from_servo_channel(servo_channel);
  latestServoAngles[joint_idx] = (((float)(pulse - SERVOMIN) * (2.0f * (76.0f * M_PI / 180.0f)) / (float)(SERVOMAX - SERVOMIN)) - (76.0f * M_PI / 180.0f))
                                  + servo_center_angle[get_servo_type(servo_channel)];
  #ifndef SCRAP_MECHANIC
  if((servo_channel < 16)){
    pwm0.setPWM(servo_channel, 0, pulse);
  }
  else{
    pwm1.setPWM(servo_channel - 16, 0, pulse);
  }
  #endif
}

inline void setAngleDegrees(uint8_t servo_channel, float angle){
  setAnglePulse(servo_channel, get_pulse_from_angle_degrees(servo_channel, angle));
}

inline void setAngleRadians(uint8_t servo_channel, float angle){
  setAnglePulse(servo_channel, get_pulse_from_angle_radians(servo_channel, angle));
}

#endif //SERVO_ANGLES_H  
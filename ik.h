// ik_servo.h
#ifndef IK_SERVO_H
#define IK_SERVO_H

#include "utils.h"
#include <math.h>

// Leg mechanical parameters (in cm)
#define COXA_LENGTH  7.0    // Length from hip to knee joint
#define FEMUR_LENGTH 10.0    // Length from knee to ankle joint
#define TIBIA_LENGTH 11.6   // Length from ankle to foot

// Joint angle limits (in radians)
#define COXA_MIN_ANGLE    (-60 * M_PI / 180.0)
#define COXA_MAX_ANGLE     (60 * M_PI / 180.0)
#define FEMUR_MIN_ANGLE   (-70 * M_PI / 180.0)
#define FEMUR_MAX_ANGLE    (70 * M_PI / 180.0)
#define TIBIA_MIN_ANGLE   (-70 * M_PI / 180.0)
#define TIBIA_MAX_ANGLE    (70 * M_PI / 180.0)

#define LEG_COUNT 1

// Servo channel assignments per leg (adjust based on your setup)
const uint8_t LEG_SERVOS[LEG_COUNT][3] = {  // [leg][joint] where joints are 0:coxa, 1:femur, 2:tibia
  {9, 10, 11},   // Leg 0 servos
};


Vec3 legTargets[LEG_COUNT];  // One target position per leg

bool legActive[LEG_COUNT] = {false};  // Which legs are active
uint8_t currentLeg = 0;     // Currently selected leg for commands

void setupIK() {
  // Initialize all legs to default positions
  for (uint8_t i = 0; i < LEG_COUNT; i++) {
    legTargets[i] = {25, 0, 0};
    legActive[i] = false;
  }
}

bool calculateIK(uint8_t leg, float x, float y, float z, float& coxaAngle, float& femurAngle, float& tibiaAngle) {
  const float lC = COXA_LENGTH;
  const float lF = FEMUR_LENGTH;
  const float lT = TIBIA_LENGTH;

  // Calculate coxa angle (yaw)
  coxaAngle = atan2(y, x);
  if (coxaAngle < COXA_MIN_ANGLE || coxaAngle > COXA_MAX_ANGLE) {
    Serial.println("Coxa angle out of range!");
    return false;
  }

  //distance between femur joint and foot
  float h; {
    float hhor = sqrt(x*x + y*y) - lC;
    h = sqrt(hhor*hhor + z*z);
    
    // Check if position is reachable
    if (h > (lF + lT) || h < fabs(lF - lT)) {
      Serial.println("Target position unreachable!");
      return false;
    }
  }

  //femurAngle
  {
    float A = -acos(
      (lF*lF + h*h - lT*lT) / 
      (2*lF*h)
    );

    float ah = asin(z/h);
    femurAngle = A-ah;
    
    if (femurAngle < FEMUR_MIN_ANGLE || femurAngle > FEMUR_MAX_ANGLE) {
      Serial.println("Femur angle out of range!");
      return false;
    }
  }

  //tibiaAngle
  {
    float C = acos(
      (lF*lF + lT*lT - h*h) / 
      (2*lF*lT)
    );

    tibiaAngle = 3.14159 - C;

    if (tibiaAngle < TIBIA_MIN_ANGLE || tibiaAngle > TIBIA_MAX_ANGLE) {
      Serial.println("Tibia angle out of range!");
      return false;
    }
  }


  return true;
}

void moveLegToTarget(uint8_t leg, bool log = false) {
  //if (!legActive[leg]) return;

  float coxa, femur, tibia;
  if (calculateIK(leg, legTargets[leg].x, legTargets[leg].y, legTargets[leg].z, coxa, femur, tibia)) {
    setAngleRadians(LEG_SERVOS[leg][0], coxa);    // Coxa
    setAngleRadians(LEG_SERVOS[leg][1], femur);   // Femur
    setAngleRadians(LEG_SERVOS[leg][2], tibia);   // Tibia
    
    if(log){
      Serial.print("Moved leg ");
      Serial.print(leg);
      Serial.print(" to X:");
      Serial.print(legTargets[leg].x);
      Serial.print(" Y:");
      Serial.print(legTargets[leg].y);
      Serial.print(" Z:");
      Serial.println(legTargets[leg].z);
      Serial.print("angles: ");
      Serial.print(coxa / M_PI * 180);
      Serial.print(" ; ");
      Serial.print(femur / M_PI * 180);
      Serial.print(" ; ");
      Serial.println(tibia / M_PI * 180);
    }

  }
}

void updateIK() {
  for (uint8_t i = 0; i < LEG_COUNT; i++) {
    if (legActive[i]) {
      moveLegToTarget(i);
    }
  }
}

#endif
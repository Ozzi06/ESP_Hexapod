// ik.h
#ifndef IK_H
#define IK_H

#include "utils.h"
#include <math.h>
#include "robot_spec.h"

bool calculateIK(uint8_t leg, float x, float y, float z, float& coxaAngle, float& femurAngle, float& tibiaAngle) {
  const float lC = COXA_LENGTH;
  const float lF = FEMUR_LENGTH;
  const float lT = TIBIA_LENGTH;

  // Calculate coxa angle (yaw)
  coxaAngle = atan2(y, x);
  if (coxaAngle < COXA_MIN_ANGLE || coxaAngle > COXA_MAX_ANGLE) {
    Serial.print("Coxa angle out of range(");
    Serial.print(coxaAngle / M_PI * 180.0f);
    Serial.print(" degrees)");
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
      Serial.print("Femur angle out of range(");
      Serial.print(femurAngle / M_PI * 180.0f);
      Serial.print(" degrees)");
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
      Serial.print("Tibia angle out of range(");
      Serial.print(tibiaAngle / M_PI * 180.0f);
      Serial.print(" degrees)");
      return false;
    }
  }


  return true;
}

// Calculates IK and sends commands to move a specific leg to a target position
// specified in the Leg's Local IK Frame.
// It now takes the target position as an argument instead of reading a global.
void moveLegToTarget(uint8_t leg, const Vec3& target_leg_ik, bool log = false) {
    // Calculate IK angles required to reach the target_leg_ik position {x,y,z}
    float coxa_rad, femur_rad, tibia_rad;
    if (calculateIK(leg, target_leg_ik.x, target_leg_ik.y, target_leg_ik.z, coxa_rad, femur_rad, tibia_rad)) {
        // IK successful, send angles to servos
        // Note: LEG_SERVOS is now defined in robot_spec.cpp / declared in robot_spec.h
        setAngleRadians(LEG_SERVOS[leg][0], coxa_rad);    // Coxa
        setAngleRadians(LEG_SERVOS[leg][1], femur_rad);   // Femur
        setAngleRadians(LEG_SERVOS[leg][2], tibia_rad);   // Tibia

        if (log) {
            Serial.print("Attempting move leg ");
            Serial.print(leg_names[leg]); // Use names for clarity
            Serial.print(" ("); Serial.print(leg); Serial.print(")");
            Serial.print(" to LegIK Target: X:"); Serial.print(target_leg_ik.x, 2);
            Serial.print(" Y:"); Serial.print(target_leg_ik.y, 2);
            Serial.print(" Z:"); Serial.println(target_leg_ik.z, 2);
            Serial.print("  Calculated Angles (Deg): Coxa="); Serial.print(coxa_rad * 180.0f / M_PI, 1);
            Serial.print(" Femur="); Serial.print(femur_rad * 180.0f / M_PI, 1);
            Serial.print(" Tibia="); Serial.println(tibia_rad * 180.0f / M_PI, 1);
        }
    } else {
        // IK failed (e.g., target unreachable or angle limits exceeded)
        if (log) {
            Serial.print("IK Failed for leg ");
            Serial.print(leg_names[leg]); // Use names
            Serial.print(" ("); Serial.print(leg); Serial.print(")");
            Serial.print(" -> Target LegIK: X:"); Serial.print(target_leg_ik.x, 2);
            Serial.print(" Y:"); Serial.print(target_leg_ik.y, 2);
            Serial.print(" Z:"); Serial.println(target_leg_ik.z, 2);
        }
        // Optional: Decide what to do on failure. Currently, servos just hold their last position.
        // You could command them to a safe default, log more details, etc.
    }
}

#endif
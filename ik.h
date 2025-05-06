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
  if (coxaAngle < COXA_MIN_ANGLE + servo_center_angle[0] || coxaAngle > COXA_MAX_ANGLE + servo_center_angle[0]) {
    Serial.print("Coxa angle out of range(");
    Serial.print(coxaAngle / M_PI * 180.0f);
      Serial.println(" degrees)");

      Serial.print("Range: ");
      Serial.print((COXA_MIN_ANGLE + servo_center_angle[0])*180.0/M_PI);
      Serial.print(" to ");
      Serial.println((COXA_MAX_ANGLE + servo_center_angle[0])*180.0/M_PI);
      
      Serial.print("Target coords: ");
      Serial.print(x);
      Serial.print(", ");
      Serial.print(y);
      Serial.print(", ");
      Serial.println(z);
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
    float A = acos(
      (lF*lF + h*h - lT*lT) / 
      (2*lF*h)
    );

    float ah = asin(z/h);
    femurAngle = A + ah;
    femurAngle = fmod(femurAngle + M_PI, 2 * M_PI) - M_PI; //remapping it into the correct range -pi to +pi
    
    if (femurAngle < FEMUR_MIN_ANGLE + servo_center_angle[1] || femurAngle > FEMUR_MAX_ANGLE + servo_center_angle[1]) {
      Serial.print("Femur angle out of range(");
      Serial.print(femurAngle / M_PI * 180.0f);
      Serial.println(" degrees)");

      Serial.print("Range: ");
      Serial.print((FEMUR_MIN_ANGLE + servo_center_angle[1])*180.0/M_PI);
      Serial.print(" to ");
      Serial.println((FEMUR_MAX_ANGLE + servo_center_angle[1])*180.0/M_PI);
      
      Serial.print("Target coords: ");
      Serial.print(x);
      Serial.print(", ");
      Serial.print(y);
      Serial.print(", ");
      Serial.println(z);
      return false;
    }
  }

  //tibiaAngle
  {
    float C = -acos( //Law of cosines
      (lF*lF + lT*lT - h*h) / 
      (2*lF*lT)
    );

    tibiaAngle = M_PI - C;
    tibiaAngle = fmod(tibiaAngle + M_PI, 2 * M_PI) - M_PI; //remapping it into the correct range -pi to +pi

    if (tibiaAngle < TIBIA_MIN_ANGLE + servo_center_angle[2] || tibiaAngle > TIBIA_MAX_ANGLE + servo_center_angle[2]) {
      Serial.print("Tibia angle out of range(");
      Serial.print(tibiaAngle / M_PI * 180.0f);
      Serial.println(" degrees)");

      Serial.print("Range: ");
      Serial.print((TIBIA_MIN_ANGLE + servo_center_angle[2])*180.0/M_PI);
      Serial.print(" to ");
      Serial.println((TIBIA_MAX_ANGLE + servo_center_angle[2])*180.0/M_PI);
      
      Serial.print("Target coords: ");
      Serial.print(x);
      Serial.print(", ");
      Serial.print(y);
      Serial.print(", ");
      Serial.println(z);
      return false;
    }
  }


  return true;
}


void moveLegToTarget(uint8_t leg, const Vec3& target_leg_ik, bool log = false) {
    float coxa_rad, femur_rad, tibia_rad;

    if (calculateIK(leg, target_leg_ik.x, target_leg_ik.y, target_leg_ik.z, coxa_rad, femur_rad, tibia_rad)) {
        // IK successful, send angles to servos
        setAngleRadians(LEG_SERVOS[leg][0], coxa_rad);
        setAngleRadians(LEG_SERVOS[leg][1], femur_rad);
        setAngleRadians(LEG_SERVOS[leg][2], tibia_rad);

        // Original logging (optional, can be combined with FK logging)
        // if (log) { ... }

    } else {
        // IK failed
        if (log) {
            Serial.print("[IK Failed] Leg "); Serial.print(leg_names[leg]); Serial.print(" ("); Serial.print(leg); Serial.print(")");
            Serial.print(" -> Target LegIK: "); target_leg_ik.print(""); // Use Vec3 print helper
        }
    }
}

#endif
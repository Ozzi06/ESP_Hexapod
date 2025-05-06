#ifndef SET_POS_H
#define SET_POS_H

#include "ik.h"

bool ikRunning = false;
uint8_t currentLeg = 0;
Vec3 legTargets[LEG_COUNT];
bool legActive[LEG_COUNT] = {true, true, true, true, true, true};

void setupIKPostitioning(){
  ikRunning = false;

  Serial.println("\n==== Leg IK Control Program ====");
  Serial.println("Commands:");
  Serial.println("L[leg] - Select leg (0-3)");
  Serial.println("A - Toggle all legs");
  Serial.println("x[value] - Set x position (cm)");
  Serial.println("y[value] - Set y position (cm)");
  Serial.println("z[value] - Set z position (cm)");
  Serial.println("M - Move to current coordinates");
  Serial.println("G - Go/Start continuous IK updates");
  Serial.println("B - Break/Stop IK updates");
  Serial.println("H - Home position (retracted)");
  Serial.println("E - Extended position");
  Serial.println("D - Display current positions");
  Serial.println("X - Exit to main menu");
  Serial.println("========================");
}

bool processIKCommands() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {
      char command = input.charAt(0);
      input = input.substring(1);

      switch (command) {
        case 'X':
          ikRunning = false;
          Serial.println("Exiting IK program");
          return false;

        case 'B':
          ikRunning = false;
          Serial.println("IK updates stopped");
          break;

        case 'G':
          ikRunning = true;
          Serial.println("IK updates started");
          break;

        case 'L':
          {
            uint8_t leg = input.toInt();
            if (leg < LEG_COUNT) {
              currentLeg = leg;
              Serial.print("Selected leg ");
              Serial.println(leg);
            }
          }
          break;

        case 'A':
          {
            bool newState = !legActive[0];
            for (uint8_t i = 0; i < LEG_COUNT; i++) {
              legActive[i] = newState;
            }
            Serial.print("All legs are now ");
            Serial.println(newState ? "active" : "inactive");
          }
          break;

        case 'x':
          legTargets[currentLeg].x = input.toFloat();
          Serial.print("Leg ");
          Serial.print(currentLeg);
          Serial.print(" X set to ");
          Serial.println(legTargets[currentLeg].x);
          break;

        case 'y':
          legTargets[currentLeg].y = input.toFloat();
          Serial.print("Leg ");
          Serial.print(currentLeg);
          Serial.print(" Y set to ");
          Serial.println(legTargets[currentLeg].y);
          break;

        case 'z':
          legTargets[currentLeg].z = input.toFloat();
          Serial.print("Leg ");
          Serial.print(currentLeg);
          Serial.print(" Z set to ");
          Serial.println(legTargets[currentLeg].z);
          break;

        case 'M':
          moveLegToTarget(currentLeg, legTargets[currentLeg]);
          break;

        case 'H':
          for (uint8_t i = 0; i < LEG_COUNT; i++) {
            legTargets[i] = {25, 0, 0};
            if (legActive[i]) moveLegToTarget(i, legTargets[i]);
          }
          Serial.println("All legs moved to home position");
          break;

        case 'E':
          for (uint8_t i = 0; i < LEG_COUNT; i++) {
            legTargets[i] = {0, COXA_LENGTH + FEMUR_LENGTH, -TIBIA_LENGTH};
            if (legActive[i]) moveLegToTarget(i, legTargets[i]);
          }
          Serial.println("All legs moved to extended position");
          break;

        case 'D':
          for (uint8_t i = 0; i < LEG_COUNT; i++) {
            Serial.print("Leg ");
            Serial.print(i);
            Serial.print(": X=");
            Serial.print(legTargets[i].x);
            Serial.print(" Y=");
            Serial.print(legTargets[i].y);
            Serial.print(" Z=");
            Serial.println(legTargets[i].z);
          }
          break;

        default:
          Serial.println("Unknown command");
          break;
      }
    }
  }
  return true;
}

bool IKpositioning() {
  if (ikRunning) {
    for (uint8_t i = 0; i < LEG_COUNT; i++) {
      if (legActive[i]) moveLegToTarget(i, legTargets[i]);
    }
  }

  return processIKCommands();
}

#endif
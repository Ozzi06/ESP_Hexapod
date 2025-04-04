#ifndef SET_POS_H
#define SET_POS_H

#include "ik.h"
bool ikRunning = false;

void setupIKPostitioning(){
  setupIK();
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
      input = input.substring(1); // Remove command character
      
      switch (command) {
        case 'X': // Exit program
          ikRunning = false;
          Serial.println("Exiting IK program");
          return false;
          
        case 'B': // Break/Stop
          ikRunning = false;
          Serial.println("IK updates stopped");
          break;
          
        case 'G': // Go/Start
          ikRunning = true;
          Serial.println("IK updates started");
          break;
          
        case 'L': // Select leg
          {
            uint8_t leg = input.toInt();
            if (leg < LEG_COUNT) {
              currentLeg = leg;
              Serial.print("Selected leg ");
              Serial.println(leg);
            }
          }
          break;
          
        case 'A': // Toggle all legs
          {
            bool newState = !legActive[0]; // Toggle based on first leg
            for (uint8_t i = 0; i < LEG_COUNT; i++) {
              legActive[i] = newState;
            }
            Serial.print("All legs are now ");
            Serial.println(newState ? "active" : "inactive");
          }
          break;
          
        case 'x': // Set X position
          legTargets[currentLeg].x = input.toFloat();
          Serial.print("Leg ");
          Serial.print(currentLeg);
          Serial.print(" X set to ");
          Serial.println(legTargets[currentLeg].x);
          break;
          
        case 'y': // Set Y position
          legTargets[currentLeg].y = input.toFloat();
          Serial.print("Leg ");
          Serial.print(currentLeg);
          Serial.print(" Y set to ");
          Serial.println(legTargets[currentLeg].y);
          break;
          
        case 'z': // Set Z position
          legTargets[currentLeg].z = input.toFloat();
          Serial.print("Leg ");
          Serial.print(currentLeg);
          Serial.print(" Z set to ");
          Serial.println(legTargets[currentLeg].z);
          break;
          
        case 'M': // Move to current coordinates
          moveLegToTarget(currentLeg);
          break;
          
        case 'H': // Home position (retracted)
          for (uint8_t i = 0; i < LEG_COUNT; i++) {
            legTargets[i] = {25, 0, 0};
            if (legActive[i]) moveLegToTarget(i);
          }
          Serial.println("All legs moved to home position");
          break;
          
        case 'E': // Extended position
          for (uint8_t i = 0; i < LEG_COUNT; i++) {
            legTargets[i] = {0, COXA_LENGTH + FEMUR_LENGTH, -TIBIA_LENGTH};
            if (legActive[i]) moveLegToTarget(i);
          }
          Serial.println("All legs moved to extended position");
          break;
          
        case 'D': // Display current positions
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
  if(ikRunning) updateIK();
  
  return processIKCommands();
}

#endif

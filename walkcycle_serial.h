#ifndef WALK_CYCLE_SERIAL_H
#define WALK_CYCLE_SERIAL_H

#include "walkcycle.h"

void setupWalkcycleSerial() {
  setupWalkcycle();
  
  Serial.println("\n==== Leg Walk Cycle Control ====");
  Serial.println("Commands:");
  Serial.println("x[value] - Set x velocity (cm/s)");
  Serial.println("y[value] - Set y velocity (cm/s)");
  Serial.println("z[value] - Set z velocity (cm/s)");
  Serial.println("G - Go/Start walk cycle");
  Serial.println("B - Break/Stop walk cycle");
  Serial.println("D - Display current status");
  Serial.println("X - Exit to main menu");
  Serial.println("S - Set step height");
  Serial.println("F - Set step frequency");
  Serial.println("========================");

}

bool processWalkCommands() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.length() > 0) {
      char command = input.charAt(0);
      input = input.substring(1); // Remove command character
      
      switch (command) {
        case 'X': // Exit program
          walkCycleRunning = false;
          Serial.println("Exiting walk cycle");
          return false;
          
        case 'B': // Break/Stop
          walkCycleRunning = false;
          Serial.println("Walk cycle stopped");
          break;
          
        case 'G': // Go/Start
          walkCycleRunning = true;
          globalPhase = 0.0f;
          Serial.println("Walk cycle started");
          break;
          
        case 'x': // Set X velocity
          bodyVelocity.x = input.toFloat();
          Serial.print("X velocity set to ");
          Serial.println(bodyVelocity.x);
          break;
          
        case 'y': // Set Y velocity
          bodyVelocity.y = input.toFloat();
          Serial.print("Y velocity set to ");
          Serial.println(bodyVelocity.y);
          break;
          
        case 'z': // Set Z velocity
          bodyVelocity.z = input.toFloat();
          Serial.print("Z velocity set to ");
          Serial.println(bodyVelocity.z);
          break;
          
        case 'S': // Set step height
          walkParams.stepHeight = input.toFloat();
          Serial.print("Step height set to ");
          Serial.println(walkParams.stepHeight);
          break;
          
        case 'F': // Set step frequency
          walkParams.stepFrequency = input.toFloat();
          Serial.print("Step frequency set to ");
          Serial.println(walkParams.stepFrequency);
          break;
          
        case 'D': // Display current status
          Serial.print("Body velocity: X=");
          Serial.print(bodyVelocity.x);
          Serial.print(" Y=");
          Serial.print(bodyVelocity.y);
          Serial.print(" Z=");
          Serial.println(bodyVelocity.z);
          
          Serial.print("Step height: ");
          Serial.print(walkParams.stepHeight);
          Serial.print("cm, Frequency: ");
          Serial.print(walkParams.stepFrequency);
          Serial.println("Hz");
          
          for (uint8_t i = 0; i < LEG_COUNT; i++) {
            Serial.print("Leg ");
            Serial.print(i);
            Serial.print(": ");
            Serial.print("X=");
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

bool walkcycleSerialUpdate() {
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();
  float dt = (now - lastUpdate) / 1000.0f;
  lastUpdate = now;
  
  if (walkCycleRunning) updateWalkCycle(dt);
  
  return processWalkCommands();
}

#endif
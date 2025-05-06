#include "utils.h"

// Movement parameters
#define DEFAULT_SPEED 5       // Steps per movement iteration
#define DEFAULT_SMOOTHNESS 10 // Delay between steps (ms)

struct ServoChannel {
  uint16_t currentPulse;
  uint16_t targetPulse;
};

ServoChannel channels[16]; // All 16 channels

uint8_t movementSpeed = DEFAULT_SPEED;
uint8_t smoothness = DEFAULT_SMOOTHNESS;


void centerAllServos() {
  for (uint8_t i = 0; i < 16; i++) {
    channels[i].currentPulse = SERVOMIDDLE;
    channels[i].targetPulse = SERVOMIDDLE;
    
    // Directly set the hardware pulse
    setAnglePulse(i, SERVOMIDDLE); 
  }
  Serial.println("All servos centered (Pulse: " + String(SERVOMIDDLE) + ")");
}

void setup_serial_control(){
  Serial.println("Xiao ESP32-S3 16-Channel Servo Control");
  Serial.println("Commands (angles in degrees, roughly -76 to +76):");
  Serial.println("M[channel],[angle] - Move servo (e.g., M5,30.5)"); // Updated help text
  Serial.println("P[angle] - Move all servos (e.g., P-15.0)");       // Updated help text
  Serial.println("S[speed] - Set movement speed (1-50)");
  Serial.println("D[delay] - Set smoothness delay (1-50ms)");
  Serial.println("C - Center all servos (Angle: 0.0)"); // Updated help text
  Serial.println("X - Exit this mode and return to main menu");
}

bool processSerialCommands() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.length() > 0) {
      char command = input.charAt(0);
      // Keep the rest of the string, including potential negative signs for angles
      String valueString = input.substring(1); 
      
      switch (command) {
        case 'X':
          Serial.println("Exiting serial control mode.");
          return false; // Signal to exit the mode/loop

        case 'M': // Move command: M[channel],[angle_degrees]
          {
            int commaIndex = valueString.indexOf(',');
            if (commaIndex > 0) {
              // Extract channel number
              String channelStr = valueString.substring(0, commaIndex);
              uint8_t channel = channelStr.toInt(); 

              // Extract angle value
              String angleStr = valueString.substring(commaIndex + 1);
              float angle_degrees = angleStr.toFloat(); 

              if (channel < 16) {
                // Calculate the target pulse from the angle
                uint16_t target_pulse = get_pulse_from_angle_degrees(0, angle_degrees); //TODO! this is not great since i have hardcoded the servo_idx

                // Set the target pulse for smooth movement
                channels[channel].targetPulse = target_pulse; 

                // Print confirmation with both angle and resulting pulse
                Serial.print("Moving channel ");
                Serial.print(channel);
                Serial.print(" to angle ");
                Serial.print(angle_degrees, 2); // Print angle with 2 decimal places
                Serial.print(" degrees (Pulse: ");
                Serial.print(target_pulse);
                Serial.println(")");
              } else {
                Serial.println("Invalid channel (0-15)");
              }
            } else {
              Serial.println("Invalid format. Use M[channel],[angle_degrees]");
            }
          }
          break;
        
        case 'P': // Move all servos: P[angle_degrees]
          {
            float angle_degrees = valueString.toFloat();
            uint16_t target_pulse = get_pulse_from_angle_degrees(0, angle_degrees); //TODO! this is not great since i have hardcoded the servo_idx

            for(int i = 0; i < 16; i++) {
              channels[i].targetPulse = target_pulse;
            }

            Serial.print("Moving all channels to angle ");
            Serial.print(angle_degrees, 2); // Print angle with 2 decimal places
            Serial.print(" degrees (Pulse: ");
            Serial.print(target_pulse);
            Serial.println(")");
          }
          break;
      
        case 'S': // Set speed: S[speed]
          {
            // Use valueString directly here as it contains the number after 'S'
            uint8_t speed = valueString.toInt(); 
            if (speed >= 1 && speed <= 50) {
              movementSpeed = speed;
              Serial.print("Movement speed set to ");
              Serial.println(movementSpeed);
            } else {
              Serial.println("Speed must be between 1-50");
            }
          }
          break;
          
        case 'D': // Set smoothness delay: D[delay]
          {
             // Use valueString directly here as it contains the number after 'D'
            uint8_t delayTime = valueString.toInt();
            if (delayTime >= 1 && delayTime <= 50) {
              smoothness = delayTime;
              Serial.print("Smoothness delay set to ");
              Serial.print(smoothness);
              Serial.println("ms");
            } else {
              Serial.println("Delay must be between 1-50ms");
            }
          }
          break;
          
        case 'C': // Center all servos
          centerAllServos(); // This already sets targetPulse and currentPulse to SERVOMIDDLE
          break;
          
        default:
          Serial.println("Unknown command");
          break;
      }
      
    }
    return true; // Command processed (or no command received), continue loop
  }
  return true; // No command available, continue loop
}

// moveServosSmoothly remains the same as it works with pulse values
void moveServosSmoothly() {
  bool changed = false; // Optional: check if any servo actually moved
  for (uint8_t i = 0; i < 16; i++) {
    if (channels[i].currentPulse != channels[i].targetPulse) {
      changed = true;
      // Calculate direction and step
      int16_t difference = channels[i].targetPulse - channels[i].currentPulse;
      
      // Determine the step size, ensuring we don't overshoot
      int16_t step;
      if (abs(difference) <= movementSpeed) {
          step = difference; // Take the final step
      } else {
          step = (difference > 0) ? movementSpeed : -movementSpeed;
      }
      
      channels[i].currentPulse += step;
      
      // Clamp currentPulse just in case (though step calculation should prevent overshoot)
      // channels[i].currentPulse = constrain(channels[i].currentPulse, SERVOMIN, SERVOMAX); 
      // Note: get_pulse_from_angle already clamps, so targetPulse is safe.
      // Clamping here might be redundant unless there's a bug elsewhere.

      setAnglePulse(i, channels[i].currentPulse); // Use the utility function to set PWM
    }
  }
  // Only delay if a servo actually moved or if smoothness > 0
  if (changed || smoothness > 0) { 
      delay(smoothness);
  }
}

// Main loop function for serial control mode
bool update_serial() {
  // First, process any incoming commands which might update targetPulse
  bool continueRunning = processSerialCommands(); 
  if (!continueRunning) {
    return false; // User entered 'X'
  }

  // Then, move servos smoothly towards their targets
  moveServosSmoothly(); 

  return true; // Continue running serial control mode
}
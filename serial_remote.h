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
    
    setAnglePulse(i, SERVOMIDDLE);
  }
  Serial.println("All servos centered");
}

void setup_serial_control(){
  Serial.println("Xiao ESP32-S3 16-Channel Servo Control");
  Serial.println("Commands:");
  Serial.println("M[channel],[pulse] - Move servo (e.g., M5,355)");
  Serial.println("P[pulse] - Move all servos (e.g., 355)");
  Serial.println("S[speed] - Set movement speed (1-50)");
  Serial.println("D[delay] - Set smoothness delay (1-50ms)");
  Serial.println("C - Center all servos (355)");
  Serial.println("X - exit this mode and return to main menu");
}

bool processSerialCommands() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.length() > 0) {
      char command = input.charAt(0);
      input = input.substring(1); // Remove command character
      
      switch (command) {
        case 'X':
        {
          Serial.println("exiting");
          return false;
        }
        break;
        case 'M': // Move command: M[channel],[pulse]
          {
            int commaIndex = input.indexOf(',');
            if (commaIndex > 0) {
              uint8_t channel = input.substring(0, commaIndex).toInt();
              float angle = input.substring(commaIndex + 1).toFloat();
              uint16_t pulse = get_pulse_from_angle_degrees(angle);

              if (channel < 16) {
                channels[channel].targetPulse = pulse;
                Serial.print("Moving channel ");
                Serial.print(channel);
                Serial.print(" to ");
                Serial.println(pulse);
              } else {
                Serial.println("Invalid channel (0-15)");
              }
            } else {
              Serial.println("Invalid format. Use M[channel],[pulse]");
            }
          }
          break;
        
        case 'P':
          {
            float angle = input.toFloat();
            uint16_t pulse = get_pulse_from_angle_degrees(angle);
            for(int i = 0; i < 16; i++) {
              channels[i].targetPulse = pulse;
            }
            Serial.print("Moving all channels");
            Serial.print(" to ");
            Serial.println(pulse);
          }
          break;
      

          
        case 'S': // Set speed: S[speed]
          {
            uint8_t speed = input.toInt();
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
            uint8_t delayTime = input.toInt();
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
          centerAllServos();
          break;
          
        default:
          Serial.println("Unknown command");
          break;
      }
      
    }
    return true;
  }
  return true;
}

void moveServosSmoothly() {
  for (uint8_t i = 0; i < 16; i++) {
    if (channels[i].currentPulse != channels[i].targetPulse) {
      // Calculate direction and step
      int16_t difference = channels[i].targetPulse - channels[i].currentPulse;
      int16_t step = constrain(difference, -movementSpeed, movementSpeed);
      
      channels[i].currentPulse += step;
      setAnglePulse(i, channels[i].currentPulse);
    }
  }
  delay(smoothness);
}

bool update_serial() {
  moveServosSmoothly();
  return processSerialCommands();
}
// wave_servo.h
#ifndef WAVE_SERVO_H
#define WAVE_SERVO_H

#include "utils.h"

// Wave parameters
#define DEFAULT_WAVE_SPEED 5       // Speed multiplier
#define DEFAULT_WAVE_AMPLITUDE 180  // Default amplitude (half of full range)

struct WaveChannel {
  bool active;
  float phase;
  uint16_t center;
  uint16_t amplitude;
};

WaveChannel waveChannels[16]; // All 16 channels
uint8_t waveSpeed = DEFAULT_WAVE_SPEED;
bool waveRunning = false;
unsigned long lastWaveUpdate = 0;

void setupWaveProgram() {
  Serial.println("\n==== Servo Wave Program ====");
  Serial.println("Commands:");
  Serial.println("M[channel] - Toggle motor (e.g., M5)");
  Serial.println("A - Toggle all motors");
  Serial.println("S[speed] - Set wave speed (1-20)");
  Serial.println("C[center] - Set center position (175-535)");
  Serial.println("V[amplitude] - Set amplitude (1-180)");
  Serial.println("G - Go/Start waving");
  Serial.println("B - Break/Stop waving");
  Serial.println("X - Exit to main menu");
  Serial.println("========================");
  
  // Initialize all channels to inactive
  for (uint8_t i = 0; i < 16; i++) {
    waveChannels[i].active = false;
    waveChannels[i].phase = 0;
    waveChannels[i].center = SERVOMIDDLE;
    waveChannels[i].amplitude = DEFAULT_WAVE_AMPLITUDE;
  }
  
  waveRunning = false;
}

bool processWaveCommands() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.length() > 0) {
      char command = input.charAt(0);
      input = input.substring(1); // Remove command character
      
      switch (command) {
        case 'X': // Exit program
          waveRunning = false;
          Serial.println("Exiting wave program");
          return false;
          
        case 'B': // Break/Stop
          waveRunning = false;
          Serial.println("Waving stopped");
          break;
          
        case 'G': // Go/Start
          waveRunning = true;
          lastWaveUpdate = millis();
          Serial.println("Waving started");
          break;
          
        case 'M': // Toggle motor
          {
            uint8_t channel = input.toInt();
            if (channel < 16) {
              waveChannels[channel].active = !waveChannels[channel].active;
              Serial.print("Channel ");
              Serial.print(channel);
              Serial.print(" is now ");
              Serial.println(waveChannels[channel].active ? "active" : "inactive");
            }
          }
          break;
          
        case 'A': // Toggle all motors
          {
            bool newState = !waveChannels[0].active; // Toggle based on first channel
            for (uint8_t i = 0; i < 16; i++) {
              waveChannels[i].active = newState;
            }
            Serial.print("All channels are now ");
            Serial.println(newState ? "active" : "inactive");
          }
          break;
          
        case 'S': // Set speed
          {
            uint8_t speed = input.toInt();
            if (speed >= 1 && speed <= 20) {
              waveSpeed = speed;
              Serial.print("Wave speed set to ");
              Serial.println(waveSpeed);
            } else {
              Serial.println("Speed must be between 1-20");
            }
          }
          break;
          
        case 'C': // Set center position
          {
            uint16_t center = input.toInt();
            if (center >= SERVOMIN && center <= SERVOMAX) {
              for (uint8_t i = 0; i < 16; i++) {
                if (waveChannels[i].active) {
                  waveChannels[i].center = center;
                }
              }
              Serial.print("Active channels center set to ");
              Serial.println(center);
            } else {
              Serial.println("Center must be between 175-535");
            }
          }
          break;
          
        case 'V': // Set amplitude
          {
            uint16_t amplitude = input.toInt();
            if (amplitude >= 1 && amplitude <= 180) {
              for (uint8_t i = 0; i < 16; i++) {
                if (waveChannels[i].active) {
                  waveChannels[i].amplitude = amplitude;
                }
              }
              Serial.print("Active channels amplitude set to ");
              Serial.println(amplitude);
            } else {
              Serial.println("Amplitude must be between 1-180");
            }
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

void updateWave() {
  if (!waveRunning) return;
  
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastWaveUpdate) / 1000.0; // Convert to seconds
  lastWaveUpdate = currentTime;
  
  for (uint8_t i = 0; i < 16; i++) {
    if (waveChannels[i].active) {
      // Update phase based on speed
      waveChannels[i].phase += deltaTime * waveSpeed * 0.1;
      if (waveChannels[i].phase > 2 * M_PI) {
        waveChannels[i].phase -= 2 * M_PI;
      }
      
      // Calculate sinusoidal position
      float sinValue = sin(waveChannels[i].phase);
      uint16_t pulse = waveChannels[i].center + (sinValue * waveChannels[i].amplitude);
      pulse = constrain(pulse, SERVOMIN, SERVOMAX);
      
      setAnglePulse(i, pulse);
    }
  }
}

bool updateWaveProgram() {
  updateWave();
  return processWaveCommands();
}

#endif
//hexapod_proj.ino
// Suggested servo limits (optional)
#define SERVOMIN  175 //-71 degrees (assume -76 degrees)
#define SERVOMAX  535 //81 degrees (Assume 76 degrees)
#define SERVOMIDDLE 355
#define SERVO_FREQ 50


#include "utils.h"


#include "serial_remote.h"
#include "servo_wave.h"
#include "set_pos.h"
#include "walkcycle_serial.h"
#include "walkcycle_remote.h"

// Program states
enum ProgramState {
  MAIN_MENU,
  SERIAL_CONTROL,
  WAVE_PROGRAM,
  IK_PROGRAM,
  WALKCYCLE_SERIAL,
  WALKCYCLE_REMOTE,
  KINEMATICS_DEBUG,
  // Add more program states here as you create them
};

ProgramState currentState = MAIN_MENU;

void printMainMenu() {
  Serial.println("\n===== Main Menu =====");
  Serial.println("1 - Serial Servo Control");
  Serial.println("2 - Sinus Wave");
  Serial.println("3 - Inverse kinematics");
  Serial.println("4 - Walkcycle Serial");
  Serial.println("5 - Walkcycle Remote");
  Serial.println("====================");
}

void setup() {
  Serial.begin(115200);
  
  setupPwm();

  printMainMenu();
}


void handleMainMenu() {
  if (Serial.available() > 0) {
    Serial.println("enter: ");
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    char input_char;
    if (input.length() > 0)
      input_char = input.charAt(0);
    else input_char = '\n';
    
    switch (input_char) {
      case '1':
        currentState = SERIAL_CONTROL;
        setup_serial_control();
        break;
        
      case '2':
        currentState = WAVE_PROGRAM;
        setupWaveProgram();
        break;
        
      case '3':
        currentState = IK_PROGRAM;
        setupIKPostitioning();
        break;
      case '4':
        currentState = WALKCYCLE_SERIAL;
        setupWalkcycleSerial();
        break;
      case '5':
        currentState = WALKCYCLE_REMOTE;
        setupWalkcycleRemote();
        break;
        
      case 'X':
        // Already in main menu
        Serial.println("Already in main menu");
        printMainMenu();
        break;
        
      default:
        Serial.println("Invalid selection");
        printMainMenu();
        break;
    }
  }
}

void stateMachine() {
switch (currentState) {
    case MAIN_MENU:
      handleMainMenu();
      break;
      
    case SERIAL_CONTROL:
      if (!update_serial()) { // If update_serial returns false, exit to main menu
        Serial.println("update_serial exited, returning to menu");
        currentState = MAIN_MENU;
        printMainMenu();
      }
      break;

    case WAVE_PROGRAM:
      if (!updateWaveProgram()) { // If update_serial returns false, exit to main menu
        Serial.println("updateWaveProgram exited, returning to menu");
        currentState = MAIN_MENU;
        printMainMenu();
      }
      break;

    case IK_PROGRAM:
      if (!IKpositioning()) { // If update_serial returns false, exit to main menu
        Serial.println("updateIKProgram exited, returning to menu");
        currentState = MAIN_MENU;
        printMainMenu();
      }
      break;

    case WALKCYCLE_SERIAL:
      if (!walkcycleSerialUpdate()) { // If update_serial returns false, exit to main menu
        Serial.println("walkCycleSerial exited, returning to menu");
        currentState = MAIN_MENU;
        printMainMenu();
      }
      break;

    case WALKCYCLE_REMOTE:
      if (!walkcycleRemoteUpdate()) { // If update_serial returns false, exit to main menu
        Serial.println("walkCycleRemote exited, returning to menu");
        currentState = MAIN_MENU;
        printMainMenu();
      }
      break;
      
    // Add more program cases here as you create them
  }
}

void loop() {
  stateMachine();
}
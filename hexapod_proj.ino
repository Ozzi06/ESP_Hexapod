#define OSSIAN_HEMMA
// HexapodESP32.ino (or your main sketch file)
// #############################################################################
// ### LIBRARIES ###
#include <WiFi.h>
#include <WiFiUdp.h>
#include "servo_angles.h"       // For setupPwm, servo control
#include "robot_spec.h"         // For robot constants and global state
#include "remote_control.h"     // Handles all remote operations
#include "servo_test_mode.h"    // For the servo test mode
#include "walkcycle.h"          // For basic walk cycle setup if needed by remote_control
#include "passwords.h"          // For WiFi credentials

// #############################################################################
// ### DEFINES AND GLOBAL VARIABLES ###

// --- Network ---
WiFiUDP udp; // UDP instance used by remote_control.cpp as well

// --- Robot Operating Modes ---
enum RobotOperatingMode {
  MAIN_MENU,          // Serial command menu
  REMOTE_CONTROL,     // Default: UDP JSON control, mobile app input
  SERVO_TEST          // Direct servo control via serial
};
RobotOperatingMode currentOperatingMode = REMOTE_CONTROL; // DEFAULT TO REMOTE CONTROL

unsigned long lastLoopTimeMicros = 0;

// #############################################################################
// ### SETUP ###
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000); // Wait for serial, but timeout
  Serial.println("\n\n--- Hexapod Control System Booting ---");

  // --- Hardware Initialization ---
  setupPwm(); // Initialize PCA9685 servo drivers
  Serial.println("PWM Drivers Initialized.");

  // --- WiFi Connection ---
  Serial.print("Connecting to WiFi: ");
  Serial.println(SSID);
  WiFi.begin(SSID, PASSWORD);
  int wifi_retries = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_retries < 60) { // Retry for ~30 seconds
    delay(500);
    Serial.print(".");
    wifi_retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi Connection Failed! Remote control will not work.");
    // Proceeding, but remote mode will be non-functional without WiFi.
    // Consider a fallback or error state here if WiFi is absolutely critical.
  }

  // --- Initialize Control Modes ---
  // setupWalkcycle(); // Basic walkcycle params - called within setupRemoteControl if needed
  setupRemoteControl();    // Initialize remote control systems (UDP, JSON parsing states)
  setupServoTestMode(); // Initialize servo test mode (prints help, etc.)

  Serial.println("All systems initialized.");

  // --- Default Operating Mode ---
  // currentOperatingMode is already set to REMOTE_CONTROL by default.
  // If you wanted to start in MAIN_MENU for debugging, you'd change the default.
  if (currentOperatingMode == REMOTE_CONTROL) {
    Serial.println("Entering Remote Control Mode by default.");
    // setupRemoteControl() already printed its ready messages.
  } else if (currentOperatingMode == MAIN_MENU) {
    printMainMenuHelp();
  }

  lastLoopTimeMicros = micros();
}

// #############################################################################
// ### MAIN LOOP ###
void loop() {
  unsigned long startMicros = micros();

  switch (currentOperatingMode) {
    case MAIN_MENU:
      if (processMainMenuCommands()) {
        // Command processed, potentially changed mode
      }
      break;

    case REMOTE_CONTROL:
      if (!remoteControlUpdate()) { // remoteControlUpdate returns false if it wants to exit
        Serial.println("Exiting Remote Control Mode, returning to Main Menu.");
        currentOperatingMode = MAIN_MENU;
        printMainMenuHelp();
      }
      break;

    case SERVO_TEST:
      if (!servoTestModeUpdate()) { // servoTestModeUpdate returns false if 'X' is entered
        Serial.println("Exiting Servo Test Mode, returning to Main Menu.");
        currentOperatingMode = MAIN_MENU;
        printMainMenuHelp();
      }
      break;

    default:
      Serial.println("[ERROR] Unknown operating mode! Reverting to Main Menu.");
      currentOperatingMode = MAIN_MENU;
      printMainMenuHelp();
      break;
  }

  // Simple loop timing print for performance monitoring (optional)
  unsigned long endMicros = micros();
  // if (currentOperatingMode == REMOTE_CONTROL) { // Only print if relevant
  //   Serial.printf("Loop time: %lu us\n", endMicros - startMicros);
  // }
  lastLoopTimeMicros = startMicros; // For dt calculation if needed globally

  // A small delay can be useful if loops are too fast and unthrottled,
  // but remote_control_update will likely have its own effective rate.
  // delay(1);
}

// #############################################################################
// ### MAIN MENU SERIAL COMMANDS ### (Keep this for fallback)
void printMainMenuHelp() {
  Serial.println("\n===== Main Menu =====");
  Serial.println("Enter command:");
  Serial.println("  R - Enter Remote Control Mode");
  Serial.println("  T - Enter Servo Test Mode");
  Serial.println("  H / ? - Display this help");
  Serial.println("=====================");
}

bool processMainMenuCommands() {
  if (Serial.available() > 0) {
    char command = toupper(Serial.read());
    while (Serial.available() > 0 && (Serial.peek() == '\n' || Serial.peek() == '\r')) { Serial.read(); } // Clear buffer

    Serial.print("MainMenu CMD> "); Serial.println(command);

    switch (command) {
      case 'R':
        Serial.println("Transitioning to Remote Control Mode...");
        // setupRemoteControl(); // Should be robust to re-entry or already setup
        currentOperatingMode = REMOTE_CONTROL;
        // remote_control specific "entering mode" messages are handled by setupRemoteControl
        // or when the first packet is received.
        break;
      case 'T':
        Serial.println("Transitioning to Servo Test Mode...");
        // setupServoTestMode(); // Already called in main setup, prints help
        currentOperatingMode = SERVO_TEST;
        printServoTestHelp_STM(); // Re-print help for this mode
        break;
      case 'H':
      case '?':
        printMainMenuHelp();
        break;
      default:
        Serial.println("Unknown command. Type 'H' or '?' for help.");
        return false;
    }
    return true;
  }
  return false;
}
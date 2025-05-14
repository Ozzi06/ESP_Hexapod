//hexapod_proj.ino
// Suggested servo limits

//#define OSSIAN_HEMMA

#include <WiFi.h>
#include <WiFiUdp.h>
#include "servo_angles.h"
// --- Global Variables ---
WiFiUDP udp;

#include "walkcycle_remote.h"
#include "servo_test_mode.h"

// Program states
enum ProgramState {
  MAIN_MENU,
  WALKCYCLE_REMOTE,
  SERVO_TEST_MODE,
  // Add more program states here as you create them
};

ProgramState currentState = MAIN_MENU;

void printMainMenu() {
  Serial.println("\n===== Main Menu =====");
  Serial.println("1 - Walkcycle Remote");
  Serial.println("2 - Servo Test Mode");
  Serial.println("====================");
}

void setup() {
  Serial.begin(115200);
  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
  // ... (WiFi connection loop - same as before) ...
  int wifi_retries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    wifi_retries++;
    if (wifi_retries > 20) {
        Serial.println("\nWiFi Connection Failed! Halting.");
        while(1) { delay(1000); }
    }
  }
  Serial.println(" Connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
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
        currentState = WALKCYCLE_REMOTE;
        setupWalkcycleRemote();
        break;
    case '2':
        currentState = SERVO_TEST_MODE;
        setupServoTestMode();
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

    case WALKCYCLE_REMOTE:
      if (!walkcycleRemoteUpdate()) { // If update_serial returns false, exit to main menu
        Serial.println("walkCycleRemote exited, returning to menu");
        currentState = MAIN_MENU;
        printMainMenu();
      }
      break;

    case SERVO_TEST_MODE:
        if (!servoTestModeUpdate()) {
            currentState = MAIN_MENU;
            printMainMenu();
        }
        break;
      
    // Add more program cases here as you create them
  }
}

const int VOLTAGE_SENSOR_PIN = 9; // A10 on XIAO ESP32-S3 corresponds to GPIO1
unsigned long lastVoltagePrintTime = 0;
const unsigned long voltagePrintInterval = 1000; // 1000 milliseconds = 1 second

// --- Voltage Divider and ADC Configuration ---
// IMPORTANT: Verify R1 and R2 match your physical connections!
// R1 is the resistor from your actual voltage source (Vin) to the ADC pin.
// R2 is the resistor from the ADC pin to Ground.
const float R1_OHMS = 10000.0f; // e.g., 10k Ohm
const float R2_OHMS = 3800.0f;  // e.g., 3.84k Ohm

const float ADC_VREF = 3.3f;       // ADC reference voltage for ESP32 (usually 3.3V)
const float ADC_MAX_VALUE = 4095.0f; // ESP32 has a 12-bit ADC (2^12 - 1)

void loop() {
  if (millis() - lastVoltagePrintTime >= voltagePrintInterval) {
    lastVoltagePrintTime = millis(); // Update the last print time

    int sensorValue = analogRead(VOLTAGE_SENSOR_PIN); // Read the analog value (0-4095)
    
    // Calculate voltage at the ADC pin
    float v_at_adc_pin = sensorValue * (ADC_VREF / ADC_MAX_VALUE);
    
    // Calculate the actual input voltage before the divider
    // Vin = V_at_adc_pin * (R1 + R2) / R2
    float v_in_actual = 0.0f;
    if (R2_OHMS > 0.001f) { // Avoid division by zero
          v_in_actual = v_at_adc_pin * (R1_OHMS + R2_OHMS) / R2_OHMS;
    }
    /*
    Serial.print("A10(GPIO1) Raw: ");
    Serial.print(sensorValue);
    Serial.print(" | V_adc: ");
    Serial.print(v_at_adc_pin, 2); // Voltage at ADC pin, 2 decimal places
    Serial.print(" V | Vin_actual: ");
    Serial.print(v_in_actual, 2);   // Calculated actual input voltage, 2 decimal places
    Serial.println(" V");*/
  }

  stateMachine();
  #ifdef OSSIAN_HEMMA
    // Check if WiFi is connected before attempting to send
    if (WiFi.status() == WL_CONNECTED && (millis() % 20) == 0) {
        udp.beginPacket(angleBroadcastIp, angleBroadcastPort);
        // Send the raw bytes of the entire latestServoAngles array
        udp.write((uint8_t*)latestServoAngles, sizeof(latestServoAngles));
        udp.endPacket();
    }
  #endif // OSSIAN_HEMMA
}
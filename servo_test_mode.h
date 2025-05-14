// servo_test_mode.h
#ifndef SERVO_TEST_MODE_H
#define SERVO_TEST_MODE_H

#include "robot_spec.h"   // For LEG_COUNT, LEG_SERVOS, leg_names, servo_center_angle
#include "servo_angles.h" // For setAngleDegrees, setAngleRadians (which use setAnglePulse)
#include <Arduino.h>      // For Serial, String, etc.
#include <stdio.h>        // For sscanf

// Forward declarations for functions within this header
void setupServoTestMode();
bool servoTestModeUpdate();
void printServoTestHelp_STM(); // Renamed to avoid conflict if other helps exist
bool processSerialCommands_STM(); // Renamed
void centerSpecificServo_STM(uint8_t leg_idx, uint8_t joint_idx); // Renamed
void setSpecificServoAngle_STM(uint8_t leg_idx, uint8_t joint_idx, float angle_deg); // Renamed
void centerAllServos_STM(); // Renamed

/**
 * @brief Sets up the servo test mode.
 * Prints help message. Servos remain in their current state (not actively driven by this mode yet).
 */
inline void setupServoTestMode() {
    Serial.println("\n===== Servo Test Mode =====");
    Serial.println("Servos are initially 'loose' (not actively commanded by this mode).");
    Serial.println("They will move when first commanded.");
    printServoTestHelp_STM();
    Serial.println("===========================");
    // No servos are commanded at setup. They retain their state from the previous mode
    // or power-on state.
}

/**
 * @brief Prints available serial commands for servo test mode.
 */
inline void printServoTestHelp_STM() {
    Serial.println("\nAvailable Commands (Servo Test Mode):");
    Serial.println("  CA            - Center All servos to their logical 0 (respecting offsets)");
    Serial.println("  CL <L> <J>    - Center Leg <L> (0-5), Joint <J> (0-2: Coxa,Femur,Tibia)");
    Serial.println("                  (e.g., 'CL 0 1' centers BackRight Femur)");
    Serial.println("  S <L> <J> <A> - Set Leg <L> (0-5), Joint <J> (0-2) to Angle <A> (degrees, logical)");
    Serial.println("                  (e.g., 'S 3 2 -30.5' sets BackLeft Tibia to -30.5 deg)");
    Serial.println("  H / ?         - Display this Help");
    Serial.println("  X             - Exit Servo Test Mode");
}

/**
 * @brief Centers a specific servo to its logical 0 position.
 * The logical 0 position respects the servo_center_angle offset defined in robot_spec.
 * @param leg_idx Index of the leg (0 to LEG_COUNT-1).
 * @param joint_idx Index of the joint (0:Coxa, 1:Femur, 2:Tibia).
 */
inline void centerSpecificServo_STM(uint8_t leg_idx, uint8_t joint_idx) {
    if (leg_idx >= LEG_COUNT) {
        Serial.printf("[Error] Invalid leg index: %d. Must be 0-%d.\n", leg_idx, LEG_COUNT - 1);
        return;
    }
    if (joint_idx >= 3) {
        Serial.printf("[Error] Invalid joint index: %d. Must be 0-2.\n", joint_idx);
        return;
    }

    uint8_t servo_channel = LEG_SERVOS[leg_idx][joint_idx];
    const char* current_leg_name = (leg_idx < LEG_COUNT) ? leg_names[leg_idx] : "Unknown";
    const char* joint_name_str[] = {"Coxa", "Femur", "Tibia"};

    Serial.printf("Centering Leg %d (%s), Joint %d (%s) (Servo Channel %d) to logical 0.0 degrees.\n",
                  leg_idx, current_leg_name, joint_idx, joint_name_str[joint_idx], servo_channel);

    // Command the servo to logical 0.0 degrees.
    // setAngleDegrees will internally use servo_center_angle from robot_spec.h
    // (via get_pulse_from_angle_degrees) to calculate the actual physical angle the servo needs to achieve.
    setAngleDegrees(servo_channel, 0.0f);
}

/**
 * @brief Sets a specific servo to a given logical angle.
 * @param leg_idx Index of the leg (0 to LEG_COUNT-1).
 * @param joint_idx Index of the joint (0:Coxa, 1:Femur, 2:Tibia).
 * @param angle_deg The desired logical angle in degrees.
 */
inline void setSpecificServoAngle_STM(uint8_t leg_idx, uint8_t joint_idx, float angle_deg) {
    if (leg_idx >= LEG_COUNT) {
        Serial.printf("[Error] Invalid leg index: %d. Must be 0-%d.\n", leg_idx, LEG_COUNT - 1);
        return;
    }
    if (joint_idx >= 3) {
        Serial.printf("[Error] Invalid joint index: %d. Must be 0-2.\n", joint_idx);
        return;
    }

    uint8_t servo_channel = LEG_SERVOS[leg_idx][joint_idx];
    const char* current_leg_name = (leg_idx < LEG_COUNT) ? leg_names[leg_idx] : "Unknown";
    const char* joint_name_str[] = {"Coxa", "Femur", "Tibia"};


    Serial.printf("Setting Leg %d (%s), Joint %d (%s) (Servo Channel %d) to %.2f degrees (logical).\n",
                  leg_idx, current_leg_name, joint_idx, joint_name_str[joint_idx], servo_channel, angle_deg);

    setAngleDegrees(servo_channel, angle_deg);
}

/**
 * @brief Centers all servos to their logical 0 positions.
 */
inline void centerAllServos_STM() {
    Serial.println("Centering ALL servos to their logical 0 positions...");
    for (uint8_t leg = 0; leg < LEG_COUNT; ++leg) {
        for (uint8_t joint = 0; joint < 3; ++joint) {
            // Call the specific centering function which already has safety checks and messaging.
            centerSpecificServo_STM(leg, joint);
            // A small delay might be good if power supply is limited,
            // but PCA9685 can handle rapid updates.
            // delay(10); // Optional: small delay
        }
    }
    Serial.println("All servos commanded to center.");
}


/**
 * @brief Processes incoming serial commands for servo test mode.
 * @return True to continue running, False to exit.
 */
inline bool processSerialCommands_STM() {
    if (Serial.available() > 0) {
        String commandStr = Serial.readStringUntil('\n');
        commandStr.trim(); // Remove leading/trailing whitespace and newlines

        if (commandStr.length() == 0) return true; // Ignore empty commands

        // Echo command for clarity
        Serial.print("CMD_STM> "); Serial.println(commandStr);

        // Use a char array for sscanf compatibility
        char cmdBuffer[64]; // Increased buffer size
        strncpy(cmdBuffer, commandStr.c_str(), sizeof(cmdBuffer) - 1);
        cmdBuffer[sizeof(cmdBuffer) - 1] = '\0'; // Ensure null termination

        // Variables for parsed commands
        int leg_idx_param = -1, joint_idx_param = -1;
        float angle_val_param = 0.0f;

        // Command parsing
        if (commandStr.equalsIgnoreCase("X")) {
            Serial.println("\nExiting Servo Test Mode...");
            return false; // Signal to exit
        } else if (commandStr.equalsIgnoreCase("H") || commandStr.equals("?")) {
            printServoTestHelp_STM();
        } else if (commandStr.equalsIgnoreCase("CA")) {
            centerAllServos_STM();
        }
        // Parse "CL <L> <J>" - case insensitive for "CL"
        else if (commandStr.length() >= 2 && (commandStr.substring(0, 2).equalsIgnoreCase("CL")) ) {
            if (sscanf(cmdBuffer + 2, "%d %d", &leg_idx_param, &joint_idx_param) == 2) { // Start scanning after "CL"
                 centerSpecificServo_STM(leg_idx_param, joint_idx_param);
            } else {
                Serial.println("[Error] Invalid format for CL command. Usage: CL <leg_idx_0_5> <joint_idx_0_2>");
                printServoTestHelp_STM();
            }
        }
        // Parse "S <L> <J> <A>" - case insensitive for "S"
        else if (commandStr.length() >= 1 && (commandStr.substring(0, 1).equalsIgnoreCase("S")) ) {
            if (sscanf(cmdBuffer + 1, "%d %d %f", &leg_idx_param, &joint_idx_param, &angle_val_param) == 3) { // Start scanning after "S"
                setSpecificServoAngle_STM(leg_idx_param, joint_idx_param, angle_val_param);
            } else {
                Serial.println("[Error] Invalid format for S command. Usage: S <leg_idx_0_5> <joint_idx_0_2> <angle_degrees>");
                printServoTestHelp_STM();
            }
        }
        else {
            Serial.print("Unknown command: '"); Serial.print(commandStr); Serial.println("'. Type 'H' or '?' for help.");
        }
         // Clear remaining serial buffer to prevent processing partial/old commands on next loop.
        while(Serial.available()) { Serial.read(); }
    }
    return true; // Continue running
}


/**
 * @brief Main update loop for the servo test mode.
 * This function is called repeatedly by the main state machine.
 * @return True to continue running this mode, False to exit to the main menu.
 */
inline bool servoTestModeUpdate() {
    // This mode is driven by serial commands.
    // processSerialCommands_STM will return false if 'X' is entered.
    return processSerialCommands_STM();
}

#endif // SERVO_TEST_MODE_H
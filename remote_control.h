// remote_control.h
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <ArduinoJson.h> // For JSON processing
#include "robot_spec.h"  // For global robot state
#include "walkcycle.h"   // For walk cycle parameters and update function
#include <WiFi.h>
#include <WiFiUdp.h>

// Forward declaration for the global UDP object from main .ino
extern WiFiUDP udp;

// Configuration
constexpr unsigned int UDP_LISTEN_PORT = 5005; // Port ESP32 listens on for commands
constexpr unsigned int TCP_LISTEN_PORT = 5006; // Port ESP32 listens on for commands
constexpr unsigned long GLOBAL_PACKET_TIMEOUT_MS = 7000; // 7 seconds

// Function Declarations
void setupRemoteControl();
bool remoteControlUpdate(); // Main update loop for this mode, returns false to exit mode
void printRemoteControlSerialHelp(); // For serial commands specific to this mode

#endif // REMOTE_CONTROL_H
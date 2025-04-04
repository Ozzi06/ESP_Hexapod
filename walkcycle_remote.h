#ifndef WALK_CYCLE_REMOTE_H
#define WALK_CYCLE_REMOTE_H

#include <WiFi.h>
#include <WiFiUdp.h>
#include <stdint.h>
#include "walkcycle.h" // Base walk cycle logic and types
#include "ik.h"        // Inverse Kinematics
#include "utils.h"     // Utilities like Vec3, clampf

// --- Network Configuration ---
const char* ssid = "guestnet";         // Replace with your WiFi SSID
const char* password = "VolvoAmazon"; // Replace with your WiFi Password
unsigned int localUdpPort = 5005;           // Port to listen on (matches Python sender)

// --- Packet Structure Definition ---
// Matches Python format: '<IQBffffff' (Little Endian)
// I=uint32_t, Q=uint64_t (ms), B=uint8_t, f=float x 6
#define WALK_PACKET_IDENTIFIER 0xDEADBEEF

#pragma pack(push, 1) // Prevent padding
struct WalkControlPacket {
  uint32_t identifier;       // 'I'
  uint64_t timestampMs;      // 'Q' (Timestamp in milliseconds)
  uint8_t  walkCycleRunning; // 'B' (0=False, 1=True)
  float    velX;             // 'f'
  float    velY;             // 'f'
  float    velZ;             // 'f'
  float    stepHeight;       // 'f'
  float    stepFrequency;    // 'f'
  float    dutyFactor;       // 'f'
};
#pragma pack(pop) // Restore default packing

// Verify size at compile time if possible (should be 37 bytes)
// static_assert(sizeof(WalkControlPacket) == 37, "WalkControlPacket size mismatch");

// --- Global Variables ---
WiFiUDP udp;
uint8_t incomingPacketBuffer[sizeof(WalkControlPacket)];
uint64_t lastPacketTimestampMs = 0; // Track the last valid timestamp
bool logPackets = false;            // Control verbose packet logging via Serial

// Forward declarations
bool walkcycleRemoteUpdate();
void printSerialHelp();
bool processSerialCommands_remote();

void setupWalkcycleRemote() {
  // Serial must be initialized in your main .ino's setup()
  Serial.println("\nSetting up Walk Cycle Remote Control (UDP)...");

  // Setup base walk cycle systems (IK, initial leg positions, etc.)
  setupWalkcycle(); // Calls setupIK internally
  walkCycleRunning = false; // Start stopped
  logPackets = false;      // Logging off by default

  // Connect to Wi-Fi
  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" Connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Start UDP listener
  Serial.printf("Starting UDP listener on port %d\n", localUdpPort);
  if (udp.begin(localUdpPort)) {
      Serial.printf("UDP Listener started. Expected packet size: %d bytes.\n", sizeof(WalkControlPacket));
  } else {
      Serial.println("Failed to start UDP Listener! Halting.");
      while(1) { delay(1000); } // Critical error, stop here
  }

  Serial.println("==== Walk Cycle Remote Ready ====");
  printSerialHelp(); // Show available serial commands
  Serial.println("================================");
}

void printSerialHelp() {
    Serial.println("\n==== Serial Commands ====");
    Serial.println(" L - Toggle UDP Packet Logging (currently " + String(logPackets ? "ON" : "OFF") + ")");
    Serial.println(" X - Exit Remote Control Mode");
    Serial.println(" H - Display this Help");
    Serial.println("========================");
}

bool processWalkPacket() {
  int packetSize = udp.parsePacket();
  if (!packetSize) {
    return false; // No packet available
  }

  IPAddress remoteIp = udp.remoteIP();
  unsigned int remotePort = udp.remotePort();

  // Always log basic reception info if logging is on
  if (logPackets) {
    Serial.printf("\nReceived packet: %d bytes from %s:%d\n", packetSize, remoteIp.toString().c_str(), remotePort);
  }

  // --- Validation 1: Packet Size ---
  if (packetSize != sizeof(WalkControlPacket)) {
    Serial.printf("  [Error] Packet size mismatch! Expected %d, Got %d. Discarding.\n", sizeof(WalkControlPacket), packetSize);
    udp.flush(); // Clear the UDP buffer
    return true; // Handled (by discarding)
  }

  // Read packet data
  int len = udp.read(incomingPacketBuffer, sizeof(WalkControlPacket));
  if (len != sizeof(WalkControlPacket)) {
      Serial.printf("  [Error] Failed to read correct # bytes! Read %d. Discarding.\n", len);
      udp.flush();
      return true;
  }

  // Cast buffer to struct pointer (safe due to size check and matching endianness)
  WalkControlPacket* receivedPacket = (WalkControlPacket*)incomingPacketBuffer;

  // --- Validation 2: Identifier ---
  if (receivedPacket->identifier != WALK_PACKET_IDENTIFIER) {
      Serial.printf("  [Error] Invalid packet identifier! Expected 0x%X, Got 0x%X. Discarding.\n",
                    WALK_PACKET_IDENTIFIER, receivedPacket->identifier);
      return true; // Handled (by discarding)
  }

  // --- Validation 3: Timestamp ---
  bool isNewer = receivedPacket->timestampMs > lastPacketTimestampMs;
  if (!isNewer) {
      // Always log error about old packets
      Serial.printf("  [Warning] Old/duplicate packet timestamp (%llu ms <= %llu ms). Ignoring.\n",
                    receivedPacket->timestampMs, lastPacketTimestampMs);
      return true; // Handled (by ignoring)
  }

  // If we reach here, the packet is valid and newer than the last processed one
  uint64_t oldTimestamp = lastPacketTimestampMs; // Store for logging if needed
  lastPacketTimestampMs = receivedPacket->timestampMs; // Update timestamp FIRST

  // --- Optional Detailed Logging ---
  if (logPackets) {
    Serial.println("  --- Processing Valid Packet ---");
    Serial.printf("    Identifier: 0x%X\n", receivedPacket->identifier);
    Serial.printf("    Timestamp: %llu ms (Prev: %llu ms)\n", receivedPacket->timestampMs, oldTimestamp); // Show new and previous
    Serial.printf("    Raw Running Flag: %d\n", receivedPacket->walkCycleRunning);
    Serial.printf("    Raw Velocity (X,Y,Z): (%.2f, %.2f, %.2f)\n", receivedPacket->velX, receivedPacket->velY, receivedPacket->velZ);
    Serial.printf("    Raw Step Height: %.2f\n", receivedPacket->stepHeight);
    Serial.printf("    Raw Step Freq: %.2f\n", receivedPacket->stepFrequency);
    Serial.printf("    Raw Duty Factor: %.2f\n", receivedPacket->dutyFactor);
    Serial.println("  ---------------------------------");
  }

  // --- Apply Packet Data to Walk Cycle State ---
  bool newRunningState = (receivedPacket->walkCycleRunning != 0);

  // Reset phase only when starting from a stopped state
  if (newRunningState && !walkCycleRunning) {
      globalPhase = 0.0f;
      if (logPackets) Serial.println("  State change: Starting walk cycle, phase reset.");
  } else if (!newRunningState && walkCycleRunning) {
       if (logPackets) Serial.println("  State change: Stopping walk cycle.");
       // Optional: Add code here to move legs to a default resting pose immediately
       // e.g., for(int i=0; i<LEG_COUNT; ++i) legTargets[i] = legCycleData[i].basePosition;
       //      for(int i=0; i<LEG_COUNT; ++i) moveLegToTarget(i, true); // Move smoothly to base
  }
  walkCycleRunning = newRunningState;

  bodyVelocity.x = receivedPacket->velX;
  bodyVelocity.y = receivedPacket->velY;
  bodyVelocity.z = receivedPacket->velZ; // Apply Z velocity (though original walk might not use it much)

  walkParams.stepHeight = receivedPacket->stepHeight;
  walkParams.stepFrequency = receivedPacket->stepFrequency;
  // Clamp duty factor to prevent issues in calculations
  walkParams.dutyFactor = clampf(receivedPacket->dutyFactor, 0.01f, 0.99f);

   if (logPackets) {
     Serial.println("  Applied state:");
     Serial.printf("    walkCycleRunning: %s\n", walkCycleRunning ? "true":"false");
     Serial.printf("    bodyVelocity: (%.2f, %.2f, %.2f)\n", bodyVelocity.x, bodyVelocity.y, bodyVelocity.z);
     Serial.printf("    walkParams.stepHeight: %.2f\n", walkParams.stepHeight);
     Serial.printf("    walkParams.stepFrequency: %.2f\n", walkParams.stepFrequency);
     Serial.printf("    walkParams.dutyFactor: %.2f\n", walkParams.dutyFactor);
   }


  return true; // Packet processed successfully
}


// Process incoming serial commands
// Returns true to continue, false to exit
bool processSerialCommands_remote() {
  if (Serial.available() > 0) {
    char command = Serial.read(); // Read single character command
    // Consume any extra newline characters
    while (Serial.available() > 0 && Serial.peek() == '\n') {
        Serial.read();
    }

    switch (toupper(command)) { // Use uppercase for case-insensitivity
      case 'L':
        logPackets = !logPackets;
        Serial.print("\nPacket Logging is now ");
        Serial.println(logPackets ? "ON" : "OFF");
        break;

      case 'X':
        Serial.println("\nExiting Remote Control Mode via serial command.");
        walkCycleRunning = false; // Ensure robot stops
        // Optional: Move legs to resting pose before exiting
        return false; // Signal exit to main loop

      case 'H':
      case '?':
        printSerialHelp();
        break;

      default:
        // Ignore unknown characters or print message
         if (isprint(command)) { // Only print if it's a printable character
             Serial.print("\nUnknown serial command: '");
             Serial.print(command);
             Serial.println("'. Type 'H' for help.");
         }
        break;
    }
  }
  return true; // Continue running
}


// This function should be called repeatedly in the main loop()
// Returns true to continue running, false to exit mode
bool walkcycleRemoteUpdate() {
  // 1. Check for Serial Commands (like Exit or Log Toggle)
  if (!processSerialCommands_remote()) {
    return false; // Exit command received
  }

  // 2. Process Incoming UDP Packets (updates state variables if valid packet arrives)
  processWalkPacket(); // Process one packet per call, if available

  // 3. Calculate Time Delta for Walk Cycle Update
  static unsigned long lastUpdateTimeMicros = 0;
  unsigned long nowMicros = micros(); // Use microseconds for potentially higher resolution dt
  // Handle first run or potential rollover (though less likely with micros)
  if (lastUpdateTimeMicros == 0) {
    lastUpdateTimeMicros = nowMicros;
  }
  float dt = (nowMicros - lastUpdateTimeMicros) / 1000000.0f;
  lastUpdateTimeMicros = nowMicros;

  // Prevent extreme dt values due to lag spikes or timing issues
  if (dt < 0.0f || dt > 0.5f) {
      // If logging is on, report the issue
      if (logPackets && dt != 0.0f) { // Avoid spamming if dt is simply 0 on first valid run
           Serial.printf("[Warning] Unusual dt detected: %.4f s. Using default 0.01s\n", dt);
      }
      dt = 0.01f; // Use a small default delta time
  }


  // 4. Update Walk Cycle Logic if Running
  if (walkCycleRunning) {
    // This function (from walkcycle.h) calculates leg targets based on
    // globalPhase, bodyVelocity, walkParams and calls moveLegToTarget
    updateWalkCycle(dt);
  } else {
    // If stopped, the walk cycle logic isn't running.
    // Servos might hold their last position depending on your servo library.
    // You could optionally add code here to explicitly move legs to a
    // defined resting/home position if needed when stopped.
    // For example:
    // bool legsNeedHoming = true; // Set this flag when stopping
    // if (legsNeedHoming) {
    //   for(int i=0; i<LEG_COUNT; ++i) { legTargets[i] = legCycleData[i].basePosition; moveLegToTarget(i, true); }
    //   legsNeedHoming = false;
    // }
  }

  return true; // Continue running
}

#endif // WALK_CYCLE_REMOTE_H
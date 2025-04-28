#ifndef WALK_CYCLE_REMOTE_H
#define WALK_CYCLE_REMOTE_H

#include <WiFi.h>
#include <WiFiUdp.h>
#include "packets.h"      // Include the new header with FullControlPacket definition
#include "walkcycle.h"    // Base walk cycle logic
#include "ik.h"           // Inverse Kinematics (needed indirectly via walkcycle)
#include "utils.h"        // Utilities like clampf
#include "robot_spec.h"   // Global state variables (bodyVelocity, bodyOrientation, etc.)

// --- Network Configuration ---
const char* ssid = "guestnet";         // Replace with your WiFi SSID
const char* password = "VolvoAmazon"; // Replace with your WiFi Password
unsigned int localUdpPort = 5005;           // Port to listen on

// --- Global Variables ---
WiFiUDP udp;
uint8_t incomingPacketBuffer[sizeof(FullControlPacket)]; // Use size of the new packet
uint64_t lastPacketTimestampMs = 0; // Track the last valid timestamp (for logging/sanity check)
uint32_t lastSequenceNumber = 0;    // Track the last processed sequence number
bool logPackets = false;            // Control verbose packet logging via Serial

// Forward declarations
void setupWalkcycleRemote();
bool walkcycleRemoteUpdate();
bool processFullControlPacket(); // Renamed packet processing function
bool processSerialCommands_remote();
void printSerialHelp();

/**
 * @brief Sets up WiFi connection, UDP listener, and initializes walk cycle state for remote control.
 */
void setupWalkcycleRemote() {
  // Serial must be initialized in your main .ino's setup()
  Serial.println("\nSetting up Walk Cycle Remote Control (UDP) - Full Control Mode...");

  // Setup base walk cycle systems (IK, initial leg positions, etc.)
  setupWalkcycle();         // Initializes walk cycle data structures
  walkCycleRunning = false; // Start stopped
  logPackets = false;       // Logging off by default
  lastSequenceNumber = 0;   // Reset sequence number tracking
  lastPacketTimestampMs = 0;// Reset timestamp tracking

  // Connect to Wi-Fi
  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
  int wifi_retries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    wifi_retries++;
    if (wifi_retries > 20) { // Timeout after ~10 seconds
        Serial.println("\nWiFi Connection Failed! Check SSID/Password. Halting.");
        while(1) { delay(1000); } // Critical error
    }
  }
  Serial.println(" Connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Start UDP listener
  Serial.printf("Starting UDP listener on port %d\n", localUdpPort);
  if (udp.begin(localUdpPort)) {
      Serial.printf("UDP Listener started. Expected packet size: %d bytes (FullControlPacket).\n", sizeof(FullControlPacket));
  } else {
      Serial.println("Failed to start UDP Listener! Halting.");
      while(1) { delay(1000); } // Critical error, stop here
  }

  Serial.println("==== Walk Cycle Remote Ready ====");
  printSerialHelp(); // Show available serial commands
  Serial.println("================================");
}

/**
 * @brief Prints available serial commands for this mode to the Serial monitor.
 */
void printSerialHelp() {
    Serial.println("\n==== Serial Commands (Remote Mode) ====");
    Serial.println(" L - Toggle UDP Packet Logging (currently " + String(logPackets ? "ON" : "OFF") + ")");
    Serial.println(" X - Exit Remote Control Mode");
    Serial.println(" H / ? - Display this Help");
    Serial.println("=======================================");
}

/**
 * @brief Processes one incoming UDP packet, validates it, and updates robot state if valid.
 * @return True if a packet was available (processed or discarded), False if no packet was waiting.
 */
bool processFullControlPacket() {
  int packetSize = udp.parsePacket();
  if (!packetSize) {
    return false; // No packet available
  }

  IPAddress remoteIp = udp.remoteIP();
  unsigned int remotePort = udp.remotePort();

  // Basic logging if enabled
  if (logPackets) {
    Serial.printf("\nReceived packet: %d bytes from %s:%d\n", packetSize, remoteIp.toString().c_str(), remotePort);
  }

  // --- Validation 1: Packet Size ---
  if (packetSize != sizeof(FullControlPacket)) {
    Serial.printf("  [Error] Packet size mismatch! Expected %d, Got %d. Discarding.\n", sizeof(FullControlPacket), packetSize);
    udp.flush(); // Clear the UDP buffer of the invalid packet
    return true; // Handled (by discarding)
  }

  // Read packet data into buffer
  int len = udp.read(incomingPacketBuffer, sizeof(FullControlPacket));
  if (len != sizeof(FullControlPacket)) {
      Serial.printf("  [Error] Failed to read correct # bytes! Read %d. Discarding.\n", len);
      udp.flush();
      return true;
  }

  // Cast buffer to the packet struct pointer
  FullControlPacket* receivedPacket = (FullControlPacket*)incomingPacketBuffer;

  // --- Validation 2: Identifier ---
  if (receivedPacket->identifier != FULL_CONTROL_PACKET_IDENTIFIER) {
      Serial.printf("  [Error] Invalid packet identifier! Expected 0x%X, Got 0x%X. Discarding.\n",
                    FULL_CONTROL_PACKET_IDENTIFIER, receivedPacket->identifier);
      return true; // Handled (by discarding)
  }

  // --- Validation 3: Sequence Number ---
  // Check if the packet is newer than the last processed one. Handles rollover correctly.
  if (receivedPacket->sequenceNumber <= lastSequenceNumber && lastSequenceNumber != 0) { // Allow first packet (seq > 0 if lastSeq == 0)
       // Log if needed, but don't process old/duplicate sequence numbers
       if (logPackets) {
          Serial.printf("  [Info] Stale/duplicate sequence number (%u <= %u). Ignoring.\n",
                         receivedPacket->sequenceNumber, lastSequenceNumber);
       }
       // Also check timestamp as a secondary diagnostic
       if (receivedPacket->timestampMs <= lastPacketTimestampMs && logPackets) {
             Serial.printf("         Timestamp also old (%llu ms <= %llu ms).\n",
                         receivedPacket->timestampMs, lastPacketTimestampMs);
       }
       return true; // Handled (by ignoring)
  }
  // --- Check timestamp for potential clock skew warnings ---
   if (receivedPacket->timestampMs <= lastPacketTimestampMs && logPackets && lastSequenceNumber != 0) {
         Serial.printf("  [Warning] Newer sequence number (%u > %u) but older timestamp (%llu ms <= %llu ms). Clock skew?\n",
                       receivedPacket->sequenceNumber, lastSequenceNumber,
                       receivedPacket->timestampMs, lastPacketTimestampMs);
   }


  // If we reach here, the packet is valid and newer based on sequence number.
  uint32_t oldSequenceNumber = lastSequenceNumber; // Store for logging
  lastSequenceNumber = receivedPacket->sequenceNumber; // Update sequence number *before* applying data
  lastPacketTimestampMs = receivedPacket->timestampMs; // Update timestamp


  // --- Optional Detailed Logging ---
  if (logPackets) {
    Serial.println("  --- Processing Valid Packet ---");
    Serial.printf("    Identifier: 0x%X\n", receivedPacket->identifier);
    Serial.printf("    Timestamp: %llu ms\n", receivedPacket->timestampMs);
    Serial.printf("    Sequence #: %u (Prev: %u)\n", receivedPacket->sequenceNumber, oldSequenceNumber);
    Serial.printf("    Raw Flags: 0x%02X\n", receivedPacket->controlFlags);
    Serial.printf("    Raw LinVel(X,Y,Z): (%.2f, %.2f, %.2f) cm/s\n", receivedPacket->velocityX, receivedPacket->velocityY, receivedPacket->velocityZ);
    Serial.printf("    Raw AngVel(Yaw): %.3f rad/s\n", receivedPacket->angularVelocityYaw);
    Serial.printf("    Raw Gait (H, F, D): (%.2f cm, %.2f Hz, %.2f)\n", receivedPacket->stepHeight, receivedPacket->stepFrequency, receivedPacket->dutyFactor);
    Serial.printf("    Raw PosOff(X,Y,Z): (%.2f, %.2f, %.2f) cm\n", receivedPacket->bodyPositionOffsetX, receivedPacket->bodyPositionOffsetY, receivedPacket->bodyPositionOffsetZ);
    Serial.printf("    Raw Orient(W,X,Y,Z): (%.3f, %.3f, %.3f, %.3f)\n", receivedPacket->bodyOrientationW, receivedPacket->bodyOrientationX, receivedPacket->bodyOrientationY, receivedPacket->bodyOrientationZ);
    Serial.println("  ---------------------------------");
  }

  // --- Apply Packet Data to Robot State ---

  // 1. Update Walk Cycle Running State
  bool newRunningState = (receivedPacket->controlFlags & FLAG_WALK_RUNNING) != 0;
  // Reset phase only when starting from a stopped state
  if (newRunningState && !walkCycleRunning) {
      globalPhase = 0.0f; // Reset phase for a clean start
      if (logPackets) Serial.println("  State change: Starting walk cycle, phase reset.");
  } else if (!newRunningState && walkCycleRunning) {
       if (logPackets) Serial.println("  State change: Stopping walk cycle.");
       // Optional: Command legs to a resting pose immediately upon stop?
  }
  walkCycleRunning = newRunningState;

  // 2. Update Velocities
  bodyVelocity.x = receivedPacket->velocityX;
  bodyVelocity.y = receivedPacket->velocityY;
  bodyVelocity.z = receivedPacket->velocityZ;
  //bodyAngularVelocityYaw = receivedPacket->angularVelocityYaw; // Update the new global variable //TODO! implement the yaw control

  // 3. Update Walk Parameters
  walkParams.stepHeight = receivedPacket->stepHeight;
  walkParams.stepFrequency = receivedPacket->stepFrequency;
  // Clamp duty factor to a safe range to prevent issues in walk cycle math
  walkParams.dutyFactor = clampf(receivedPacket->dutyFactor, 0.01f, 0.99f);

  // 4. Update Body Pose (Position Offset and Orientation)
  // Based on the user's packet definition, the FLAG_USE_BODY_POSE was removed,
  // so we assume the pose should be updated with *every* valid packet now.
  bodyPositionOffset.x = receivedPacket->bodyPositionOffsetX;
  bodyPositionOffset.y = receivedPacket->bodyPositionOffsetY;
  bodyPositionOffset.z = receivedPacket->bodyPositionOffsetZ;

  bodyOrientation.w = receivedPacket->bodyOrientationW;
  bodyOrientation.x = receivedPacket->bodyOrientationX;
  bodyOrientation.y = receivedPacket->bodyOrientationY;
  bodyOrientation.z = receivedPacket->bodyOrientationZ;
  bodyOrientation.normalize(); // CRITICAL: Ensure the quaternion remains normalized


   if (logPackets) {
     Serial.println("  Applied state:");
     Serial.printf("    walkCycleRunning: %s\n", walkCycleRunning ? "true":"false");
     Serial.printf("    bodyVelocity: (%.2f, %.2f, %.2f)\n", bodyVelocity.x, bodyVelocity.y, bodyVelocity.z);
     //Serial.printf("    bodyAngVelYaw: %.3f\n", bodyAngularVelocityYaw); //TODO! uncomment
     Serial.printf("    walkParams (H, F, D): (%.2f, %.2f, %.2f)\n", walkParams.stepHeight, walkParams.stepFrequency, walkParams.dutyFactor);
     Serial.printf("    bodyPosOffset: (%.2f, %.2f, %.2f)\n", bodyPositionOffset.x, bodyPositionOffset.y, bodyPositionOffset.z);
     Serial.printf("    bodyOrient (WXYZ): (%.3f, %.3f, %.3f, %.3f)\n", bodyOrientation.w, bodyOrientation.x, bodyOrientation.y, bodyOrientation.z);
   }

  return true; // Packet processed successfully
}


/**
 * @brief Processes incoming serial commands (like toggling logs or exiting).
 * @return True to continue running this mode, False to exit back to the main menu.
 */
bool processSerialCommands_remote() {
  if (Serial.available() > 0) {
    char command = Serial.read(); // Read single character command
    // Consume any extra newline characters that might follow
    while (Serial.available() > 0 && (Serial.peek() == '\n' || Serial.peek() == '\r')) {
        Serial.read();
    }

    switch (toupper(command)) { // Use uppercase for case-insensitivity
      case 'L':
        logPackets = !logPackets;
        Serial.print("\nPacket Logging is now ");
        Serial.println(logPackets ? "ON" : "OFF");
        printSerialHelp(); // Show help again as context
        break;

      case 'X':
        Serial.println("\nExiting Remote Control Mode via serial command.");
        walkCycleRunning = false; // Ensure robot stops moving
        // Optional: Add code here to move legs to a defined resting pose before exiting
        return false; // Signal exit to main loop

      case 'H':
      case '?':
        printSerialHelp();
        break;

      default:
        // Ignore unknown characters or print message if printable
         if (isprint(command)) {
             Serial.print("\nUnknown serial command: '");
             Serial.print(command);
             Serial.println("'. Type 'H' or '?' for help.");
         }
        break;
    }
  }
  return true; // Continue running
}


/**
 * @brief Main update loop for the remote walk cycle mode.
 * Checks serial, processes UDP packets, calculates dt, and calls the core walk cycle update.
 * @return True to continue running this mode, False to exit back to the main menu.
 */
bool walkcycleRemoteUpdate() {
  // 1. Check for Serial Commands (like Exit or Log Toggle)
  if (!processSerialCommands_remote()) {
    return false; // Exit command received
  }

  // 2. Process Incoming UDP Packets (updates state variables if a valid packet arrives)
  processFullControlPacket(); // Process one packet per call, if available

  // 3. Calculate Time Delta (dt) for Walk Cycle Update
  static unsigned long lastUpdateTimeMicros = 0;
  unsigned long nowMicros = micros();
  // Handle first run or potential timer rollover
  if (lastUpdateTimeMicros == 0 || nowMicros < lastUpdateTimeMicros) {
    lastUpdateTimeMicros = nowMicros;
  }
  float dt = (nowMicros - lastUpdateTimeMicros) / 1000000.0f;
  lastUpdateTimeMicros = nowMicros;

  // Prevent unreasonably large dt values if loop execution stalls significantly
  // Also handles the case where dt might be slightly negative due to micros() rollover
  // (though less frequent than millis()). Max dt of 0.1s (10Hz min update for simulation)
  if (dt < 0.0f || dt > 0.1f) {
      if (logPackets && dt != 0.0f) { // Avoid spamming if dt is 0 on first valid run
           Serial.printf("[Warning] Unusual dt detected: %.4f s. Clamping to 0.01s\n", dt);
      }
      dt = 0.01f; // Use a small, safe default delta time if calculation is suspect
  }

  // 4. Update Walk Cycle Logic (if enabled by UDP command)
  // This function uses the global variables (bodyVelocity, bodyAngularVelocityYaw, walkParams, etc.)
  // that were potentially updated by processFullControlPacket().
  if (walkCycleRunning) {
      updateWalkCycle(dt); // Pass the calculated time delta
      // Note: updateWalkCycle currently doesn't USE bodyAngularVelocityYaw.
      // That modification needs to be made within walkcycle.h itself.
  }

  return true; // Continue running this mode
}

#endif // WALK_CYCLE_REMOTE_H
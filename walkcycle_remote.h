#ifndef WALK_CYCLE_REMOTE_H
#define WALK_CYCLE_REMOTE_H

#include <WiFi.h>
#include <WiFiUdp.h>
#include "robot_spec.h"   // Include AFTER defining LEG_COUNT potentially
#include "packets.h"      // Include the new packet definitions
#include "walkcycle.h"    // Base walk cycle logic
#include "utils.h"        // Utilities like clampf
#include "passwords.h"

// --- Network Configuration ---
#ifndef OSSIAN_HEMMA
const char* ssid = "guestnet";         // Replace with your WiFi SSID
const char* password = "VolvoAmazon"; // Replace with your WiFi Password
unsigned int localUdpPort = 5005;           // Port to listen on
#endif
#ifdef OSSIAN_HEMMA
const char* ssid = SSID;         // Replace with your WiFi SSID
const char* password = PASSWORD; // Replace with your WiFi Password
unsigned int localUdpPort = 5005;           // Port to listen on
// --- Angle Broadcast Configuration ---
const char* angleBroadcastIp = "255.255.255.255"; // Broadcast address (adjust if needed, e.g., "192.168.1.255")
unsigned int angleBroadcastPort = 5006;           // Different port for sending angles
// Array to hold the angles before sending. Size = Legs * JointsPerLeg (assuming 3)
// Make sure LEG_COUNT is defined correctly in robot_spec.h!
float jointAnglesToSend[LEG_COUNT * 3];
#endif



// ### Update buffer size for the new packet ###
uint8_t incomingPacketBuffer[sizeof(FullControlPacket)];
uint64_t lastPacketTimestampMs = 0;
uint32_t lastSequenceNumber = 0;
const uint32_t SEQUENCE_IGNORE_WINDOW = 25;
bool logPackets = false;

// Forward declarations
void setupWalkcycleRemote();
bool walkcycleRemoteUpdate();
bool processFullControlPacket();
bool processSerialCommands_remote();
void printSerialHelp();

/**
 * @brief Sets up WiFi connection, UDP listener, and initializes walk cycle state.
 */
void setupWalkcycleRemote() {
  Serial.println("\nSetting up Walk Cycle Remote Control (UDP) - Full Control Mode...");
  setupWalkcycle(); // Initializes walk cycle data using initial defaultFootPositionWalk
  walkCycleRunning = false;
  logPackets = false;
  lastSequenceNumber = 0;
  lastPacketTimestampMs = 0;



  Serial.printf("Starting UDP listener on port %d\n", localUdpPort);
  if (udp.begin(localUdpPort)) {
      // ### Updated expected size message ###
      Serial.printf("UDP Listener started. Expected packet size: %d bytes (FullControlPacket).\n", sizeof(FullControlPacket));
  } else {
      Serial.println("Failed to start UDP Listener! Halting.");
      while(1) { delay(1000); }
  }

  Serial.println("==== Walk Cycle Remote Ready ====");
  printSerialHelp();
  Serial.println("================================");
}

/**
 * @brief Prints available serial commands for this mode.
 */
void printSerialHelp() {
    // ... (Help text remains the same) ...
    Serial.println("\n==== Serial Commands (Remote Mode) ====");
    Serial.println(" L - Toggle UDP Packet Logging (currently " + String(logPackets ? "ON" : "OFF") + ")");
    Serial.println(" X - Exit Remote Control Mode");
    Serial.println(" H / ? - Display this Help");
    Serial.println("=======================================");
}

/**
 * @brief Processes one incoming UDP packet, validates it, and updates robot state.
 * @return True if a packet was available, False otherwise.
 */
bool processFullControlPacket() {
  int packetSize = udp.parsePacket();
  if (!packetSize) {
    return false; // No packet
  }

  IPAddress remoteIp = udp.remoteIP();
  unsigned int remotePort = udp.remotePort();

  if (logPackets) {
    Serial.printf("\nReceived packet: %d bytes from %s:%d\n", packetSize, remoteIp.toString().c_str(), remotePort);
  }

  // --- Validation 1: Packet Size --- ### Use new size ###
  if (packetSize != sizeof(FullControlPacket)) {
    Serial.printf("  [Error] Packet size mismatch! Expected %d, Got %d. Discarding.\n", sizeof(FullControlPacket), packetSize);
    udp.flush();
    return true;
  }

  // Read packet data
  int len = udp.read(incomingPacketBuffer, sizeof(FullControlPacket));
  if (len != sizeof(FullControlPacket)) {
      Serial.printf("  [Error] Failed to read correct # bytes! Read %d. Discarding.\n", len);
      udp.flush();
      return true;
  }

  // Cast buffer to the packet struct pointer
  FullControlPacket* receivedPacket = (FullControlPacket*)incomingPacketBuffer;

  // --- Validation 2: Identifier --- ### Use new identifier ###
  if (receivedPacket->identifier != FULL_CONTROL_PACKET_IDENTIFIER) {
      Serial.printf("  [Error] Invalid packet identifier! Expected 0x%X, Got 0x%X. Discarding.\n",
                    FULL_CONTROL_PACKET_IDENTIFIER, receivedPacket->identifier);
      return true;
  }

  // --- Validation 3: Sequence Number --- (Same logic as before)
  if (receivedPacket->sequenceNumber <= lastSequenceNumber && lastSequenceNumber != 0 && (lastSequenceNumber - receivedPacket->sequenceNumber < SEQUENCE_IGNORE_WINDOW)) {
       if (logPackets) { // Log if needed
          Serial.printf("  [Info] Stale/duplicate sequence number (%u <= %u). Ignoring.\n",
                         receivedPacket->sequenceNumber, lastSequenceNumber);
       }
       return true; // Handled (by ignoring)
  }
  // --- Timestamp checks (same as before) ---
   if (receivedPacket->timestampMs <= lastPacketTimestampMs && logPackets && lastSequenceNumber != 0) {
         Serial.printf("  [Warning] Newer seq (%u > %u) but older timestamp (%llu ms <= %llu ms). Clock skew?\n",
                       receivedPacket->sequenceNumber, lastSequenceNumber,
                       receivedPacket->timestampMs, lastPacketTimestampMs);
   }

  // Update sequence and timestamp tracking
  uint32_t oldSequenceNumber = lastSequenceNumber;
  lastSequenceNumber = receivedPacket->sequenceNumber;
  lastPacketTimestampMs = receivedPacket->timestampMs;


  // --- Optional Detailed Logging (Updated for new fields) ---
  if (logPackets) {
    Serial.println("  --- Processing Valid Packet ---");
    Serial.printf("    Identifier: 0x%X\n", receivedPacket->identifier);
    Serial.printf("    Timestamp: %llu ms\n", receivedPacket->timestampMs);
    Serial.printf("    Sequence #: %u (Prev: %u)\n", receivedPacket->sequenceNumber, oldSequenceNumber);
    Serial.printf("    Raw Flags: 0x%02X\n", receivedPacket->controlFlags);
    Serial.printf("    Raw LinVel(X,Y,Z): (%.2f, %.2f, %.2f)\n", receivedPacket->velocityX, receivedPacket->velocityY, receivedPacket->velocityZ);
    Serial.printf("    Raw AngVel(Yaw): %.3f\n", receivedPacket->angularVelocityYaw);
    Serial.printf("    Raw Gait (H, F, D): (%.2f, %.2f, %.2f)\n", receivedPacket->stepHeight, receivedPacket->stepFrequency, receivedPacket->dutyFactor);
    Serial.printf("    Raw PosOff(X,Y,Z): (%.2f, %.2f, %.2f)\n", receivedPacket->bodyPositionOffsetX, receivedPacket->bodyPositionOffsetY, receivedPacket->bodyPositionOffsetZ);
    Serial.printf("    Raw Orient(W,X,Y,Z): (%.3f, %.3f, %.3f, %.3f)\n", receivedPacket->bodyOrientationW, receivedPacket->bodyOrientationX, receivedPacket->bodyOrientationY, receivedPacket->bodyOrientationZ);
    // ### Log received base foot positions ###
    Serial.println("    Raw Base Foot Pos (XYZ):");
    for (int i = 0; i < LEG_COUNT; ++i) {
        Serial.printf("      L%d: (%.1f, %.1f, %.1f)\n", i,
                      receivedPacket->baseFootPosXYZ[i * 3 + 0],
                      receivedPacket->baseFootPosXYZ[i * 3 + 1],
                      receivedPacket->baseFootPosXYZ[i * 3 + 2]);
    }
    Serial.println("  ---------------------------------");
  }

  // --- Apply Packet Data to Robot State ---

  // 1. Update Walk Cycle Running State (Same as before)
  bool newRunningState = (receivedPacket->controlFlags & FLAG_WALK_RUNNING) != 0;
  if (newRunningState && !walkCycleRunning) { globalPhase = 0.0f; /* Reset phase */ }
  walkCycleRunning = newRunningState;

  // 2. Update Velocities (Same as before)
  bodyVelocity.x = receivedPacket->velocityX;
  bodyVelocity.y = receivedPacket->velocityY;
  bodyVelocity.z = receivedPacket->velocityZ;
  bodyAngularVelocityYaw = receivedPacket->angularVelocityYaw;

  // 3. Update Walk Parameters (Same as before)
  walkParams.stepHeight = receivedPacket->stepHeight;
  walkParams.stepFrequency = receivedPacket->stepFrequency;
  walkParams.dutyFactor = clampf(receivedPacket->dutyFactor, 0.01f, 0.99f);

  // 4. Update Body Pose (Same as before)
  bodyPositionOffset.x = receivedPacket->bodyPositionOffsetX;
  bodyPositionOffset.y = receivedPacket->bodyPositionOffsetY;
  bodyPositionOffset.z = receivedPacket->bodyPositionOffsetZ;
  bodyOrientation.w = receivedPacket->bodyOrientationW;
  bodyOrientation.x = receivedPacket->bodyOrientationX;
  bodyOrientation.y = receivedPacket->bodyOrientationY;
  bodyOrientation.z = receivedPacket->bodyOrientationZ;
  bodyOrientation.normalize();

  // 5. ### Update Base Foot Positions ###
  for (int i = 0; i < LEG_COUNT; ++i) {
      // Direct copy from packet into the global array
      baseFootPositionWalk[i].x = receivedPacket->baseFootPosXYZ[i * 3 + 0];
      baseFootPositionWalk[i].y = receivedPacket->baseFootPosXYZ[i * 3 + 1];
      baseFootPositionWalk[i].z = receivedPacket->baseFootPosXYZ[i * 3 + 2];
  }

   if (logPackets) { // Log applied state (optional: add base pos here too)
     Serial.println("  Applied state:");
     Serial.printf("    walkCycleRunning: %s\n", walkCycleRunning ? "true":"false");
     // ... (log other applied states) ...
     // Example logging applied base positions:
     Serial.println("    Applied Base Foot Pos (XYZ):");
     for (int i = 0; i < LEG_COUNT; ++i) {
         Serial.printf("      L%d: (%.1f, %.1f, %.1f)\n", i,
                       baseFootPositionWalk[i].x,
                       baseFootPositionWalk[i].y,
                       baseFootPositionWalk[i].z);
     }
   }

  return true; // Packet processed
}


/**
 * @brief Processes incoming serial commands (like toggling logs or exiting).
 * @return True to continue running, False to exit.
 */
bool processSerialCommands_remote() {
    // ... (This function remains unchanged) ...
    if (Serial.available() > 0) {
        char command = Serial.read();
        while (Serial.available() > 0 && (Serial.peek() == '\n' || Serial.peek() == '\r')) { Serial.read(); }
        switch (toupper(command)) {
            case 'L': logPackets = !logPackets; Serial.printf("\nPacket Logging: %s\n", logPackets ? "ON" : "OFF"); printSerialHelp(); break;
            case 'X': Serial.println("\nExiting Remote Mode..."); walkCycleRunning = false; return false;
            case 'H': case '?': printSerialHelp(); break;
            default: if (isprint(command)) { Serial.printf("\nUnknown command: '%c'. H for help.\n", command); } break;
        }
    }
    return true;
}

/**
 * @brief Main update loop for the remote walk cycle mode.
 * @return True to continue running, False to exit.
 */
bool walkcycleRemoteUpdate() {
  // ... (Structure remains the same) ...
  // 1. Check Serial Commands
  if (!processSerialCommands_remote()) {
    return false;
  }

  // 2. Process Incoming UDP Packet(s)
  // Process multiple packets if available quickly? Or just one per loop?
  // Processing just one is usually fine and simpler.
  processFullControlPacket();

  // 3. Calculate Time Delta (dt)
  // ... (dt calculation and clamping logic remains the same) ...
  static unsigned long lastUpdateTimeMicros = 0;
  unsigned long nowMicros = micros();
  if (lastUpdateTimeMicros == 0 || nowMicros < lastUpdateTimeMicros) { lastUpdateTimeMicros = nowMicros; }
  float dt = (nowMicros - lastUpdateTimeMicros) / 1000000.0f;
  lastUpdateTimeMicros = nowMicros;
  if (dt < 0.0f || dt > 0.1f) {
      if (logPackets && dt != 0.0f) { Serial.printf("[Warning] Unusual dt: %.4f s. Clamping to 0.01s\n", dt); }
      dt = 0.01f;
  }


  // 4. Update Walk Cycle Logic if Running
  if (walkCycleRunning) {
      updateWalkCycle(dt); // This now implicitly uses the updated defaultFootPositionWalk values
  }

  return true;
}

#endif // WALK_CYCLE_REMOTE_H
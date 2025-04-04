#ifndef WALK_CYCLE_REMOTE_H
#define WALK_CYCLE_REMOTE_H

#include <WiFi.h>        // Or appropriate networking library for your board
#include <WiFiUdp.h>     // Or appropriate UDP library
#include <stdint.h>      // For fixed-width integer types (uint32_t, uint64_t)
#include "walkcycle.h"   // Include the original walk cycle logic (for setup/types)
#include "ik.h"          // Assuming ik.h is needed by walkcycle.h/setupIK
#include "utils.h"       // Assuming utils.h is needed

// --- Network Configuration ---
const char* ssid = "guestnet";         // Replace with your WiFi SSID
const char* password = "VolvoAmazon"; // Replace with your WiFi Password
unsigned int localUdpPort = 5005;           // <<< MUST MATCH PYTHON SENDER PORT

// --- Packet Structure Definition ---
// This MUST exactly match the Python struct format: '<IQBffffff'
// '<' (Little-Endian) is the default for ESP32, so direct casting should work.
// I = uint32_t (4 bytes)
// Q = uint64_t (8 bytes) - For timestamp in milliseconds
// B = uint8_t  (1 byte)  - For boolean flag
// f = float    (4 bytes) - For parameters
#define WALK_PACKET_IDENTIFIER 0xDEADBEEF // Magic number (must match Python)

#pragma pack(push, 1) // Ensure compiler doesn't add padding
struct WalkControlPacket {
  uint32_t identifier;       // Corresponds to 'I'
  uint64_t timestampMs;      // Corresponds to 'Q' (Timestamp in milliseconds)
  uint8_t  walkCycleRunning; // Corresponds to 'B' (0 or 1)
  float    velX;             // Corresponds to 'f'
  float    velY;             // Corresponds to 'f'
  float    velZ;             // Corresponds to 'f'
  float    stepHeight;       // Corresponds to 'f'
  float    stepFrequency;    // Corresponds to 'f'
  float    dutyFactor;       // Corresponds to 'f'
};
#pragma pack(pop) // Restore default packing

// Check that the compiler's size calculation matches Python's
// sizeof(uint32_t) + sizeof(uint64_t) + sizeof(uint8_t) + 6 * sizeof(float)
// 4 + 8 + 1 + 6 * 4 = 13 + 24 = 37 bytes. This should match Python's calculation.
// You can add a static_assert here if your compiler supports it:
// static_assert(sizeof(WalkControlPacket) == 37, "WalkControlPacket size mismatch");


// --- Global Variables ---
WiFiUDP udp;
// Use a byte array for the buffer. Size MUST match the struct size.
uint8_t incomingPacketBuffer[sizeof(WalkControlPacket)];
uint64_t lastPacketTimestampMs = 0; // Track the timestamp of the last processed packet


// Forward declaration
bool walkcycleRemoteUpdate();

void setupWalkcycleRemote() {
  // Initialize Serial for debugging (ensure this is done only once in your project)
  // Serial.begin(115200);
  // while (!Serial); // Wait for Serial if needed

  Serial.println("\nSetting up Walk Cycle Remote Control (UDP)...");

  // Setup base walk cycle systems (like IK, initial leg positions)
  // This ensures legs don't start in weird positions before first command
  setupWalkcycle();
  walkCycleRunning = false; // Start in stopped state

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
      Serial.printf("UDP Listener started successfully. Expected packet size: %d bytes.\n", sizeof(WalkControlPacket));
  } else {
      Serial.println("Failed to start UDP Listener! Halting.");
      // Handle error appropriately - maybe blink LED, halt, etc.
      while(1) { delay(1000); }
  }

  Serial.println("==== Walk Cycle Remote Ready ====");
  Serial.println("Waiting for UDP control packets...");
  Serial.println("================================");
}

bool processWalkPacket() {
  int packetSize = udp.parsePacket();
  if (!packetSize) {
    return false; // No packet available
  }

  // Read the packet data
  IPAddress remoteIp = udp.remoteIP();
  unsigned int remotePort = udp.remotePort();

  // Use Serial.printf for cleaner formatting
  Serial.printf("\nReceived packet: %d bytes from %s:%d\n", packetSize, remoteIp.toString().c_str(), remotePort);

  // --- Validation 1: Packet Size ---
  if (packetSize != sizeof(WalkControlPacket)) {
    Serial.printf("  [Error] Packet size mismatch! Expected %d, Got %d. Discarding.\n", sizeof(WalkControlPacket), packetSize);
    udp.flush(); // Discard invalid packet data
    return true; // Packet was handled (by discarding)
  }

  // Read packet data into the buffer
  int len = udp.read(incomingPacketBuffer, sizeof(WalkControlPacket));
  if (len != sizeof(WalkControlPacket)) {
      Serial.printf("  [Error] Failed to read correct number of bytes! Read %d. Discarding.\n", len);
      udp.flush();
      return true;
  }

  // --- De-serialization: Cast buffer to struct pointer ---
  // Assumes microcontroller and sender have the same endianness (ESP32 is Little Endian, Python '<' format is Little Endian)
  WalkControlPacket* receivedPacket = (WalkControlPacket*)incomingPacketBuffer;

  // --- Validation 2: Identifier ---
  if (receivedPacket->identifier != WALK_PACKET_IDENTIFIER) {
      Serial.printf("  [Error] Invalid packet identifier! Expected 0x%X, Got 0x%X. Discarding.\n",
                    WALK_PACKET_IDENTIFIER, receivedPacket->identifier);
      return true; // Invalid identifier, discard
  }

  // --- Validation 3: Timestamp ---
  // Compare incoming timestamp with the last processed one
  bool isNewer = receivedPacket->timestampMs > lastPacketTimestampMs;
  const char* timestampStatus = "";
  if (!isNewer) {
      timestampStatus = " <-- OLD or DUPLICATE PACKET! IGNORING.";
      // Optionally print details only if old/duplicate for less clutter
      Serial.printf("  Timestamp: %llu ms %s (Last good: %llu ms)\n",
          receivedPacket->timestampMs, timestampStatus, lastPacketTimestampMs);
      return true; // Discard old/duplicate packet
  } else {
      timestampStatus = " (Newer)";
      // Update the last known good timestamp *before* processing
      lastPacketTimestampMs = receivedPacket->timestampMs;
  }

  // --- Print Received Data (Only for Newer Packets) ---
  Serial.println("  --- Processing Valid Packet ---");
  Serial.printf("    Identifier: 0x%X\n", receivedPacket->identifier);
  // Use %llu for uint64_t
  Serial.printf("    Timestamp: %llu ms %s\n", receivedPacket->timestampMs, timestampStatus);
  Serial.printf("    Running: %s\n", (receivedPacket->walkCycleRunning != 0) ? "True" : "False");
  Serial.printf("    Velocity (X, Y, Z): (%.2f, %.2f, %.2f) cm/s\n",
                receivedPacket->velX, receivedPacket->velY, receivedPacket->velZ);
  Serial.printf("    Step Height: %.2f cm\n", receivedPacket->stepHeight);
  Serial.printf("    Step Frequency: %.2f Hz\n", receivedPacket->stepFrequency);
  Serial.printf("    Duty Factor: %.2f\n", receivedPacket->dutyFactor);
  Serial.println("  ---------------------------------");


  // --- TODO: Update Actual Walk Cycle State ---
  // This section will be enabled later, once packet reception is confirmed.
  /*
  bool newRunningState = (receivedPacket->walkCycleRunning != 0);
  if (newRunningState && !walkCycleRunning) {
      globalPhase = 0.0f; // Reset phase when starting
      Serial.println("  Remote command: Starting walk cycle.");
  } else if (!newRunningState && walkCycleRunning) {
      Serial.println("  Remote command: Stopping walk cycle.");
  }
  walkCycleRunning = newRunningState;

  bodyVelocity.x = receivedPacket->velX;
  bodyVelocity.y = receivedPacket->velY;
  bodyVelocity.z = receivedPacket->velZ; // Update Z velocity if used

  walkParams.stepHeight = receivedPacket->stepHeight;
  walkParams.stepFrequency = receivedPacket->stepFrequency;
  walkParams.dutyFactor = clampf(receivedPacket->dutyFactor, 0.01f, 0.99f); // Clamp received value
  */

  return true; // Packet processed successfully
}

// This function replaces walkcycleSerialUpdate in your main loop
bool walkcycleRemoteUpdate() {
  // Check for and process incoming UDP packets
  processWalkPacket(); // Process one packet per call, if available

  // --- TODO: Update Actual Walk Cycle Logic ---
  // This section will be enabled later. It needs the dt calculation.
  /*
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();
  float dt = (now - lastUpdate) / 1000.0f;
  // Prevent large dt spikes on first run or after delays
  if (dt > 0.5f || dt < 0.0f ) { // also check for negative dt if millis rolls over, unlikely with uint64_t timestamps though
      dt = 0.01f; // Use a small default dt instead of 0
      // Serial.println("Warning: Large/invalid dt detected, using default.");
  }
  lastUpdate = now;

  // Update the actual walk cycle IF it's running AND we have received valid data
  if (walkCycleRunning) {
    updateWalkCycle(dt);
  } else {
      // Optional: Add logic for stopped state (e.g., hold position)
  }
  */

  // This remote version currently doesn't have an 'exit' concept.
  return true; // Keep the main loop going
}

#endif // WALK_CYCLE_REMOTE_H
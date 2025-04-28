// packets.h
#ifndef PACKETS_H
#define PACKETS_H

#include <stdint.h> // For fixed-width integer types like uint32_t, uint64_t

// --- Packet Identifiers ---
// Use distinct values to differentiate packet types if more are added later.
#define FULL_CONTROL_PACKET_IDENTIFIER 0xFEEDF00D // Identifier for the comprehensive control packet

// --- Full Control Packet Structure ---
// Used for sending comprehensive commands (velocity, pose, gait) via UDP.
// Ensure sender and receiver use the same packing and endianness (Little Endian assumed).

#pragma pack(push, 1) // Ensure packed structure with no padding for consistent network transmission
struct FullControlPacket {
    // --- Metadata (16 bytes) ---
    uint32_t identifier;       // Packet identifier (e.g., FULL_CONTROL_PACKET_IDENTIFIER)
    uint64_t timestampMs;      // Sender timestamp in milliseconds (for ordering/latency checks)
    uint32_t sequenceNumber;   // Monotonically increasing sequence number (for detecting loss)

    // --- Control State (1 byte) ---
    uint8_t  controlFlags;     // Bit 0: walkCycleRunning (1 = run, 0 = stop)

    // --- Locomotion Control (28 bytes) ---
    // Desired Body Velocity (relative to Walk Frame)
    float    velocityX;        // Desired sideways speed (cm/s) in Walk Frame X (Right)
    float    velocityY;        // Desired forward/backward speed (cm/s) in Walk Frame Y (Forward)
    float    velocityZ;        // Desired vertical speed (cm/s) in Walk Frame Z (Up) - (Often 0 for ground walking)
    // Desired Body Angular Velocity (relative to Walk Frame)
    float    angularVelocityYaw; // Desired turning speed (radians/s around Walk Frame Z)
    // Walk Gait Parameters
    float    stepHeight;       // Max height foot lifts during swing (cm)
    float    stepFrequency;    // Steps per second for a full cycle (Hz)
    float    dutyFactor;       // Proportion of cycle time foot is on the ground (e.g., 0.5 for 50%)

    // --- Body Pose Control (28 bytes) ---
    // Desired Body Position Offset (relative to Walk Frame origin)
    float    bodyPositionOffsetX; // Desired X offset (cm) - Right/Left relative to walk direction
    float    bodyPositionOffsetY; // Desired Y offset (cm) - Forward/Backward relative to walk direction
    float    bodyPositionOffsetZ; // Desired Z offset (ride height) (cm) - Up/Down from ground
    // Desired Body Orientation (relative to Walk Frame axes) - Quaternion
    float    bodyOrientationW;  // Quaternion W component (scalar part)
    float    bodyOrientationX;  // Quaternion X component (vector part)
    float    bodyOrientationY;  // Quaternion Y component (vector part)
    float    bodyOrientationZ;  // Quaternion Z component (vector part)

    // --- Optional: Checksum/CRC could be added here ---
    // uint32_t crc32;
};
#pragma pack(pop) // Restore default packing alignment

// --- Total Size Calculation ---
// Metadata: 4 (ID) + 8 (TS) + 4 (Seq) = 16 bytes
// Control State: 1 (Flags) = 1 byte
// Locomotion: (3 linear vel + 1 angular vel + 3 walk params) * 4 bytes/float = 7 * 4 = 28 bytes
// Body Pose: (3 pos offset + 4 quat) * 4 bytes/float = 7 * 4 = 28 bytes
// Total = 16 + 1 + 28 + 28 = 73 bytes

// --- Compile-time size check (optional, might require C++11 or newer features enabled) ---
static_assert(sizeof(FullControlPacket) == 73, "FullControlPacket size mismatch. Expected 73 bytes.");

// --- Bit Masks for controlFlags ---
#define FLAG_WALK_RUNNING  (1 << 0) // Bit 0: Controls if the walk cycle logic executes

#endif // PACKETS_H
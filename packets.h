// packets.h (Should already be like this from previous steps)
#ifndef PACKETS_H
#define PACKETS_H

#include <stdint.h>

// --- Packet Identifiers ---
#define FULL_CONTROL_PACKET_IDENTIFIER 0xFEEDF00D

#pragma pack(push, 1)
struct FullControlPacket {
    // --- Metadata (16 bytes) ---
    uint32_t identifier;
    uint64_t timestampMs;
    uint32_t sequenceNumber;

    // --- Control State (1 byte) ---
    uint8_t  controlFlags; // Bit 0: walkCycleRunning

    // --- Locomotion Control (28 bytes) ---
    float    velocityX;
    float    velocityY;
    float    velocityZ;
    float    angularVelocityYaw;
    float    stepHeight;
    float    stepTime;
    float    dutyFactor;

    // --- Body Pose Control (28 bytes) ---
    float    bodyPositionOffsetX;
    float    bodyPositionOffsetY;
    float    bodyPositionOffsetZ;
    float    bodyOrientationW;
    float    bodyOrientationX;
    float    bodyOrientationY;
    float    bodyOrientationZ;

    // --- Base Foot Positions (Walk Frame) (6 legs * 3 coords * 4 bytes/float = 72 bytes) ---
    float    baseFootPosXYZ[6 * 3];

};
#pragma pack(pop)

// Total Size: 16 + 1 + 28 + 28 + 72 = 145 bytes
static_assert(sizeof(FullControlPacket) == 145, "FullControlPacket size mismatch. Expected 145 bytes.");

// --- Bit Masks for controlFlags ---
#define FLAG_WALK_RUNNING  (1 << 0)

#endif // PACKETS_H
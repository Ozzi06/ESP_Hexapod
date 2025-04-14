      
#ifndef ROBOT_SPEC_H
#define ROBOT_SPEC_H

#include "Vec3.h"
#include "quat.h"

// --- Constants ---
#define LEG_COUNT 6 // Make sure this matches LEG_SERVOS etc.

// Leg mechanical parameters (in cm) - Moved from ik_servo.h
extern const float COXA_LENGTH;
extern const float FEMUR_LENGTH;
extern const float TIBIA_LENGTH;

// Joint angle limits (in radians) - Moved from ik_servo.h
extern const float COXA_MIN_ANGLE;
extern const float COXA_MAX_ANGLE;
extern const float FEMUR_MIN_ANGLE;
extern const float FEMUR_MAX_ANGLE;
extern const float TIBIA_MIN_ANGLE;
extern const float TIBIA_MAX_ANGLE;

// Leg Geometry & Configuration
// Position of each leg's coxa joint relative to the Body Frame origin (X=Right, Y=Fwd, Z=Up)
extern const Vec3 legOriginOffset[LEG_COUNT];
// Mounting angle (yaw/Z-rotation) of each leg relative to the Body Frame's Y-axis (radians)
// Example: 0 = straight forward/backward, +PI/4 = 45deg right, -PI/4 = 45deg left
extern const float legMountingAngle[LEG_COUNT];
// Flags for mirroring logic (true if leg is on the robot's left side)
extern const bool isLeftLeg[LEG_COUNT];

// Servo channel assignments per leg [leg][joint: 0=coxa, 1=femur, 2=tibia]
// Consider moving this here too for complete spec centralization?
// extern const uint8_t LEG_SERVOS[LEG_COUNT][3];

// --- Global State Variables ---
// Desired body pose relative to the Walk Frame (Walk Frame: X=Right, Y=Fwd, Z=Up, fixed orientation)
extern Vec3 bodyPositionOffset;    // Body center translation from Walk Frame origin
extern Quaternion bodyOrientation; // Body orientation relative to Walk Frame axes

#endif // ROBOT_SPEC_H

    
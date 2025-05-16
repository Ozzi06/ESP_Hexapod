#ifndef ROBOT_SPEC_H
#define ROBOT_SPEC_H

#include "math_utils.h"

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

//Leg names
extern const char* leg_names[LEG_COUNT];

// Leg Geometry & Configuration
// Position of each leg's coxa joint relative to the Body Frame origin (X=Right, Y=Fwd, Z=Up)
extern const Vec3 legOriginOffset[LEG_COUNT];
// Mounting angle (yaw/Z-rotation) of each leg relative to the Body Frame's Y-axis (radians)
// Example: 0 = straight forward/backward, +PI/4 = 45deg right, -PI/4 = 45deg left
extern const float legMountingAngle[LEG_COUNT];
extern const float servo_center_angle[3];

// Servo channel assignments per leg [leg][joint: 0=coxa, 1=femur, 2=tibia]
// Maps logical leg/joint to the physical PCA9685 channel (0-15 range, board dependent)
/*constexpr uint8_t LEG_SERVOS[LEG_COUNT][3] = {
  { 2,  1,  0},        // Leg 0 (BR)
  { 10,  9,  8},       // Leg 1 (CR)
  { 14,  13,  12},     // Leg 2 (FR)
  { 16+14, 16+13, 16+12}, // Leg 3 (BL)
  { 16+6, 16+5, 16+4},    // Leg 4 (CL)
  { 16+2, 16+1, 16+0},    // Leg 5 (FL)
};*/
constexpr uint8_t LEG_SERVOS[LEG_COUNT][3] = {
  { 16+2, 16+1, 16+0},    // Leg 0 (BR)
  { 16+6, 16+5, 16+4},    // Leg 1 (CR)
  { 16+14, 16+13, 16+12}, // Leg 2 (FR)
  { 14,  13,  12},     // Leg 3 (BL)
  { 10,  9,  8},       // Leg 4 (CL)
  { 2,  1,  0},        // Leg 5 (FL)
};

// --- Global State Variables ---
// Desired body pose relative to the Walk Frame (Walk Frame: X=Right, Y=Fwd, Z=Up, fixed orientation)
extern float latestServoAngles[LEG_COUNT * 3];
extern Vec3 bodyPositionOffset;    // Body center translation from Walk Frame origin
extern Quaternion bodyOrientation; // Body orientation relative to Walk Frame axes
extern Vec3 bodyVelocity; // Desired body velocity relative to the Walk Frame (cm/s)
extern float bodyAngularVelocityYaw;
    

// --- Default Stance Positions ---
// Neutral position for each foot relative to the Walk Frame origin (X=Right, Y=Fwd, Z=Up)
// Used as the center point for stepping calculations. Z=0 usually means foot on the ground.
extern Vec3 baseFootPositionWalk[LEG_COUNT];


#endif // ROBOT_SPEC_H

    
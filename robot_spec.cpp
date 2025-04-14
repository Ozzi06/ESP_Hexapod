#include "robot_spec.h"

// Define constants
const float COXA_LENGTH = 7.0f;
// ... other lengths and angle limits ...

const Vec3 legOriginOffset[LEG_COUNT] = {
    { 10.0f,  10.0f, 0.0f}, // Leg 0 (Front Right)
    { 12.0f,   0.0f, 0.0f}, // Leg 1 (Mid Right)
    { 10.0f, -10.0f, 0.0f}, // Leg 2 (Rear Right)
    {-10.0f,  10.0f, 0.0f}, // Leg 3 (Front Left)
    {-12.0f,   0.0f, 0.0f}, // Leg 4 (Mid Left)
    {-10.0f, -10.0f, 0.0f}  // Leg 5 (Rear Left) - ADJUST THESE VALUES!
};

const float legMountingAngle[LEG_COUNT] = {
     M_PI / 4.0f, // Leg 0 (+45 deg) - Assuming your +45 deg example
     0.0f,        // Leg 1 (Straight)
    -M_PI / 4.0f, // Leg 2 (-45 deg)
    -M_PI / 4.0f, // Leg 3 (-45 deg mirrored from Leg 0) - Check convention!
     0.0f,        // Leg 4 (Straight)
     M_PI / 4.0f  // Leg 5 (+45 deg mirrored from Leg 2) - Check convention! - ADJUST THESE VALUES!
     // Ensure angles match your X=Right, Y=Fwd convention.
};

const bool isLeftLeg[LEG_COUNT] = {
    false, false, false, // Right legs
    true,  true,  true   // Left legs
};

// Define global state variables (initial values)
Vec3 bodyPositionOffset = {0.0f, 0.0f, 15.0f}; // Start 15cm above walk frame origin
Quaternion bodyOrientation = Quaternion::identity();
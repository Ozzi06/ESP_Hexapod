#ifndef WALK_CYCLE_H
#define WALK_CYCLE_H

#include <math.h>       // For fmod, M_PI, etc.
#include "ik.h"         // For calculateIK
#include "math_utils.h"      // For Vec3, clampf, setAngleRadians, etc.
#include "servo_angles.h"
#include "robot_spec.h" // For LEG_COUNT, defaultFootPositionWalk, bodyVelocity, etc.
#include "body_transform.h" // For transformWalkFrameToLegFrame

// --- Walk Cycle Parameters ---
// These parameters control the gait characteristics.
// They can be modified dynamically (e.g., by serial or remote commands).
struct WalkParams {
  float stepHeight = 3.0f;
  float stepTime = 1.0f;
  float dutyFactor = 0.5f; //TODO! remove this
};

// --- Per-Leg Walk Cycle State ---
// Stores state information specific to each leg's movement within the cycle.
struct LegCycleData {
  Vec3 currentPosition;
  Vec3 swingStartPosition;
};

extern WalkParams walkParams;
extern bool walkCycleRunning;

// --- Function Declarations ---

/**
 * @brief Initializes the walk cycle state for each leg.
 * Does NOT move the legs; just sets up internal data structures.
 */
void setupWalkcycle();

/**
 * @brief Main update function for the hexapod walk cycle.
 * Calculates target foot positions in the Walk Frame based on phase and velocity,
 * transforms them into the Leg IK Frame considering body pose,
 * performs IK, and sends commands to servos.
 *
 * @param dt Time elapsed since the last update (in seconds).
 */
void updateWalkCycle(float dt); // Main update function

#endif // WALK_CYCLE_H
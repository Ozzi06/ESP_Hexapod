#ifndef BODY_TRANSFORM_H
#define BODY_TRANSFORM_H

#include "Vec3.h"
#include "quat.h"
#include "robot_spec.h" // Includes global state and constants

/**
 * @brief Transforms a target foot position from the Walk Frame to the Leg Frame required by IK.
 *
 * Takes a desired foot position calculated relative to the Walk Frame (gravity-aligned, non-tilting),
 * applies the robot's desired body position offset and orientation, and accounts for the specific
 * leg's origin offset and mounting angle to produce coordinates suitable for the inverse kinematics
 * calculation (`calculateIK`).
 *
 * @param P_foot_walk Input: The target foot position {x,y,z} in the Walk Frame (X=Right, Y=Fwd, Z=Up).
 * @param leg_index Input: The index (0 to LEG_COUNT-1) of the leg being calculated.
 * @param P_foot_leg_ik_input Output: The calculated foot position {x,y,z} relative to the leg's coxa joint,
 *                                  expressed in the leg's local coordinate system (where the leg's
 *                                  X/Y plane is horizontal when coxa/femur/tibia angles are zero,
 *                                  and Y typically points along the femur direction).
 */
inline void transformWalkFrameToLegFrame(
    const Vec3& P_foot_walk,        // Input: Target foot pos calculated by walk_cycle.h (in Walk Frame)
    uint8_t leg_index,              // Input: Index of the leg (0 to LEG_COUNT-1)
    Vec3& P_foot_leg_ik_input)      // Output: Target foot pos for calculateIK (in leg's local frame)
{
    // 1. Get desired body pose relative to Walk Frame (from globals in robot_spec.h)
    const Vec3& P_body_walk = bodyPositionOffset;     // Body origin position in Walk Frame
    const Quaternion& Q_body_walk = bodyOrientation;  // Body orientation relative to Walk Frame

    // 2. Calculate foot position relative to Body origin, expressed in Body Frame axes
    // Vector from body origin to foot in Walk Frame
    Vec3 V_body_to_foot_walk = P_foot_walk - P_body_walk;

    // Inverse of body orientation to rotate from Walk Frame to Body Frame
    // Assumes Q_body_walk used for rotation is normalized. Normalize if unsure.
    // Q_body_walk.normalize(); // Optional: Ensure normalization if input isn't guaranteed
    Quaternion Q_body_walk_inv = Q_body_walk.conjugate();

    // Rotate the vector into the Body Frame
    Vec3 P_foot_body = rotate_vector_by_quaternion(V_body_to_foot_walk, Q_body_walk_inv);

    // 3. Calculate foot position relative to the Leg's coxa origin, still in Body Frame axes
    // Get this leg's coxa joint position relative to body origin (from globals)
    const Vec3& legOrigin_body = legOriginOffset[leg_index];

    // Vector from leg coxa origin to foot target, in Body Frame axes
    Vec3 V_leg_to_foot_body = P_foot_body - legOrigin_body;

    // 4. Rotate the vector from Body Frame axes into the Leg's local IK Frame axes
    // Get this leg's mounting angle (yaw rotation) relative to Body Frame (from globals)
    const float mountAngle = legMountingAngle[leg_index];

    // Create a quaternion representing the inverse rotation (-mountAngle around Z axis)
    // This rotates coordinates from the Body Frame into the Leg's coordinate system expected by IK.
    Quaternion Q_mount_inv = Quaternion::from_axis_angle({0.0f, 0.0f, 1.0f}, -mountAngle);

    // Rotate the vector from Body Frame coords to Leg IK Frame coords
    P_foot_leg_ik_input = rotate_vector_by_quaternion(V_leg_to_foot_body, Q_mount_inv);

    // P_foot_leg_ik_input now holds the coordinates {x,y,z} that should be passed to calculateIK
}


#endif // BODY_TRANSFORM_H
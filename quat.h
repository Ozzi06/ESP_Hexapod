#ifndef QUAT_H
#define QUAT_H

#include <math.h>    // For sqrt, sin, cos
#include "Vec3.h"    // Include the Vec3 definition

// Quaternion Structure
struct Quaternion {
    float w = 1.0f; // Real part
    float x = 0.0f; // i component
    float y = 0.0f; // j component
    float z = 0.0f; // k component

    // Default constructor (initializes to identity)
    Quaternion() = default;

    // Constructor with explicit values
    Quaternion(float w_val, float x_val, float y_val, float z_val)
        : w(w_val), x(x_val), y(y_val), z(z_val) {}

    // --- Basic Properties ---

    // Get the identity quaternion
    static Quaternion identity() {
        return Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
    }

    // Calculate the magnitude (norm) of the quaternion
    inline float norm() const {
        return sqrtf(w*w + x*x + y*y + z*z);
    }

    // --- Operations ---

    // Calculate the conjugate of the quaternion
    // For unit quaternions (used for rotation), conjugate is the inverse.
    inline Quaternion conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }

    // Return a normalized version of the quaternion (unit quaternion)
    // Rotations should always use unit quaternions.
    inline Quaternion normalized() const {
        float n = norm();
        if (n < 1e-9f) { // Avoid division by zero or near-zero
            fprintf(stderr, "Warning: Normalizing near-zero quaternion. Returning identity.\n");
            return Quaternion::identity();
        }
        float inv_norm = 1.0f / n;
        return Quaternion(w * inv_norm, x * inv_norm, y * inv_norm, z * inv_norm);
    }

    // Normalize the quaternion in-place
    inline void normalize() {
        float n = norm();
        if (n < 1e-9f) {
            fprintf(stderr, "Warning: Normalizing near-zero quaternion in-place. Setting to identity.\n");
            w = 1.0f; x = 0.0f; y = 0.0f; z = 0.0f; // Set to identity
            return;
        }
        float inv_norm = 1.0f / n;
        w *= inv_norm;
        x *= inv_norm;
        y *= inv_norm;
        z *= inv_norm;
    }

    // --- Creation from Axis-Angle ---
    // Creates a quaternion representing a rotation of angle_rad around axis.
    // Assumes axis is a non-zero vector (doesn't need to be normalized beforehand).
    static Quaternion from_axis_angle(const Vec3& axis, float angle_rad) {
        float half_angle = angle_rad * 0.5f;
        float s = sinf(half_angle);
        float c = cosf(half_angle);

        // Normalize the axis vector
        Vec3 normalized_axis = axis.normalized(); // Uses Vec3::normalized()

        return Quaternion(
            c,
            normalized_axis.x * s,
            normalized_axis.y * s,
            normalized_axis.z * s
        );
    }
};

// --- Standalone Functions ---

// Quaternion multiplication (Hamilton product: q_result = q_left * q_right)
// Note: Order matters! Corresponds to applying q_right rotation then q_left.
inline Quaternion operator*(const Quaternion& q_left, const Quaternion& q_right) {
    return Quaternion(
        q_left.w * q_right.w - q_left.x * q_right.x - q_left.y * q_right.y - q_left.z * q_right.z,  // New w
        q_left.w * q_right.x + q_left.x * q_right.w + q_left.y * q_right.z - q_left.z * q_right.y,  // New x
        q_left.w * q_right.y - q_left.x * q_right.z + q_left.y * q_right.w + q_left.z * q_right.x,  // New y
        q_left.w * q_right.z + q_left.x * q_right.y - q_left.y * q_right.x + q_left.z * q_right.w   // New z
    );
}

// Rotate a 3D vector by a unit quaternion
// Assumes q is normalized (unit quaternion).
// Implements v' = q * v * conjugate(q) using the efficient formula:
// v' = v + 2 * cross(q.xyz, cross(q.xyz, v) + q.w * v)
inline Vec3 rotate_vector_by_quaternion(const Vec3& v, const Quaternion& q) {
    Vec3 q_vec = {q.x, q.y, q.z}; // The vector part of the quaternion

    // Calculate the cross products using standalone cross function from Vec3.h
    Vec3 uv = cross(q_vec, v);    // cross(q.xyz, v)
    Vec3 uuv = cross(q_vec, uv);  // cross(q.xyz, cross(q.xyz, v))

    // Calculate the scaled vectors (using Vec3 operators)
    Vec3 scaled_uv = uv * (2.0f * q.w);
    Vec3 scaled_uuv = uuv * 2.0f;

    // Combine them (using Vec3 operator)
    return v + scaled_uv + scaled_uuv;
}


#endif // QUAT_H
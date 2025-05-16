#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <math.h>   // For sqrtf, fabsf, sinf, cosf
#include <stdio.h>  // For fprintf in error messages within inline functions

// --- Function Declarations (definitions in math_utils.cpp) ---
float mapf(float x, float in_min, float in_max, float out_min, float out_max);
float clampf(float val, float min, float max);
float clampmapf(float x, float in_min, float in_max, float out_min, float out_max);

// --- Vec3 Structure and Inline Methods ---
struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    // --- Constructors ---
    Vec3() = default; // Default constructor (0, 0, 0)
    Vec3(float x_val, float y_val, float z_val) : x(x_val), y(y_val), z(z_val) {}

    // --- Basic Arithmetic Operators ---
    inline Vec3 operator+(const Vec3& other) const {
        return Vec3(x + other.x, y + other.y, z + other.z);
    }
    inline Vec3 operator-(const Vec3& other) const {
        return Vec3(x - other.x, y - other.y, z - other.z);
    }
    inline Vec3 operator*(float scalar) const {
        return Vec3(x * scalar, y * scalar, z * scalar);
    }
    inline Vec3 operator/(float scalar) const {
        if (fabsf(scalar) < 1e-9f) {
             fprintf(stderr, "Warning: Vec3 division by zero/near-zero.\n");
             return Vec3(0.0f, 0.0f, 0.0f);
        }
        float inv_scalar = 1.0f / scalar;
        return Vec3(x * inv_scalar, y * inv_scalar, z * inv_scalar);
    }
    inline Vec3 operator-() const {
        return Vec3(-x, -y, -z);
    }

    // --- Compound Assignment Operators ---
    inline Vec3& operator+=(const Vec3& other) {
        x += other.x; y += other.y; z += other.z; return *this;
    }
    inline Vec3& operator-=(const Vec3& other) {
        x -= other.x; y -= other.y; z -= other.z; return *this;
    }
    inline Vec3& operator*=(float scalar) {
        x *= scalar; y *= scalar; z *= scalar; return *this;
    }
    inline Vec3& operator/=(float scalar) {
        if (fabsf(scalar) < 1e-9f) {
             fprintf(stderr, "Warning: Vec3 compound division by zero/near-zero.\n");
             return *this;
        }
        float inv_scalar = 1.0f / scalar;
        x *= inv_scalar; y *= inv_scalar; z *= inv_scalar; return *this;
    }
    
    inline bool operator==(const Vec3& other) const {
        const float epsilon = 1e-6f;
        return fabsf(x - other.x) < epsilon &&
            fabsf(y - other.y) < epsilon &&
            fabsf(z - other.z) < epsilon;
    }
    inline bool operator!=(const Vec3& other) const {
        return !(*this == other);
    }

    // --- Vector Operations ---
    inline float normSq() const {
        return x*x + y*y + z*z;
    }
    inline float norm() const {
        return sqrtf(normSq());
    }
    inline Vec3 normalized() const {
        float n = norm();
        if (n < 1e-9f) {
            return Vec3(0.0f, 0.0f, 0.0f);
        }
        return *this / n;
    }
    inline void normalize() {
        float n = norm();
        if (n < 1e-9f) {
            x = 0.0f; y = 0.0f; z = 0.0f;
            return;
        }
        *this /= n;
    }

    // --- Utility ---
    inline void print(const char* label = "Vec3") const {
        printf("%s: (%.4f, %.4f, %.4f)\n", label, x, y, z);
    }
};

// --- Standalone Vec3 Functions (Inline) ---
inline Vec3 operator*(float scalar, const Vec3& vec) {
    return vec * scalar;
}
inline float dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return Vec3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

// --- Quaternion Structure and Inline Methods ---
struct Quaternion {
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Quaternion() = default;
    Quaternion(float w_val, float x_val, float y_val, float z_val)
        : w(w_val), x(x_val), y(y_val), z(z_val) {}

    static Quaternion identity() {
        return Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
    }
    inline float norm() const {
        return sqrtf(w*w + x*x + y*y + z*z);
    }
    inline Quaternion conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }
    inline Quaternion normalized() const {
        float n = norm();
        if (n < 1e-9f) {
            fprintf(stderr, "Warning: Normalizing near-zero quaternion. Returning identity.\n");
            return Quaternion::identity();
        }
        float inv_norm = 1.0f / n;
        return Quaternion(w * inv_norm, x * inv_norm, y * inv_norm, z * inv_norm);
    }
    inline void normalize() {
        float n = norm();
        if (n < 1e-9f) {
            fprintf(stderr, "Warning: Normalizing near-zero quaternion in-place. Setting to identity.\n");
            w = 1.0f; x = 0.0f; y = 0.0f; z = 0.0f;
            return;
        }
        float inv_norm = 1.0f / n;
        w *= inv_norm; x *= inv_norm; y *= inv_norm; z *= inv_norm;
    }
    static Quaternion from_axis_angle(const Vec3& axis, float angle_rad) {
        float half_angle = angle_rad * 0.5f;
        float s = sinf(half_angle);
        float c = cosf(half_angle);
        Vec3 normalized_axis = axis.normalized();
        return Quaternion(
            c,
            normalized_axis.x * s,
            normalized_axis.y * s,
            normalized_axis.z * s
        );
    }
};

// --- Standalone Quaternion Functions ---
Quaternion slerp(const Quaternion& qa, const Quaternion& qb, float t);

// --- Standalone Quaternion Functions (Inline) ---
inline Quaternion operator*(const Quaternion& q_left, const Quaternion& q_right) {
    return Quaternion(
        q_left.w * q_right.w - q_left.x * q_right.x - q_left.y * q_right.y - q_left.z * q_right.z,
        q_left.w * q_right.x + q_left.x * q_right.w + q_left.y * q_right.z - q_left.z * q_right.y,
        q_left.w * q_right.y - q_left.x * q_right.z + q_left.y * q_right.w + q_left.z * q_right.x,
        q_left.w * q_right.z + q_left.x * q_right.y - q_left.y * q_right.x + q_left.z * q_right.w
    );
}
inline float dot(const Quaternion& qa, const Quaternion& qb) {
    return qa.w * qb.w + qa.x * qb.x + qa.y * qb.y + qa.z * qb.z;
}
inline Vec3 rotate_vector_by_quaternion(const Vec3& v, const Quaternion& q) {
    Vec3 q_vec = {q.x, q.y, q.z};
    Vec3 uv = cross(q_vec, v);
    Vec3 uuv = cross(q_vec, uv);
    Vec3 scaled_uv = uv * (2.0f * q.w);
    Vec3 scaled_uuv = uuv * 2.0f;
    return v + scaled_uv + scaled_uuv;
}

#endif //MATH_UTILS_H
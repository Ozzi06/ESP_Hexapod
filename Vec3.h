#ifndef VEC3_H
#define VEC3_H

#include <math.h> // For sqrtf
#include <stdio.h> // For basic printing/debugging if needed

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    // --- Constructors ---
    Vec3() = default; // Default constructor (0, 0, 0)
    Vec3(float x_val, float y_val, float z_val) : x(x_val), y(y_val), z(z_val) {}

    // --- Basic Arithmetic Operators ---

    // Addition
    inline Vec3 operator+(const Vec3& other) const {
        return Vec3(x + other.x, y + other.y, z + other.z);
    }

    // Subtraction
    inline Vec3 operator-(const Vec3& other) const {
        return Vec3(x - other.x, y - other.y, z - other.z);
    }

    // Scalar Multiplication (Vec3 * scalar)
    inline Vec3 operator*(float scalar) const {
        return Vec3(x * scalar, y * scalar, z * scalar);
    }

    // Scalar Division (Vec3 / scalar)
    inline Vec3 operator/(float scalar) const {
        // Basic check for division by zero
        if (fabsf(scalar) < 1e-9f) {
             // Handle error: return zero vector, print warning, or assert
             // Returning zero vector for now
             fprintf(stderr, "Warning: Vec3 division by zero/near-zero.\n");
             return Vec3(0.0f, 0.0f, 0.0f);
        }
        float inv_scalar = 1.0f / scalar;
        return Vec3(x * inv_scalar, y * inv_scalar, z * inv_scalar);
    }

    // Unary Negation
    inline Vec3 operator-() const {
        return Vec3(-x, -y, -z);
    }

    // --- Compound Assignment Operators ---

    inline Vec3& operator+=(const Vec3& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    inline Vec3& operator-=(const Vec3& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    inline Vec3& operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    inline Vec3& operator/=(float scalar) {
        // Basic check for division by zero
        if (fabsf(scalar) < 1e-9f) {
             // Handle error: print warning, leave vector unchanged, or assert
             fprintf(stderr, "Warning: Vec3 compound division by zero/near-zero.\n");
             return *this; // Return unchanged vector
        }
        float inv_scalar = 1.0f / scalar;
        x *= inv_scalar;
        y *= inv_scalar;
        z *= inv_scalar;
        return *this;
    }

    // --- Vector Operations ---

    // Calculate magnitude (length) squared
    inline float normSq() const {
        return x*x + y*y + z*z;
    }

    // Calculate magnitude (length)
    inline float norm() const {
        return sqrtf(normSq());
    }

    // Return a normalized copy of the vector (unit vector)
    inline Vec3 normalized() const {
        float n = norm();
        if (n < 1e-9f) {
            // Handle zero-length vector case
            return Vec3(0.0f, 0.0f, 0.0f); // Or return Vec3(1,0,0) or handle error
        }
        return *this / n;
    }

    // Normalize the vector in-place
    inline void normalize() {
        float n = norm();
        if (n < 1e-9f) {
            // Handle zero-length vector case, maybe set to (0,0,0) or default
            x = 0.0f; y = 0.0f; z = 0.0f;
            return;
        }
        *this /= n;
    }

    // --- Utility ---
    // Basic print function for debugging
    inline void print(const char* label = "Vec3") const {
        printf("%s: (%.4f, %.4f, %.4f)\n", label, x, y, z);
    }
};

// --- Standalone Functions ---

// Scalar Multiplication (scalar * Vec3)
inline Vec3 operator*(float scalar, const Vec3& vec) {
    return vec * scalar; // Reuse the Vec3 * scalar operator
}

// Dot Product
inline float dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

// Cross Product
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return Vec3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}


#endif // VEC3_H
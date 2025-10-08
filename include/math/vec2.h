#pragma once

#include <cmath>
#include <iostream>

#ifdef __ARM_NEON
#include <arm_neon.h>
#define USE_NEON 1
#endif

namespace parallax {

/**
 * @brief 2D vector class with SIMD optimizations
 * 
 * On Apple Silicon, this uses ARM NEON intrinsics for fast
 * parallel operations on vector components.
 */
class Vec2 {
public:
    float x, y;

    // Constructors
    Vec2() : x(0.0f), y(0.0f) {}
    Vec2(float x, float y) : x(x), y(y) {}
    Vec2(float val) : x(val), y(val) {}

    // Basic operations
    Vec2 operator+(const Vec2& other) const;
    Vec2 operator-(const Vec2& other) const;
    Vec2 operator*(float scalar) const;
    Vec2 operator/(float scalar) const;
    
    Vec2& operator+=(const Vec2& other);
    Vec2& operator-=(const Vec2& other);
    Vec2& operator*=(float scalar);
    Vec2& operator/=(float scalar);

    Vec2 operator-() const { return Vec2(-x, -y); }

    // Comparison
    bool operator==(const Vec2& other) const;
    bool operator!=(const Vec2& other) const;

    // Vector operations
    float dot(const Vec2& other) const;
    float cross(const Vec2& other) const; // Returns scalar (z-component)
    float length() const;
    float lengthSquared() const;
    float distance(const Vec2& other) const;
    float distanceSquared(const Vec2& other) const;
    
    Vec2 normalized() const;
    void normalize();
    
    Vec2 perpendicular() const { return Vec2(-y, x); }
    Vec2 rotated(float angle) const;
    
    // Linear interpolation
    Vec2 lerp(const Vec2& other, float t) const;
    
    // Projection
    Vec2 project(const Vec2& onto) const;
    Vec2 reflect(const Vec2& normal) const;
    
    // Clamping
    Vec2 clamp(float minVal, float maxVal) const;
    Vec2 clampLength(float maxLength) const;

    // Static helpers
    static Vec2 zero() { return Vec2(0.0f, 0.0f); }
    static Vec2 one() { return Vec2(1.0f, 1.0f); }
    static Vec2 up() { return Vec2(0.0f, 1.0f); }
    static Vec2 down() { return Vec2(0.0f, -1.0f); }
    static Vec2 left() { return Vec2(-1.0f, 0.0f); }
    static Vec2 right() { return Vec2(1.0f, 0.0f); }
    
    static Vec2 fromAngle(float angle) {
        return Vec2(std::cos(angle), std::sin(angle));
    }
    
    float angle() const {
        return std::atan2(y, x);
    }

    // For debugging
    friend std::ostream& operator<<(std::ostream& os, const Vec2& v) {
        os << "(" << v.x << ", " << v.y << ")";
        return os;
    }
};

// Scalar multiplication from left
inline Vec2 operator*(float scalar, const Vec2& vec) {
    return vec * scalar;
}

} // namespace parallax

#pragma once

#include "vec2.h"

namespace parallax {

/**
 * @brief 2x2 matrix class for rotations and transformations
 * 
 * Stored in column-major order:
 * | m00  m10 |
 * | m01  m11 |
 */
class Mat2 {
public:
    float m00, m01;  // First column
    float m10, m11;  // Second column

    // Constructors
    Mat2() : m00(1.0f), m01(0.0f), m10(0.0f), m11(1.0f) {}
    
    Mat2(float m00, float m01, float m10, float m11) 
        : m00(m00), m01(m01), m10(m10), m11(m11) {}

    // Matrix operations
    Mat2 operator+(const Mat2& other) const;
    Mat2 operator-(const Mat2& other) const;
    Mat2 operator*(const Mat2& other) const;
    Mat2 operator*(float scalar) const;
    
    Vec2 operator*(const Vec2& vec) const;

    Mat2& operator+=(const Mat2& other);
    Mat2& operator-=(const Mat2& other);
    Mat2& operator*=(const Mat2& other);
    Mat2& operator*=(float scalar);

    // Matrix properties
    float determinant() const;
    Mat2 transpose() const;
    Mat2 inverse() const;
    
    // Static factory methods
    static Mat2 identity() {
        return Mat2(1.0f, 0.0f, 0.0f, 1.0f);
    }
    
    static Mat2 rotation(float angle);
    static Mat2 scale(float sx, float sy);
    static Mat2 scale(float s) { return scale(s, s); }

    // For debugging
    friend std::ostream& operator<<(std::ostream& os, const Mat2& m) {
        os << "[" << m.m00 << " " << m.m10 << "]\n";
        os << "[" << m.m01 << " " << m.m11 << "]";
        return os;
    }
};

} // namespace parallax

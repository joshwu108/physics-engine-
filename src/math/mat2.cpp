#include "math/mat2.h"
#include <cmath>

namespace parallax {

Mat2 Mat2::operator+(const Mat2& other) const {
    return Mat2(
        m00 + other.m00, m01 + other.m01,
        m10 + other.m10, m11 + other.m11
    );
}

Mat2 Mat2::operator-(const Mat2& other) const {
    return Mat2(
        m00 - other.m00, m01 - other.m01,
        m10 - other.m10, m11 - other.m11
    );
}

Mat2 Mat2::operator*(const Mat2& other) const {
    return Mat2(
        m00 * other.m00 + m10 * other.m01,
        m01 * other.m00 + m11 * other.m01,
        m00 * other.m10 + m10 * other.m11,
        m01 * other.m10 + m11 * other.m11
    );
}

Mat2 Mat2::operator*(float scalar) const {
    return Mat2(
        m00 * scalar, m01 * scalar,
        m10 * scalar, m11 * scalar
    );
}

Vec2 Mat2::operator*(const Vec2& vec) const {
    return Vec2(
        m00 * vec.x + m10 * vec.y,
        m01 * vec.x + m11 * vec.y
    );
}

Mat2& Mat2::operator+=(const Mat2& other) {
    *this = *this + other;
    return *this;
}

Mat2& Mat2::operator-=(const Mat2& other) {
    *this = *this - other;
    return *this;
}

Mat2& Mat2::operator*=(const Mat2& other) {
    *this = *this * other;
    return *this;
}

Mat2& Mat2::operator*=(float scalar) {
    *this = *this * scalar;
    return *this;
}

float Mat2::determinant() const {
    return m00 * m11 - m10 * m01;
}

Mat2 Mat2::transpose() const {
    return Mat2(m00, m10, m01, m11);
}

Mat2 Mat2::inverse() const {
    float det = determinant();
    if (std::abs(det) < 1e-6f) {
        return Mat2::identity(); // Return identity for singular matrices
    }
    
    float invDet = 1.0f / det;
    return Mat2(
        m11 * invDet, -m01 * invDet,
        -m10 * invDet, m00 * invDet
    );
}

Mat2 Mat2::rotation(float angle) {
    float c = std::cos(angle);
    float s = std::sin(angle);
    return Mat2(c, s, -s, c);
}

Mat2 Mat2::scale(float sx, float sy) {
    return Mat2(sx, 0.0f, 0.0f, sy);
}

} // namespace parallax

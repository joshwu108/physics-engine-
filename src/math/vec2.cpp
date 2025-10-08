#include "math/vec2.h"
#include <cmath>
#include <algorithm>

namespace parallax {

Vec2 Vec2::operator+(const Vec2& other) const {
#ifdef USE_NEON
    float32x2_t a = vld1_f32(&x);
    float32x2_t b = vld1_f32(&other.x);
    float32x2_t result = vadd_f32(a, b);
    Vec2 out;
    vst1_f32(&out.x, result);
    return out;
#else
    return Vec2(x + other.x, y + other.y);
#endif
}

Vec2 Vec2::operator-(const Vec2& other) const {
#ifdef USE_NEON
    float32x2_t a = vld1_f32(&x);
    float32x2_t b = vld1_f32(&other.x);
    float32x2_t result = vsub_f32(a, b);
    Vec2 out;
    vst1_f32(&out.x, result);
    return out;
#else
    return Vec2(x - other.x, y - other.y);
#endif
}

Vec2 Vec2::operator*(float scalar) const {
#ifdef USE_NEON
    float32x2_t a = vld1_f32(&x);
    float32x2_t s = vdup_n_f32(scalar);
    float32x2_t result = vmul_f32(a, s);
    Vec2 out;
    vst1_f32(&out.x, result);
    return out;
#else
    return Vec2(x * scalar, y * scalar);
#endif
}

Vec2 Vec2::operator/(float scalar) const {
    float inv = 1.0f / scalar;
    return *this * inv;
}

Vec2& Vec2::operator+=(const Vec2& other) {
    *this = *this + other;
    return *this;
}

Vec2& Vec2::operator-=(const Vec2& other) {
    *this = *this - other;
    return *this;
}

Vec2& Vec2::operator*=(float scalar) {
    *this = *this * scalar;
    return *this;
}

Vec2& Vec2::operator/=(float scalar) {
    *this = *this / scalar;
    return *this;
}

bool Vec2::operator==(const Vec2& other) const {
    const float epsilon = 1e-6f;
    return std::abs(x - other.x) < epsilon && std::abs(y - other.y) < epsilon;
}

bool Vec2::operator!=(const Vec2& other) const {
    return !(*this == other);
}

float Vec2::dot(const Vec2& other) const {
#ifdef USE_NEON
    float32x2_t a = vld1_f32(&x);
    float32x2_t b = vld1_f32(&other.x);
    float32x2_t result = vmul_f32(a, b);
    return vget_lane_f32(result, 0) + vget_lane_f32(result, 1);
#else
    return x * other.x + y * other.y;
#endif
}

float Vec2::cross(const Vec2& other) const {
    // Returns the z-component of the 3D cross product (a.x, a.y, 0) x (b.x, b.y, 0)
    return x * other.y - y * other.x;
}

float Vec2::lengthSquared() const {
    return dot(*this);
}

float Vec2::length() const {
    return std::sqrt(lengthSquared());
}

float Vec2::distanceSquared(const Vec2& other) const {
    return (*this - other).lengthSquared();
}

float Vec2::distance(const Vec2& other) const {
    return std::sqrt(distanceSquared(other));
}

Vec2 Vec2::normalized() const {
    float len = length();
    if (len < 1e-6f) {
        return Vec2(0.0f, 0.0f);
    }
    return *this / len;
}

void Vec2::normalize() {
    *this = normalized();
}

Vec2 Vec2::rotated(float angle) const {
    float c = std::cos(angle);
    float s = std::sin(angle);
    return Vec2(x * c - y * s, x * s + y * c);
}

Vec2 Vec2::lerp(const Vec2& other, float t) const {
    return *this + (other - *this) * t;
}

Vec2 Vec2::project(const Vec2& onto) const {
    float d = onto.dot(onto);
    if (d < 1e-6f) {
        return Vec2::zero();
    }
    return onto * (dot(onto) / d);
}

Vec2 Vec2::reflect(const Vec2& normal) const {
    return *this - normal * (2.0f * dot(normal));
}

Vec2 Vec2::clamp(float minVal, float maxVal) const {
    return Vec2(
        std::clamp(x, minVal, maxVal),
        std::clamp(y, minVal, maxVal)
    );
}

Vec2 Vec2::clampLength(float maxLength) const {
    float len = length();
    if (len > maxLength && len > 1e-6f) {
        return *this * (maxLength / len);
    }
    return *this;
}

} // namespace parallax

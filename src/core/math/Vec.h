#pragma once

#include <cmath>

namespace atspilot {

struct Vec2 {
    double x = 0.0;
    double y = 0.0;

    constexpr Vec2() = default;
    constexpr Vec2(double x_, double y_) : x(x_), y(y_) {}

    constexpr Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    constexpr Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    constexpr Vec2 operator*(double s) const { return {x * s, y * s}; }
    constexpr Vec2 operator/(double s) const { return {x / s, y / s}; }
    constexpr Vec2 operator-() const { return {-x, -y}; }
    Vec2& operator+=(const Vec2& o) { x += o.x; y += o.y; return *this; }
    Vec2& operator-=(const Vec2& o) { x -= o.x; y -= o.y; return *this; }

    double length() const { return std::hypot(x, y); }
    constexpr double lengthSq() const { return x * x + y * y; }
    Vec2 normalized() const {
        const double l = length();
        return l > 1e-12 ? Vec2{x / l, y / l} : Vec2{};
    }
    // Rotates +90 degrees counterclockwise in the (x, y) plane.
    constexpr Vec2 perp() const { return {-y, x}; }
};

constexpr double dot(const Vec2& a, const Vec2& b) { return a.x * b.x + a.y * b.y; }
// z-component of the 3D cross product; positive when b is counterclockwise from a.
constexpr double cross(const Vec2& a, const Vec2& b) { return a.x * b.y - a.y * b.x; }
inline double distance(const Vec2& a, const Vec2& b) { return (a - b).length(); }

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    constexpr Vec3() = default;
    constexpr Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    constexpr Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    constexpr Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
    constexpr Vec3 operator-() const { return {-x, -y, -z}; }

    double length() const { return std::sqrt(x * x + y * y + z * z); }
    constexpr double lengthSq() const { return x * x + y * y + z * z; }
    Vec3 normalized() const {
        const double l = length();
        return l > 1e-12 ? Vec3{x / l, y / l, z / l} : Vec3{};
    }
};

constexpr double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
constexpr Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double distance(const Vec3& a, const Vec3& b) { return (a - b).length(); }

struct Quat {
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Vec3 rotate(const Vec3& v) const {
        // v' = v + 2w(q x v) + 2 q x (q x v)
        const Vec3 q{x, y, z};
        const Vec3 t = cross(q, v) * 2.0;
        return v + t * w + cross(q, t);
    }
};

}  // namespace atspilot

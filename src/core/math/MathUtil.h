#pragma once

#include <algorithm>
#include <cmath>
#include <optional>

#include "math/Vec.h"

namespace atspilot {

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kTwoPi = 2.0 * kPi;
inline constexpr double kMphToMps = 0.44704;
inline constexpr double kKphToMps = 1.0 / 3.6;

template <typename T>
constexpr T clamp(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }

constexpr double lerp(double a, double b, double t) { return a + (b - a) * t; }
inline Vec2 lerp(const Vec2& a, const Vec2& b, double t) { return a + (b - a) * t; }
inline Vec3 lerp(const Vec3& a, const Vec3& b, double t) { return a + (b - a) * t; }

constexpr double degToRad(double d) { return d * kPi / 180.0; }
constexpr double radToDeg(double r) { return r * 180.0 / kPi; }

// Wraps to (-pi, pi].
inline double normalizeAngle(double a) {
    a = std::fmod(a + kPi, kTwoPi);
    if (a <= 0.0) a += kTwoPi;
    return a - kPi;
}

// Signed smallest rotation from `from` to `to`, positive counterclockwise.
inline double headingDifference(double from, double to) { return normalizeAngle(to - from); }

inline double sign(double v) { return (v > 0.0) - (v < 0.0); }

// Moves `current` towards `target` by at most `maxDelta`.
inline double approach(double current, double target, double maxDelta) {
    if (target > current) return std::min(target, current + maxDelta);
    return std::max(target, current - maxDelta);
}

// Signed curvature (1/m) of the circle through three points; positive when
// the points turn counterclockwise. Returns 0 for (near) collinear points.
inline double curvatureFromPoints(const Vec2& a, const Vec2& b, const Vec2& c) {
    const double ab = distance(a, b);
    const double bc = distance(b, c);
    const double ca = distance(c, a);
    const double denom = ab * bc * ca;
    if (denom < 1e-9) return 0.0;
    // Circumradius R = abc / (4 * area), so curvature = 4 * area / abc,
    // and cross(b-a, c-a) = 2 * signed area.
    return 2.0 * cross(b - a, c - a) / denom;
}

// Closest point on segment [a, b] to p, returned as the interpolation factor in [0, 1].
inline double projectOntoSegment(const Vec2& p, const Vec2& a, const Vec2& b) {
    const Vec2 ab = b - a;
    const double len2 = ab.lengthSq();
    if (len2 < 1e-12) return 0.0;
    return clamp(dot(p - a, ab) / len2, 0.0, 1.0);
}

// Cubic Hermite interpolation.
inline Vec3 hermite(const Vec3& p0, const Vec3& m0, const Vec3& p1, const Vec3& m1, double t) {
    const double t2 = t * t;
    const double t3 = t2 * t;
    return p0 * (2 * t3 - 3 * t2 + 1) + m0 * (t3 - 2 * t2 + t) + p1 * (-2 * t3 + 3 * t2) + m1 * (t3 - t2);
}

inline Vec3 hermiteTangent(const Vec3& p0, const Vec3& m0, const Vec3& p1, const Vec3& m1, double t) {
    const double t2 = t * t;
    return p0 * (6 * t2 - 6 * t) + m0 * (3 * t2 - 4 * t + 1) + p1 * (-6 * t2 + 6 * t) + m1 * (3 * t2 - 2 * t);
}

// Intersection of the circle (center c, radius r) with segment [a, b]; returns the
// factor along the segment of the intersection furthest along the segment.
inline std::optional<double> circleSegmentIntersection(const Vec2& c, double r, const Vec2& a, const Vec2& b) {
    const Vec2 d = b - a;
    const Vec2 f = a - c;
    const double A = dot(d, d);
    if (A < 1e-12) return std::nullopt;
    const double B = 2.0 * dot(f, d);
    const double C = dot(f, f) - r * r;
    const double disc = B * B - 4.0 * A * C;
    if (disc < 0.0) return std::nullopt;
    const double sq = std::sqrt(disc);
    const double t2 = (-B + sq) / (2.0 * A);
    if (t2 >= 0.0 && t2 <= 1.0) return t2;
    const double t1 = (-B - sq) / (2.0 * A);
    if (t1 >= 0.0 && t1 <= 1.0) return t1;
    return std::nullopt;
}

}  // namespace atspilot

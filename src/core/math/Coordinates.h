#pragma once

// ATS world coordinate conventions (see docs/sdk.md, "Coordinate systems").
//
//   World axes:  +X = east, +Y = up, +Z = south. Units are metres.
//   SDK heading: unit range [0, 1), counterclockwise seen from above,
//                0 = north (-Z), 0.25 = west (-X), 0.5 = south, 0.75 = east.
//   Vehicle space: -Z is forward, +X is right, +Y is up.
//
// ATSPilot plans on a 2D ground plane ("plan" coordinates) that is
// right-handed when seen from above:
//
//   plan.x = world.x   (east)
//   plan.y = -world.z  (north)
//   yaw    = mathematical angle from +plan.x, counterclockwise, radians.
//
// Converting the SDK heading: north (h = 0) is yaw = pi/2, and both are
// counterclockwise, so yaw = h * 2pi + pi/2.
//
// Keeping every inversion in this file means controllers never need to know
// that ATS uses a left-handed-looking top-down view.

#include "math/MathUtil.h"
#include "math/Vec.h"

namespace atspilot::coords {

inline Vec2 worldToPlan(const Vec3& world) { return {world.x, -world.z}; }

inline Vec3 planToWorld(const Vec2& plan, double worldY) { return {plan.x, worldY, -plan.y}; }

inline double sdkHeadingToYaw(double headingUnit) { return normalizeAngle(headingUnit * kTwoPi + kPi / 2.0); }

inline double yawToSdkHeading(double yaw) {
    double h = (yaw - kPi / 2.0) / kTwoPi;
    h -= std::floor(h);
    return h;
}

inline Vec2 yawToDirection(double yaw) { return {std::cos(yaw), std::sin(yaw)}; }

inline double directionToYaw(const Vec2& dir) { return std::atan2(dir.y, dir.x); }

// Direction a world-space forward vector points to on the plan.
inline double worldDirectionToYaw(const Vec3& worldDir) { return std::atan2(-worldDir.z, worldDir.x); }

// SDK "vehicle space" forward speed: velocity.z is negative when moving forward.
inline double forwardSpeedFromLocalVelocity(const Vec3& localVelocity) { return -localVelocity.z; }

}  // namespace atspilot::coords

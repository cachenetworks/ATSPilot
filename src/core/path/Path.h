#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include "math/Vec.h"

namespace atspilot {

struct PathPoint {
    Vec2 pos;
    double s = 0.0;           // arc length from the first point, metres
    double speedLimit = 0.0;  // m/s, 0 = unknown
    std::uint64_t segmentId = 0;
    double height = std::numeric_limits<double>::quiet_NaN();  // road surface, world up axis; NaN = unknown
};

struct PathProjection {
    std::size_t index = 0;  // segment [index, index + 1]
    double t = 0.0;         // factor along the segment
    double s = 0.0;         // arc length of the projected point
    Vec2 point;
    double crossTrackError = 0.0;  // signed; positive when the query point is left of the path
    double yaw = 0.0;              // path direction at the projection
};

// Immutable-after-build 2D polyline with arc-length parameterisation.
class Path {
public:
    Path() = default;
    explicit Path(std::vector<PathPoint> points);

    void append(const Vec2& p, double speedLimit = 0.0, std::uint64_t segmentId = 0,
                double height = std::numeric_limits<double>::quiet_NaN());

    bool valid() const { return points_.size() >= 2 && length() > 0.1; }
    std::size_t size() const { return points_.size(); }
    const std::vector<PathPoint>& points() const { return points_; }
    const PathPoint& operator[](std::size_t i) const { return points_[i]; }
    double length() const { return points_.empty() ? 0.0 : points_.back().s; }

    // Projects p onto the path. With a hint, the search is limited to a window
    // around the hinted segment so parallel stretches of the same path are not confused.
    std::optional<PathProjection> project(const Vec2& p, std::optional<std::size_t> hint = std::nullopt,
                                          std::size_t window = 40) const;

    Vec2 positionAt(double s) const;
    double yawAt(double s) const;
    double speedLimitAt(double s) const;
    // Road surface height at s, NaN where the path carries none.
    double heightAt(double s) const;
    // Signed curvature (1/m) estimated from points spaced `span` metres apart around s.
    double curvatureAt(double s, double span = 10.0) const;

    // Resamples to roughly uniform spacing.
    Path resampled(double spacing) const;
    // Sub-path covering arc lengths [from, to], re-based so it starts at s = 0.
    Path trimmed(double from, double to) const;

    // Index of the segment containing arc length s.
    std::size_t segmentIndexAt(double s) const;

private:
    std::vector<PathPoint> points_;
};

}  // namespace atspilot

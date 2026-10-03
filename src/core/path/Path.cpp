#include "path/Path.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "math/MathUtil.h"

namespace atspilot {

Path::Path(std::vector<PathPoint> points) : points_(std::move(points)) {
    double s = 0.0;
    for (std::size_t i = 0; i < points_.size(); ++i) {
        if (i > 0) s += distance(points_[i - 1].pos, points_[i].pos);
        points_[i].s = s;
    }
}

void Path::append(const Vec2& p, double speedLimit, std::uint64_t segmentId) {
    if (!points_.empty() && distance(points_.back().pos, p) < 1e-3) return;
    PathPoint pp;
    pp.pos = p;
    pp.speedLimit = speedLimit;
    pp.segmentId = segmentId;
    pp.s = points_.empty() ? 0.0 : points_.back().s + distance(points_.back().pos, p);
    points_.push_back(pp);
}

std::optional<PathProjection> Path::project(const Vec2& p, std::optional<std::size_t> hint, std::size_t window) const {
    if (points_.size() < 2) return std::nullopt;
    std::size_t begin = 0;
    std::size_t end = points_.size() - 1;
    if (hint && *hint < end) {
        begin = *hint > window ? *hint - window : 0;
        end = std::min(end, *hint + window + 1);
    }

    double best = std::numeric_limits<double>::max();
    PathProjection result;
    for (std::size_t i = begin; i < end; ++i) {
        const Vec2& a = points_[i].pos;
        const Vec2& b = points_[i + 1].pos;
        const double t = projectOntoSegment(p, a, b);
        const Vec2 q = lerp(a, b, t);
        const double d2 = (p - q).lengthSq();
        if (d2 < best) {
            best = d2;
            result.index = i;
            result.t = t;
            result.point = q;
        }
    }

    const Vec2& a = points_[result.index].pos;
    const Vec2& b = points_[result.index + 1].pos;
    const Vec2 dir = (b - a).normalized();
    result.s = lerp(points_[result.index].s, points_[result.index + 1].s, result.t);
    result.yaw = std::atan2(dir.y, dir.x);
    result.crossTrackError = cross(dir, p - result.point);
    return result;
}

std::size_t Path::segmentIndexAt(double s) const {
    if (points_.size() < 2) return 0;
    if (s <= 0.0) return 0;
    if (s >= length()) return points_.size() - 2;
    auto it = std::upper_bound(points_.begin(), points_.end(), s,
                               [](double v, const PathPoint& pp) { return v < pp.s; });
    const auto idx = static_cast<std::size_t>(std::distance(points_.begin(), it));
    return std::min(idx == 0 ? 0 : idx - 1, points_.size() - 2);
}

Vec2 Path::positionAt(double s) const {
    if (points_.empty()) return {};
    if (points_.size() == 1) return points_[0].pos;
    const std::size_t i = segmentIndexAt(s);
    const PathPoint& a = points_[i];
    const PathPoint& b = points_[i + 1];
    const double segLen = b.s - a.s;
    const double t = segLen > 1e-9 ? (s - a.s) / segLen : 0.0;
    // Extrapolates linearly beyond either end, which keeps lookahead targets
    // well-defined for a short time near the end of the path.
    return lerp(a.pos, b.pos, t);
}

double Path::yawAt(double s) const {
    if (points_.size() < 2) return 0.0;
    const std::size_t i = segmentIndexAt(s);
    const Vec2 d = points_[i + 1].pos - points_[i].pos;
    return std::atan2(d.y, d.x);
}

double Path::speedLimitAt(double s) const {
    if (points_.empty()) return 0.0;
    return points_[segmentIndexAt(s)].speedLimit;
}

double Path::curvatureAt(double s, double span) const {
    if (!valid()) return 0.0;
    const double s0 = std::max(0.0, s - span);
    const double s2 = std::min(length(), s + span);
    if (s2 - s0 < span) return 0.0;
    const double s1 = 0.5 * (s0 + s2);
    return curvatureFromPoints(positionAt(s0), positionAt(s1), positionAt(s2));
}

Path Path::trimmed(double from, double to) const {
    Path out;
    if (points_.size() < 2) return *this;
    from = std::max(0.0, from);
    to = std::min(length(), to);
    if (to <= from) return out;
    out.append(positionAt(from), points_[segmentIndexAt(from)].speedLimit, points_[segmentIndexAt(from)].segmentId);
    for (const auto& p : points_) {
        if (p.s > from && p.s < to) out.append(p.pos, p.speedLimit, p.segmentId);
    }
    out.append(positionAt(to), points_[segmentIndexAt(to)].speedLimit, points_[segmentIndexAt(to)].segmentId);
    return out;
}

Path Path::resampled(double spacing) const {
    Path out;
    if (!valid() || spacing <= 0.0) return *this;
    const double len = length();
    for (double s = 0.0; s < len; s += spacing) {
        const std::size_t i = segmentIndexAt(s);
        out.append(positionAt(s), points_[i].speedLimit, points_[i].segmentId);
    }
    out.append(points_.back().pos, points_.back().speedLimit, points_.back().segmentId);
    return out;
}

}  // namespace atspilot

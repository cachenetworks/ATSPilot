#include "control/Lateral.h"

#include <cmath>

#include "math/MathUtil.h"

namespace atspilot {

LateralAlgorithm lateralAlgorithmFromString(const std::string& s, bool* ok) {
    if (ok) *ok = true;
    if (s == "pure_pursuit") return LateralAlgorithm::PurePursuit;
    if (s == "stanley") return LateralAlgorithm::Stanley;
    if (ok) *ok = false;
    return LateralAlgorithm::PurePursuit;
}

const char* toString(LateralAlgorithm a) {
    return a == LateralAlgorithm::Stanley ? "stanley" : "pure_pursuit";
}

double lookaheadDistance(const LateralParams& p, double speed) {
    return clamp(p.lookaheadBase + std::max(0.0, speed) * p.lookaheadSpeedFactor, p.lookaheadMin, p.lookaheadMax);
}

double purePursuitWheelAngle(const Vec2& rearAxle, double yaw, const Vec2& target, double wheelbase) {
    const Vec2 toTarget = target - rearAxle;
    const double ld = toTarget.length();
    if (ld < 1e-3) return 0.0;
    const double alpha = headingDifference(yaw, std::atan2(toTarget.y, toTarget.x));
    // Arc through the rear axle tangent to the heading and through the target:
    // curvature = 2 sin(alpha) / ld; bicycle model: tan(delta) = L * curvature.
    return std::atan(2.0 * wheelbase * std::sin(alpha) / ld);
}

double stanleyWheelAngle(double headingError, double crossTrackError, double speed, double gain, double softening) {
    // Positive cross-track error means the axle is left of the path, which needs a
    // rightward (negative) correction.
    return headingError + std::atan2(-gain * crossTrackError, std::max(0.0, speed) + softening);
}

LateralOutput computeLateral(const LateralParams& params, const Path& path, const LateralInput& in,
                             std::optional<std::size_t> hint) {
    LateralOutput out;
    if (!path.valid()) return out;

    const auto rearProj = path.project(in.rearAxle, hint);
    if (!rearProj) return out;

    out.projectionIndex = rearProj->index;
    out.pathS = rearProj->s;
    out.lookahead = lookaheadDistance(params, in.speed);
    out.headingError = headingDifference(in.yaw, rearProj->yaw);
    out.crossTrackError = rearProj->crossTrackError;

    if (params.algorithm == LateralAlgorithm::PurePursuit) {
        out.target = path.positionAt(rearProj->s + out.lookahead);
        out.wheelAngle = purePursuitWheelAngle(in.rearAxle, in.yaw, out.target, in.wheelbase);
    } else {
        const auto frontProj = path.project(in.frontAxle, rearProj->index);
        if (!frontProj) return out;
        out.target = frontProj->point;
        const double headingErr = headingDifference(in.yaw, frontProj->yaw);
        out.wheelAngle = stanleyWheelAngle(headingErr, frontProj->crossTrackError, in.speed, params.stanleyGain,
                                           params.stanleySoftening);
    }
    out.valid = std::isfinite(out.wheelAngle);
    return out;
}

}  // namespace atspilot

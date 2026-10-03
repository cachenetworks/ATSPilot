#include "control/SteeringShaper.h"

#include <cmath>

#include "math/MathUtil.h"

namespace atspilot {

double SteeringShaper::authorityLimit(double maxWheelAngle, double speed, double wheelbase) const {
    if (maxWheelAngle <= 1e-6) return 0.0;
    const double v = std::max(speed, 1.0);
    // a_lat = v^2 * curvature and tan(delta) = L * curvature.
    const double maxCurvature = p_.maxLateralAccel * p_.authorityMargin / (v * v);
    const double maxDelta = std::atan(wheelbase * maxCurvature);
    return clamp(maxDelta / maxWheelAngle, 0.05, 1.0);
}

double SteeringShaper::update(double desiredWheelAngle, double maxWheelAngle, double speed, double wheelbase,
                              int trailerCount, double dt) {
    if (dt <= 0.0) return output_;
    const double limit = authorityLimit(maxWheelAngle, speed, wheelbase);
    const double raw = maxWheelAngle > 1e-6 ? desiredWheelAngle / maxWheelAngle : 0.0;
    const double target = clamp(raw, -limit, limit);

    const double alpha = p_.smoothingTime > 0.0 ? dt / (p_.smoothingTime + dt) : 1.0;
    filtered_ += (target - filtered_) * alpha;

    const double speedFactor = lerp(1.0, p_.highSpeedRateScale, clamp(speed / p_.highSpeed, 0.0, 1.0));
    const double trailerFactor = std::pow(p_.trailerConservatism, std::max(0, trailerCount));
    const double maxStep = p_.maxRate * speedFactor * trailerFactor * dt;
    output_ = clamp(approach(output_, filtered_, maxStep), -1.0, 1.0);
    return output_;
}

}  // namespace atspilot

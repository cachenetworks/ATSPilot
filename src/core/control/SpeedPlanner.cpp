#include "control/SpeedPlanner.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "math/MathUtil.h"

namespace atspilot {

namespace {

// Map joins and sparse polyline vertices can create a one-sample curvature
// spike. Requiring the bend to persist into one neighbouring sample keeps the
// planner conservative for real curves while avoiding a hard brake for a
// single bad vertex.
double sustainedCurvature(const Path& path, double s, double span, double spacing) {
    const double probe = std::max(1.0, spacing);
    const double k = std::abs(path.curvatureAt(s, span));
    const bool havePrev = s - probe >= 0.0;
    const bool haveNext = s + probe <= path.length();
    if (!havePrev && !haveNext) return k;
    if (!havePrev) return std::min(k, std::abs(path.curvatureAt(s + probe, span)));
    if (!haveNext) return std::min(k, std::abs(path.curvatureAt(s - probe, span)));

    const double a = std::abs(path.curvatureAt(s - probe, span));
    const double b = k;
    const double c = std::abs(path.curvatureAt(s + probe, span));
    return a + b + c - std::min({a, b, c}) - std::max({a, b, c});
}

}  // namespace

double effectiveLateralAccel(const SpeedPlannerParams& p, const VehicleLoadFactors& f) {
    double a = p.maxLateralAccel * clamp(f.aggressiveness, 0.5, 1.5);
    // Each articulation point adds rollover and off-tracking risk.
    a *= std::pow(0.85, clamp(f.trailerCount, 0, 3));
    if (f.cargoMassKg > 15000.0) {
        // Raised centre of gravity: scale down to 75% at 35 t and beyond.
        a *= lerp(1.0, 0.75, clamp((f.cargoMassKg - 15000.0) / 20000.0, 0.0, 1.0));
    }
    if (f.wet) a *= 0.85;
    return a;
}

double curveSpeed(double curvature, double maxLateralAccel, double minSpeed) {
    const double k = std::abs(curvature);
    if (k < 1e-6) return std::numeric_limits<double>::infinity();
    return std::max(minSpeed, std::sqrt(maxLateralAccel / k));
}

SpeedPlan planSpeed(const SpeedPlannerParams& p, const VehicleLoadFactors& f, const Path& path, double currentS,
                    double cruiseSpeed, const std::vector<SpeedConstraint>& constraints) {
    SpeedPlan plan;
    plan.targetSpeed = cruiseSpeed;
    if (!path.valid()) {
        plan.targetSpeed = 0.0;
        return plan;
    }

    const double aLat = effectiveLateralAccel(p, f);
    const double end = std::min(path.length(), currentS + p.horizon);
    double minK = 0.0;

    const double spacing = std::max(1.0, p.sampleSpacing);
    for (double s = currentS; s <= end; s += spacing) {
        const double k = sustainedCurvature(path, s, p.curvatureSpan, spacing);
        minK = std::max(minK, k);
        double vLimit = curveSpeed(k, aLat, p.minCurveSpeed);
        const double posted = path.speedLimitAt(s);
        if (posted > 0.0) vLimit = std::min(vLimit, posted);
        const double d = std::max(0.0, s - currentS);
        // Highest speed now from which we can still slow to vLimit within d.
        const double allowed = std::sqrt(vLimit * vLimit + 2.0 * p.comfortDecel * d);
        if (allowed < plan.targetSpeed) {
            plan.targetSpeed = allowed;
            plan.limitingDistance = d;
            plan.limitingCurveRadius = k > 1e-6 ? 1.0 / k : 0.0;
        }
    }

    if (p.stopAtPathEnd && path.length() - currentS < p.horizon) {
        // Running out of path is treated as a stop line so the truck never
        // drives beyond known geometry at speed.
        const double d = std::max(0.0, path.length() - currentS);
        const double allowed = std::sqrt(2.0 * p.comfortDecel * d);
        if (allowed < plan.targetSpeed) {
            plan.targetSpeed = allowed;
            plan.limitingDistance = d;
            plan.limitingCurveRadius = 0.0;
        }
    }

    for (const auto& c : constraints) {
        const double d = std::max(0.0, c.s - currentS);
        const double allowed = std::sqrt(c.speed * c.speed + 2.0 * p.comfortDecel * d);
        if (allowed < plan.targetSpeed) {
            plan.targetSpeed = allowed;
            plan.limitingDistance = d;
            plan.limitingCurveRadius = 0.0;
        }
    }

    plan.minRadiusAhead = minK > 1e-6 ? 1.0 / minK : 0.0;
    plan.targetSpeed = std::max(0.0, plan.targetSpeed);
    return plan;
}

}  // namespace atspilot

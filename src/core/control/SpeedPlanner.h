#pragma once

#include <vector>

#include "path/Path.h"

namespace atspilot {

struct SpeedPlannerParams {
    double maxLateralAccel = 1.6;      // m/s^2 for a bobtail on dry road; deliberately conservative
    double comfortDecel = 1.2;         // m/s^2 used to slow ahead of curves
    double limitDecel = 0.7;           // m/s^2 when the speed limit or maximum drops
    double minCurveSpeed = 4.0;        // m/s floor so very tight geometry still makes progress
    double horizon = 400.0;            // m of path examined
    double sampleSpacing = 5.0;        // m
    double curvatureSpan = 18.0;       // m; smooths curvature noise from polyline vertices and lane joins
    bool stopAtPathEnd = true;
};

struct VehicleLoadFactors {
    int trailerCount = 0;
    double cargoMassKg = 0.0;
    bool wet = false;          // wipers running is used as a proxy for rain
    double aggressiveness = 1.0;
};

// Lateral acceleration bound after accounting for trailers, load and weather.
double effectiveLateralAccel(const SpeedPlannerParams& p, const VehicleLoadFactors& f);

// v = sqrt(a_lat / |kappa|).
double curveSpeed(double curvature, double maxLateralAccel, double minSpeed);

struct SpeedPlan {
    double targetSpeed = 0.0;        // speed to command now, m/s
    double limitingDistance = 0.0;   // where the binding constraint lies, m ahead
    double limitingCurveRadius = 0.0;  // radius of the binding curve, 0 if none
    double minRadiusAhead = 0.0;     // tightest radius within the horizon
};

// A speed that must not be exceeded from arc length `s` on (0 = stop there).
struct SpeedConstraint {
    double s = 0.0;
    double speed = 0.0;
};

// Computes the highest speed that still allows decelerating at `comfortDecel`
// to every curve speed, every extra constraint and the path end within the horizon.
SpeedPlan planSpeed(const SpeedPlannerParams& p, const VehicleLoadFactors& f, const Path& path, double currentS,
                    double cruiseSpeed, const std::vector<SpeedConstraint>& constraints = {});

}  // namespace atspilot

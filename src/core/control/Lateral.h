#pragma once

#include <string>

#include "math/Vec.h"
#include "path/Path.h"

namespace atspilot {

enum class LateralAlgorithm { PurePursuit, Stanley };

LateralAlgorithm lateralAlgorithmFromString(const std::string& s, bool* ok = nullptr);
const char* toString(LateralAlgorithm a);

struct LateralParams {
    LateralAlgorithm algorithm = LateralAlgorithm::PurePursuit;
    double lookaheadBase = 15.0;         // m
    double lookaheadSpeedFactor = 0.5;   // s (m per m/s)
    double lookaheadMin = 4.0;           // m
    // In tight curves the lookahead is limited to this fraction of the smallest
    // radius ahead, which stops Pure Pursuit cutting corners at junctions. 0 = off.
    double curveLookaheadFactor = 0.35;
    double lookaheadMax = 60.0;          // m
    double stanleyGain = 1.0;
    double stanleySoftening = 2.0;       // m/s; keeps the gain finite at low speed
};

struct LateralInput {
    Vec2 rearAxle;    // plan coordinates
    Vec2 frontAxle;   // plan coordinates
    double yaw = 0.0;
    double speed = 0.0;   // m/s, forward positive
    double wheelbase = 6.0;
};

struct LateralOutput {
    double wheelAngle = 0.0;  // desired road-wheel angle, radians, positive = left
    double lookahead = 0.0;
    Vec2 target;
    double crossTrackError = 0.0;
    double headingError = 0.0;
    double pathS = 0.0;
    std::size_t projectionIndex = 0;
    bool valid = false;
};

double lookaheadDistance(const LateralParams& p, double speed);

// Pure geometric pursuit of the point `lookahead` metres along the path from the
// rear-axle projection. Returns the bicycle-model wheel angle.
double purePursuitWheelAngle(const Vec2& rearAxle, double yaw, const Vec2& target, double wheelbase);

// Stanley: heading error plus a cross-track term evaluated at the front axle.
double stanleyWheelAngle(double headingError, double crossTrackError, double speed, double gain, double softening);

LateralOutput computeLateral(const LateralParams& params, const Path& path, const LateralInput& in,
                             std::optional<std::size_t> hint);

}  // namespace atspilot

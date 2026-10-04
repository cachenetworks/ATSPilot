#pragma once

#include <optional>
#include <vector>

#include "control/SpeedPlanner.h"
#include "path/Path.h"
#include "pilot/PilotTypes.h"
#include "world/World.h"

namespace atspilot {

struct TrafficParams {
    double corridorHalfWidth = 1.5;  // half the truck's width plus a margin, either side of the path
    double horizon = 200.0;          // m of path ahead that is checked
    double standstillGap = 5.0;      // m left to a stopped vehicle
    double timeGap = 2.0;            // s headway to a moving vehicle
    double predictionTime = 4.0;     // s crossing vehicles are extrapolated
    double conflictMargin = 2.5;     // s: a crossing vehicle we would meet within this is a conflict
    double startAccel = 1.0;         // m/s², assumed when estimating our arrival time from low speed
    double lightMatchRadius = 60.0;  // m between a stop line and its traffic light
    double amberMaxDecel = 3.0;      // m/s²: an amber light needing more braking than this is passed
};

// Something on, or about to cross, the truck's path.
struct PathObstacle {
    int id = 0;
    double s = 0.0;           // path arc length of its nearest point
    double speed = 0.0;       // m/s along the path (0 for crossing traffic)
    bool crossing = false;    // predicted to cross rather than occupying the path now
};

struct TrafficPicture {
    std::vector<SpeedConstraint> constraints;
    std::optional<PathObstacle> nearest;  // the closest obstacle ahead
    double requiredDecel = 0.0;           // m/s² needed to stop behind the nearest obstacle
};

// Turns the vehicles around the truck into speed constraints along `path`.
// `frontS` is the arc length of the truck's front bumper, `speed` its speed.
TrafficPicture assessTraffic(const Path& path, double frontS, double speed, const std::vector<WorldVehicle>& vehicles,
                             const TrafficParams& p);

struct LightMatch {
    const WorldLight* light = nullptr;
    bool byId = false;  // matched by semaphore id rather than by position
};

// The live traffic light controlling a signal stop. Preferably the light with the
// stop's semaphore id nearest the stop line; otherwise, by position: a light just
// before or beyond the line, near the lane, facing along the road. `facing` is
// +1 when lights are known to face the way their traffic drives, -1 when they
// face against it, 0 when not yet known; then position alone is only trusted
// when all candidate lights agree.
LightMatch lightForStop(const PathStop& stop, const Path& path, const std::vector<WorldLight>& lights,
                        const TrafficParams& p, int facing = 0);

enum class SignalDecision { Unknown, Stop, Go, GiveWay };

// What to do at a stop line `distance` metres ahead, at `speed`, given its light.
SignalDecision decideSignal(LightState state, double distance, double speed, const TrafficParams& p);

}  // namespace atspilot

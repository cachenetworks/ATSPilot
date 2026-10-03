#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "control/Longitudinal.h"
#include "path/Path.h"

namespace atspilot {

enum class PilotMode {
    Off,          // nothing commanded
    Cruise,       // speed only
    LaneAssist,   // steering only
    Autopilot,    // steering and speed
    EmergencyStop
};

const char* toString(PilotMode m);

enum class PilotRequest {
    ToggleAutopilot,
    ToggleLaneAssist,
    ToggleCruise,
    SpeedUp,
    SpeedDown,
    Resume,
    Cancel,
    EmergencyDisable,
};

const char* toString(PilotRequest r);

// What the game should receive this frame. `active == false` means neutral output.
struct ControlCommand {
    bool active = false;
    bool steerActive = false;
    bool pedalsActive = false;
    double steering = 0.0;  // [-1, 1], left positive (SDK convention)
    double throttle = 0.0;
    double brake = 0.0;
    double time = 0.0;      // simulation time the command was computed for
};

// A driving path published by the planning side, immutable once shared.
struct PathSnapshot {
    Path path;
    double time = 0.0;              // simulation time of the vehicle state used to build it
    double truckS = 0.0;            // truck's arc length on `path` when it was built
    std::string roadName;
    std::string nextManeuver;
    double nextManeuverDistance = 0.0;
    std::uint64_t generation = 0;   // increments when the route/lane choice changes discontinuously
};

using PathSnapshotPtr = std::shared_ptr<const PathSnapshot>;

struct ControllerDebug {
    double crossTrackError = 0.0;
    double headingError = 0.0;
    double lookAheadDistance = 0.0;
    double targetWheelAngle = 0.0;
    double targetSteering = 0.0;
    double currentSpeed = 0.0;
    double targetSpeed = 0.0;
    double throttleOutput = 0.0;
    double brakeOutput = 0.0;
    double curveRadius = 0.0;
    double maxWheelAngle = 0.0;
    double pathS = 0.0;
    BrakeLevel brakeLevel = BrakeLevel::None;
};

// Presentation model for any HUD, overlay or status file. Never exposes controllers.
struct PilotStatus {
    PilotMode mode = PilotMode::Off;
    bool available = false;
    bool telemetryConnected = false;
    bool mapLoaded = false;
    double speed = 0.0;        // m/s
    double setSpeed = 0.0;     // m/s
    double targetSpeed = 0.0;  // m/s
    double crossTrackError = 0.0;
    double routeDistance = 0.0;
    std::string road;
    std::string nextManeuver;
    double nextManeuverDistance = 0.0;
    std::string statusMessage;
};

}  // namespace atspilot

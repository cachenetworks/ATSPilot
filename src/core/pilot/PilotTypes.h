#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "control/GameCruise.h"
#include "control/Longitudinal.h"
#include "path/Path.h"

namespace atspilot {

enum class PilotMode {
    Off,           // nothing commanded
    Autopilot,     // ATSPilot steers; speed via the game's cruise control or its own pedals
    EmergencyStop  // controlled stop after losing a valid state
};

const char* toString(PilotMode m);

enum class PilotRequest {
    Toggle,           // the single ATSPilot key
    Cancel,           // programmatic off (harness, world transitions)
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
    GameButtons buttons;    // one-frame presses of the game's own controls
    double time = 0.0;      // simulation time the command was computed for
};

enum class StopKind : std::uint8_t { Signal, StopSign, Yield, RailCrossing };

const char* toString(StopKind k);

// A junction lane on the path that is controlled by a traffic light, stop sign,
// give-way rule or railway crossing; `s` is where that lane begins (the stop line).
struct PathStop {
    double s = 0.0;
    StopKind kind = StopKind::Signal;
    std::uint32_t segment = 0;
    int semaphoreId = -1;  // signals: the prefab's semaphore id, matched against live light states
};

// Where along the path the turn signal should be on, and on which side. Derived
// from the path's geometry: lane changes, turns at junctions, exits and forks,
// and merges (on-ramps and lane drops).
enum class IndicationKind : std::uint8_t { LaneChange, Turn, Exit, Merge };

const char* toString(IndicationKind k);

struct Indication {
    double sStart = 0.0;  // path arc length where the signal goes on (front of the truck)
    double sEnd = 0.0;    // and off again
    int side = 0;         // +1 left, -1 right
    IndicationKind kind = IndicationKind::LaneChange;
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
    bool navigationActive = false;  // path follows a planned route to the job destination
    double routeRemaining = 0.0;    // m
    bool gpsMatched = false;        // route agrees with the in-game navigation distance
    std::vector<PathStop> stops;    // controlled junction lanes ahead, in path order
    std::vector<Indication> indications;  // turn signal intervals, in path order
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

// Presentation model for the HUD, status file and logs. Never exposes controllers.
struct PilotStatus {
    PilotMode mode = PilotMode::Off;
    bool available = false;
    bool telemetryConnected = false;
    bool mapLoaded = false;
    double speed = 0.0;          // m/s
    double setSpeed = 0.0;       // m/s, maximum chosen by the player
    double targetSpeed = 0.0;    // m/s, what ATSPilot is aiming for now
    double crossTrackError = 0.0;
    double routeDistance = 0.0;
    bool navigationActive = false;
    bool gpsMatched = false;
    bool gameCruiseActive = false;
    double cruiseSetSpeed = 0.0; // m/s, the game's cruise control set speed (0 = off)
    bool waitingAtIntersection = false;
    bool trafficAware = false;   // live traffic and light states in use
    bool directSteering = false; // steering written to the truck directly
    double leadDistance = -1.0;  // m to the nearest vehicle on the path, < 0 = none
    double leadSpeed = 0.0;      // m/s
    std::string signalState;     // next traffic light, when known
    std::string profile;
    std::string road;
    std::string nextManeuver;
    double nextManeuverDistance = 0.0;
    std::string statusMessage;
};

}  // namespace atspilot

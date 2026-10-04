#pragma once

#include <cstdint>
#include <deque>
#include <functional>
#include <optional>
#include <string>

#include "config/Config.h"
#include "control/GameCruise.h"
#include "control/Lateral.h"
#include "control/Longitudinal.h"
#include "control/SpeedPlanner.h"
#include "control/SteeringShaper.h"
#include "pilot/PilotTypes.h"
#include "pilot/Safety.h"
#include "pilot/VehicleState.h"
#include "util/Logger.h"
#include "world/World.h"

namespace atspilot {

enum class PilotEvent { Engaged, Disengaged, DriverOverride, Unavailable, EmergencyBraking, SetSpeedChanged,
                        WaitingAtIntersection, Arrived };

// Orchestrates planning outputs, controllers and safety for one truck. Free of
// any SDK dependency so the same code runs inside ATS and in the simulator.
//
// Per frame:  VehicleState ─▶ safety checks ─▶ steering + speed plan ─▶ ControlCommand
//
// One key switches it on and off. Speed is held by the game's own cruise
// control where possible (see GameCruiseManager); ATSPilot's pedals take over
// below cruise speed, for braking, and at stop lines.
class Autopilot {
public:
    explicit Autopilot(const Config& cfg, Logger* log = nullptr);

    void setConfig(const Config& cfg);
    void setEventCallback(std::function<void(PilotEvent, const std::string&)> cb) { onEvent_ = std::move(cb); }

    // `wallNow` and `pathWall` are monotonic wall-clock seconds used for staleness checks.
    void request(PilotRequest r, const VehicleState& s, const VehicleConfig& vc, const PathSnapshotPtr& path,
                 double wallNow, double pathWall);

    ControlCommand update(const VehicleState& s, const VehicleConfig& vc, const PathSnapshotPtr& path,
                          double wallNow, double telemetryWall, double pathWall);

    // Immediate neutral output, e.g. on pause, world unload or an internal error.
    void disengage(const std::string& reason, PilotEvent evt = PilotEvent::Disengaged);

    // Traffic and traffic-light states around the truck, when the game's memory
    // can be read; null or invalid means ATSPilot drives on map data alone.
    void setWorld(WorldSnapshotPtr world) { world_ = std::move(world); }
    // ATSPilot's steering is written to the truck directly instead of being added
    // to the driver's input through the input mix.
    void setDirectSteering(bool direct) { directSteering_ = direct; }
    bool directSteering() const { return directSteering_; }

    PilotMode mode() const { return mode_; }
    const PilotStatus& status() const { return status_; }
    const ControllerDebug& debug() const { return debug_; }
    double maxWheelAngle() const { return maxWheelAngle_; }
    double setSpeed() const { return setSpeed_; }
    bool waitingAtIntersection() const { return waitingStop_.has_value(); }
    const OverrideDetector& overrideDetector() const { return override_; }
    const Config& effectiveConfig() const { return eff_; }

    // Validates the preconditions for engaging without changing state.
    std::optional<std::string> checkAvailability(const VehicleState& s, const PathSnapshotPtr& path, double wallNow,
                                                 double pathWall) const;

private:
    void engage(const VehicleState& s);
    void enterEmergency(const std::string& reason);
    void learnSteeringRatio(const VehicleState& s);
    void updateProfile(const VehicleConfig& vc);
    double cruiseTarget(const VehicleState& s) const;
    void setMessage(const std::string& msg);
    void emit(PilotEvent e, const std::string& msg);
    GameButtons blinkers(const VehicleState& s, const PathSnapshotPtr& path, double time);
    void logSignal(const PathStop& stop, LightState state, SignalDecision decision);

    Config cfg_;   // as configured
    Config eff_;   // with the active driving profile applied
    DrivingProfile activeProfile_ = DrivingProfile::Normal;
    Logger* log_ = nullptr;
    std::function<void(PilotEvent, const std::string&)> onEvent_;

    PilotMode mode_ = PilotMode::Off;
    double setSpeed_ = 0.0;  // m/s, the maximum: config default, then the player's cruise-control speed
    double lastTime_ = -1.0;
    double maxWheelAngle_ = 0.6;
    int invertedSteeringFrames_ = 0;
    double lastCrossTrackWarn_ = -100.0;

    SteeringShaper shaper_;
    LongitudinalController longitudinal_;
    GameCruiseManager cruise_;
    OverrideDetector override_;
    Watchdog watchdog_;

    // Intersections: the stop being waited at, stops the driver released, and the
    // throttle tap that releases one.
    std::optional<PathStop> waitingStop_;
    std::deque<std::uint32_t> clearedStops_;
    double tapStart_ = -1.0;
    double stopSignSince_ = -1.0;  // stopped at a stop sign since (s), with traffic in view

    WorldSnapshotPtr world_;
    bool directSteering_ = false;
    std::uint32_t loggedSignalSegment_ = 0xFFFFFFFFu;
    LightState loggedSignalState_ = LightState::Unknown;
    std::uint32_t unmatchedSignalLogged_ = 0xFFFFFFFFu;

    // Turn signals ATSPilot switched on (it never cancels the driver's own).
    bool ourLeftBlinker_ = false;
    bool ourRightBlinker_ = false;
    double lastBlinkerPress_ = -100.0;

    ControlCommand last_;
    PathSnapshotPtr hintPath_;
    std::size_t hintIndex_ = 0;

    PilotStatus status_;
    ControllerDebug debug_;
};

}  // namespace atspilot

#pragma once

#include <functional>
#include <optional>
#include <string>

#include "config/Config.h"
#include "control/Lateral.h"
#include "control/Longitudinal.h"
#include "control/SpeedPlanner.h"
#include "control/SteeringShaper.h"
#include "pilot/PilotTypes.h"
#include "pilot/Safety.h"
#include "pilot/VehicleState.h"
#include "util/Logger.h"

namespace atspilot {

enum class PilotEvent { Engaged, Disengaged, DriverOverride, Unavailable, EmergencyBraking, SetSpeedChanged };

// Orchestrates planning outputs, controllers and safety for one truck. Free of
// any SDK dependency so the same code runs inside ATS and in the simulator.
//
// Per frame:  VehicleState ─▶ safety checks ─▶ lateral + speed control ─▶ ControlCommand
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

    PilotMode mode() const { return mode_; }
    const PilotStatus& status() const { return status_; }
    const ControllerDebug& debug() const { return debug_; }
    double maxWheelAngle() const { return maxWheelAngle_; }
    double setSpeed() const { return setSpeed_; }
    const OverrideDetector& overrideDetector() const { return override_; }

    // Validates the preconditions for a mode without changing state.
    std::optional<std::string> checkAvailability(PilotMode m, const VehicleState& s, const PathSnapshotPtr& path,
                                                 double wallNow, double pathWall) const;

private:
    void engage(PilotMode m, const VehicleState& s);
    void enterEmergency(const std::string& reason);
    void learnSteeringRatio(const VehicleState& s);
    double cruiseTarget(const VehicleState& s) const;
    void setMessage(const std::string& msg);
    void emit(PilotEvent e, const std::string& msg);

    Config cfg_;
    Logger* log_ = nullptr;
    std::function<void(PilotEvent, const std::string&)> onEvent_;

    PilotMode mode_ = PilotMode::Off;
    PilotMode lastActiveMode_ = PilotMode::Off;
    double setSpeed_ = 0.0;  // m/s
    double lastTime_ = -1.0;
    double maxWheelAngle_ = 0.6;
    int invertedSteeringFrames_ = 0;
    double lastCrossTrackWarn_ = -100.0;

    SteeringShaper shaper_;
    LongitudinalController longitudinal_;
    OverrideDetector override_;
    Watchdog watchdog_;

    ControlCommand last_;
    PathSnapshotPtr hintPath_;
    std::size_t hintIndex_ = 0;

    PilotStatus status_;
    ControllerDebug debug_;
};

}  // namespace atspilot

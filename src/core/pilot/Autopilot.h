#pragma once

#include <cstdint>
#include <deque>
#include <functional>
#include <optional>
#include <string>
#include <unordered_map>

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
    // Whether the game accepted ATSPilot's input device (pedals, cruise control,
    // signals); without it ATSPilot cannot drive.
    void setInputReady(bool ready) { inputReady_ = ready; }
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
    double cruiseTarget(const VehicleState& s);
    // Where the truck's front stops relative to a service stop's point.
    double serviceFrontOffset(const PathStop& stop, const VehicleConfig& vc) const;
    // Handles one fuel or weigh stop: adds its stop constraint and runs the
    // refuelling / weighing sequence once the truck stands there.
    void serviceStop(const PathStop& stop, const VehicleState& s, const VehicleConfig& vc, const Path& path,
                     double frontS, double toFront, std::vector<SpeedConstraint>& constraints);
    // Right on red: whether the truck, at a red light where its lane turns right,
    // may go now (after a full stop, with no traffic coming).
    bool rightOnRed(const PathStop& stop, const VehicleState& s, const Path& path, double lineDistance);
    void setMessage(const std::string& msg);
    void emit(PilotEvent e, const std::string& msg);
    GameButtons blinkers(const VehicleState& s, const PathSnapshotPtr& path, double time);
    void logSignal(const PathStop& stop, LightState state, SignalDecision decision, bool byId);
    void clearStop(std::uint32_t segment);
    int lightFacing() const;
    bool laneTrafficBehind(const Path& path, double s, const std::vector<WorldVehicle>& vehicles) const;
    // Pulling into the lane from beside it: the larger deviation allowed meanwhile.
    double crossTrackLimit() const { return std::max(eff_.safety.maxCrossTrack, joinAllowance_); }
    void learnLightFacing(const WorldLight& light, double travelYaw);

    Config cfg_;   // as configured
    Config eff_;   // with the active driving profile applied
    DrivingProfile activeProfile_ = DrivingProfile::Normal;
    Logger* log_ = nullptr;
    std::function<void(PilotEvent, const std::string&)> onEvent_;

    PilotMode mode_ = PilotMode::Off;
    double setSpeed_ = 0.0;  // m/s, the maximum: speed.max (or the player's cruise speed with cruise_sets_max)
    double lastLimit_ = 0.0; // m/s, the last posted speed limit reported
    bool cruiseIgnoredLogged_ = false;
    int crossingId_ = -1;        // crossing vehicle currently calling for strong braking
    double crossingSince_ = 0.0;

    // Service stops (fuel pump, weigh station scale).
    std::uint32_t serviceSegment_ = 0xFFFFFFFFu;
    double serviceSince_ = -1.0;   // arrived and stopped at the spot
    double serviceNudge_ = 0.0;    // m moved forward after a refuelling attempt did nothing
    int fuelTries_ = 0;
    double fuelBest_ = -1.0;
    double fuelLastRise_ = -1.0;
    bool refuelling_ = false;
    bool activate_ = false;

    // Right turn on red.
    std::uint32_t rorSegment_ = 0xFFFFFFFFu;     // waiting at this red light to turn right
    double rorSince_ = -1.0;
    double rorClearSince_ = -1.0;
    std::uint32_t rorCommitted_ = 0xFFFFFFFFu;   // turning right on red through this lane
    double lastUpdateWall_ = -1e9;  // wall time of the previous update (hitch detection)
    double hitchFrom_ = -1e9;
    double hitchUntil_ = -1e9;
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
    // Stop-and-go junctions (stop signs, unreadable lights): stopped at the line since (s).
    std::uint32_t stopAndGoSegment_ = 0xFFFFFFFFu;
    double stopAndGoSince_ = -1.0;
    // The last stop line passed on green, so further signal lanes of the same
    // junction follow it; and whether lights face along or against their traffic.
    Vec2 lastGreenLine_;
    double lastGreenTime_ = -1e9;
    int lightFacingVotes_ = 0;
    double joinAllowance_ = 0.0;  // > 0 while pulling into the lane
    double baseTarget_ = -1.0;    // set speed / limit, with decreases rate-limited
    int joinSide_ = 0;            // +1 the lane is to the left, -1 to the right
    std::unordered_map<std::uint32_t, LightState> loggedSignals_;

    WorldSnapshotPtr world_;
    bool directSteering_ = false;
    bool inputReady_ = true;
    std::uint32_t unmatchedSignalLogged_ = 0xFFFFFFFFu;

    // Turn signals ATSPilot switched on (it never cancels the driver's own).
    bool ourLeftBlinker_ = false;
    bool ourRightBlinker_ = false;
    double lastBlinkerPress_ = -100.0;
    bool holdIndicators_ = true;   // use the game's hold controls until shown not to work
    bool holdConfirmed_ = false;
    double holdSince_ = -1.0;
    int indicatorHold_ = 0;
    double frontS_ = 0.0;              // rear axle to front bumper, m
    double activeIndication_ = -1e9;

    ControlCommand last_;
    PathSnapshotPtr hintPath_;
    std::size_t hintIndex_ = 0;

    PilotStatus status_;
    ControllerDebug debug_;
};

}  // namespace atspilot

#include "pilot/Autopilot.h"

#include <cmath>

#include "math/Coordinates.h"
#include "math/MathUtil.h"

namespace atspilot {

const char* toString(PilotMode m) {
    switch (m) {
        case PilotMode::Off: return "OFF";
        case PilotMode::Cruise: return "CRUISE";
        case PilotMode::LaneAssist: return "LANE ASSIST";
        case PilotMode::Autopilot: return "AUTOPILOT";
        case PilotMode::EmergencyStop: return "EMERGENCY STOP";
    }
    return "?";
}

const char* toString(PilotRequest r) {
    switch (r) {
        case PilotRequest::ToggleAutopilot: return "toggle_autopilot";
        case PilotRequest::ToggleLaneAssist: return "toggle_lane_assist";
        case PilotRequest::ToggleCruise: return "toggle_cruise";
        case PilotRequest::SpeedUp: return "speed_up";
        case PilotRequest::SpeedDown: return "speed_down";
        case PilotRequest::Resume: return "resume";
        case PilotRequest::Cancel: return "cancel";
        case PilotRequest::EmergencyDisable: return "emergency_disable";
    }
    return "?";
}

namespace {

bool steers(PilotMode m) { return m == PilotMode::LaneAssist || m == PilotMode::Autopilot; }
bool drivesPedals(PilotMode m) { return m == PilotMode::Cruise || m == PilotMode::Autopilot; }

struct AxlePoints {
    Vec2 rear;
    Vec2 front;
    double yaw = 0.0;
};

AxlePoints axlePoints(const VehicleState& s, const VehicleConfig& vc) {
    const double yaw = coords::sdkHeadingToYaw(s.headingUnit);
    const Vec2 origin = coords::worldToPlan(s.worldPosition);
    const Vec2 forward = coords::yawToDirection(yaw);
    // Vehicle space -z is forward, so an axle at z sits -z metres ahead of the origin.
    return {origin + forward * (-vc.rearAxleZ), origin + forward * (-vc.frontAxleZ), yaw};
}

}  // namespace

Autopilot::Autopilot(const Config& cfg, Logger* log)
    : cfg_(cfg), log_(log), override_(cfg.safety), watchdog_(cfg.safety) {
    setConfig(cfg);
    status_.statusMessage = "ATSPilot Ready";
}

void Autopilot::setConfig(const Config& cfg) {
    cfg_ = cfg;
    shaper_.setParams(cfg.steering.shaper);
    longitudinal_.setParams(cfg.cruise);
    override_.setConfig(cfg.safety);
    watchdog_.setConfig(cfg.safety);
    maxWheelAngle_ = degToRad(cfg.steering.initialMaxWheelAngleDeg);
    if (setSpeed_ <= 0.0) setSpeed_ = speedToMps(cfg.speed.maxSpeed, cfg.speed.units);
}

void Autopilot::setMessage(const std::string& msg) { status_.statusMessage = msg; }

void Autopilot::emit(PilotEvent e, const std::string& msg) {
    if (onEvent_) onEvent_(e, msg);
}

std::optional<std::string> Autopilot::checkAvailability(PilotMode m, const VehicleState& s,
                                                        const PathSnapshotPtr& path, double wallNow,
                                                        double pathWall) const {
    if (!cfg_.autopilotEnabled) return "ATSPilot disabled in config";
    if (!s.valid) return "Telemetry Unavailable";
    if (s.paused) return "Game paused";
    if (s.speed < -0.5 || s.gear < 0) return "Not available in reverse";
    if (drivesPedals(m) && s.parkingBrake) return "Release parking brake";
    if (drivesPedals(m) && !s.engineEnabled) return "Engine off";
    if (steers(m)) {
        if (!path || !path->path.valid()) return "No valid road path";
        if (wallNow - pathWall > cfg_.safety.pathTimeout) return "Path stale";
        const double yaw = coords::sdkHeadingToYaw(s.headingUnit);
        const auto proj = path->path.project(coords::worldToPlan(s.worldPosition));
        if (!proj) return "No valid road path";
        if (std::abs(proj->crossTrackError) > cfg_.safety.maxCrossTrack) return "Truck not on the planned lane";
        if (std::abs(headingDifference(yaw, proj->yaw)) > degToRad(cfg_.safety.maxHeadingErrorDeg))
            return "Truck not aligned with the road";
    }
    return std::nullopt;
}

void Autopilot::engage(PilotMode m, const VehicleState& s) {
    const PilotMode previous = mode_;
    mode_ = m;
    if (m != PilotMode::Off) lastActiveMode_ = m;
    if (!steers(previous) && steers(m)) shaper_.reset(clamp(s.effectiveSteering, -1.0, 1.0));
    if (!drivesPedals(previous) && drivesPedals(m)) longitudinal_.reset(0.0, 0.0);
    override_.reset();
    invertedSteeringFrames_ = 0;
    hintPath_.reset();

    std::string msg = m == PilotMode::Autopilot ? "Autopilot Engaged"
                      : m == PilotMode::LaneAssist ? "Lane Assist Engaged"
                                                   : "Cruise Engaged";
    setMessage(msg);
    if (log_) {
        log_->info("{} (set speed {:.0f} {})", msg, mpsToSpeed(setSpeed_, cfg_.speed.units), unitLabel(cfg_.speed.units));
    }
    emit(PilotEvent::Engaged, msg);
}

void Autopilot::disengage(const std::string& reason, PilotEvent evt) {
    const bool wasActive = mode_ != PilotMode::Off;
    mode_ = PilotMode::Off;
    last_ = ControlCommand{};
    hintPath_.reset();
    setMessage(reason);
    if (wasActive) {
        if (log_) log_->info("Disengaged: {}", reason);
        emit(evt, reason);
    }
}

void Autopilot::enterEmergency(const std::string& reason) {
    if (mode_ == PilotMode::EmergencyStop) return;
    const bool hadPedals = drivesPedals(mode_);
    mode_ = PilotMode::EmergencyStop;
    if (!hadPedals) longitudinal_.reset(0.0, 0.0);
    setMessage("Emergency Stop: " + reason);
    if (log_) log_->warn("Emergency stop: {}", reason);
    emit(PilotEvent::EmergencyBraking, reason);
}

void Autopilot::request(PilotRequest r, const VehicleState& s, const VehicleConfig& vc, const PathSnapshotPtr& path,
                        double wallNow, double pathWall) {
    (void)vc;
    const double step = speedToMps(cfg_.speed.step, cfg_.speed.units);
    const double maxSet = speedToMps(cfg_.speed.maxSpeed, cfg_.speed.units);

    auto toggle = [&](PilotMode target) {
        if (mode_ == target || mode_ == PilotMode::EmergencyStop) {
            disengage(mode_ == PilotMode::EmergencyStop ? "Emergency stop cancelled" : "ATSPilot Disengaged");
            return;
        }
        if (const auto why = checkAvailability(target, s, path, wallNow, pathWall)) {
            setMessage("ATSPilot unavailable: " + *why);
            if (log_) log_->info("Engage {} refused: {}", toString(target), *why);
            emit(PilotEvent::Unavailable, *why);
            return;
        }
        if (drivesPedals(target) && !drivesPedals(mode_)) {
            // Engaging while moving holds the current speed; from standstill use the maximum.
            setSpeed_ = s.speed > 5.0 ? std::round(mpsToSpeed(s.speed, cfg_.speed.units)) : cfg_.speed.maxSpeed;
            setSpeed_ = clamp(speedToMps(setSpeed_, cfg_.speed.units), step, maxSet);
        }
        engage(target, s);
    };

    switch (r) {
        case PilotRequest::ToggleAutopilot: toggle(PilotMode::Autopilot); break;
        case PilotRequest::ToggleLaneAssist: toggle(PilotMode::LaneAssist); break;
        case PilotRequest::ToggleCruise: toggle(PilotMode::Cruise); break;
        case PilotRequest::SpeedUp:
        case PilotRequest::SpeedDown: {
            const double delta = r == PilotRequest::SpeedUp ? step : -step;
            // Snap to the step grid in display units so repeated presses give round numbers.
            const double display = mpsToSpeed(setSpeed_, cfg_.speed.units);
            const double snapped = std::round((display + (delta > 0 ? cfg_.speed.step : -cfg_.speed.step)) /
                                              cfg_.speed.step) * cfg_.speed.step;
            setSpeed_ = clamp(speedToMps(snapped, cfg_.speed.units), step, maxSet);
            const std::string msg = "Set speed " + std::to_string(static_cast<int>(std::round(
                                        mpsToSpeed(setSpeed_, cfg_.speed.units)))) + " " + unitLabel(cfg_.speed.units);
            setMessage(msg);
            emit(PilotEvent::SetSpeedChanged, msg);
            break;
        }
        case PilotRequest::Resume:
            if (mode_ == PilotMode::Off && lastActiveMode_ != PilotMode::Off) {
                const PilotMode target = lastActiveMode_;
                if (const auto why = checkAvailability(target, s, path, wallNow, pathWall)) {
                    setMessage("ATSPilot unavailable: " + *why);
                    emit(PilotEvent::Unavailable, *why);
                } else {
                    engage(target, s);
                }
            }
            break;
        case PilotRequest::Cancel:
            disengage("ATSPilot Disengaged");
            break;
        case PilotRequest::EmergencyDisable:
            lastActiveMode_ = PilotMode::Off;  // resume must not re-engage after an emergency disable
            disengage("ATSPilot Emergency Disable");
            break;
    }
}

void Autopilot::learnSteeringRatio(const VehicleState& s) {
    if (!cfg_.steering.learnSteeringRatio) return;
    if (std::abs(s.effectiveSteering) < 0.08 || std::abs(s.speed) < 1.0) return;
    const double ratio = std::abs(s.steerableWheelAngle / s.effectiveSteering);
    // Reject samples that cannot be a real steering rack (e.g. while wheels are still
    // catching up with a fast input change).
    if (ratio < degToRad(10.0) || ratio > degToRad(70.0)) return;
    maxWheelAngle_ += (ratio - maxWheelAngle_) * 0.01;
}

double Autopilot::cruiseTarget(const VehicleState& s) const {
    double target = setSpeed_;
    if (cfg_.speed.followSpeedLimit && s.navigationSpeedLimit > 0.5) {
        target = std::min(target, s.navigationSpeedLimit + speedToMps(cfg_.speed.limitOffset, cfg_.speed.units));
    }
    return std::max(0.0, target);
}

ControlCommand Autopilot::update(const VehicleState& s, const VehicleConfig& vc, const PathSnapshotPtr& path,
                                 double wallNow, double telemetryWall, double pathWall) {
    // The first frame has no previous timestamp; assume a nominal 60 Hz step.
    double dt = lastTime_ < 0.0 ? 1.0 / 60.0 : s.time - lastTime_;
    lastTime_ = s.time;
    dt = clamp(dt, 0.0, 0.1);

    status_.telemetryConnected = s.valid;
    status_.speed = s.speed;
    status_.setSpeed = setSpeed_;
    status_.mode = mode_;
    if (path) {
        status_.road = path->roadName;
        status_.nextManeuver = path->nextManeuver;
        status_.nextManeuverDistance = path->nextManeuverDistance;
    }
    status_.routeDistance = s.navigationDistance;
    status_.available = !checkAvailability(PilotMode::Autopilot, s, path, wallNow, pathWall).has_value();

    if (s.valid) learnSteeringRatio(s);
    debug_.maxWheelAngle = maxWheelAngle_;
    debug_.currentSpeed = s.speed;

    if (mode_ == PilotMode::Off || dt <= 0.0) {
        if (mode_ == PilotMode::Off) last_ = ControlCommand{};
        last_.time = s.time;
        return last_;
    }

    if (s.paused) {
        disengage("Game paused");
        return last_;
    }

    if (const auto stale = watchdog_.check(wallNow, telemetryWall, pathWall, steers(mode_))) {
        if (*stale == "Telemetry stale") {
            disengage(*stale);
            return last_;
        }
        enterEmergency(*stale);
    }

    const OverrideKind ov = override_.update(s, last_);
    const bool steeringOverride = ov == OverrideKind::Steering && steers(mode_);
    const bool pedalOverride = (ov == OverrideKind::Brake || ov == OverrideKind::Throttle) &&
                               (drivesPedals(mode_) || mode_ == PilotMode::EmergencyStop);
    if (steeringOverride || pedalOverride || (mode_ == PilotMode::EmergencyStop && ov != OverrideKind::None)) {
        disengage(std::string("Driver Override (") + toString(ov) + ")", PilotEvent::DriverOverride);
        return last_;
    }

    ControlCommand cmd;
    cmd.active = true;
    cmd.time = s.time;
    const bool steeringWanted = steers(mode_) || mode_ == PilotMode::EmergencyStop;
    const bool pedalsWanted = drivesPedals(mode_) || mode_ == PilotMode::EmergencyStop;

    // --- Lateral ---------------------------------------------------------------
    bool pathUsable = path && path->path.valid() && wallNow - pathWall <= cfg_.safety.pathTimeout;
    std::optional<LateralOutput> lat;
    if (steeringWanted && pathUsable) {
        const AxlePoints axles = axlePoints(s, vc);
        LateralInput in;
        in.rearAxle = axles.rear;
        in.frontAxle = axles.front;
        in.yaw = axles.yaw;
        in.speed = std::max(0.0, s.speed);
        in.wheelbase = vc.wheelbase;

        std::optional<std::size_t> hint;
        if (hintPath_ == path) hint = hintIndex_;
        else hint = std::size_t{0};
        LateralOutput out = computeLateral(cfg_.steering.lateral, path->path, in, hint);
        if (out.valid && std::abs(out.crossTrackError) > cfg_.safety.maxCrossTrack && hint) {
            // The windowed search may have locked onto the wrong stretch; retry globally.
            out = computeLateral(cfg_.steering.lateral, path->path, in, std::nullopt);
        }
        if (out.valid) {
            hintPath_ = path;
            hintIndex_ = out.projectionIndex;
            lat = out;
        }
    }

    if (steers(mode_)) {
        if (!lat) {
            enterEmergency("Path Lost");
        } else if (std::abs(lat->crossTrackError) > cfg_.safety.maxCrossTrack) {
            enterEmergency("Dangerous path deviation");
        } else if (std::abs(lat->headingError) > degToRad(cfg_.safety.maxHeadingErrorDeg)) {
            enterEmergency("Heading deviates from path");
        } else if (std::abs(lat->crossTrackError) > cfg_.safety.warnCrossTrack && s.time - lastCrossTrackWarn_ > 5.0) {
            lastCrossTrackWarn_ = s.time;
            if (log_) log_->warn("Cross-track error {:.2f} m", lat->crossTrackError);
        }
    }

    if (steeringWanted) {
        double desiredWheel = 0.0;
        if (lat && mode_ != PilotMode::EmergencyStop) {
            desiredWheel = lat->wheelAngle;
        } else if (lat && std::abs(lat->crossTrackError) <= cfg_.safety.maxCrossTrack) {
            // Emergency with usable geometry: keep tracking the lane while braking.
            desiredWheel = lat->wheelAngle;
        } else {
            // No trustworthy geometry: unwind steering slowly instead of snapping to centre.
            desiredWheel = shaper_.output() * maxWheelAngle_ * std::max(0.0, 1.0 - 0.5 * dt);
        }
        cmd.steering = shaper_.update(desiredWheel, maxWheelAngle_, std::max(0.0, s.speed), vc.wheelbase,
                                      vc.trailerCount, dt);
        cmd.steerActive = true;
        debug_.targetWheelAngle = desiredWheel;
        debug_.targetSteering = cmd.steering;
        if (lat) {
            debug_.crossTrackError = lat->crossTrackError;
            debug_.headingError = lat->headingError;
            debug_.lookAheadDistance = lat->lookahead;
            debug_.pathS = lat->pathS;
            status_.crossTrackError = lat->crossTrackError;
        }

        // Runtime check of the steering sign assumption: with the driver hands-off the
        // effective steering must follow the command, never oppose it.
        if (std::abs(cmd.steering) > 0.04 && s.effectiveSteering * cmd.steering < 0.0 &&
            std::abs(s.effectiveSteering) > 0.03) {
            if (++invertedSteeringFrames_ > 45) {
                if (log_) {
                    log_->error("Steering response opposes command; check steering.output_sign in atspilot.toml");
                }
                disengage("Steering direction mismatch");
                return last_;
            }
        } else {
            invertedSteeringFrames_ = 0;
        }
    }

    // --- Longitudinal ------------------------------------------------------------
    if (pedalsWanted) {
        PedalCommand pedals;
        if (mode_ == PilotMode::EmergencyStop) {
            pedals = longitudinal_.emergency(dt);
            debug_.targetSpeed = 0.0;
            if (s.speed < 0.5) {
                disengage("Emergency stop complete");
                // Leave the brake applied for this final frame; the next frame is neutral.
                cmd.steering = 0.0;
                cmd.throttle = 0.0;
                cmd.brake = pedals.brake;
                cmd.pedalsActive = true;
                last_ = cmd;
                return cmd;
            }
        } else {
            double target = cruiseTarget(s);
            debug_.curveRadius = 0.0;
            if (mode_ == PilotMode::Autopilot && lat && path) {
                VehicleLoadFactors f;
                f.trailerCount = vc.trailerCount;
                f.cargoMassKg = vc.cargoMassKg;
                f.wet = s.wipers;
                f.aggressiveness = cfg_.aggressiveness;
                const SpeedPlan plan = planSpeed(cfg_.planner, f, path->path, lat->pathS, target);
                target = plan.targetSpeed;
                debug_.curveRadius = plan.limitingCurveRadius;
            }
            debug_.targetSpeed = target;
            status_.targetSpeed = target;
            pedals = longitudinal_.update(target, s.speed, dt);
        }
        cmd.throttle = pedals.throttle;
        cmd.brake = pedals.brake;
        cmd.pedalsActive = true;
        debug_.throttleOutput = pedals.throttle;
        debug_.brakeOutput = pedals.brake;
        debug_.brakeLevel = pedals.level;
    }

    status_.mode = mode_;
    last_ = cmd;
    return cmd;
}

}  // namespace atspilot

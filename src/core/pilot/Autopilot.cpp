#include "pilot/Autopilot.h"

#include <algorithm>
#include <cmath>

#include "math/Coordinates.h"
#include "math/MathUtil.h"

namespace atspilot {

const char* toString(PilotMode m) {
    switch (m) {
        case PilotMode::Off: return "OFF";
        case PilotMode::Autopilot: return "AUTOPILOT";
        case PilotMode::EmergencyStop: return "EMERGENCY STOP";
    }
    return "?";
}

const char* toString(PilotRequest r) {
    switch (r) {
        case PilotRequest::Toggle: return "toggle";
        case PilotRequest::Cancel: return "cancel";
        case PilotRequest::EmergencyDisable: return "emergency_disable";
    }
    return "?";
}

const char* toString(StopKind k) {
    switch (k) {
        case StopKind::Signal: return "traffic light";
        case StopKind::StopSign: return "stop sign";
        case StopKind::Yield: return "give way";
        case StopKind::RailCrossing: return "railway crossing";
    }
    return "?";
}

namespace {

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

// Distance from the rear axle to the front bumper (steer axle plus a typical overhang).
double frontFromRear(const VehicleConfig& vc) { return (vc.rearAxleZ - vc.frontAxleZ) + 1.5; }

bool startsWith(const std::string& s, const char* prefix) { return s.rfind(prefix, 0) == 0; }

}  // namespace

Autopilot::Autopilot(const Config& cfg, Logger* log)
    : log_(log), cruise_(cfg.ingame.cruise), override_(cfg.safety), watchdog_(cfg.safety) {
    setConfig(cfg);
    status_.statusMessage = "ATSPilot Ready";
}

void Autopilot::setConfig(const Config& cfg) {
    cfg_ = cfg;
    activeProfile_ = resolveProfile(cfg.profile, 0.0);
    eff_ = applyProfile(cfg_, activeProfile_);
    shaper_.setParams(eff_.steering.shaper);
    longitudinal_.setParams(eff_.cruise);
    override_.setConfig(eff_.safety);
    watchdog_.setConfig(eff_.safety);
    maxWheelAngle_ = degToRad(eff_.steering.initialMaxWheelAngleDeg);
    setSpeed_ = speedToMps(eff_.speed.maxSpeed, eff_.speed.units);
    status_.profile = toString(activeProfile_);
}

void Autopilot::updateProfile(const VehicleConfig& vc) {
    const DrivingProfile resolved = resolveProfile(cfg_.profile, vc.cargoMassKg);
    if (resolved == activeProfile_) return;
    activeProfile_ = resolved;
    eff_ = applyProfile(cfg_, resolved);
    shaper_.setParams(eff_.steering.shaper);
    setSpeed_ = std::min(setSpeed_, speedToMps(eff_.speed.maxSpeed, eff_.speed.units));
    status_.profile = toString(resolved);
    if (log_) log_->info("Driving profile: {}", toString(resolved));
}

void Autopilot::setMessage(const std::string& msg) { status_.statusMessage = msg; }

void Autopilot::emit(PilotEvent e, const std::string& msg) {
    if (onEvent_) onEvent_(e, msg);
}

std::optional<std::string> Autopilot::checkAvailability(const VehicleState& s, const PathSnapshotPtr& path,
                                                        double wallNow, double pathWall) const {
    if (!eff_.autopilotEnabled) return "ATSPilot disabled in config";
    if (!s.valid) return "Telemetry Unavailable";
    if (s.paused) return "Game paused";
    if (s.speed < -0.5 || s.gear < 0) return "Not available in reverse";
    if (s.parkingBrake) return "Release parking brake";
    if (!s.engineEnabled) return "Engine off";
    if (!path || !path->path.valid()) return "No valid road path";
    if (wallNow - pathWall > eff_.safety.pathTimeout) return "Path stale";
    const double yaw = coords::sdkHeadingToYaw(s.headingUnit);
    const auto proj = path->path.project(coords::worldToPlan(s.worldPosition));
    if (!proj) return "No valid road path";
    if (std::abs(proj->crossTrackError) > eff_.safety.maxCrossTrack) return "Truck not on the planned lane";
    if (std::abs(headingDifference(yaw, proj->yaw)) > degToRad(eff_.safety.maxHeadingErrorDeg))
        return "Truck not aligned with the road";
    return std::nullopt;
}

void Autopilot::engage(const VehicleState& s) {
    mode_ = PilotMode::Autopilot;
    // ATSPilot's input is added to the driver's, and the game keeps the driver's
    // steering where it was. Starting from the current wheel angle would double it,
    // so ATSPilot's share starts at zero. While off, it outputs nothing, so the
    // reported input is entirely the driver's.
    shaper_.reset(0.0);
    longitudinal_.reset(0.0, 0.0);
    cruise_ = GameCruiseManager(eff_.ingame.cruise);
    override_.reset(clamp(s.inputSteering, -1.0, 1.0));
    invertedSteeringFrames_ = 0;
    hintPath_.reset();
    waitingStop_.reset();
    tapStart_ = -1.0;
    setSpeed_ = speedToMps(eff_.speed.maxSpeed, eff_.speed.units);

    setMessage("Autopilot Engaged");
    if (log_) {
        log_->info("Autopilot engaged (profile {}, {})", toString(activeProfile_),
                   eff_.ingame.useCruiseControl ? "speed via game cruise control" : "speed via ATSPilot pedals");
    }
    emit(PilotEvent::Engaged, "Autopilot Engaged");
}

void Autopilot::disengage(const std::string& reason, PilotEvent evt) {
    const bool wasActive = mode_ != PilotMode::Off;
    mode_ = PilotMode::Off;
    last_ = ControlCommand{};
    hintPath_.reset();
    waitingStop_.reset();
    ourLeftBlinker_ = ourRightBlinker_ = false;
    setMessage(reason);
    if (wasActive) {
        if (log_) log_->info("Disengaged: {}", reason);
        emit(evt, reason);
    }
}

void Autopilot::enterEmergency(const std::string& reason) {
    if (mode_ == PilotMode::EmergencyStop) return;
    mode_ = PilotMode::EmergencyStop;
    setMessage("Emergency Stop: " + reason);
    if (log_) log_->warn("Emergency stop: {}", reason);
    emit(PilotEvent::EmergencyBraking, reason);
}

void Autopilot::request(PilotRequest r, const VehicleState& s, const VehicleConfig& vc, const PathSnapshotPtr& path,
                        double wallNow, double pathWall) {
    switch (r) {
        case PilotRequest::Toggle:
            if (mode_ != PilotMode::Off) {
                disengage("ATSPilot Off");
                return;
            }
            updateProfile(vc);
            if (const auto why = checkAvailability(s, path, wallNow, pathWall)) {
                setMessage("ATSPilot unavailable: " + *why);
                if (log_) log_->info("Engage refused: {}", *why);
                emit(PilotEvent::Unavailable, *why);
                return;
            }
            engage(s);
            break;
        case PilotRequest::Cancel:
            disengage("ATSPilot Off");
            break;
        case PilotRequest::EmergencyDisable:
            disengage("ATSPilot Emergency Disable");
            break;
    }
}

void Autopilot::learnSteeringRatio(const VehicleState& s) {
    if (!eff_.steering.learnSteeringRatio) return;
    if (std::abs(s.effectiveSteering) < 0.08 || std::abs(s.speed) < 1.0) return;
    const double ratio = std::abs(s.steerableWheelAngle / s.effectiveSteering);
    // Reject samples that cannot be a real steering rack (e.g. while wheels are still
    // catching up with a fast input change).
    if (ratio < degToRad(10.0) || ratio > degToRad(70.0)) return;
    maxWheelAngle_ += (ratio - maxWheelAngle_) * 0.01;
}

double Autopilot::cruiseTarget(const VehicleState& s) const {
    double target = setSpeed_;
    if (eff_.speed.followSpeedLimit && s.navigationSpeedLimit > 0.5) {
        target = std::min(target, s.navigationSpeedLimit + speedToMps(eff_.speed.limitOffset, eff_.speed.units));
    }
    return std::max(0.0, target);
}

GameButtons Autopilot::blinkers(const VehicleState& s, const PathSnapshotPtr& path, double time) {
    GameButtons b;
    if (!eff_.ingame.useBlinkers) return b;
    bool wantLeft = false, wantRight = false;
    if (path && path->nextManeuverDistance < eff_.ingame.blinkerDistance) {
        const std::string& m = path->nextManeuver;
        const bool turnLike = startsWith(m, "Turn") || startsWith(m, "Keep") || startsWith(m, "Change Lane");
        if (turnLike) {
            wantLeft = m.find("Left") != std::string::npos;
            wantRight = m.find("Right") != std::string::npos;
        }
    }
    if (time - lastBlinkerPress_ < 0.6) return b;
    // The game's indicator switch toggles; telemetry reports its position.
    if (wantLeft && !s.blinkerLeft) {
        b.leftBlinker = true;
        ourLeftBlinker_ = true;
    } else if (wantRight && !s.blinkerRight) {
        b.rightBlinker = true;
        ourRightBlinker_ = true;
    } else if (!wantLeft && ourLeftBlinker_ && s.blinkerLeft) {
        b.leftBlinker = true;
        ourLeftBlinker_ = false;
    } else if (!wantRight && ourRightBlinker_ && s.blinkerRight) {
        b.rightBlinker = true;
        ourRightBlinker_ = false;
    }
    if (!s.blinkerLeft && !wantLeft) ourLeftBlinker_ = false;
    if (!s.blinkerRight && !wantRight) ourRightBlinker_ = false;
    if (b.any()) lastBlinkerPress_ = time;
    return b;
}

ControlCommand Autopilot::update(const VehicleState& s, const VehicleConfig& vc, const PathSnapshotPtr& path,
                                 double wallNow, double telemetryWall, double pathWall) {
    // The first frame has no previous timestamp; assume a nominal 60 Hz step.
    double dt = lastTime_ < 0.0 ? 1.0 / 60.0 : s.time - lastTime_;
    lastTime_ = s.time;
    dt = clamp(dt, 0.0, 0.1);

    if (vc.valid || vc.cargoMassKg > 0.0) updateProfile(vc);
    status_.telemetryConnected = s.valid;
    status_.speed = s.speed;
    status_.setSpeed = setSpeed_;
    status_.mode = mode_;
    status_.gameCruiseActive = s.cruiseControlSpeed > 0.1;
    status_.cruiseSetSpeed = s.cruiseControlSpeed;
    status_.waitingAtIntersection = waitingStop_.has_value();
    if (path) {
        status_.road = path->roadName;
        status_.nextManeuver = path->nextManeuver;
        status_.nextManeuverDistance = path->nextManeuverDistance;
    }
    status_.navigationActive = path && path->navigationActive;
    status_.gpsMatched = path && path->gpsMatched;
    status_.routeDistance = status_.navigationActive ? path->routeRemaining : s.navigationDistance;
    status_.available = !checkAvailability(s, path, wallNow, pathWall).has_value();

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

    if (const auto stale = watchdog_.check(wallNow, telemetryWall, pathWall, true)) {
        if (*stale == "Telemetry stale") {
            disengage(*stale);
            return last_;
        }
        enterEmergency(*stale);
    }

    // Arrival: the route ends at the depot entrance. Stop there, hand parking to
    // the game's quick-park, and finish.
    if (mode_ == PilotMode::Autopilot && path && path->navigationActive && path->routeRemaining < 40.0 &&
        std::abs(s.speed) < 0.5) {
        ControlCommand done;
        done.time = s.time;
        done.buttons.quickPark = eff_.ingame.quickPark;
        disengage("Destination Reached", PilotEvent::Arrived);
        return done;
    }

    OverrideKind ov = override_.update(s, last_);
    // While waiting at a stop line a short throttle tap means "go" (handled below).
    if (waitingStop_ && ov == OverrideKind::Throttle) ov = OverrideKind::None;
    if (ov != OverrideKind::None) {
        if (log_) {
            log_->info("Override input: steering {:.3f} (ours {:.3f}, driver {:.3f}, held at engage {:.3f}, "
                       "mixing {}), throttle {:.2f}, brake {:.2f} (ours {:.2f}), effective steering {:.3f}",
                       s.inputSteering, last_.steering, override_.driverSteering(), override_.steeringBaseline(),
                       static_cast<int>(override_.steeringMixing()), s.inputThrottle, s.inputBrake, last_.brake,
                       s.effectiveSteering);
        }
        disengage(std::string("Driver Override (") + toString(ov) + ")", PilotEvent::DriverOverride);
        return last_;
    }

    ControlCommand cmd;
    cmd.active = true;
    cmd.time = s.time;

    // --- Lateral ---------------------------------------------------------------
    const bool pathUsable = path && path->path.valid() && wallNow - pathWall <= eff_.safety.pathTimeout;
    std::optional<LateralOutput> lat;
    if (pathUsable) {
        const AxlePoints axles = axlePoints(s, vc);
        LateralInput in;
        in.rearAxle = axles.rear;
        in.frontAxle = axles.front;
        in.yaw = axles.yaw;
        in.speed = std::max(0.0, s.speed);
        in.wheelbase = vc.wheelbase;

        // Same snapshot: continue from the last projection. New snapshot: start from
        // where the planner placed the truck when it built the path.
        std::optional<std::size_t> hint =
            hintPath_ == path ? hintIndex_ : path->path.segmentIndexAt(path->truckS);
        LateralOutput out = computeLateral(eff_.steering.lateral, path->path, in, hint);
        if (out.valid && std::abs(out.crossTrackError) > eff_.safety.maxCrossTrack && hint) {
            // The windowed search may have locked onto the wrong stretch; retry globally.
            out = computeLateral(eff_.steering.lateral, path->path, in, std::nullopt);
        }
        if (out.valid) {
            hintPath_ = path;
            hintIndex_ = out.projectionIndex;
            lat = out;
        }
    }

    if (mode_ == PilotMode::Autopilot) {
        if (!lat) {
            enterEmergency("Path Lost");
        } else if (std::abs(lat->crossTrackError) > eff_.safety.maxCrossTrack) {
            enterEmergency("Dangerous path deviation");
        } else if (std::abs(lat->headingError) > degToRad(eff_.safety.maxHeadingErrorDeg)) {
            enterEmergency("Heading deviates from path");
        } else if (std::abs(lat->crossTrackError) > eff_.safety.warnCrossTrack && s.time - lastCrossTrackWarn_ > 5.0) {
            lastCrossTrackWarn_ = s.time;
            if (log_) log_->warn("Cross-track error {:.2f} m", lat->crossTrackError);
        }
    }

    double desiredWheel = 0.0;
    if (lat && (mode_ != PilotMode::EmergencyStop || std::abs(lat->crossTrackError) <= eff_.safety.maxCrossTrack)) {
        // Normal driving, or an emergency stop with geometry still trusted: track the lane.
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
    if (std::abs(cmd.steering) > 0.04 && s.effectiveSteering * cmd.steering < 0.0 && std::abs(s.effectiveSteering) > 0.03) {
        if (++invertedSteeringFrames_ > 45) {
            if (log_) log_->error("Steering response opposes command; check steering.output_sign in atspilot.toml");
            disengage("Steering direction mismatch");
            return last_;
        }
    } else {
        invertedSteeringFrames_ = 0;
    }

    // --- Longitudinal ------------------------------------------------------------
    PedalCommand pedals;
    bool ownPedals = true;
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
        std::vector<SpeedConstraint> constraints;
        std::optional<PathStop> nextStop;
        double nextStopRearS = 0.0;
        if (lat && path) {
            const double front = frontFromRear(vc) + eff_.intersections.stopLineMargin;
            for (const auto& stop : path->stops) {
                if (stop.s < lat->pathS + frontFromRear(vc) - 1.0) continue;  // already at or past it
                const bool hardStop = (stop.kind == StopKind::Signal && eff_.intersections.stopAtSignals) ||
                                      (stop.kind == StopKind::StopSign && eff_.intersections.stopAtStopSigns);
                const bool cleared = std::find(clearedStops_.begin(), clearedStops_.end(), stop.segment) !=
                                     clearedStops_.end();
                if (hardStop && !cleared) {
                    constraints.push_back({stop.s - front, 0.0});
                    if (!nextStop) {
                        nextStop = stop;
                        nextStopRearS = stop.s - front;
                    }
                } else if (stop.kind == StopKind::Yield || stop.kind == StopKind::RailCrossing) {
                    constraints.push_back({stop.s - frontFromRear(vc), eff_.intersections.yieldSpeed});
                }
            }
            VehicleLoadFactors f;
            f.trailerCount = vc.trailerCount;
            f.cargoMassKg = vc.cargoMassKg;
            f.wet = s.wipers;
            f.aggressiveness = eff_.aggressiveness;
            const SpeedPlan plan = planSpeed(eff_.planner, f, path->path, lat->pathS, target, constraints);
            target = plan.targetSpeed;
            debug_.curveRadius = plan.limitingCurveRadius;
        }

        // Arriving at an unreleased stop line: wait for the driver's throttle tap.
        if (!waitingStop_ && nextStop && lat && nextStopRearS - lat->pathS < 3.0 && std::abs(s.speed) < 0.4) {
            waitingStop_ = nextStop;
            tapStart_ = -1.0;
            const std::string msg = std::string("Waiting at ") + toString(nextStop->kind) + " - tap throttle to go";
            setMessage(msg);
            if (log_) log_->info("{}", msg);
            emit(PilotEvent::WaitingAtIntersection, msg);
        }
        if (waitingStop_) {
            const double driverThrottle = s.inputThrottle;  // ATSPilot sends no throttle while waiting
            if (driverThrottle > eff_.safety.throttleOverrideThreshold) {
                if (tapStart_ < 0.0) tapStart_ = s.time;
                if (s.time - tapStart_ > eff_.intersections.goTapMaxSeconds) {
                    disengage("Driver Override (throttle)", PilotEvent::DriverOverride);
                    return last_;
                }
            } else if (tapStart_ >= 0.0 && driverThrottle < 0.1) {
                clearedStops_.push_back(waitingStop_->segment);
                if (clearedStops_.size() > 8) clearedStops_.pop_front();
                if (log_) log_->info("Proceeding through {}", toString(waitingStop_->kind));
                setMessage("Proceeding");
                waitingStop_.reset();
                tapStart_ = -1.0;
            }
        }
        if (waitingStop_) target = 0.0;

        debug_.targetSpeed = target;
        status_.targetSpeed = target;
        status_.waitingAtIntersection = waitingStop_.has_value();

        const bool wantBrake = waitingStop_.has_value() || s.speed > target + eff_.ingame.cruise.brakeMargin ||
                               (target < 0.5 && s.speed > 0.2);
        if (eff_.ingame.useCruiseControl) {
            const auto out = cruise_.update(s.time, s.speed, s.cruiseControlSpeed, target, wantBrake,
                                            last_.brake > 0.02);
            if (out.playerSetSpeed) {
                setSpeed_ = std::max(2.0, out.playerSpeed);
                status_.setSpeed = setSpeed_;
                if (log_) {
                    log_->info("Maximum speed {:.0f} {} (game cruise control)",
                               mpsToSpeed(setSpeed_, eff_.speed.units), unitLabel(eff_.speed.units));
                }
                emit(PilotEvent::SetSpeedChanged, "Set speed changed");
            }
            if (out.cancelledExternally) {
                // The game switched its cruise control off on its own (driver input,
                // emergency brake assist): the driver must take over.
                disengage("Cruise control cancelled - take over", PilotEvent::DriverOverride);
                return last_;
            }
            cmd.buttons.merge(out.buttons);
            ownPedals = out.ownPedals;
        }
        if (ownPedals) {
            pedals = longitudinal_.update(target, s.speed, dt);
            if (waitingStop_) {
                pedals.throttle = 0.0;
                pedals.brake = std::max(pedals.brake, 0.35);
            }
        } else {
            longitudinal_.reset(0.0, 0.0);
        }
    }
    cmd.throttle = pedals.throttle;
    cmd.brake = pedals.brake;
    cmd.pedalsActive = true;
    debug_.throttleOutput = pedals.throttle;
    debug_.brakeOutput = pedals.brake;
    debug_.brakeLevel = pedals.level;

    cmd.buttons.merge(blinkers(s, path, s.time));

    status_.mode = mode_;
    last_ = cmd;
    return cmd;
}

}  // namespace atspilot

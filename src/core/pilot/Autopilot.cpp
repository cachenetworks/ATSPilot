#include "pilot/Autopilot.h"

#include <algorithm>
#include <cmath>
#include <format>

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

const char* toString(IndicationKind k) {
    switch (k) {
        case IndicationKind::LaneChange: return "lane change";
        case IndicationKind::Turn: return "turn";
        case IndicationKind::Exit: return "exit";
        case IndicationKind::Merge: return "merge";
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
    if (!inputReady_) return "Game input device not available (see game.log.txt)";
    if (s.paused) return "Game paused";
    if (s.speed < -0.5 || s.gear < 0) return "Not available in reverse";
    if (s.parkingBrake) return "Release parking brake";
    if (!s.engineEnabled) return "Engine off";
    if (!path || !path->path.valid()) return "No valid road path";
    if (wallNow - pathWall > eff_.safety.pathTimeout) return "Path stale";
    const double yaw = coords::sdkHeadingToYaw(s.headingUnit);
    const auto proj = path->path.project(coords::worldToPlan(s.worldPosition));
    if (!proj) return "No valid road path";
    // Off the lane (on the shoulder, pulling out of a lay-by) ATSPilot still engages
    // and pulls into it, up to max_join_offset_m.
    if (std::abs(proj->crossTrackError) > std::max(eff_.safety.maxCrossTrack, eff_.safety.maxJoinOffset))
        return "Truck not on the planned lane";
    if (std::abs(headingDifference(yaw, proj->yaw)) > degToRad(eff_.safety.maxHeadingErrorDeg))
        return "Truck not aligned with the road";
    return std::nullopt;
}

void Autopilot::engage(const VehicleState& s) {
    mode_ = PilotMode::Autopilot;
    // Through the input mix, ATSPilot's input is added to the driver's, and the
    // game keeps the driver's steering where it was: starting from the current
    // wheel angle would double it. While off, ATSPilot outputs nothing, so the
    // reported input is entirely the driver's.
    // Direct steering sets the truck's steering absolutely, so it starts where the
    // wheel is; through the input mix it is added to the driver's and starts at zero.
    shaper_.reset(directSteering_ ? clamp(s.effectiveSteering, -1.0, 1.0) : 0.0);
    stopAndGoSince_ = -1.0;
    baseTarget_ = -1.0;
    joinAllowance_ = 0.0;
    joinSide_ = 0;
    longitudinal_.reset(0.0, 0.0);
    cruise_ = GameCruiseManager(eff_.ingame.cruise);
    override_.reset(clamp(s.inputSteering, -1.0, 1.0));
    invertedSteeringFrames_ = 0;
    hintPath_.reset();
    waitingStop_.reset();
    tapStart_ = -1.0;
    setSpeed_ = speedToMps(eff_.speed.maxSpeed, eff_.speed.units);
    cruiseIgnoredLogged_ = false;

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
            {
                const auto proj = path->path.project(coords::worldToPlan(s.worldPosition));
                if (proj && std::abs(proj->crossTrackError) > eff_.safety.warnCrossTrack) {
                    joinAllowance_ = std::abs(proj->crossTrackError) + 1.0;
                    joinSide_ = proj->crossTrackError > 0.0 ? -1 : 1;  // lane is to the right when we are left of it
                    if (log_) log_->info("Pulling into the lane from {:.1f} m to its {}", std::abs(proj->crossTrackError),
                                         proj->crossTrackError > 0.0 ? "left" : "right");
                    setMessage("Pulling into the lane");
                }
            }
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

double Autopilot::cruiseTarget(const VehicleState& s) {
    double target = setSpeed_;
    if (eff_.speed.followSpeedLimit) {
        // The posted limit; where none is reported (some junctions, depots) the
        // last one still applies, and before any has been seen a moderate default.
        if (s.navigationSpeedLimit > 0.5) lastLimit_ = s.navigationSpeedLimit;
        const double limit = lastLimit_ > 0.0 ? lastLimit_ : speedToMps(eff_.speed.unknownLimit, eff_.speed.units);
        target = std::min(target, limit + speedToMps(eff_.speed.limitOffset, eff_.speed.units));
    }
    return std::max(0.0, target);
}

void Autopilot::logSignal(const PathStop& stop, LightState state, SignalDecision decision, bool byId) {
    const auto it = loggedSignals_.find(stop.segment);
    if (it != loggedSignals_.end() && it->second == state) return;
    if (loggedSignals_.size() > 64) loggedSignals_.clear();
    loggedSignals_[stop.segment] = state;
    const char* action = decision == SignalDecision::Go       ? "go"
                         : decision == SignalDecision::Stop    ? "stop"
                         : decision == SignalDecision::GiveWay ? "give way"
                                                               : "unknown";
    if (log_) log_->info("Traffic light {} ahead: {} (matched by {})", toString(state), action, byId ? "id" : "position");
    if (decision == SignalDecision::Stop) setMessage(std::string("Traffic light ") + toString(state));
}

void Autopilot::clearStop(std::uint32_t segment) {
    clearedStops_.push_back(segment);
    if (clearedStops_.size() > 8) clearedStops_.pop_front();
}

bool Autopilot::laneTrafficBehind(const Path& path, double s, const std::vector<WorldVehicle>& vehicles) const {
    // The lane's frame where the truck is; vehicles behind in it that would arrive
    // within 8 s are traffic to let pass before pulling in.
    const Vec2 p0 = path.positionAt(s);
    const double yaw = path.yawAt(s);
    const Vec2 dir{std::cos(yaw), std::sin(yaw)};
    const Vec2 left{-dir.y, dir.x};
    for (const auto& v : vehicles) {
        const Vec2 rel = v.position - p0;
        const double along = dot(rel, dir);
        const double lateral = dot(rel, left);
        if (along > 15.0 || along < -200.0 || std::abs(lateral) > 2.5) continue;
        if (std::cos(v.yaw - yaw) < 0.7 || v.speed < 2.0) continue;
        if (along > 0.0 || -along / v.speed < 8.0) return true;
    }
    return false;
}

int Autopilot::lightFacing() const {
    if (lightFacingVotes_ >= 3) return 1;
    if (lightFacingVotes_ <= -3) return -1;
    return 0;
}

void Autopilot::learnLightFacing(const WorldLight& light, double travelYaw) {
    const double c = std::cos(light.yaw - travelYaw);
    const int before = lightFacing();
    if (c > 0.8) lightFacingVotes_ = std::min(lightFacingVotes_ + 1, 20);
    else if (c < -0.8) lightFacingVotes_ = std::max(lightFacingVotes_ - 1, -20);
    if (lightFacing() != before && lightFacing() != 0 && log_) {
        log_->info("Traffic lights face {} their traffic; matching unnumbered lights by position",
                   lightFacing() > 0 ? "along with" : "against");
    }
}

GameButtons Autopilot::blinkers(const VehicleState& s, const PathSnapshotPtr& path, double time) {
    GameButtons b;
    if (!eff_.ingame.useBlinkers) return b;
    bool wantLeft = false, wantRight = false;
    if (path && hintPath_ == path) {
        // The truck's front on the path: the signal is on within an indication's interval.
        const double frontS = debug_.pathS + frontS_;
        for (const auto& ind : path->indications) {
            if (ind.sStart > frontS) break;
            if (frontS <= ind.sEnd) {
                wantLeft = ind.side > 0;
                wantRight = ind.side < 0;
                if (ind.sStart != activeIndication_) {
                    activeIndication_ = ind.sStart;
                    if (log_) log_->debug("Signalling {} for {}", ind.side > 0 ? "left" : "right", toString(ind.kind));
                }
                break;
            }
        }
    }
    if (joinAllowance_ > 0.0 && joinSide_ != 0) {
        wantLeft = joinSide_ > 0;
        wantRight = joinSide_ < 0;
    }

    // Preferred: the game's hold controls (lblinkerh/rblinkerh, as ETS2LA's
    // controller uses). The signal is on exactly while held, so it can never be
    // left on. If the game does not show the signal while held, fall back to
    // toggling the ordinary blinker switch.
    indicatorHold_ = 0;
    if (holdIndicators_) {
        const int want = wantLeft ? 1 : wantRight ? -1 : 0;
        indicatorHold_ = want;
        const bool shown = (want > 0 && s.blinkerLeft) || (want < 0 && s.blinkerRight);
        if (want == 0 || shown) {
            holdSince_ = -1.0;
            if (shown) holdConfirmed_ = true;
        } else if (holdSince_ < 0.0) {
            holdSince_ = time;
        } else if (!holdConfirmed_ && time - holdSince_ > 1.5) {
            holdIndicators_ = false;
            indicatorHold_ = 0;
            if (log_) log_->info("Hold-type turn signal controls have no effect; using the blinker switch");
        }
        if (holdIndicators_) return b;
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

    // After a game hitch (no frames for a while) the planning thread needs a
    // moment to catch up; a path that was fresh when the hitch began stays
    // usable meanwhile instead of counting as lost.
    if (lastUpdateWall_ > -1e8 && wallNow - lastUpdateWall_ > 0.3) {
        hitchFrom_ = lastUpdateWall_;
        hitchUntil_ = wallNow + eff_.safety.pathTimeout;
    }
    lastUpdateWall_ = wallNow;
    if (wallNow < hitchUntil_ && hitchFrom_ - pathWall <= eff_.safety.pathTimeout) {
        pathWall = std::max(pathWall, wallNow - 0.5 * eff_.safety.pathTimeout);
    }

    if (vc.valid || vc.cargoMassKg > 0.0) updateProfile(vc);
    status_.telemetryConnected = s.valid;
    status_.speed = s.speed;
    status_.setSpeed = setSpeed_;
    status_.mode = mode_;
    status_.gameCruiseActive = s.cruiseControlSpeed > 0.1;
    status_.cruiseSetSpeed = s.cruiseControlSpeed;
    status_.waitingAtIntersection = waitingStop_.has_value();
    status_.trafficAware = world_ && world_->valid;
    frontS_ = frontFromRear(vc);
    status_.directSteering = directSteering_;
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

    // With direct steering none of ATSPilot's steering reaches the input mix, so
    // all reported steering input is the driver's.
    ControlCommand sent = last_;
    if (directSteering_) sent.steering = 0.0;
    OverrideKind ov = override_.update(s, sent);
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
        if (out.valid && std::abs(out.crossTrackError) > crossTrackLimit() && hint) {
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
        } else if (std::abs(lat->crossTrackError) > crossTrackLimit()) {
            enterEmergency("Dangerous path deviation");
        } else if (std::abs(lat->headingError) > degToRad(eff_.safety.maxHeadingErrorDeg)) {
            enterEmergency("Heading deviates from path");
        } else if (joinAllowance_ <= 0.0 && std::abs(lat->crossTrackError) > eff_.safety.warnCrossTrack &&
                   s.time - lastCrossTrackWarn_ > 5.0) {
            lastCrossTrackWarn_ = s.time;
            if (log_) log_->warn("Cross-track error {:.2f} m", lat->crossTrackError);
        }
    }

    double desiredWheel = 0.0;
    // Pulling into the lane: the allowed deviation shrinks as the truck closes in.
    if (lat && joinAllowance_ > 0.0) {
        joinAllowance_ = std::min(joinAllowance_, std::abs(lat->crossTrackError) + 1.0);
        if (std::abs(lat->crossTrackError) < eff_.safety.warnCrossTrack) {
            joinAllowance_ = 0.0;
            joinSide_ = 0;
            if (log_) log_->info("In the lane");
            setMessage("Autopilot Engaged");
        }
    }
    if (lat && (mode_ != PilotMode::EmergencyStop || std::abs(lat->crossTrackError) <= crossTrackLimit())) {
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
        // A lower speed limit is approached gently rather than braked for at once
        // (the sign is only reported once passed); the player's maximum applies as set.
        {
            const double limit = cruiseTarget(s);
            if (baseTarget_ < 0.0 || limit >= baseTarget_) baseTarget_ = limit;
            else baseTarget_ = std::max(limit, baseTarget_ - eff_.planner.limitDecel * dt);
        }
        double target = std::min(setSpeed_, baseTarget_);
        std::vector<SpeedConstraint> constraints;
        std::optional<PathStop> nextStop;  // the next stop that needs the driver's throttle tap
        double nextStopRearS = 0.0;
        double strongBrake = 0.0;
        status_.leadDistance = -1.0;
        status_.signalState.clear();
        const bool aware = world_ && world_->valid;  // live traffic and light states available
        if (lat && path) {
            const double toFront = frontFromRear(vc);
            const double frontS = lat->pathS + toFront;
            const double front = toFront + eff_.intersections.stopLineMargin;

            // Traffic on, or about to cross, the path. Constraints are produced at
            // the truck's front and moved to the rear axle the planner works with.
            TrafficPicture traffic;
            if (aware && !world_->vehicles.empty()) {
                traffic = assessTraffic(path->path, frontS, std::max(0.0, s.speed), world_->vehicles, eff_.traffic);
                for (auto c : traffic.constraints) {
                    c.s -= toFront;
                    constraints.push_back(c);
                }
                if (traffic.nearest) {
                    status_.leadDistance = traffic.nearest->s - frontS;
                    status_.leadSpeed = traffic.nearest->speed;
                }
                // More than comfortable braking needed: brake directly rather than
                // waiting for the speed loop to catch up. A crossing that is only
                // predicted must persist briefly first (unless close), so one odd
                // reading never slams the brakes on.
                if (traffic.requiredDecel > 2.5) {
                    strongBrake = clamp(traffic.requiredDecel / 7.0, 0.35, 0.9);
                    if (traffic.nearest->crossing) {
                        if (traffic.nearest->id != crossingId_) {
                            crossingId_ = traffic.nearest->id;
                            crossingSince_ = s.time;
                        }
                        if (s.time - crossingSince_ < 0.4 && traffic.nearest->s - frontS > 15.0) strongBrake = 0.0;
                    } else {
                        crossingId_ = -1;
                    }
                } else {
                    crossingId_ = -1;
                }
            }
            const bool junctionBusy = traffic.nearest && traffic.nearest->s - frontS < 40.0;

            // Pulling into the lane: slowly, and only when nothing is coming up behind in it.
            if (joinAllowance_ > 0.0) {
                target = std::min(target, 8.0);
                if (aware && laneTrafficBehind(path->path, lat->pathS, world_->vehicles)) {
                    target = 0.0;
                    setMessage("Waiting for traffic to pass");
                }
            }

            for (const auto& stop : path->stops) {
                if (stop.s < frontS - 1.0) continue;  // already at or past it
                const bool cleared = std::find(clearedStops_.begin(), clearedStops_.end(), stop.segment) !=
                                     clearedStops_.end();
                if (cleared) continue;
                const double distance = stop.s - eff_.intersections.stopLineMargin - frontS;

                // Without live states: stop and wait for the driver's throttle tap.
                auto waitForTap = [&] {
                    constraints.push_back({stop.s - front, 0.0});
                    if (!nextStop) {
                        nextStop = stop;
                        nextStopRearS = stop.s - front;
                    }
                };
                // Stop fully at the line, then go once nothing is on or crossing the
                // path nearby: stop signs, and lights whose state cannot be read.
                auto stopAndGo = [&](const char* what) {
                    constraints.push_back({stop.s - front, 0.0});
                    const bool atLine = distance < 3.0 && std::abs(s.speed) < 0.3;
                    if (!atLine) {
                        if (stopAndGoSegment_ == stop.segment) stopAndGoSince_ = -1.0;
                        return;
                    }
                    if (stopAndGoSegment_ != stop.segment || stopAndGoSince_ < 0.0) {
                        stopAndGoSegment_ = stop.segment;
                        stopAndGoSince_ = s.time;
                        setMessage("Waiting for the junction to clear");
                    } else if (s.time - stopAndGoSince_ > 1.5 && !junctionBusy) {
                        clearStop(stop.segment);
                        stopAndGoSince_ = -1.0;
                        if (log_) log_->info("Junction clear, proceeding from {}", what);
                        setMessage("Proceeding");
                    }
                };

                if (stop.kind == StopKind::Signal && eff_.intersections.stopAtSignals) {
                    if (!aware) {
                        waitForTap();
                        continue;
                    }
                    const double lineS = std::clamp(stop.s, 0.0, path->path.length());
                    const Vec2 line = path->path.positionAt(lineS);
                    const LightMatch match = lightForStop(stop, path->path, world_->lights, eff_.traffic, lightFacing());
                    SignalDecision d = SignalDecision::Unknown;
                    if (match.light) {
                        if (match.byId) learnLightFacing(*match.light, path->path.yawAt(lineS));
                        d = decideSignal(match.light->state, distance, std::max(0.0, s.speed), eff_.traffic);
                        if (status_.signalState.empty()) status_.signalState = toString(match.light->state);
                        logSignal(stop, match.light->state, d, match.byId);
                    } else if (s.time - lastGreenTime_ < 30.0 && atspilot::distance(line, lastGreenLine_) < 60.0) {
                        // A further signal lane of the junction just entered on green.
                        d = SignalDecision::Go;
                    } else if (stop.segment != unmatchedSignalLogged_ && distance < 80.0 && log_) {
                        unmatchedSignalLogged_ = stop.segment;
                        std::string ids;
                        for (const auto& l : world_->lights) {
                            if (atspilot::distance(l.position, line) > 80.0) continue;
                            ids += std::format(" {}@{:.0f}m:{}", l.semaphoreId, atspilot::distance(l.position, line),
                                               toString(l.state));
                        }
                        log_->info("No live light for the signal ahead (semaphore {}), treating it as an all-way "
                                   "stop; nearby:{}",
                                   stop.semaphoreId, ids.empty() ? " none" : ids);
                    }
                    if (d == SignalDecision::Go) {
                        lastGreenLine_ = line;
                        lastGreenTime_ = s.time;
                        if (waitingStop_ && waitingStop_->segment == stop.segment) waitingStop_.reset();
                        continue;
                    }
                    if (d == SignalDecision::GiveWay) {
                        constraints.push_back({stop.s - toFront, eff_.intersections.yieldSpeed});
                        continue;
                    }
                    if (d == SignalDecision::Stop) {
                        constraints.push_back({stop.s - front, 0.0});
                        continue;
                    }
                    stopAndGo("traffic light (state unknown)");
                } else if (stop.kind == StopKind::StopSign && eff_.intersections.stopAtStopSigns) {
                    if (!aware) waitForTap();
                    else stopAndGo("stop sign");
                } else if (stop.kind == StopKind::Yield || stop.kind == StopKind::RailCrossing) {
                    constraints.push_back({stop.s - toFront, eff_.intersections.yieldSpeed});
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

        const bool wantBrake = waitingStop_.has_value() || strongBrake > 0.0 ||
                               s.speed > target + eff_.ingame.cruise.brakeMargin ||
                               (target < 0.5 && s.speed > 0.2);
        if (eff_.ingame.useCruiseControl) {
            const auto out = cruise_.update(s.time, s.speed, s.cruiseControlSpeed, target, wantBrake,
                                            last_.brake > 0.02);
            if (out.playerSetSpeed && eff_.speed.cruiseSetsMax) {
                setSpeed_ = std::min(std::max(2.0, out.playerSpeed), speedToMps(eff_.speed.maxSpeed, eff_.speed.units));
                status_.setSpeed = setSpeed_;
                if (log_) {
                    log_->info("Maximum speed {:.0f} {} (game cruise control)",
                               mpsToSpeed(setSpeed_, eff_.speed.units), unitLabel(eff_.speed.units));
                }
                emit(PilotEvent::SetSpeedChanged, "Set speed changed");
            } else if (out.playerSetSpeed && !cruiseIgnoredLogged_) {
                cruiseIgnoredLogged_ = true;
                if (log_) {
                    log_->info("Driving at the posted speed limit; the game's cruise set speed ({:.0f} {}) is not a "
                               "maximum (speed.cruise_sets_max)",
                               mpsToSpeed(out.playerSpeed, eff_.speed.units), unitLabel(eff_.speed.units));
                }
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
            // Routine slowing (curves ahead, a lower limit) is planned with comfortable
            // deceleration, so it never needs more than light braking; the speed
            // error alone would otherwise ask for strong braking.
            if (strongBrake <= 0.0 && !waitingStop_ && target > 2.0) {
                pedals.brake = std::min(pedals.brake, eff_.cruise.routineBrake);
            }
            if (strongBrake > 0.0 && s.speed > 0.3) {
                pedals.throttle = 0.0;
                pedals.brake = std::max(pedals.brake, strongBrake);
            }
            // Hold the truck while queued or held at a light rather than creeping.
            if (target < 0.2 && std::abs(s.speed) < 0.5) {
                pedals.throttle = 0.0;
                pedals.brake = std::max(pedals.brake, 0.3);
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
    cmd.indicator = indicatorHold_;

    status_.mode = mode_;
    last_ = cmd;
    return cmd;
}

}  // namespace atspilot

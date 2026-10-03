#include "sim/Scenario.h"

#include <cmath>

#include "math/Coordinates.h"
#include "math/MathUtil.h"
#include "pilot/Autopilot.h"

namespace atspilot::sim {

ScenarioResult runScenario(const Path& path, const Config& cfg, const SimParams& simParams,
                           const ScenarioOptions& options) {
    ScenarioResult r;
    r.name = options.name;

    VehicleSim vehicle(simParams);
    const double yaw0 = path.yawAt(0.0);
    const Vec2 start = path.positionAt(0.0) + coords::yawToDirection(yaw0 + kPi / 2.0) * options.lateralOffset;
    vehicle.reset(start, yaw0 + options.headingOffset, options.initialSpeed);

    auto snapshot = std::make_shared<PathSnapshot>();
    snapshot->path = path;
    snapshot->roadName = "synthetic";
    PathSnapshotPtr snap = snapshot;

    Config c = cfg;
    if (options.setSpeedOverride > 0.0) c.speed.maxSpeed = mpsToSpeed(options.setSpeedOverride, c.speed.units);
    Autopilot pilot(c);
    pilot.setEventCallback([&](PilotEvent e, const std::string& msg) {
        if (e == PilotEvent::Disengaged || e == PilotEvent::DriverOverride || e == PilotEvent::EmergencyBraking) {
            if (!r.disengaged) r.disengageReason = msg;
            r.disengaged = true;
        }
    });

    const double dt = 1.0 / options.rate;
    VehicleState s = vehicle.state();
    pilot.request(options.engage, s, vehicle.config(), snap, 0.0, 0.0);
    if (pilot.mode() == PilotMode::Off) {
        r.disengaged = true;
        r.disengageReason = pilot.status().statusMessage;
        return r;
    }
    if (options.setSpeedOverride > 0.0) {
        // Engaging while moving holds the current speed; step towards the requested one.
        while (pilot.setSpeed() + 0.1 < options.setSpeedOverride) {
            const double before = pilot.setSpeed();
            pilot.request(PilotRequest::SpeedUp, s, vehicle.config(), snap, 0.0, 0.0);
            if (pilot.setSpeed() <= before) break;
        }
        while (pilot.setSpeed() - 0.1 > options.setSpeedOverride) {
            const double before = pilot.setSpeed();
            pilot.request(PilotRequest::SpeedDown, s, vehicle.config(), snap, 0.0, 0.0);
            if (pilot.setSpeed() >= before) break;
        }
    }

    double sumSq = 0.0;
    int samples = 0;
    double prevSteer = 0.0;
    double prevRate = 0.0;
    std::size_t hint = 0;

    while (vehicle.time() < options.maxTime) {
        s = vehicle.state();
        const double now = s.time;
        const ControlCommand cmd = pilot.update(s, vehicle.config(), snap, now, now, now);
        if (options.observer) options.observer(s, cmd, pilot.debug());
        if (pilot.mode() == PilotMode::Off || pilot.mode() == PilotMode::EmergencyStop) {
            if (!r.disengaged) {
                r.disengaged = true;
                r.disengageReason = pilot.status().statusMessage;
            }
            break;
        }

        const auto proj = path.project(vehicle.rearAxle(), hint, 30);
        if (proj) {
            hint = proj->index;
            const double e = std::abs(proj->crossTrackError);
            r.maxCrossTrack = std::max(r.maxCrossTrack, e);
            sumSq += e * e;
            ++samples;
            r.distance = proj->s;
            if (now > 15.0) {
                r.maxSpeedError = std::max(r.maxSpeedError, std::abs(pilot.debug().targetSpeed - s.speed));
            }
            if (proj->s >= path.length() - 60.0) {
                r.completed = true;
                break;
            }
        }
        const double latAccel = std::abs(vehicle.speed() * s.localAngularVelocity.y * kTwoPi);
        r.maxLateralAccel = std::max(r.maxLateralAccel, latAccel);

        const double rate = cmd.steering - prevSteer;
        if (std::abs(rate) > 1e-4) {
            if (rate * prevRate < 0.0) ++r.steeringReversals;
            prevRate = rate;
        }
        prevSteer = cmd.steering;
        vehicle.step(cmd, dt);
    }
    r.rmsCrossTrack = samples ? std::sqrt(sumSq / samples) : 0.0;
    return r;
}

}  // namespace atspilot::sim

#include "sim/VehicleSim.h"

#include <cmath>

#include "math/Coordinates.h"
#include "math/MathUtil.h"

namespace atspilot::sim {

void VehicleSim::reset(const Vec2& rearAxle, double yaw, double speed) {
    rear_ = rearAxle;
    yaw_ = yaw;
    v_ = speed;
    wheel_ = 0.0;
    t_ = 0.0;
    lastCmd_ = {};
}

void VehicleSim::step(const ControlCommand& cmd, double dt) {
    lastCmd_ = cmd;
    const double steerCmd = cmd.active && cmd.steerActive ? cmd.steering : 0.0;
    const double throttle = cmd.active && cmd.pedalsActive ? cmd.throttle : 0.0;
    const double brake = cmd.active && cmd.pedalsActive ? cmd.brake : 0.0;

    const double targetWheel = clamp(steerCmd, -1.0, 1.0) * p_.maxWheelAngle;
    wheel_ += (targetWheel - wheel_) * (dt / (p_.steeringLag + dt));

    const double powerFalloff = v_ > p_.maxPowerSpeed ? p_.maxPowerSpeed / v_ : 1.0;
    double accel = throttle * p_.maxAccel * powerFalloff / p_.massFactor;
    accel -= brake * p_.maxBrakeDecel;
    accel -= p_.dragCoeff * v_ * v_ + (v_ > 0.01 ? p_.rolling : 0.0);
    v_ = std::max(0.0, v_ + accel * dt);

    yaw_ = normalizeAngle(yaw_ + v_ / p_.wheelbase * std::tan(wheel_) * dt);
    rear_ += coords::yawToDirection(yaw_) * (v_ * dt);
    t_ += dt;
}

VehicleState VehicleSim::state() const {
    VehicleState s;
    s.valid = true;
    s.paused = false;
    s.time = t_;
    const Vec2 origin = rear_ + coords::yawToDirection(yaw_) * p_.rearAxleZ;
    s.worldPosition = coords::planToWorld(origin, 0.0);
    s.headingUnit = coords::yawToSdkHeading(yaw_);
    s.speed = v_;
    s.localVelocity = {0.0, 0.0, -v_};
    s.localAngularVelocity = {0.0, v_ / p_.wheelbase * std::tan(wheel_) / kTwoPi, 0.0};
    const double steerCmd = lastCmd_.active && lastCmd_.steerActive ? lastCmd_.steering : 0.0;
    s.inputSteering = steerCmd;
    s.inputThrottle = lastCmd_.active && lastCmd_.pedalsActive ? lastCmd_.throttle : 0.0;
    s.inputBrake = lastCmd_.active && lastCmd_.pedalsActive ? lastCmd_.brake : 0.0;
    s.effectiveSteering = wheel_ / p_.maxWheelAngle;
    s.effectiveThrottle = s.inputThrottle;
    s.effectiveBrake = s.inputBrake;
    s.steerableWheelAngle = wheel_;
    s.gear = v_ > 0.1 ? 6 : 1;
    s.displayedGear = s.gear;
    s.engineEnabled = true;
    s.electricEnabled = true;
    return s;
}

VehicleConfig VehicleSim::config() const {
    VehicleConfig c;
    c.valid = true;
    c.truckId = "sim";
    c.wheelbase = p_.wheelbase;
    c.frontAxleZ = p_.frontAxleZ;
    c.rearAxleZ = p_.rearAxleZ;
    c.trailerCount = p_.trailerCount;
    return c;
}

namespace {

// Appends an arc continuing from the path's current end and heading.
void appendArc(Path& path, double& yaw, double radius, double angle, double spacing) {
    const Vec2 start = path.points().back().pos;
    const double dir = sign(angle);
    const Vec2 center = start + coords::yawToDirection(yaw + dir * kPi / 2.0) * radius;
    const double startAngle = yaw - dir * kPi / 2.0;
    const int steps = std::max(1, static_cast<int>(std::ceil(std::abs(angle) * radius / spacing)));
    for (int i = 1; i <= steps; ++i) {
        const double a = startAngle + angle * i / steps;
        path.append(center + coords::yawToDirection(a) * radius);
    }
    yaw = normalizeAngle(yaw + angle);
}

void appendStraight(Path& path, double yaw, double length, double spacing) {
    const Vec2 start = path.points().back().pos;
    const int steps = std::max(1, static_cast<int>(std::ceil(length / spacing)));
    for (int i = 1; i <= steps; ++i) path.append(start + coords::yawToDirection(yaw) * (length * i / steps));
}

}  // namespace

Path makeStraight(double length, double spacing) {
    Path p;
    p.append({0.0, 0.0});
    appendStraight(p, 0.0, length, spacing);
    return p;
}

Path makeArc(double straightBefore, double radius, double angleRad, double straightAfter, double spacing) {
    Path p;
    double yaw = 0.0;
    p.append({0.0, 0.0});
    appendStraight(p, yaw, straightBefore, spacing);
    appendArc(p, yaw, radius, angleRad, spacing);
    appendStraight(p, yaw, straightAfter, spacing);
    return p;
}

Path makeSCurve(double straightBefore, double radius, double angleRad, double straightAfter, double spacing) {
    Path p;
    double yaw = 0.0;
    p.append({0.0, 0.0});
    appendStraight(p, yaw, straightBefore, spacing);
    appendArc(p, yaw, radius, angleRad, spacing);
    appendArc(p, yaw, radius, -angleRad, spacing);
    appendStraight(p, yaw, straightAfter, spacing);
    return p;
}

Path makeHighwayRamp(double spacing) {
    Path p;
    double yaw = 0.0;
    p.append({0.0, 0.0});
    appendStraight(p, yaw, 400.0, spacing);
    appendArc(p, yaw, 600.0, degToRad(-6.0), spacing);   // diverge right
    appendArc(p, yaw, 600.0, degToRad(6.0), spacing);
    appendStraight(p, yaw, 150.0, spacing);
    appendArc(p, yaw, 70.0, degToRad(-270.0), spacing);  // cloverleaf loop
    appendStraight(p, yaw, 300.0, spacing);
    return p;
}

}  // namespace atspilot::sim

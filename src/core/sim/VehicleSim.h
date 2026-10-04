#pragma once

#include <string>
#include <vector>

#include "path/Path.h"
#include "pilot/PilotTypes.h"
#include "pilot/VehicleState.h"

namespace atspilot::sim {

// Kinematic bicycle model with actuator lag and simple longitudinal dynamics.
// Development aid only: it validates controller logic, not ATS physics.
struct SimParams {
    double wheelbase = 6.0;
    double frontAxleZ = -1.0;
    double rearAxleZ = 5.0;
    double maxWheelAngle = 0.61;     // rad at steering = 1
    double steeringLag = 0.15;       // s, first-order lag of the steering rack
    double maxAccel = 1.2;           // m/s^2 at full throttle, low speed
    double maxPowerSpeed = 15.0;     // m/s above which acceleration falls off (power limited)
    double maxBrakeDecel = 6.0;      // m/s^2 at full brake
    double dragCoeff = 0.00045;      // aerodynamic, per (m/s)^2
    double rolling = 0.08;           // m/s^2
    double massFactor = 1.0;         // >1 for heavy loads, scales acceleration down
    int trailerCount = 0;
    // Model of the game's cruise control: switches on at >= cruiseMinSpeed, steps
    // by cruiseStep per +/- press, and is cancelled by braking.
    double cruiseMinSpeed = 8.33;
    double cruiseStep = 1.0 / 3.6;
};

class VehicleSim {
public:
    explicit VehicleSim(SimParams p = {}) : p_(p) {}

    // Places the rear axle at `rearAxle` facing `yaw` at `speed`.
    void reset(const Vec2& rearAxle, double yaw, double speed);
    void step(const ControlCommand& cmd, double dt);

    VehicleState state() const;
    VehicleConfig config() const;

    Vec2 rearAxle() const { return rear_; }
    double yaw() const { return yaw_; }
    double speed() const { return v_; }
    double time() const { return t_; }
    double cruiseSet() const { return cruiseSet_; }
    // Lead-vehicle style slowdown of the game's (adaptive) cruise control for tests:
    // while set, cruise holds at most this speed.
    void setCruiseCap(double v) { cruiseCap_ = v; }
    // Simulates the game cancelling its cruise control on its own.
    void cancelCruise() { cruiseSet_ = 0.0; }
    // Blinker switch state, toggled by button presses.
    bool blinkerLeft() const { return blinkerLeft_; }
    bool blinkerRight() const { return blinkerRight_; }
    int quickParkPresses() const { return quickParks_; }

private:
    SimParams p_;
    Vec2 rear_;
    double yaw_ = 0.0;
    double v_ = 0.0;
    double wheel_ = 0.0;  // actual road-wheel angle
    double t_ = 0.0;
    ControlCommand lastCmd_;
    double cruiseSet_ = 0.0;
    double cruiseCap_ = 1e9;
    double cruiseIntegral_ = 0.0;
    bool blinkerLeft_ = false;
    bool blinkerRight_ = false;
    int quickParks_ = 0;
};

// Synthetic test geometry.
Path makeStraight(double length, double spacing = 4.0);
Path makeArc(double straightBefore, double radius, double angleRad, double straightAfter, double spacing = 4.0);
Path makeSCurve(double straightBefore, double radius, double angleRad, double straightAfter, double spacing = 4.0);
// Highway exit: straight, gentle diverge, tight 270 degree loop, merge straight.
Path makeHighwayRamp(double spacing = 4.0);

struct ScenarioResult {
    std::string name;
    double maxCrossTrack = 0.0;
    double rmsCrossTrack = 0.0;
    double maxLateralAccel = 0.0;
    double maxSpeedError = 0.0;   // vs. target after settling
    int steeringReversals = 0;    // sign changes of the steering rate, an oscillation indicator
    double distance = 0.0;
    bool completed = false;
    bool disengaged = false;
    std::string disengageReason;
};

}  // namespace atspilot::sim

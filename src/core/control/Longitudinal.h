#pragma once

#include "control/Pid.h"

namespace atspilot {

enum class BrakeLevel { None, Minor, Normal, Strong, Emergency };

const char* toString(BrakeLevel b);

struct LongitudinalParams {
    PidGains gains{0.30, 0.04, 0.08};
    double integralLimit = 8.0;
    // PID demand below -brakeEnter switches to braking, above -brakeExit back to throttle.
    // The band between is coasting, where engine drag handles small corrections.
    double brakeEnter = 0.10;
    double brakeExit = 0.03;
    double throttleRate = 1.2;  // pedal units per second
    double brakeRate = 0.8;
    double maxNormalBrake = 0.45;
    double maxStrongBrake = 0.75;
    double emergencyBrake = 0.85;
    double emergencyBrakeRate = 0.6;
};

struct PedalCommand {
    double throttle = 0.0;
    double brake = 0.0;
    BrakeLevel level = BrakeLevel::None;
};

class LongitudinalController {
public:
    explicit LongitudinalController(LongitudinalParams p = {});

    void setParams(const LongitudinalParams& p);
    void reset(double currentThrottle = 0.0, double currentBrake = 0.0);

    PedalCommand update(double targetSpeed, double currentSpeed, double dt);
    // Controlled emergency stop: no throttle, brake ramps up to `emergencyBrake`.
    PedalCommand emergency(double dt);

    const PedalCommand& last() const { return out_; }

private:
    LongitudinalParams p_;
    Pid pid_;
    bool braking_ = false;
    PedalCommand out_;
};

}  // namespace atspilot

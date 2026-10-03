#include "control/Longitudinal.h"

#include "math/MathUtil.h"

namespace atspilot {

const char* toString(BrakeLevel b) {
    switch (b) {
        case BrakeLevel::None: return "none";
        case BrakeLevel::Minor: return "minor";
        case BrakeLevel::Normal: return "normal";
        case BrakeLevel::Strong: return "strong";
        case BrakeLevel::Emergency: return "emergency";
    }
    return "?";
}

LongitudinalController::LongitudinalController(LongitudinalParams p) { setParams(p); }

void LongitudinalController::setParams(const LongitudinalParams& p) {
    p_ = p;
    pid_.setGains(p.gains);
    pid_.setOutputLimits(-1.0, 1.0);
    pid_.setIntegralLimit(p.integralLimit);
}

void LongitudinalController::reset(double currentThrottle, double currentBrake) {
    pid_.preload(currentThrottle - currentBrake);
    braking_ = currentBrake > 0.0;
    out_ = {currentThrottle, currentBrake, BrakeLevel::None};
}

PedalCommand LongitudinalController::update(double targetSpeed, double currentSpeed, double dt) {
    const double demand = pid_.update(targetSpeed, currentSpeed, dt);

    if (braking_ && demand > -p_.brakeExit) braking_ = false;
    else if (!braking_ && demand < -p_.brakeEnter) braking_ = true;

    double throttleTarget = 0.0;
    double brakeTarget = 0.0;
    BrakeLevel level = BrakeLevel::None;
    if (braking_) {
        brakeTarget = clamp(-demand - p_.brakeExit, 0.0, p_.maxStrongBrake);
        if (brakeTarget <= 0.05) level = BrakeLevel::Minor;
        else if (brakeTarget <= p_.maxNormalBrake) level = BrakeLevel::Normal;
        else level = BrakeLevel::Strong;
    } else if (demand > 0.0) {
        throttleTarget = demand;
    }

    // Pedals never jump: releasing one pedal is fast, applying is rate-limited.
    out_.throttle = throttleTarget < out_.throttle ? approach(out_.throttle, throttleTarget, 3.0 * dt)
                                                   : approach(out_.throttle, throttleTarget, p_.throttleRate * dt);
    out_.brake = brakeTarget < out_.brake ? approach(out_.brake, brakeTarget, 2.0 * dt)
                                          : approach(out_.brake, brakeTarget, p_.brakeRate * dt);
    out_.level = level;
    return out_;
}

PedalCommand LongitudinalController::emergency(double dt) {
    out_.throttle = 0.0;
    out_.brake = approach(out_.brake, p_.emergencyBrake, p_.emergencyBrakeRate * dt);
    out_.level = BrakeLevel::Emergency;
    braking_ = true;
    return out_;
}

}  // namespace atspilot

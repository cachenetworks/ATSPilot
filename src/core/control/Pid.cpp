#include "control/Pid.h"

#include "math/MathUtil.h"

namespace atspilot {

void Pid::reset() {
    integral_ = 0.0;
    lastMeasurement_ = 0.0;
    lastError_ = 0.0;
    hasLast_ = false;
}

void Pid::preload(double output) {
    reset();
    if (gains_.ki > 1e-9) integral_ = clamp(output / gains_.ki, -integralLimit_, integralLimit_);
}

double Pid::update(double setpoint, double measurement, double dt) {
    if (dt <= 0.0) dt = 1e-3;
    const double error = setpoint - measurement;
    const double derivative = hasLast_ ? -(measurement - lastMeasurement_) / dt : 0.0;

    const double unclamped = gains_.kp * error + gains_.ki * integral_ + gains_.kd * derivative;
    // Only integrate when it would not push an already saturated output further.
    const bool saturatedHigh = unclamped >= outMax_ && error > 0.0;
    const bool saturatedLow = unclamped <= outMin_ && error < 0.0;
    if (!saturatedHigh && !saturatedLow) {
        integral_ = clamp(integral_ + error * dt, -integralLimit_, integralLimit_);
    }

    lastMeasurement_ = measurement;
    lastError_ = error;
    hasLast_ = true;
    return clamp(gains_.kp * error + gains_.ki * integral_ + gains_.kd * derivative, outMin_, outMax_);
}

}  // namespace atspilot

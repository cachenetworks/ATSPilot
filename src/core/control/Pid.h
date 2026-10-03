#pragma once

namespace atspilot {

struct PidGains {
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
};

// PID with conditional-integration anti-windup and derivative on measurement,
// so set-point steps do not produce derivative kicks.
class Pid {
public:
    Pid() = default;
    Pid(PidGains gains, double outMin, double outMax) : gains_(gains), outMin_(outMin), outMax_(outMax) {}

    void setGains(PidGains gains) { gains_ = gains; }
    void setOutputLimits(double outMin, double outMax) { outMin_ = outMin; outMax_ = outMax; }
    void setIntegralLimit(double limit) { integralLimit_ = limit; }
    void reset();
    // Seeds the integrator so the next output starts at `output` (bumpless engagement).
    void preload(double output);

    double update(double setpoint, double measurement, double dt);

    double integral() const { return integral_; }
    double lastError() const { return lastError_; }

private:
    PidGains gains_;
    double outMin_ = -1.0;
    double outMax_ = 1.0;
    double integralLimit_ = 1.0;
    double integral_ = 0.0;
    double lastMeasurement_ = 0.0;
    double lastError_ = 0.0;
    bool hasLast_ = false;
};

}  // namespace atspilot

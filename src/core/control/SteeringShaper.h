#pragma once

namespace atspilot {

struct SteeringShaperParams {
    double maxRate = 1.5;              // normalized steering units per second at low speed
    double highSpeedRateScale = 0.35;  // fraction of maxRate kept at highSpeed
    double highSpeed = 25.0;           // m/s
    double smoothingTime = 0.08;       // s, first-order low-pass time constant
    double maxLateralAccel = 2.5;      // m/s^2 used to bound commanded curvature
    double authorityMargin = 1.6;      // multiplier on the curvature bound for corrections
    double trailerConservatism = 0.75; // rate multiplier per extra articulation (trailer)
};

// Converts a desired road-wheel angle into a normalized steering command
// [-1, 1] (positive = left) and limits how fast and how far it may move.
class SteeringShaper {
public:
    explicit SteeringShaper(SteeringShaperParams p = {}) : p_(p) {}

    void setParams(const SteeringShaperParams& p) { p_ = p; }
    void reset(double current) { output_ = current; filtered_ = current; }

    double update(double desiredWheelAngle, double maxWheelAngle, double speed, double wheelbase, int trailerCount,
                  double dt);

    double output() const { return output_; }

    // Largest normalized steering allowed at `speed` given the lateral acceleration bound.
    double authorityLimit(double maxWheelAngle, double speed, double wheelbase) const;

private:
    SteeringShaperParams p_;
    double output_ = 0.0;
    double filtered_ = 0.0;
};

}  // namespace atspilot

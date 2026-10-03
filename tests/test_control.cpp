#include "TestSupport.h"

#include "control/Lateral.h"
#include "control/Longitudinal.h"
#include "control/Pid.h"
#include "control/SpeedPlanner.h"
#include "control/SteeringShaper.h"
#include "math/MathUtil.h"
#include "sim/VehicleSim.h"

using namespace atspilot;
using doctest::Approx;

TEST_CASE("PID proportional, integral and derivative-on-measurement") {
    Pid p({1.0, 0.0, 0.0}, -10, 10);
    CHECK(p.update(5.0, 3.0, 0.1) == Approx(2.0));

    Pid i({0.0, 1.0, 0.0}, -10, 10);
    i.setIntegralLimit(100);
    i.update(1.0, 0.0, 0.5);
    CHECK(i.update(1.0, 0.0, 0.5) == Approx(1.0));

    // A set-point step produces no derivative kick; a measurement change does.
    Pid d({0.0, 0.0, 1.0}, -10, 10);
    d.update(0.0, 0.0, 0.1);
    CHECK(d.update(5.0, 0.0, 0.1) == Approx(0.0));
    CHECK(d.update(5.0, 1.0, 0.1) == Approx(-10.0));
}

TEST_CASE("PID anti-windup stops integrating while saturated") {
    Pid p({1.0, 1.0, 0.0}, -1, 1);
    p.setIntegralLimit(1000);
    for (int k = 0; k < 100; ++k) p.update(10.0, 0.0, 0.1);
    const double integralWhileSaturated = p.integral();
    CHECK(integralWhileSaturated < 1.0);
    // Once the error flips, the output leaves saturation promptly.
    CHECK(p.update(-1.0, 0.0, 0.1) < 0.5);
}

TEST_CASE("PID preload gives bumpless start") {
    Pid p({0.5, 0.2, 0.0}, -1, 1);
    p.setIntegralLimit(10.0);
    p.preload(0.4);
    CHECK(p.update(10.0, 10.0, 0.05) == Approx(0.4).epsilon(0.01));
}

TEST_CASE("pure pursuit steers toward the target") {
    const double L = 6.0;
    // Target straight ahead: no steering.
    CHECK(purePursuitWheelAngle({0, 0}, 0.0, {20, 0}, L) == Approx(0.0));
    // Target to the left: positive (left) wheel angle with the textbook value.
    const double a = purePursuitWheelAngle({0, 0}, 0.0, {20, 5}, L);
    const double ld = std::hypot(20.0, 5.0);
    const double alpha = std::atan2(5.0, 20.0);
    CHECK(a == Approx(std::atan(2 * L * std::sin(alpha) / ld)));
    CHECK(purePursuitWheelAngle({0, 0}, 0.0, {20, -5}, L) == Approx(-a));
}

TEST_CASE("pure pursuit on an arc reproduces the arc's steering angle") {
    // On a circle of radius R the steady-state bicycle steering angle is atan(L / R).
    const double R = 100.0, L = 6.0;
    const Path arc = sim::makeArc(0.0, R, degToRad(120.0), 0.0, 1.0);
    LateralParams params;
    LateralInput in;
    in.rearAxle = arc.positionAt(50.0);
    in.yaw = arc.yawAt(50.0);
    in.speed = 15.0;
    in.wheelbase = L;
    const LateralOutput out = computeLateral(params, arc, in, std::nullopt);
    REQUIRE(out.valid);
    CHECK(out.wheelAngle == Approx(std::atan(L / R)).epsilon(0.03));
    CHECK(out.crossTrackError == Approx(0.0).epsilon(0.05));
}

TEST_CASE("Stanley corrects cross-track error towards the path") {
    // Left of path (positive error) needs a right (negative) correction.
    CHECK(stanleyWheelAngle(0.0, 1.0, 10.0, 1.0, 1.0) < 0.0);
    CHECK(stanleyWheelAngle(0.0, -1.0, 10.0, 1.0, 1.0) > 0.0);
    CHECK(stanleyWheelAngle(0.1, 0.0, 10.0, 1.0, 1.0) == Approx(0.1));
}

TEST_CASE("lookahead grows with speed within limits") {
    LateralParams p;
    CHECK(lookaheadDistance(p, 0.0) == Approx(15.0));
    CHECK(lookaheadDistance(p, 20.0) == Approx(25.0));
    CHECK(lookaheadDistance(p, 500.0) == Approx(p.lookaheadMax));
    p.lookaheadBase = 1.0;
    CHECK(lookaheadDistance(p, 0.0) == Approx(p.lookaheadMin));
}

TEST_CASE("steering shaper rate-limits and bounds authority at speed") {
    SteeringShaperParams sp;
    sp.smoothingTime = 0.0;
    SteeringShaper s(sp);
    const double maxWheel = 0.6;
    // One 1/60 s step cannot move more than maxRate * dt.
    const double out = s.update(0.6, maxWheel, 0.0, 6.0, 0, 1.0 / 60.0);
    CHECK(out == Approx(sp.maxRate / 60.0));
    // At highway speed the commanded curvature is bounded by the lateral-acceleration limit.
    const double limit = s.authorityLimit(maxWheel, 30.0, 6.0);
    CHECK(limit < 0.1);
    for (int k = 0; k < 600; ++k) s.update(0.6, maxWheel, 30.0, 6.0, 0, 1.0 / 60.0);
    CHECK(s.output() == Approx(limit));
    // Trailers make the shaper slower.
    SteeringShaper a(sp), b(sp);
    a.update(0.6, maxWheel, 5.0, 6.0, 0, 0.1);
    b.update(0.6, maxWheel, 5.0, 6.0, 2, 0.1);
    CHECK(b.output() < a.output());
}

TEST_CASE("curve speed from lateral acceleration") {
    CHECK(curveSpeed(1.0 / 100.0, 1.6, 2.0) == Approx(std::sqrt(160.0)));
    CHECK(curveSpeed(0.0, 1.6, 2.0) > 1e6);
    CHECK(curveSpeed(1.0, 1.6, 4.0) == Approx(4.0));
}

TEST_CASE("load factors make the planner more conservative") {
    SpeedPlannerParams p;
    VehicleLoadFactors bobtail;
    VehicleLoadFactors loaded;
    loaded.trailerCount = 1;
    loaded.cargoMassKg = 30000;
    loaded.wet = true;
    CHECK(effectiveLateralAccel(p, loaded) < effectiveLateralAccel(p, bobtail));
}

TEST_CASE("speed planner slows before a curve, not after it") {
    SpeedPlannerParams p;
    VehicleLoadFactors f;
    const double R = 80.0;
    const Path path = sim::makeArc(500.0, R, degToRad(90.0), 500.0, 2.0);
    const double vCurve = curveSpeed(1.0 / R, effectiveLateralAccel(p, f), p.minCurveSpeed);
    const double cruise = 29.0;

    const SpeedPlan farAway = planSpeed(p, f, path, 0.0, cruise);
    CHECK(farAway.targetSpeed == Approx(cruise));

    const SpeedPlan approaching = planSpeed(p, f, path, 450.0, cruise);
    CHECK(approaching.targetSpeed < cruise);
    CHECK(approaching.targetSpeed > vCurve);

    const SpeedPlan inCurve = planSpeed(p, f, path, 500.0 + R * 0.6, cruise);
    CHECK(inCurve.targetSpeed == Approx(vCurve).epsilon(0.08));
}

TEST_CASE("speed planner treats the end of known path as a stop") {
    SpeedPlannerParams p;
    VehicleLoadFactors f;
    const Path path = sim::makeStraight(100.0, 2.0);
    const SpeedPlan plan = planSpeed(p, f, path, 50.0, 30.0);
    CHECK(plan.targetSpeed == Approx(std::sqrt(2 * p.comfortDecel * 50.0)).epsilon(0.01));
    CHECK(planSpeed(p, f, Path{}, 0.0, 30.0).targetSpeed == 0.0);
}

TEST_CASE("longitudinal controller: throttle when slow, coast in band, brake when fast") {
    LongitudinalParams lp;
    LongitudinalController c(lp);
    PedalCommand out;
    for (int k = 0; k < 120; ++k) out = c.update(25.0, 20.0, 1.0 / 60.0);
    CHECK(out.throttle > 0.5);
    CHECK(out.brake == 0.0);

    LongitudinalController coast(lp);
    for (int k = 0; k < 60; ++k) out = coast.update(25.0, 25.2, 1.0 / 60.0);
    CHECK(out.brake == 0.0);
    CHECK(out.throttle == Approx(0.0).epsilon(0.05));

    LongitudinalController brake(lp);
    for (int k = 0; k < 120; ++k) out = brake.update(15.0, 25.0, 1.0 / 60.0);
    CHECK(out.throttle == 0.0);
    CHECK(out.brake > 0.2);
    CHECK(out.brake <= lp.maxStrongBrake + 1e-9);
    CHECK((out.level != BrakeLevel::None));
}

TEST_CASE("brake hysteresis prevents pedal flicker around the threshold") {
    LongitudinalParams lp;
    lp.gains = {1.0, 0.0, 0.0};
    LongitudinalController c(lp);
    // Demand oscillating between -0.08 and -0.12 must not toggle brake on every cycle.
    int toggles = 0;
    bool wasBraking = false;
    for (int k = 0; k < 200; ++k) {
        const double err = (k % 2 == 0) ? 0.12 : 0.06;  // speed above target
        const PedalCommand out = c.update(20.0, 20.0 + err, 1.0 / 60.0);
        const bool braking = out.brake > 0.0;
        if (braking != wasBraking) ++toggles;
        wasBraking = braking;
    }
    CHECK(toggles <= 1);
}

TEST_CASE("emergency braking ramps instead of slamming") {
    LongitudinalParams lp;
    LongitudinalController c(lp);
    const PedalCommand first = c.emergency(1.0 / 60.0);
    CHECK(first.throttle == 0.0);
    CHECK(first.brake < 0.05);
    PedalCommand out;
    for (int k = 0; k < 600; ++k) out = c.emergency(1.0 / 60.0);
    CHECK(out.brake == Approx(lp.emergencyBrake));
    CHECK((out.level == BrakeLevel::Emergency));
}

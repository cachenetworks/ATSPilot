// Closed-loop controller tests against the kinematic simulator. Thresholds are
// deliberately loose enough to be robust, but tight enough that a regression in
// path tracking, curve speed planning or stability fails them.

#include "TestSupport.h"

#include "math/MathUtil.h"
#include "sim/Scenario.h"

using namespace atspilot;
using namespace atspilot::sim;

namespace {

void report(const ScenarioResult& r) {
    MESSAGE(r.name << ": maxXTE=" << r.maxCrossTrack << " rmsXTE=" << r.rmsCrossTrack
                   << " maxLatAcc=" << r.maxLateralAccel << " reversals=" << r.steeringReversals
                   << " dist=" << r.distance << " completed=" << r.completed
                   << (r.disengaged ? " disengaged: " + r.disengageReason : std::string()));
}

}  // namespace

TEST_CASE("straight highway: holds lane and speed") {
    ScenarioOptions o;
    o.name = "straight";
    o.initialSpeed = 25.0;
    const ScenarioResult r = runScenario(makeStraight(3000.0), Config{}, SimParams{}, o);
    report(r);
    CHECK(r.completed);
    CHECK_FALSE(r.disengaged);
    CHECK(r.maxCrossTrack < 0.05);
}

TEST_CASE("recovers from an initial lateral offset without overshooting the lane") {
    ScenarioOptions o;
    o.name = "offset recovery";
    o.initialSpeed = 25.0;
    o.lateralOffset = 1.5;
    o.headingOffset = degToRad(2.0);
    const ScenarioResult r = runScenario(makeStraight(2000.0), Config{}, SimParams{}, o);
    report(r);
    CHECK(r.completed);
    CHECK(r.maxCrossTrack < 1.8);
    CHECK(r.steeringReversals < 20);
}

TEST_CASE("gentle highway curve at speed") {
    ScenarioOptions o;
    o.name = "gentle curve R=800";
    o.initialSpeed = 29.0;
    const ScenarioResult r = runScenario(makeArc(300.0, 800.0, degToRad(60.0), 500.0), Config{}, SimParams{}, o);
    report(r);
    CHECK(r.completed);
    CHECK(r.maxCrossTrack < 0.6);
    CHECK(r.maxLateralAccel < 1.5);
}

TEST_CASE("sharp curve forces slowdown before entry") {
    ScenarioOptions o;
    o.name = "sharp curve R=60";
    o.initialSpeed = 25.0;
    const ScenarioResult r = runScenario(makeArc(500.0, 60.0, degToRad(90.0), 400.0), Config{}, SimParams{}, o);
    report(r);
    CHECK(r.completed);
    CHECK(r.maxCrossTrack < 0.6);
    // Planner bound is 1.6 m/s^2; allow for tracking transients.
    CHECK(r.maxLateralAccel < 2.4);
}

TEST_CASE("S curve stays stable") {
    ScenarioOptions o;
    o.name = "S curve R=150";
    o.initialSpeed = 20.0;
    const ScenarioResult r = runScenario(makeSCurve(300.0, 150.0, degToRad(50.0), 400.0), Config{}, SimParams{}, o);
    report(r);
    CHECK(r.completed);
    CHECK(r.maxCrossTrack < 0.8);
    CHECK(r.steeringReversals < 40);
}

TEST_CASE("highway ramp with cloverleaf loop") {
    ScenarioOptions o;
    o.name = "highway ramp";
    o.initialSpeed = 27.0;
    const ScenarioResult r = runScenario(makeHighwayRamp(), Config{}, SimParams{}, o);
    report(r);
    CHECK(r.completed);
    CHECK(r.maxCrossTrack < 1.2);
    CHECK(r.maxLateralAccel < 2.4);
}

TEST_CASE("heavy truck with trailer and slow steering still tracks") {
    ScenarioOptions o;
    o.name = "loaded trailer, slow rack";
    o.initialSpeed = 22.0;
    SimParams p;
    p.trailerCount = 1;
    p.massFactor = 2.5;
    p.steeringLag = 0.35;
    p.wheelbase = 6.5;
    p.rearAxleZ = 5.5;
    p.frontAxleZ = -1.0;
    const ScenarioResult r = runScenario(makeSCurve(300.0, 200.0, degToRad(40.0), 400.0), Config{}, p, o);
    report(r);
    CHECK(r.completed);
    CHECK(r.maxCrossTrack < 0.6);
}

TEST_CASE("Stanley controller variant also tracks") {
    Config c;
    c.steering.lateral.algorithm = LateralAlgorithm::Stanley;
    ScenarioOptions o;
    o.name = "stanley S curve";
    o.initialSpeed = 20.0;
    const ScenarioResult r = runScenario(makeSCurve(300.0, 150.0, degToRad(50.0), 400.0), c, SimParams{}, o);
    report(r);
    CHECK(r.completed);
    CHECK(r.maxCrossTrack < 0.6);
}

TEST_CASE("cruise holds a set speed on a straight") {
    ScenarioOptions o;
    o.name = "cruise 55 mph";
    o.initialSpeed = 15.0;
    o.setSpeedOverride = speedToMps(55.0, SpeedUnits::Mph);
    o.engage = PilotRequest::Toggle;
    const ScenarioResult r = runScenario(makeStraight(6000.0), Config{}, SimParams{}, o);
    report(r);
    CHECK(r.completed);
    CHECK(r.maxSpeedError < 2.0);
}

TEST_CASE("tight city turn: curvature-limited lookahead keeps the corner") {
    ScenarioOptions o;
    o.name = "city turn R=20";
    o.initialSpeed = 6.0;
    Config c;
    c.speed.maxSpeed = 20.0;  // mph
    const ScenarioResult r = runScenario(makeArc(80.0, 20.0, degToRad(90.0), 150.0, 2.0), c, SimParams{}, o);
    report(r);
    CHECK(r.completed);
    CHECK(r.maxCrossTrack < 0.6);

    Config noLimit = c;
    noLimit.steering.lateral.curveLookaheadFactor = 0.0;
    o.name = "city turn R=20 without curve lookahead";
    const ScenarioResult before = runScenario(makeArc(80.0, 20.0, degToRad(90.0), 150.0, 2.0), noLimit, SimParams{}, o);
    report(before);
    CHECK(r.maxCrossTrack < before.maxCrossTrack);
}

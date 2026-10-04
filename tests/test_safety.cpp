#include "TestSupport.h"

#include "pilot/Safety.h"

using namespace atspilot;

namespace {

ControlCommand steerCmd(double steer) {
    ControlCommand c;
    c.active = true;
    c.steerActive = true;
    c.pedalsActive = true;
    c.steering = steer;
    return c;
}

}  // namespace

TEST_CASE("no override while the game reports exactly the command") {
    SafetyConfig cfg;
    OverrideDetector d(cfg);
    VehicleState s;
    s.inputSteering = 0.3;
    for (int k = 0; k < 100; ++k) CHECK((d.update(s, steerCmd(0.3)) == OverrideKind::None));
    CHECK((d.steeringMixing() == InputMixing::IncludesCommand));
}

TEST_CASE("steering override requires several consecutive frames") {
    SafetyConfig cfg;
    OverrideDetector d(cfg);
    VehicleState s;
    s.inputSteering = 0.1 + 0.5;  // driver adds 0.5 on top of the 0.1 command
    CHECK((d.update(s, steerCmd(0.1)) == OverrideKind::None));
    CHECK((d.update(s, steerCmd(0.1)) == OverrideKind::None));
    CHECK((d.update(s, steerCmd(0.1)) == OverrideKind::Steering));

    // A single spike does not disengage.
    OverrideDetector e(cfg);
    VehicleState spike;
    spike.inputSteering = 0.9;
    VehicleState calm;
    calm.inputSteering = 0.1;
    CHECK((e.update(spike, steerCmd(0.1)) == OverrideKind::None));
    CHECK((e.update(calm, steerCmd(0.1)) == OverrideKind::None));
    CHECK((e.update(spike, steerCmd(0.1)) == OverrideKind::None));
}

TEST_CASE("learned exclusive mixing attributes all reported input to the driver") {
    SafetyConfig cfg;
    OverrideDetector d(cfg);
    VehicleState s;
    s.inputSteering = 0.0;  // game does not include our command in input.steering
    for (int k = 0; k < 40; ++k) d.update(s, steerCmd(0.3));
    CHECK((d.steeringMixing() == InputMixing::ExcludesCommand));
    s.inputSteering = 0.25;
    d.update(s, steerCmd(0.3));
    d.update(s, steerCmd(0.3));
    CHECK((d.update(s, steerCmd(0.3)) == OverrideKind::Steering));
}

TEST_CASE("steering held at engagement only overrides when the driver moves beyond it") {
    SafetyConfig cfg;
    OverrideDetector d(cfg);
    d.reset(0.5);
    VehicleState s;
    for (double held = 0.5; held > 0.0; held -= 0.02) {  // re-centring
        s.inputSteering = held;
        CHECK((d.update(s, steerCmd(0.0)) == OverrideKind::None));
    }
    s.inputSteering = -0.3;  // a fresh move the other way
    d.update(s, steerCmd(0.0));
    d.update(s, steerCmd(0.0));
    CHECK((d.update(s, steerCmd(0.0)) == OverrideKind::Steering));
}

TEST_CASE("brake pedal overrides immediately") {
    SafetyConfig cfg;
    OverrideDetector d(cfg);
    VehicleState s;
    s.inputBrake = 0.4;
    CHECK((d.update(s, steerCmd(0.0)) == OverrideKind::Brake));
}

TEST_CASE("our own braking is not mistaken for the driver's") {
    SafetyConfig cfg;
    OverrideDetector d(cfg);
    VehicleState s;
    s.inputBrake = 0.4;
    ControlCommand c = steerCmd(0.0);
    c.brake = 0.4;
    CHECK((d.update(s, c) == OverrideKind::None));
}

TEST_CASE("override detection can be disabled") {
    SafetyConfig cfg;
    cfg.driverOverride = false;
    OverrideDetector d(cfg);
    VehicleState s;
    s.inputBrake = 1.0;
    CHECK((d.update(s, steerCmd(0.0)) == OverrideKind::None));
}

TEST_CASE("watchdog flags stale telemetry and stale paths") {
    SafetyConfig cfg;
    Watchdog w(cfg);
    CHECK_FALSE(w.check(10.0, 9.9, 9.5, true));
    CHECK(*w.check(10.0, 9.0, 9.9, true) == "Telemetry stale");
    CHECK(*w.check(10.0, 9.95, 8.0, true) == "Path stale");
    CHECK_FALSE(w.check(10.0, 9.95, 8.0, false));
}

TEST_CASE("a brake ATSPilot just released is not the driver's") {
    // In game the reported brake input trails the command by a frame or more.
    SafetyConfig cfg;
    OverrideDetector d(cfg);
    VehicleState s;
    ControlCommand c = steerCmd(0.0);
    c.brake = 0.3;
    s.inputBrake = 0.3;
    for (int k = 0; k < 5; ++k) CHECK((d.update(s, c) == OverrideKind::None));
    c.brake = 0.0;  // released; the game still reports 0.3 for two frames
    CHECK((d.update(s, c) == OverrideKind::None));
    CHECK((d.update(s, c) == OverrideKind::None));
    s.inputBrake = 0.0;
    CHECK((d.update(s, c) == OverrideKind::None));
}

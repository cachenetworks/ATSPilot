#include "TestSupport.h"

#include "math/MathUtil.h"
#include "pilot/Autopilot.h"
#include "sim/VehicleSim.h"

using namespace atspilot;

namespace {

struct Rig {
    Config cfg;
    sim::VehicleSim vehicle;
    PathSnapshotPtr path;
    Autopilot pilot{cfg};

    Rig() {
        vehicle.reset({0.0, 0.0}, 0.0, 20.0);
        auto snap = std::make_shared<PathSnapshot>();
        snap->path = sim::makeStraight(2000.0);
        path = snap;
    }

    VehicleState state() const { return vehicle.state(); }
    void request(PilotRequest r, double wall = 0.0) {
        pilot.request(r, state(), vehicle.config(), path, wall, wall);
    }
    ControlCommand step(double wall, double telemetryWall, double pathWall) {
        const ControlCommand c = pilot.update(state(), vehicle.config(), path, wall, telemetryWall, pathWall);
        vehicle.step(c, 1.0 / 60.0);
        return c;
    }
};

}  // namespace

TEST_CASE("off mode commands nothing") {
    Rig rig;
    const ControlCommand c = rig.step(0.0, 0.0, 0.0);
    CHECK_FALSE(c.active);
    CHECK_FALSE(c.steerActive);
    CHECK_FALSE(c.pedalsActive);
}

TEST_CASE("autopilot engages only with valid state and toggles off") {
    Rig rig;
    rig.request(PilotRequest::ToggleAutopilot);
    CHECK((rig.pilot.mode() == PilotMode::Autopilot));
    rig.request(PilotRequest::ToggleAutopilot);
    CHECK((rig.pilot.mode() == PilotMode::Off));
}

TEST_CASE("engagement is refused without a path, with the reason reported") {
    Rig rig;
    rig.path.reset();
    rig.request(PilotRequest::ToggleAutopilot);
    CHECK((rig.pilot.mode() == PilotMode::Off));
    CHECK(rig.pilot.status().statusMessage == "ATSPilot unavailable: No valid road path");
    // Cruise does not need a path.
    rig.request(PilotRequest::ToggleCruise);
    CHECK((rig.pilot.mode() == PilotMode::Cruise));
}

TEST_CASE("engagement is refused when far from the path or misaligned") {
    Rig rig;
    rig.vehicle.reset({0.0, 10.0}, 0.0, 20.0);
    rig.request(PilotRequest::ToggleAutopilot);
    CHECK((rig.pilot.mode() == PilotMode::Off));
    Rig rig2;
    rig2.vehicle.reset({0.0, 0.0}, degToRad(90.0), 20.0);
    rig2.request(PilotRequest::ToggleLaneAssist);
    CHECK((rig2.pilot.mode() == PilotMode::Off));
}

TEST_CASE("engaging while moving holds current speed; set speed steps and clamps") {
    Rig rig;
    rig.request(PilotRequest::ToggleCruise);
    const double held = mpsToSpeed(rig.pilot.setSpeed(), SpeedUnits::Mph);
    CHECK(held == doctest::Approx(45.0));  // 20 m/s = 44.7 mph, rounded
    rig.request(PilotRequest::SpeedUp);
    CHECK(mpsToSpeed(rig.pilot.setSpeed(), SpeedUnits::Mph) == doctest::Approx(50.0));
    for (int k = 0; k < 20; ++k) rig.request(PilotRequest::SpeedUp);
    CHECK(mpsToSpeed(rig.pilot.setSpeed(), SpeedUnits::Mph) == doctest::Approx(65.0));
}

TEST_CASE("lane assist steers but leaves pedals to the driver") {
    Rig rig;
    rig.request(PilotRequest::ToggleLaneAssist);
    const ControlCommand c = rig.step(0.0, 0.0, 0.0);
    CHECK(c.active);
    CHECK(c.steerActive);
    CHECK_FALSE(c.pedalsActive);
}

TEST_CASE("stale telemetry disengages to neutral output") {
    Rig rig;
    rig.request(PilotRequest::ToggleAutopilot);
    rig.step(0.0, 0.0, 0.0);
    const ControlCommand c = rig.step(5.0, 0.0, 5.0);
    CHECK((rig.pilot.mode() == PilotMode::Off));
    CHECK_FALSE(c.active);
}

TEST_CASE("stale path triggers a controlled emergency stop that ends in neutral") {
    Rig rig;
    rig.request(PilotRequest::ToggleAutopilot);
    double t = 0.0;
    rig.step(t, t, t);
    bool sawEmergency = false;
    double maxBrake = 0.0;
    for (int k = 0; k < 60 * 60 && rig.pilot.mode() != PilotMode::Off; ++k) {
        t += 1.0 / 60.0;
        const ControlCommand c = rig.step(t, t, 0.0);  // path never refreshed
        if (rig.pilot.mode() == PilotMode::EmergencyStop) {
            sawEmergency = true;
            CHECK(c.throttle == 0.0);
            maxBrake = std::max(maxBrake, c.brake);
        }
    }
    CHECK(sawEmergency);
    CHECK((rig.pilot.mode() == PilotMode::Off));
    CHECK(rig.vehicle.speed() < 0.6);
    CHECK(maxBrake <= Config{}.cruise.emergencyBrake + 1e-9);
}

TEST_CASE("driver braking disengages autopilot") {
    Rig rig;
    rig.request(PilotRequest::ToggleAutopilot);
    rig.step(0.0, 0.0, 0.0);
    VehicleState s = rig.state();
    s.inputBrake = 0.5;
    s.time += 1.0 / 60.0;
    const ControlCommand c = rig.pilot.update(s, rig.vehicle.config(), rig.path, 0.02, 0.02, 0.02);
    CHECK((rig.pilot.mode() == PilotMode::Off));
    CHECK_FALSE(c.active);
    CHECK(rig.pilot.status().statusMessage.find("Driver Override") == 0);
}

TEST_CASE("pause disengages and resume restores the previous mode") {
    Rig rig;
    rig.request(PilotRequest::ToggleAutopilot);
    rig.step(0.0, 0.0, 0.0);
    VehicleState s = rig.state();
    s.paused = true;
    s.time += 0.02;
    rig.pilot.update(s, rig.vehicle.config(), rig.path, 0.02, 0.02, 0.02);
    CHECK((rig.pilot.mode() == PilotMode::Off));
    rig.request(PilotRequest::Resume, 0.03);
    CHECK((rig.pilot.mode() == PilotMode::Autopilot));
    rig.request(PilotRequest::EmergencyDisable, 0.04);
    CHECK((rig.pilot.mode() == PilotMode::Off));
    rig.request(PilotRequest::Resume, 0.05);
    CHECK((rig.pilot.mode() == PilotMode::Off));
}

TEST_CASE("arriving at the destination entrance ends the drive") {
    Rig rig;
    auto snap = std::make_shared<PathSnapshot>();
    snap->path = sim::makeStraight(60.0);
    snap->navigationActive = true;
    snap->routeRemaining = 300.0;
    rig.path = snap;
    rig.vehicle.reset({0.0, 0.0}, 0.0, 10.0);
    rig.request(PilotRequest::ToggleAutopilot);
    REQUIRE((rig.pilot.mode() == PilotMode::Autopilot));
    // Still far from the end of the route: moving or not, keep driving.
    double t = 0.0;
    rig.step(t, t, t);
    CHECK((rig.pilot.mode() == PilotMode::Autopilot));
    // Close to the end and stopped: done.
    auto arrived = std::make_shared<PathSnapshot>(*snap);
    arrived->routeRemaining = 10.0;
    VehicleState s = rig.state();
    s.speed = 0.0;
    s.time += 1.0 / 60.0;
    t += 1.0 / 60.0;
    rig.pilot.update(s, rig.vehicle.config(), arrived, t, t, t);
    CHECK((rig.pilot.mode() == PilotMode::Off));
    CHECK(rig.pilot.status().statusMessage == "Destination Reached");
}

TEST_CASE("inverted steering response is detected and disengages") {
    Rig rig;
    auto snap = std::make_shared<PathSnapshot>();
    snap->path = sim::makeArc(5.0, 40.0, degToRad(90.0), 100.0);
    rig.path = snap;
    rig.vehicle.reset({0.0, 0.0}, 0.0, 8.0);
    rig.request(PilotRequest::ToggleAutopilot);
    double t = 0.0;
    for (int k = 0; k < 600 && rig.pilot.mode() != PilotMode::Off; ++k) {
        t += 1.0 / 60.0;
        VehicleState s = rig.state();
        s.time = t;
        s.effectiveSteering = -rig.pilot.debug().targetSteering;  // game steers the other way
        const ControlCommand c = rig.pilot.update(s, rig.vehicle.config(), rig.path, t, t, t);
        rig.vehicle.step(c, 1.0 / 60.0);
    }
    CHECK((rig.pilot.mode() == PilotMode::Off));
    CHECK(rig.pilot.status().statusMessage == "Steering direction mismatch");
}

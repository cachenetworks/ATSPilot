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
    double t = 0.0;

    explicit Rig(Config c = Config{}) : cfg(c), pilot(c) {
        vehicle.reset({0.0, 0.0}, 0.0, 20.0);
        auto snap = std::make_shared<PathSnapshot>();
        snap->path = sim::makeStraight(4000.0);
        path = snap;
    }

    VehicleState state() const { return vehicle.state(); }
    void toggle() { pilot.request(PilotRequest::Toggle, state(), vehicle.config(), path, t, t); }
    ControlCommand step() {
        t += 1.0 / 60.0;
        const ControlCommand c = pilot.update(state(), vehicle.config(), path, t, t, t);
        vehicle.step(c, 1.0 / 60.0);
        return c;
    }
    ControlCommand stepWith(const VehicleState& s) {
        t += 1.0 / 60.0;
        return pilot.update(s, vehicle.config(), path, t, t, t);
    }
};

}  // namespace

TEST_CASE("off mode commands nothing") {
    Rig rig;
    const ControlCommand c = rig.step();
    CHECK_FALSE(c.active);
    CHECK_FALSE(c.steerActive);
    CHECK_FALSE(c.pedalsActive);
    CHECK_FALSE(c.buttons.any());
}

TEST_CASE("a single key switches ATSPilot on and off") {
    Rig rig;
    rig.toggle();
    CHECK((rig.pilot.mode() == PilotMode::Autopilot));
    rig.toggle();
    CHECK((rig.pilot.mode() == PilotMode::Off));
    CHECK(rig.pilot.status().statusMessage == "ATSPilot Off");
}

TEST_CASE("engagement is refused without a path, with the reason reported") {
    Rig rig;
    rig.path.reset();
    rig.toggle();
    CHECK((rig.pilot.mode() == PilotMode::Off));
    CHECK(rig.pilot.status().statusMessage == "ATSPilot unavailable: No valid road path");
}

TEST_CASE("engagement is refused when far from the path or misaligned") {
    Rig rig;
    rig.vehicle.reset({0.0, 10.0}, 0.0, 20.0);
    rig.toggle();
    CHECK((rig.pilot.mode() == PilotMode::Off));
    Rig rig2;
    rig2.vehicle.reset({0.0, 0.0}, degToRad(90.0), 20.0);
    rig2.toggle();
    CHECK((rig2.pilot.mode() == PilotMode::Off));
}

TEST_CASE("speed is handed to the game's cruise control, which ATSPilot switches on") {
    Rig rig;
    rig.toggle();
    bool pressedToggle = false;
    for (int k = 0; k < 120; ++k) {
        const ControlCommand c = rig.step();
        pressedToggle = pressedToggle || c.buttons.cruiseToggle;
    }
    CHECK(pressedToggle);
    CHECK(rig.vehicle.cruiseSet() > 0.0);
    CHECK(rig.pilot.status().gameCruiseActive);
    // With cruise control on, ATSPilot leaves the pedals to the game.
    const ControlCommand c = rig.step();
    CHECK(c.throttle == 0.0);
    CHECK(c.brake == 0.0);
}

TEST_CASE("the player's cruise control speed becomes the maximum ATSPilot drives") {
    Rig rig;
    rig.toggle();
    // Let ATSPilot bring the cruise control up to its target first.
    for (int k = 0; k < 60 * 25; ++k) rig.step();
    REQUIRE(rig.vehicle.cruiseSet() > 0.0);
    // The player presses the game's "slower" key several times.
    ControlCommand player;
    player.buttons.cruiseDec = true;
    for (int k = 0; k < 5; ++k) {
        rig.vehicle.step(player, 1.0 / 60.0);
        rig.vehicle.step(ControlCommand{}, 1.0 / 60.0);
    }
    const double playerSet = rig.vehicle.cruiseSet();
    for (int k = 0; k < 120; ++k) rig.step();
    CHECK(rig.pilot.setSpeed() == doctest::Approx(playerSet).epsilon(0.02));
    CHECK(rig.vehicle.cruiseSet() == doctest::Approx(playerSet).epsilon(0.02));
}

TEST_CASE("ATSPilot lowers the cruise set speed for a sharp curve and restores it afterwards") {
    Rig rig;
    auto snap = std::make_shared<PathSnapshot>();
    snap->path = sim::makeArc(400.0, 120.0, degToRad(90.0), 1200.0);
    rig.path = snap;
    rig.toggle();
    double minSet = 1e9;
    for (int k = 0; k < 60 * 120 && rig.pilot.mode() == PilotMode::Autopilot; ++k) {
        rig.step();
        if (rig.vehicle.cruiseSet() > 0.0) minSet = std::min(minSet, rig.vehicle.cruiseSet());
    }
    CHECK((rig.pilot.mode() == PilotMode::Autopilot));
    CHECK(minSet < 15.0);  // curve speed for R = 120 m is about 13.9 m/s
}

TEST_CASE("the game cancelling its cruise control on its own hands control back") {
    Rig rig;
    rig.toggle();
    for (int k = 0; k < 120; ++k) rig.step();
    REQUIRE(rig.vehicle.cruiseSet() > 0.0);
    rig.vehicle.cancelCruise();  // e.g. emergency brake assist
    rig.step();
    CHECK((rig.pilot.mode() == PilotMode::Off));
    CHECK(rig.pilot.status().statusMessage == "Cruise control cancelled - take over");
}

TEST_CASE("without the game's cruise control ATSPilot drives the pedals itself") {
    Config c;
    c.ingame.useCruiseControl = false;
    Rig rig(c);
    rig.toggle();
    bool anyPress = false;
    for (int k = 0; k < 120; ++k) anyPress = anyPress || rig.step().buttons.cruiseToggle;
    CHECK_FALSE(anyPress);
    CHECK(rig.vehicle.cruiseSet() == 0.0);
}

TEST_CASE("stale telemetry disengages to neutral output") {
    Rig rig;
    rig.toggle();
    rig.step();
    const ControlCommand c = rig.pilot.update(rig.state(), rig.vehicle.config(), rig.path, rig.t + 5.0, rig.t, rig.t + 5.0);
    CHECK((rig.pilot.mode() == PilotMode::Off));
    CHECK_FALSE(c.active);
}

TEST_CASE("stale path triggers a controlled emergency stop that ends in neutral") {
    Rig rig;
    rig.toggle();
    double maxBrake = 0.0;
    bool sawEmergency = false;
    const double pathWall = rig.t;
    for (int k = 0; k < 60 * 60 && (k == 0 || rig.pilot.mode() != PilotMode::Off); ++k) {
        rig.t += 1.0 / 60.0;
        const ControlCommand c = rig.pilot.update(rig.state(), rig.vehicle.config(), rig.path, rig.t, rig.t, pathWall);
        rig.vehicle.step(c, 1.0 / 60.0);
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
    rig.toggle();
    rig.step();
    VehicleState s = rig.state();
    s.inputBrake = 0.5;
    s.time += 1.0 / 60.0;
    const ControlCommand c = rig.stepWith(s);
    CHECK((rig.pilot.mode() == PilotMode::Off));
    CHECK_FALSE(c.active);
    CHECK(rig.pilot.status().statusMessage.find("Driver Override") == 0);
}

TEST_CASE("steering the driver still holds at engagement is not an override") {
    // In game: engaged at a junction with the wheels turned. The game keeps the
    // driver's keyboard steering and re-centres it, and reports it plus ours.
    Rig rig;
    VehicleState s = rig.state();
    double held = 0.45;
    s.inputSteering = held;
    s.effectiveSteering = held;
    rig.pilot.request(PilotRequest::Toggle, s, rig.vehicle.config(), rig.path, rig.t, rig.t);
    REQUIRE((rig.pilot.mode() == PilotMode::Autopilot));
    double ours = 0.0;
    for (int k = 0; k < 90; ++k) {
        held = std::max(0.0, held - 0.6 / 60.0);
        s.time += 1.0 / 60.0;
        s.inputSteering = held + ours;
        s.effectiveSteering = held + ours;
        const ControlCommand c = rig.stepWith(s);
        if (k == 0) CHECK(std::abs(c.steering) < 0.05);  // not the held angle again
        ours = c.steering;
    }
    CHECK((rig.pilot.mode() == PilotMode::Autopilot));

    // Steering beyond what was held is still a takeover.
    for (int k = 0; k < 5; ++k) {
        s.time += 1.0 / 60.0;
        s.inputSteering = ours - 0.4;
        rig.stepWith(s);
    }
    CHECK(rig.pilot.status().statusMessage == "Driver Override (steering)");
}

TEST_CASE("pause disengages") {
    Rig rig;
    rig.toggle();
    rig.step();
    VehicleState s = rig.state();
    s.paused = true;
    s.time += 0.02;
    rig.stepWith(s);
    CHECK((rig.pilot.mode() == PilotMode::Off));
}

TEST_CASE("stops at a traffic light and goes after a throttle tap") {
    Rig rig;
    auto snap = std::make_shared<PathSnapshot>();
    snap->path = sim::makeStraight(600.0);
    snap->stops.push_back({300.0, StopKind::Signal, 77});
    rig.path = snap;
    rig.vehicle.reset({0.0, 0.0}, 0.0, 15.0);
    rig.toggle();
    for (int k = 0; k < 60 * 80 && !rig.pilot.waitingAtIntersection(); ++k) rig.step();
    REQUIRE(rig.pilot.waitingAtIntersection());
    // Front of the truck stops before the line.
    const double front = rig.vehicle.rearAxle().x + (rig.vehicle.config().rearAxleZ - rig.vehicle.config().frontAxleZ);
    CHECK(front < 300.0);
    CHECK(front > 290.0);
    CHECK(rig.vehicle.speed() < 0.5);

    // Waiting holds the brake.
    for (int k = 0; k < 120; ++k) CHECK(rig.step().brake > 0.2);
    CHECK((rig.pilot.mode() == PilotMode::Autopilot));

    // A short throttle tap releases it.
    for (int k = 0; k < 20; ++k) {
        VehicleState s = rig.state();
        s.inputThrottle = 0.6;
        rig.vehicle.step(rig.stepWith(s), 1.0 / 60.0);
    }
    rig.step();
    CHECK_FALSE(rig.pilot.waitingAtIntersection());
    CHECK((rig.pilot.mode() == PilotMode::Autopilot));
    for (int k = 0; k < 60 * 20; ++k) rig.step();
    CHECK(rig.vehicle.rearAxle().x > 300.0);
}

TEST_CASE("holding the throttle while waiting is a driver takeover") {
    Rig rig;
    auto snap = std::make_shared<PathSnapshot>();
    snap->path = sim::makeStraight(600.0);
    snap->stops.push_back({200.0, StopKind::StopSign, 5});
    rig.path = snap;
    rig.vehicle.reset({0.0, 0.0}, 0.0, 12.0);
    rig.toggle();
    for (int k = 0; k < 60 * 80 && !rig.pilot.waitingAtIntersection(); ++k) rig.step();
    REQUIRE(rig.pilot.waitingAtIntersection());
    for (int k = 0; k < 150 && rig.pilot.mode() == PilotMode::Autopilot; ++k) {
        VehicleState s = rig.state();
        s.inputThrottle = 0.6;
        rig.vehicle.step(rig.stepWith(s), 1.0 / 60.0);
    }
    CHECK((rig.pilot.mode() == PilotMode::Off));
}

TEST_CASE("give-way lanes slow the truck without stopping") {
    Rig rig;
    auto snap = std::make_shared<PathSnapshot>();
    snap->path = sim::makeStraight(800.0);
    snap->stops.push_back({400.0, StopKind::Yield, 9});
    rig.path = snap;
    rig.vehicle.reset({0.0, 0.0}, 0.0, 18.0);
    rig.toggle();
    double minSpeedNear = 1e9;
    for (int k = 0; k < 60 * 60; ++k) {
        rig.step();
        const double x = rig.vehicle.rearAxle().x;
        if (x > 380.0 && x < 400.0) minSpeedNear = std::min(minSpeedNear, rig.vehicle.speed());
    }
    CHECK_FALSE(rig.pilot.waitingAtIntersection());
    CHECK(minSpeedNear < Config{}.intersections.yieldSpeed + 1.0);
    CHECK(minSpeedNear > 1.0);
}

TEST_CASE("blinkers are switched on before a manoeuvre and off afterwards") {
    Rig rig;
    auto snap = std::make_shared<PathSnapshot>();
    snap->path = sim::makeStraight(4000.0);
    // Exit to the right: signal from the truck's position to 150 m on.
    snap->indications = {{0.0, 150.0, -1, IndicationKind::Exit}, {400.0, 2000.0, 1, IndicationKind::Merge}};
    rig.path = snap;
    rig.toggle();
    for (int k = 0; k < 60; ++k) rig.step();
    CHECK(rig.vehicle.blinkerRight());
    CHECK_FALSE(rig.vehicle.blinkerLeft());
    // Past the end of the interval: off again, and nothing until the merge.
    for (int k = 0; k < 60 * 8; ++k) rig.step();
    CHECK_FALSE(rig.vehicle.blinkerRight());
    CHECK_FALSE(rig.vehicle.blinkerLeft());
    for (int k = 0; k < 60 * 12; ++k) rig.step();
    CHECK(rig.vehicle.blinkerLeft());
}

TEST_CASE("arriving at the destination entrance ends the drive and asks the game to park") {
    Rig rig;
    auto snap = std::make_shared<PathSnapshot>();
    snap->path = sim::makeStraight(60.0);
    snap->navigationActive = true;
    snap->routeRemaining = 300.0;
    rig.path = snap;
    rig.vehicle.reset({0.0, 0.0}, 0.0, 10.0);
    rig.toggle();
    REQUIRE((rig.pilot.mode() == PilotMode::Autopilot));
    rig.step();
    CHECK((rig.pilot.mode() == PilotMode::Autopilot));
    auto arrived = std::make_shared<PathSnapshot>(*snap);
    arrived->routeRemaining = 10.0;
    rig.path = arrived;
    VehicleState s = rig.state();
    s.speed = 0.0;
    s.time += 1.0 / 60.0;
    const ControlCommand c = rig.stepWith(s);
    CHECK((rig.pilot.mode() == PilotMode::Off));
    CHECK(rig.pilot.status().statusMessage == "Destination Reached");
    CHECK(c.buttons.quickPark);
}

TEST_CASE("heavy loads switch the auto profile to heavy haul") {
    Rig rig;
    VehicleConfig vc = rig.vehicle.config();
    vc.cargoMassKg = 32000.0;
    rig.pilot.update(rig.state(), vc, rig.path, 0.0, 0.0, 0.0);
    CHECK(rig.pilot.status().profile == "heavy_haul");
    CHECK(rig.pilot.effectiveConfig().planner.maxLateralAccel < Config{}.planner.maxLateralAccel);
    CHECK(mpsToSpeed(rig.pilot.setSpeed(), SpeedUnits::Mph) <= 55.0 + 1e-6);
}

TEST_CASE("inverted steering response is detected and disengages") {
    Rig rig;
    auto snap = std::make_shared<PathSnapshot>();
    snap->path = sim::makeArc(5.0, 40.0, degToRad(90.0), 100.0);
    rig.path = snap;
    rig.vehicle.reset({0.0, 0.0}, 0.0, 8.0);
    rig.toggle();
    for (int k = 0; k < 600 && rig.pilot.mode() != PilotMode::Off; ++k) {
        rig.t += 1.0 / 60.0;
        VehicleState s = rig.state();
        s.time = rig.t;
        s.effectiveSteering = -rig.pilot.debug().targetSteering;  // game steers the other way
        const ControlCommand c = rig.pilot.update(s, rig.vehicle.config(), rig.path, rig.t, rig.t, rig.t);
        rig.vehicle.step(c, 1.0 / 60.0);
    }
    CHECK((rig.pilot.mode() == PilotMode::Off));
    CHECK(rig.pilot.status().statusMessage == "Steering direction mismatch");
}

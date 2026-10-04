#include "TestSupport.h"

#include "control/TrafficAwareness.h"
#include "math/MathUtil.h"
#include "pilot/Autopilot.h"
#include "sim/VehicleSim.h"

using namespace atspilot;

namespace {

WorldVehicle car(double x, double y, double yaw, double speed, int id = 1) {
    WorldVehicle v;
    v.id = id;
    v.position = {x, y};
    v.yaw = yaw;
    v.speed = speed;
    v.length = 4.5;
    v.width = 1.9;
    return v;
}

}  // namespace

TEST_CASE("a stopped car in the lane ahead becomes a stop constraint short of it") {
    const Path path = sim::makeStraight(500.0);
    const auto pic = assessTraffic(path, 10.0, 15.0, {car(60.0, 0.0, 0.0, 0.0)}, TrafficParams{});
    REQUIRE(pic.nearest);
    CHECK(pic.nearest->s == doctest::Approx(60.0 - 2.25).epsilon(0.02));
    REQUIRE(pic.constraints.size() == 1);
    CHECK(pic.constraints[0].speed == 0.0);
    CHECK(pic.constraints[0].s == doctest::Approx(60.0 - 2.25 - 5.0).epsilon(0.02));
    CHECK(pic.requiredDecel > 0.0);
}

TEST_CASE("a moving lead vehicle is followed at a time gap") {
    const Path path = sim::makeStraight(500.0);
    const auto pic = assessTraffic(path, 10.0, 20.0, {car(80.0, 0.3, 0.0, 15.0)}, TrafficParams{});
    REQUIRE(pic.constraints.size() == 1);
    CHECK(pic.constraints[0].speed == doctest::Approx(15.0).epsilon(0.01));
    CHECK(pic.constraints[0].s == doctest::Approx(80.0 - 2.25 - 5.0 - 2.0 * 15.0).epsilon(0.02));
}

TEST_CASE("traffic in the next lane, oncoming or behind is ignored") {
    const Path path = sim::makeStraight(500.0);
    const TrafficParams p;
    CHECK(assessTraffic(path, 10.0, 20.0, {car(60.0, 3.7, 0.0, 20.0)}, p).constraints.empty());   // next lane
    CHECK(assessTraffic(path, 10.0, 20.0, {car(60.0, -3.7, kPi, 20.0)}, p).constraints.empty());  // oncoming
    CHECK(assessTraffic(path, 50.0, 20.0, {car(20.0, 0.0, 0.0, 25.0)}, p).constraints.empty());   // behind
}

TEST_CASE("crossing traffic is a conflict only when the truck would meet it") {
    const Path path = sim::makeStraight(500.0);
    const TrafficParams p;
    // A car 20 m from the path, crossing at 10 m/s: it is on the path in about 2 s.
    const WorldVehicle crossing = car(60.0, -20.0, 0.5 * kPi, 10.0);
    // The truck, 50 m away at 20 m/s, would be there in 2.5 s: conflict.
    const auto meet = assessTraffic(path, 10.0, 20.0, {crossing}, p);
    REQUIRE(meet.nearest);
    CHECK(meet.nearest->crossing);
    CHECK(meet.nearest->s == doctest::Approx(60.0 - 0.95).epsilon(0.03));
    // Pulling away from a stop line 120 m back, the truck needs far longer: no conflict.
    CHECK(assessTraffic(path, 10.0, 0.0, {car(130.0, -20.0, 0.5 * kPi, 10.0)}, p).constraints.empty());
    // A parked (stopped) car beside the road is not crossing.
    CHECK(assessTraffic(path, 10.0, 20.0, {car(60.0, -20.0, 0.5 * kPi, 0.0)}, p).constraints.empty());
}

TEST_CASE("a signal stop is matched to the nearest light with its semaphore id") {
    const Path path = sim::makeStraight(500.0);
    PathStop stop;
    stop.s = 100.0;
    stop.semaphoreId = 3;
    std::vector<WorldLight> lights(4);
    lights[0] = {2, {101.0, 5.0}, 0.0, LightState::Green, 0.0};
    lights[1] = {3, {130.0, 6.0}, 0.0, LightState::Red, 0.0};
    lights[2] = {3, {300.0, 6.0}, 0.0, LightState::Green, 0.0};  // another junction
    lights[3] = {3, {104.0, -6.0}, 0.0, LightState::AmberToRed, 0.0};
    const LightMatch m = lightForStop(stop, path, lights, TrafficParams{});
    REQUIRE(m.light);
    CHECK(m.byId);
    CHECK((m.light->state == LightState::AmberToRed));
}

TEST_CASE("without a matching id a light is matched by position and facing") {
    const Path path = sim::makeStraight(500.0);
    PathStop stop;
    stop.s = 100.0;
    stop.semaphoreId = 9;  // no light carries this id
    std::vector<WorldLight> lights(3);
    lights[0] = {1, {130.0, 4.0}, 0.0, LightState::Green, 0.0};        // across the junction, facing along
    lights[1] = {2, {130.0, 8.0}, kPi, LightState::Red, 0.0};          // for oncoming traffic
    lights[2] = {4, {110.0, 20.0}, 0.5 * kPi, LightState::Red, 0.0};   // for the crossing road
    // Facing unknown: the two lights along the road disagree, so nothing is trusted.
    CHECK(lightForStop(stop, path, lights, TrafficParams{}, 0).light == nullptr);
    // Lights face along their traffic: ours is the green one.
    const LightMatch along = lightForStop(stop, path, lights, TrafficParams{}, 1);
    REQUIRE(along.light);
    CHECK_FALSE(along.byId);
    CHECK((along.light->state == LightState::Green));
    // Lights face against their traffic: the red one.
    const LightMatch against = lightForStop(stop, path, lights, TrafficParams{}, -1);
    REQUIRE(against.light);
    CHECK((against.light->state == LightState::Red));
    // Nothing near the line: unknown.
    lights.clear();
    lights.push_back({1, {400.0, 4.0}, 0.0, LightState::Green, 0.0});
    CHECK(lightForStop(stop, path, lights, TrafficParams{}, 1).light == nullptr);
}

TEST_CASE("signal decisions") {
    const TrafficParams p;
    CHECK((decideSignal(LightState::Green, 30.0, 15.0, p) == SignalDecision::Go));
    CHECK((decideSignal(LightState::Red, 80.0, 15.0, p) == SignalDecision::Stop));
    CHECK((decideSignal(LightState::AmberToGreen, 80.0, 15.0, p) == SignalDecision::Stop));
    // Amber: stop when there is room (15 m/s, 80 m: 1.4 m/s²), go when there is not (15 m: 7.5 m/s²).
    CHECK((decideSignal(LightState::AmberToRed, 80.0, 15.0, p) == SignalDecision::Stop));
    CHECK((decideSignal(LightState::AmberToRed, 15.0, 15.0, p) == SignalDecision::Go));
    CHECK((decideSignal(LightState::Flashing, 30.0, 10.0, p) == SignalDecision::GiveWay));
    CHECK((decideSignal(LightState::Unknown, 30.0, 10.0, p) == SignalDecision::Unknown));
}

namespace {

struct TrafficRig {
    Config cfg;
    sim::VehicleSim vehicle;
    std::shared_ptr<PathSnapshot> snap = std::make_shared<PathSnapshot>();
    Autopilot pilot{cfg};
    std::shared_ptr<WorldSnapshot> world = std::make_shared<WorldSnapshot>();
    double t = 0.0;

    TrafficRig() {
        vehicle.reset({0.0, 0.0}, 0.0, 15.0);
        snap->path = sim::makeStraight(2000.0);
        world->valid = true;
        pilot.setWorld(world);
        pilot.request(PilotRequest::Toggle, vehicle.state(), vehicle.config(), snap, t, t);
    }
    double frontX() const {
        const auto vc = vehicle.config();
        return vehicle.state().worldPosition.x - vc.frontAxleZ + 1.5;
    }
    void step() {
        t += 1.0 / 60.0;
        world->time = t;
        const ControlCommand c = pilot.update(vehicle.state(), vehicle.config(), snap, t, t, t);
        vehicle.step(c, 1.0 / 60.0);
    }
};

}  // namespace

TEST_CASE("the autopilot queues behind a stopped vehicle and moves off when it does") {
    TrafficRig rig;
    REQUIRE((rig.pilot.mode() == PilotMode::Autopilot));
    rig.world->vehicles = {car(150.0, 0.0, 0.0, 0.0)};
    for (int k = 0; k < 60 * 40; ++k) rig.step();
    CHECK((rig.pilot.mode() == PilotMode::Autopilot));
    CHECK(rig.vehicle.speed() < 0.3);
    const double gap = (150.0 - 2.25) - rig.frontX();
    CHECK(gap > 2.0);
    CHECK(gap < 12.0);

    // The queue moves: the car drives off and the truck follows.
    for (int k = 0; k < 60 * 20; ++k) {
        rig.world->vehicles[0].speed = 12.0;
        rig.world->vehicles[0].position.x += 12.0 / 60.0;
        rig.step();
    }
    CHECK((rig.pilot.mode() == PilotMode::Autopilot));
    CHECK(rig.vehicle.speed() > 8.0);
    CHECK((rig.world->vehicles[0].position.x - 2.25) - rig.frontX() > 5.0);
}

TEST_CASE("the autopilot stops on red and goes on green without a throttle tap") {
    TrafficRig rig;
    PathStop stop;
    stop.s = 200.0;
    stop.kind = StopKind::Signal;
    stop.segment = 7;
    stop.semaphoreId = 1;
    rig.snap->stops = {stop};
    rig.world->lights = {{1, {205.0, -6.0}, 0.0, LightState::Red, 20.0}};
    for (int k = 0; k < 60 * 40; ++k) rig.step();
    CHECK(rig.vehicle.speed() < 0.3);
    CHECK(rig.frontX() < 200.0);
    CHECK(rig.frontX() > 190.0);
    CHECK_FALSE(rig.pilot.waitingAtIntersection());  // no tap needed
    CHECK(rig.pilot.status().signalState == "red");

    rig.world->lights[0].state = LightState::Green;
    for (int k = 0; k < 60 * 15; ++k) rig.step();
    CHECK((rig.pilot.mode() == PilotMode::Autopilot));
    CHECK(rig.frontX() > 215.0);
}

TEST_CASE("a signal whose light cannot be read is an all-way stop, not a wait for the driver") {
    TrafficRig rig;
    PathStop stop;
    stop.s = 200.0;
    stop.kind = StopKind::Signal;
    stop.segment = 7;
    stop.semaphoreId = 4;
    rig.snap->stops = {stop};
    // No light anywhere near, and a car crossing the junction for a while.
    rig.world->vehicles = {car(208.0, -80.0, 0.5 * kPi, 12.0, 5)};
    bool stopped = false;
    for (int k = 0; k < 60 * 45; ++k) {
        if (!rig.world->vehicles.empty()) {
            rig.world->vehicles[0].position.y += 12.0 / 60.0;
            if (rig.world->vehicles[0].position.y > 80.0) rig.world->vehicles[0].position.y = -80.0;
        }
        if (k == 60 * 28) rig.world->vehicles.clear();
        rig.step();
        stopped = stopped || (rig.vehicle.speed() < 0.3 && rig.frontX() > 190.0);
        CHECK_FALSE(rig.pilot.waitingAtIntersection());
    }
    CHECK(stopped);
    CHECK((rig.pilot.mode() == PilotMode::Autopilot));
    CHECK(rig.frontX() > 215.0);  // through once the traffic had gone
}

TEST_CASE("a stop sign is left once crossing traffic has passed") {
    TrafficRig rig;
    PathStop stop;
    stop.s = 150.0;
    stop.kind = StopKind::StopSign;
    stop.segment = 9;
    rig.snap->stops = {stop};
    // A stream of crossing traffic that keeps the junction busy for a while.
    WorldVehicle crossing = car(158.0, -80.0, 0.5 * kPi, 12.0, 5);
    rig.world->vehicles = {crossing};
    double stoppedAt = -1.0;
    for (int k = 0; k < 60 * 40; ++k) {
        rig.world->vehicles[0].position.y += 12.0 / 60.0;
        if (rig.world->vehicles[0].position.y > 80.0) rig.world->vehicles[0].position.y = -80.0;  // next car
        if (k == 60 * 25) rig.world->vehicles.clear();  // traffic gone
        rig.step();
        if (stoppedAt < 0.0 && rig.vehicle.speed() < 0.3) stoppedAt = rig.t;
    }
    CHECK(stoppedAt > 0.0);
    CHECK(rig.frontX() > 160.0);  // through the junction after the traffic left
}

TEST_CASE("from the shoulder the autopilot pulls into the lane, signalling, after traffic behind has passed") {
    Config cfg;
    sim::VehicleSim vehicle;
    vehicle.reset({0.0, -5.0}, 0.0, 0.0);  // stopped on the shoulder, 5 m right of the lane
    auto snap = std::make_shared<PathSnapshot>();
    snap->path = sim::makeStraight(3000.0);
    auto world = std::make_shared<WorldSnapshot>();
    world->valid = true;
    // A car coming up the lane from behind.
    world->vehicles = {car(-90.0, 0.0, 0.0, 20.0, 3)};
    Autopilot pilot(cfg);
    pilot.setWorld(world);
    double t = 0.0;
    pilot.request(PilotRequest::Toggle, vehicle.state(), vehicle.config(), snap, t, t);
    REQUIRE((pilot.mode() == PilotMode::Autopilot));

    bool leftBlinker = false;
    double movedBeforeCarPassed = 0.0;
    for (int k = 0; k < 60 * 40; ++k) {
        t += 1.0 / 60.0;
        if (!world->vehicles.empty()) {
            world->vehicles[0].position.x += 20.0 / 60.0;
            if (world->vehicles[0].position.x < vehicle.state().worldPosition.x) {
                movedBeforeCarPassed = std::max(movedBeforeCarPassed, std::abs(vehicle.state().worldPosition.z - 5.0));
            }
            if (world->vehicles[0].position.x > 400.0) world->vehicles.clear();
        }
        const ControlCommand c = pilot.update(vehicle.state(), vehicle.config(), snap, t, t, t);
        vehicle.step(c, 1.0 / 60.0);
        leftBlinker = leftBlinker || vehicle.blinkerLeft();
    }
    CHECK((pilot.mode() == PilotMode::Autopilot));
    CHECK(leftBlinker);
    CHECK(movedBeforeCarPassed < 0.5);                            // waited for the car
    CHECK(std::abs(vehicle.state().worldPosition.z) < 0.6);       // in the lane (plan y = -world z)
    CHECK(vehicle.speed() > 5.0);
}

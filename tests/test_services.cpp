#include "TestSupport.h"

#include "map/ServicePlanner.h"
#include "math/MathUtil.h"
#include "pilot/Autopilot.h"
#include "sim/VehicleSim.h"

using namespace atspilot;
using doctest::Approx;

namespace {

LaneSegment lane(Vec2 from, Vec2 to, std::uint64_t item, int points = 21) {
    LaneSegment s;
    s.itemUid = item;
    for (int i = 0; i < points; ++i) {
        const Vec2 p = lerp(from, to, static_cast<double>(i) / (points - 1));
        s.points.push_back({static_cast<float>(p.x), static_cast<float>(p.y), 0.0f});
    }
    return s;
}

// A highway heading east with a parallel side road (a weigh station or a fuel
// stop) that leaves after A and rejoins before C:
//
//   A ──▶ B ──────────────▶ C ──▶ D (destination)
//    ╲                     ╱
//     R1 ──▶ S (stand) ──▶ R2
struct Highway {
    RoadNetwork net;
    std::uint32_t a, b, c, d, r1, side, r2;
    explicit Highway(ServiceKind kind) {
        a = net.add(lane({0, 0}, {400, 0}, 1));
        b = net.add(lane({400, 0}, {1200, 0}, 2));
        c = net.add(lane({1200, 0}, {2000, 0}, 3));
        d = net.add(lane({2000, 0}, {2400, 0}, 4));
        r1 = net.add(lane({400, 0}, {500, -30}, 5));
        side = net.add(lane({500, -30}, {800, -30}, 6));
        r2 = net.add(lane({800, -30}, {1200, 0}, 7));
        auto& s = net.mutableSegments();
        s[a].next = {b, r1};
        s[b].next = {c};
        s[c].next = {d};
        s[r1].next = {side};
        s[side].next = {r2};
        s[r2].next = {c};
        ServicePoint sp;
        sp.kind = kind;
        sp.position = {650.0, -30.0};
        sp.lane = side;
        sp.s = 150.0f;
        net.addService(sp);
        net.finalize();
    }
};

}  // namespace

TEST_CASE("the route pulls into a weigh station beside the highway ahead") {
    Highway h(ServiceKind::Weigh);
    ServicePlanOptions o;
    o.weigh = true;
    const Route r = planRouteWithServices(h.net, h.a, 20.0, {h.d}, RouteOptions{}, 0.0, 0.0, o);
    REQUIRE(r.found);
    CHECK(r.find(h.side) >= 0);
    CHECK(r.find(h.b) < 0);
    REQUIRE(r.services.size() == 1);
    CHECK((r.services[0].kind == ServiceKind::Weigh));
    CHECK(r.services[0].lane == h.side);
    CHECK(r.services[0].s == Approx(150.0));
    CHECK(r.steps.back().segment == h.d);

    // Once visited (or with weigh stations off) the highway is taken.
    o.skipLanes = {h.side};
    const Route again = planRouteWithServices(h.net, h.a, 20.0, {h.d}, RouteOptions{}, 0.0, 0.0, o);
    REQUIRE(again.found);
    CHECK(again.find(h.b) >= 0);
    CHECK(again.services.empty());
    o.skipLanes.clear();
    o.weigh = false;
    CHECK(planRouteWithServices(h.net, h.a, 20.0, {h.d}, RouteOptions{}, 0.0, 0.0, o).services.empty());
}

TEST_CASE("a weigh station too close ahead to reach its ramp is not routed to") {
    Highway h(ServiceKind::Weigh);
    // On the highway past the ramp: it cannot be reached, and nothing breaks.
    const Route r = planRouteWithServices(h.net, h.b, 10.0, {h.d}, RouteOptions{}, 0.0, 0.0, ServicePlanOptions{});
    REQUIRE(r.found);
    CHECK(r.services.empty());
}

TEST_CASE("low on fuel the route goes via a fuel pump, with or without a destination") {
    Highway h(ServiceKind::Fuel);
    ServicePlanOptions o;
    o.fuel = true;
    const Route r = planRouteWithServices(h.net, h.a, 20.0, {h.d}, RouteOptions{}, 0.0, 0.0, o);
    REQUIRE(r.found);
    REQUIRE(r.services.size() == 1);
    CHECK((r.services[0].kind == ServiceKind::Fuel));
    CHECK(r.find(h.side) >= 0);
    CHECK(r.steps.back().segment == h.d);

    const Route toPump = planRouteWithServices(h.net, h.a, 20.0, {}, RouteOptions{}, 0.0, 0.0, o);
    REQUIRE(toPump.found);
    CHECK(toPump.steps.back().segment == h.side);
    CHECK(toPump.services.size() == 1);
}

namespace {

struct StopRig {
    Config cfg;
    sim::VehicleSim vehicle;
    std::shared_ptr<PathSnapshot> snap = std::make_shared<PathSnapshot>();
    Autopilot pilot{cfg};
    std::shared_ptr<WorldSnapshot> world;
    double t = 0.0;

    explicit StopRig(Path path, bool withWorld = false) {
        vehicle.reset({0.0, 0.0}, 0.0, 15.0);
        snap->path = std::move(path);
        if (withWorld) {
            world = std::make_shared<WorldSnapshot>();
            world->valid = true;
            pilot.setWorld(world);
        }
    }
    void engage() { pilot.request(PilotRequest::Toggle, vehicle.state(), vehicle.config(), snap, t, t); }
    double frontX() const { return vehicle.state().worldPosition.x - vehicle.config().frontAxleZ + 1.5; }
    ControlCommand step() {
        t += 1.0 / 60.0;
        if (world) world->time = t;
        const ControlCommand c = pilot.update(vehicle.state(), vehicle.config(), snap, t, t, t);
        vehicle.step(c, 1.0 / 60.0);
        return c;
    }
};

PathStop serviceStop(StopKind kind, double s) {
    PathStop stop;
    stop.s = s;
    stop.kind = kind;
    stop.segment = 42;
    return stop;
}

}  // namespace

TEST_CASE("at a fuel pump the autopilot probes activate, detects fuel flow, then holds until full") {
    StopRig rig(sim::makeStraight(2000.0));
    rig.vehicle.setFuel(60.0, 600.0);
    rig.snap->stops = {serviceStop(StopKind::Fuel, 300.0)};
    rig.engage();
    REQUIRE((rig.pilot.mode() == PilotMode::Autopilot));
    bool stoppedAtPump = false;
    for (int k = 0; k < 60 * 120 && rig.frontX() < 330.0; ++k) {
        rig.step();
        if (rig.vehicle.speed() < 0.3 && rig.frontX() > 295.0) {
            stoppedAtPump = true;
            CHECK(rig.frontX() < 310.0);
        }
    }
    CHECK(stoppedAtPump);
    CHECK(rig.vehicle.fuel() > 0.97 * 600.0);
    CHECK((rig.pilot.mode() == PilotMode::Autopilot));
    CHECK(rig.frontX() >= 330.0);  // on its way again
    CHECK_FALSE(rig.step().activate);
}

TEST_CASE("when a pump gives no fuel the autopilot moves up, retries, then carries on") {
    StopRig rig(sim::makeStraight(2000.0));
    rig.vehicle.setFuel(60.0, 600.0, 0.0);
    rig.snap->stops = {serviceStop(StopKind::Fuel, 300.0)};
    rig.engage();
    for (int k = 0; k < 60 * 150 && rig.frontX() < 340.0; ++k) rig.step();
    CHECK(rig.vehicle.activateHeldTime() > 3.0);  // one bounded activation probe at each alignment
    CHECK(rig.vehicle.activateHeldTime() < 8.0);  // never sits there holding Enter forever
    CHECK((rig.pilot.mode() == PilotMode::Autopilot));
    CHECK(rig.frontX() >= 340.0);
}

TEST_CASE("at a weigh station the autopilot stops on the scale, waits, and drives on") {
    StopRig rig(sim::makeStraight(2000.0));
    rig.snap->stops = {serviceStop(StopKind::Weigh, 300.0)};
    rig.engage();
    double stoppedFor = 0.0;
    double activateFor = 0.0;
    for (int k = 0; k < 60 * 90 && rig.frontX() < 340.0; ++k) {
        const ControlCommand c = rig.step();
        if (c.activate) activateFor += 1.0 / 60.0;
        if (rig.vehicle.speed() < 0.3 && rig.frontX() > 295.0) stoppedFor += 1.0 / 60.0;
    }
    CHECK(stoppedFor > 4.5);
    CHECK(activateFor > 0.25);
    CHECK(activateFor < 0.6);  // one Enter/Activate pulse, not a repeated hold
    CHECK((rig.pilot.mode() == PilotMode::Autopilot));
    CHECK(rig.frontX() >= 340.0);
}

TEST_CASE("right on red: a full stop, then the turn once the road being joined is clear") {
    // 200 m straight, then a right turn (R 15 m) onto a road heading south (-y).
    StopRig rig(sim::makeArc(200.0, 15.0, -0.5 * kPi, 300.0), true);
    PathStop stop;
    stop.s = 200.0;
    stop.kind = StopKind::Signal;
    stop.segment = 7;
    stop.semaphoreId = 1;
    rig.snap->stops = {stop};
    rig.world->lights = {{1, {205.0, -6.0}, 0.0, LightState::Red, 60.0}};
    // Traffic keeps coming south along the road being joined, from the truck's left.
    WorldVehicle car;
    car.id = 9;
    car.yaw = -0.5 * kPi;
    car.speed = 14.0;
    rig.world->vehicles = {car};
    rig.engage();
    double stoppedAtLine = 0.0;
    for (int k = 0; k < 60 * 40; ++k) {
        rig.world->vehicles[0].position = {215.0, 50.0 - 14.0 * std::fmod(k / 60.0, 4.0)};
        rig.step();
        if (rig.vehicle.speed() < 0.3) stoppedAtLine += 1.0 / 60.0;
    }
    CHECK(stoppedAtLine > 10.0);
    CHECK(rig.frontX() < 200.5);  // still waiting at the line

    rig.world->vehicles.clear();
    for (int k = 0; k < 60 * 20; ++k) rig.step();
    CHECK((rig.pilot.mode() == PilotMode::Autopilot));
    CHECK(rig.vehicle.state().worldPosition.z > 20.0);  // turned and heading south (plan -y = world +z)
}

TEST_CASE("right on red is off when disabled and never applies straight on") {
    Config cfg;
    cfg.intersections.rightOnRed = false;
    StopRig rig(sim::makeArc(200.0, 15.0, -0.5 * kPi, 300.0), true);
    rig.pilot.setConfig(cfg);
    PathStop stop;
    stop.s = 200.0;
    stop.kind = StopKind::Signal;
    stop.segment = 7;
    stop.semaphoreId = 1;
    rig.snap->stops = {stop};
    rig.world->lights = {{1, {205.0, -6.0}, 0.0, LightState::Red, 60.0}};
    rig.engage();
    for (int k = 0; k < 60 * 40; ++k) rig.step();
    CHECK(rig.vehicle.speed() < 0.3);
    CHECK(rig.frontX() < 200.5);
}

#include "TestSupport.h"

#include "map/LanePlanner.h"
#include "map/RoutePlanner.h"
#include "math/MathUtil.h"

using namespace atspilot;
using doctest::Approx;

namespace {

LaneSegment lane(Vec2 from, Vec2 to, std::uint64_t item = 0, std::uint8_t index = 0, int points = 21) {
    LaneSegment s;
    s.itemUid = item;
    s.laneIndex = index;
    for (int i = 0; i < points; ++i) {
        const Vec2 p = lerp(from, to, static_cast<double>(i) / (points - 1));
        s.points.push_back({static_cast<float>(p.x), static_cast<float>(p.y), 0.0f});
    }
    return s;
}

// Two-lane road (item 1) heading east, splitting at x = 200:
//   inner lane 0 (y = 0) continues straight to the "city" lane,
//   outer lane 1 (y = -4.5) continues to an exit ramp curving south to the "depot".
struct Grid {
    RoadNetwork net;
    std::uint32_t inner, outer, straight, exitRamp, depot, city;
    Grid() {
        inner = net.add(lane({0, 0}, {200, 0}, 1, 0));
        outer = net.add(lane({0, -4.5}, {200, -4.5}, 1, 1));
        straight = net.add(lane({200, 0}, {600, 0}, 2, 0));
        exitRamp = net.add(lane({200, -4.5}, {300, -80}, 3, 0));
        depot = net.add(lane({300, -80}, {320, -150}, 4, 0));
        city = net.add(lane({600, 0}, {800, 0}, 5, 0));
        auto& s = net.mutableSegments();
        s[inner].next = {straight};
        s[outer].next = {exitRamp};
        s[straight].next = {city};
        s[exitRamp].next = {depot};
        net.finalize();
    }
};

}  // namespace

TEST_CASE("lane neighbours are parallel lanes of the same road and direction") {
    Grid g;
    const auto n = g.net.laneNeighbors(g.inner);
    REQUIRE(n.size() == 1);
    CHECK(n[0] == g.outer);
    CHECK(g.net.laneNeighbors(g.straight).empty());
}

TEST_CASE("A* finds a route that needs a lane change and never invents edges") {
    Grid g;
    const Route r = planRoute(g.net, g.inner, 20.0, {g.depot});
    REQUIRE(r.found);
    REQUIRE(r.steps.size() == 4);
    CHECK(r.steps[0].segment == g.inner);
    CHECK(r.steps[1].segment == g.outer);
    CHECK(r.steps[1].laneChange);
    CHECK(r.steps[2].segment == g.exitRamp);
    CHECK(r.steps[3].segment == g.depot);
    // Every consecutive pair is a real successor or a real lane neighbour.
    for (std::size_t i = 1; i < r.steps.size(); ++i) {
        const auto prev = r.steps[i - 1].segment;
        const auto cur = r.steps[i].segment;
        const auto& next = g.net.segment(prev).next;
        const auto neigh = g.net.laneNeighbors(prev);
        const bool linked = std::find(next.begin(), next.end(), cur) != next.end();
        const bool sideways = std::find(neigh.begin(), neigh.end(), cur) != neigh.end();
        CHECK((r.steps[i].laneChange ? sideways : linked));
    }
}

TEST_CASE("A* reports unreachable destinations") {
    Grid g;
    // The depot cannot reach the city: no edge leads there.
    const Route r = planRoute(g.net, g.depot, 0.0, {g.city});
    CHECK_FALSE(r.found);
    CHECK_FALSE(r.failure.empty());
    CHECK_FALSE(planRoute(g.net, g.inner, 0.0, {}).found);
}

TEST_CASE("A* prefers staying in lane when the lane already leads there") {
    Grid g;
    const Route r = planRoute(g.net, g.inner, 0.0, {g.city});
    REQUIRE(r.found);
    for (const auto& s : r.steps) CHECK_FALSE(s.laneChange);
    CHECK(r.length == Approx(800.0).epsilon(0.01));
}

TEST_CASE("route-following path blends across the lane change and announces it") {
    Grid g;
    const Route r = planRoute(g.net, g.inner, 20.0, {g.depot});
    REQUIRE(r.found);
    LaneMatch m;
    m.segment = g.inner;
    m.s = 20.0;
    PathBuildParams pp;
    pp.ahead = 500.0;
    const PlannedPath p = buildPlannedPath(g.net, m, pp, {}, &r);
    CHECK(p.onRoute);
    CHECK(p.nextManeuver == "Change Lane Right");
    REQUIRE(p.path.valid());
    // Starts on the inner lane at the truck, ends down the exit ramp.
    CHECK(p.path.positionAt(p.truckS).y == Approx(0.0).epsilon(0.05));
    CHECK(p.path.points().back().pos.y < -100.0);
    // The lane change (on the two-lane road, x <= 200) is gradual: every 2 m sample
    // moves sideways by well under a metre, and the blend reaches the outer lane.
    double maxStep = 0.0;
    for (std::size_t i = 1; i < p.path.size(); ++i) {
        if (p.path[i].pos.x > 200.0) break;
        maxStep = std::max(maxStep, std::abs(p.path[i].pos.y - p.path[i - 1].pos.y));
    }
    CHECK(maxStep < 0.5);
    // By the end of the two-lane road the blend has fully reached the outer lane.
    double yNearEnd = 0.0;
    for (std::size_t i = 0; i < p.path.size(); ++i) {
        if (p.path[i].pos.x <= 196.0) yNearEnd = p.path[i].pos.y;
    }
    CHECK(yNearEnd == Approx(-4.5).epsilon(0.05));
    CHECK(p.routeRemaining > 200.0);
}

TEST_CASE("without a route the path follows the road straight on") {
    Grid g;
    LaneMatch m;
    m.segment = g.inner;
    m.s = 20.0;
    PathBuildParams pp;
    pp.ahead = 500.0;
    const PlannedPath p = buildPlannedPath(g.net, m, pp, {}, nullptr);
    CHECK_FALSE(p.onRoute);
    CHECK(p.path.points().back().pos.y == Approx(0.0));
}

TEST_CASE("the in-game GPS route steers A* onto its branch") {
    // Two ways from `start` to `goal`: straight on (200 m) or a detour north (~360 m).
    RoadNetwork net;
    const auto start = net.add(lane({0, 0}, {100, 0}, 1));
    const auto direct = net.add(lane({100, 0}, {300, 0}, 2));
    const auto detourA = net.add(lane({100, 0}, {200, 150}, 3));
    const auto detourB = net.add(lane({200, 150}, {300, 0}, 4));
    const auto goal = net.add(lane({300, 0}, {400, 0}, 5));
    auto& s = net.mutableSegments();
    s[start].next = {direct, detourA};
    s[direct].next = {goal};
    s[detourA].next = {detourB};
    s[detourB].next = {goal};
    net.finalize();

    const Route shortest = planRoute(net, start, 10.0, {goal});
    REQUIRE(shortest.found);
    CHECK(shortest.find(direct) >= 0);

    // The GPS goes the long way round; its nodes sit on the road centre line.
    RouteOptions opts;
    opts.corridor = std::make_shared<const GpsCorridor>(
        std::vector<Vec2>{{0, 2}, {100, 2}, {150, 77}, {200, 152}, {250, 77}, {300, 2}, {400, 2}});
    CHECK(opts.corridor->distanceTo({0, 0}) == Approx(2.0));
    CHECK(opts.corridor->distanceTo({200, 0}) > 30.0);
    const Route gps = planRoute(net, start, 10.0, {goal}, opts);
    REQUIRE(gps.found);
    CHECK(gps.find(detourA) >= 0);
    CHECK(gps.find(direct) < 0);
}

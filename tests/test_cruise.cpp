#include "TestSupport.h"

#include "control/GameCruise.h"
#include "map/RoutePlanner.h"
#include "math/MathUtil.h"

using namespace atspilot;

TEST_CASE("cruise manager switches the game's cruise on only at a usable speed") {
    GameCruiseManager m;
    auto out = m.update(0.0, 5.0, 0.0, 20.0, false, false);
    CHECK(out.ownPedals);
    CHECK_FALSE(out.buttons.cruiseToggle);  // below the minimum
    out = m.update(1.0, 19.5, 0.0, 20.0, false, false);
    CHECK(out.buttons.cruiseToggle);
    // No repeated toggles while waiting for confirmation.
    out = m.update(1.1, 19.6, 0.0, 20.0, false, false);
    CHECK_FALSE(out.buttons.cruiseToggle);
}

TEST_CASE("cruise manager nudges the set speed and leaves the pedals alone") {
    GameCruiseManager m;
    m.update(0.0, 20.0, 20.0, 20.0, false, false);
    auto out = m.update(1.0, 20.0, 20.0, 15.0, false, false);
    CHECK_FALSE(out.ownPedals);
    CHECK(out.buttons.cruiseDec);
    // Presses are spaced so the game sees separate key presses.
    out = m.update(1.05, 20.0, 19.7, 15.0, false, false);
    CHECK_FALSE(out.buttons.cruiseDec);
    out = m.update(1.5, 20.0, 19.4, 25.0, false, false);
    CHECK(out.buttons.cruiseInc);
}

TEST_CASE("cruise manager distinguishes its own cancellations from the game's") {
    GameCruiseManager m;
    m.update(0.0, 20.0, 20.0, 20.0, false, false);
    // ATSPilot braked: cruise switching off is expected.
    auto out = m.update(0.1, 20.0, 0.0, 10.0, true, true);
    CHECK_FALSE(out.cancelledExternally);
    GameCruiseManager n;
    n.update(0.0, 20.0, 20.0, 20.0, false, false);
    out = n.update(5.0, 20.0, 0.0, 20.0, false, false);
    CHECK(out.cancelledExternally);
}

TEST_CASE("cruise manager adopts set-speed changes made by the player") {
    GameCruiseManager m;
    auto out = m.update(0.0, 20.0, 20.0, 20.0, false, false);
    CHECK(out.playerSetSpeed);  // already on at engagement
    out = m.update(2.0, 20.0, 20.0, 20.0, false, false);
    CHECK_FALSE(out.playerSetSpeed);
    out = m.update(4.0, 20.0, 22.0, 20.0, false, false);
    CHECK(out.playerSetSpeed);
    CHECK(out.playerSpeed == doctest::Approx(22.0));
}

TEST_CASE("cruise manager learns a higher minimum when the game refuses") {
    GameCruiseParams p;
    p.minSpeed = 5.0;
    GameCruiseManager m(p);
    m.update(0.0, 6.0, 0.0, 6.0, false, false);  // presses toggle
    for (double t = 0.05; t < 1.5; t += 1.0 / 60.0) m.update(t, 6.0, 0.0, 6.0, false, false);
    CHECK(m.learnedMinSpeed() > 6.0);
}

TEST_CASE("route matching picks the branch whose length equals the GPS distance") {
    // Start lane splits into a short way (500 m) and a long way (900 m) to the goal.
    RoadNetwork net;
    auto lane = [](Vec2 a, Vec2 b) {
        LaneSegment s;
        for (int i = 0; i <= 10; ++i) {
            const Vec2 p = lerp(a, b, i / 10.0);
            s.points.push_back({static_cast<float>(p.x), static_cast<float>(p.y), 0.0f});
        }
        return s;
    };
    const auto start = net.add(lane({0, 0}, {100, 0}));
    const auto shortWay = net.add(lane({100, 0}, {600, 0}));
    const auto longA = net.add(lane({100, 0}, {300, 300}));
    const auto longB = net.add(lane({300, 300}, {600, 0}));
    const auto goal = net.add(lane({600, 0}, {700, 0}));
    auto& s = net.mutableSegments();
    s[start].next = {shortWay, longA};
    s[shortWay].next = {goal};
    s[longA].next = {longB};
    s[longB].next = {goal};
    net.finalize();

    const Route shortest = planRouteMatching(net, start, 0.0, {goal}, 0.0, 0.04);
    REQUIRE(shortest.found);
    CHECK(shortest.steps[1].segment == shortWay);

    const double longLength = 100.0 + net.segment(longA).length + net.segment(longB).length + 100.0;
    const Route gps = planRouteMatching(net, start, 0.0, {goal}, longLength, 0.04);
    REQUIRE(gps.found);
    CHECK(gps.gpsMatched);
    CHECK(gps.steps[1].segment == longA);

    const Route unmatched = planRouteMatching(net, start, 0.0, {goal}, 5000.0, 0.04);
    CHECK_FALSE(unmatched.gpsMatched);
}

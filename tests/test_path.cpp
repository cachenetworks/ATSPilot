#include "TestSupport.h"

#include "math/MathUtil.h"
#include "path/Path.h"
#include "sim/VehicleSim.h"

using namespace atspilot;
using doctest::Approx;

TEST_CASE("path arc length accumulates") {
    Path p;
    p.append({0, 0});
    p.append({3, 4});
    p.append({3, 10});
    CHECK(p.length() == Approx(11.0));
    CHECK(p[1].s == Approx(5.0));
    // Duplicate points are ignored.
    p.append({3, 10});
    CHECK(p.size() == 3);
}

TEST_CASE("nearest path point and signed cross-track error") {
    const Path p = sim::makeStraight(100.0, 10.0);
    const auto left = p.project({42.0, 3.0});
    REQUIRE(left);
    CHECK(left->s == Approx(42.0));
    CHECK(left->crossTrackError == Approx(3.0));
    CHECK(left->yaw == Approx(0.0));
    const auto right = p.project({42.0, -2.0});
    REQUIRE(right);
    CHECK(right->crossTrackError == Approx(-2.0));
    // Beyond the end clamps to the final point.
    const auto beyond = p.project({150.0, 0.0});
    REQUIRE(beyond);
    CHECK(beyond->s == Approx(100.0));
}

TEST_CASE("windowed projection prefers the hinted stretch of a looping path") {
    // A loop that passes close to its own start: a global search from a point near
    // the end must not snap back to the beginning when hinted.
    const Path loop = sim::makeArc(0.0, 30.0, degToRad(350.0), 0.0, 2.0);
    const Vec2 nearEnd = loop.positionAt(loop.length() - 1.0);
    const auto hinted = loop.project(nearEnd, loop.size() - 3, 5);
    REQUIRE(hinted);
    CHECK(hinted->s > loop.length() - 10.0);
}

TEST_CASE("positionAt interpolates and yawAt follows segments") {
    Path p;
    p.append({0, 0});
    p.append({10, 0});
    p.append({10, 10});
    const Vec2 a = p.positionAt(5.0);
    CHECK(a.x == Approx(5.0));
    const Vec2 b = p.positionAt(15.0);
    CHECK(b.x == Approx(10.0));
    CHECK(b.y == Approx(5.0));
    CHECK(p.yawAt(15.0) == Approx(kPi / 2));
}

TEST_CASE("curvature estimate on a synthetic arc") {
    const Path arc = sim::makeArc(50.0, 120.0, degToRad(90.0), 50.0, 2.0);
    const double mid = 50.0 + 120.0 * degToRad(45.0);
    CHECK(arc.curvatureAt(mid, 10.0) == Approx(1.0 / 120.0).epsilon(0.02));
    CHECK(arc.curvatureAt(20.0, 10.0) == Approx(0.0));
    const Path right = sim::makeArc(50.0, 120.0, degToRad(-90.0), 50.0, 2.0);
    CHECK(right.curvatureAt(mid, 10.0) < 0.0);
}

TEST_CASE("resampling keeps geometry and length") {
    const Path arc = sim::makeArc(20.0, 60.0, degToRad(60.0), 20.0, 7.0);
    const Path r = arc.resampled(1.0);
    CHECK(r.length() == Approx(arc.length()).epsilon(0.01));
    CHECK(r.size() > arc.size());
}

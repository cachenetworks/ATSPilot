#include "TestSupport.h"

#include "math/Coordinates.h"
#include "math/MathUtil.h"

using namespace atspilot;
using doctest::Approx;

TEST_CASE("normalizeAngle wraps into (-pi, pi]") {
    CHECK(normalizeAngle(0.0) == Approx(0.0));
    CHECK(normalizeAngle(kPi) == Approx(kPi));
    CHECK(normalizeAngle(-kPi) == Approx(kPi));
    CHECK(normalizeAngle(3 * kPi) == Approx(kPi));
    CHECK(normalizeAngle(kTwoPi + 0.5) == Approx(0.5));
    CHECK(normalizeAngle(-kTwoPi - 0.5) == Approx(-0.5));
}

TEST_CASE("headingDifference takes the short way round") {
    CHECK(headingDifference(degToRad(350), degToRad(10)) == Approx(degToRad(20)));
    CHECK(headingDifference(degToRad(10), degToRad(350)) == Approx(degToRad(-20)));
    CHECK(headingDifference(degToRad(-170), degToRad(170)) == Approx(degToRad(-20)));
}

TEST_CASE("clamp, lerp and approach") {
    CHECK(clamp(5.0, 0.0, 1.0) == 1.0);
    CHECK(clamp(-5.0, 0.0, 1.0) == 0.0);
    CHECK(lerp(2.0, 4.0, 0.25) == Approx(2.5));
    CHECK(approach(0.0, 1.0, 0.3) == Approx(0.3));
    CHECK(approach(0.9, 1.0, 0.3) == Approx(1.0));
    CHECK(approach(0.0, -1.0, 0.3) == Approx(-0.3));
}

TEST_CASE("vector products") {
    const Vec2 a{1, 0}, b{0, 1};
    CHECK(dot(a, b) == Approx(0.0));
    CHECK(cross(a, b) == Approx(1.0));
    CHECK(cross(b, a) == Approx(-1.0));
    const Vec3 x{1, 0, 0}, y{0, 1, 0};
    const Vec3 z = cross(x, y);
    CHECK(z.z == Approx(1.0));
}

TEST_CASE("curvature from three points matches circle radius and turn direction") {
    const double r = 50.0;
    const Vec2 a{r * std::cos(0.0), r * std::sin(0.0)};
    const Vec2 b{r * std::cos(0.2), r * std::sin(0.2)};
    const Vec2 c{r * std::cos(0.4), r * std::sin(0.4)};
    CHECK(curvatureFromPoints(a, b, c) == Approx(1.0 / r).epsilon(1e-6));  // counterclockwise
    CHECK(curvatureFromPoints(c, b, a) == Approx(-1.0 / r).epsilon(1e-6));
    CHECK(curvatureFromPoints({0, 0}, {1, 0}, {2, 0}) == Approx(0.0));
}

TEST_CASE("quaternion rotation") {
    // 90 degrees about +Y.
    const double h = std::sqrt(0.5);
    const Quat q{h, 0.0, h, 0.0};
    const Vec3 v = q.rotate({0, 0, -1});
    CHECK(v.x == Approx(-1.0));
    CHECK(v.z == Approx(0.0).epsilon(1e-9));
}

TEST_CASE("hermite interpolation hits endpoints and tangents") {
    const Vec3 p0{0, 0, 0}, p1{10, 0, 0}, m{10, 0, 0};
    CHECK(hermite(p0, m, p1, m, 0.0).x == Approx(0.0));
    CHECK(hermite(p0, m, p1, m, 1.0).x == Approx(10.0));
    CHECK(hermite(p0, m, p1, m, 0.5).x == Approx(5.0));
    CHECK(hermiteTangent(p0, m, p1, m, 0.0).x == Approx(10.0));
}

TEST_CASE("SDK heading converts to plan yaw per the documented convention") {
    // 0 = north (-Z world = +y plan), 0.25 = west, 0.5 = south, 0.75 = east.
    CHECK(coords::sdkHeadingToYaw(0.0) == Approx(kPi / 2));
    CHECK(std::abs(coords::sdkHeadingToYaw(0.25)) == Approx(kPi));
    CHECK(coords::sdkHeadingToYaw(0.5) == Approx(-kPi / 2));
    CHECK(coords::sdkHeadingToYaw(0.75) == Approx(0.0).epsilon(1e-9));
    for (double h : {0.0, 0.1, 0.33, 0.5, 0.8, 0.999}) {
        CHECK(coords::yawToSdkHeading(coords::sdkHeadingToYaw(h)) == Approx(h).epsilon(1e-9));
    }
}

TEST_CASE("world/plan conversion keeps north up and is invertible") {
    const Vec3 north{0, 5, -100};
    const Vec2 p = coords::worldToPlan(north);
    CHECK(p.x == Approx(0.0));
    CHECK(p.y == Approx(100.0));
    const Vec3 back = coords::planToWorld(p, 5.0);
    CHECK(back.z == Approx(-100.0));
    CHECK(back.y == Approx(5.0));
    // The SDK forward vector (vehicle -Z) rotated by the heading points along the yaw.
    for (double h : {0.0, 0.125, 0.25, 0.6}) {
        const double a = h * kTwoPi;
        // Rotation about Y as in the SDK telemetry_position example.
        const Vec3 fwd{-std::sin(a), 0.0, -std::cos(a)};
        CHECK(headingDifference(coords::worldDirectionToYaw(fwd), coords::sdkHeadingToYaw(h)) ==
              Approx(0.0).epsilon(1e-9));
    }
}

TEST_CASE("circle-segment intersection returns the far intersection") {
    const auto t = circleSegmentIntersection({0, 0}, 5.0, {0, 0}, {10, 0});
    REQUIRE(t);
    CHECK(*t == Approx(0.5));
    CHECK_FALSE(circleSegmentIntersection({0, 0}, 1.0, {5, 5}, {6, 5}));
}

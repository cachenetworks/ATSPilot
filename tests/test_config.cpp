#include "TestSupport.h"

#include "config/Config.h"
#include "config/Toml.h"

using namespace atspilot;
using doctest::Approx;

TEST_CASE("TOML subset parses sections, types and comments") {
    const auto doc = TomlDocument::parse(R"(
# comment
top = 1
[a]
num = 2.5   # trailing comment
neg = -3
flag = true
name = "x # not a comment"
big = 1_000
)");
    CHECK(doc.errors().empty());
    CHECK(*doc.getNumber("", "top") == Approx(1.0));
    CHECK(*doc.getNumber("a", "num") == Approx(2.5));
    CHECK(*doc.getNumber("a", "neg") == Approx(-3.0));
    CHECK(*doc.getBool("a", "flag"));
    CHECK(*doc.getString("a", "name") == "x # not a comment");
    CHECK(*doc.getNumber("a", "big") == Approx(1000.0));
    CHECK_FALSE(doc.getBool("a", "num"));
}

TEST_CASE("TOML reports malformed lines with line numbers") {
    const auto doc = TomlDocument::parse("[ok]\nthis is wrong\nx = [1, 2]\n[bad\n");
    REQUIRE(doc.errors().size() == 3);
    CHECK(doc.errors()[0].line == 2);
    CHECK(doc.errors()[1].line == 3);
    CHECK(doc.errors()[2].line == 4);
}

TEST_CASE("default config text round-trips to default values without warnings") {
    const ConfigLoadResult r = loadConfig(defaultConfigText());
    for (const auto& w : r.warnings) MESSAGE(w);
    CHECK(r.warnings.empty());
    const Config d;
    CHECK(r.config.speed.maxSpeed == Approx(d.speed.maxSpeed));
    CHECK(r.config.cruise.gains.kp == Approx(d.cruise.gains.kp));
    CHECK(r.config.cruise.gains.ki == Approx(d.cruise.gains.ki));
    CHECK(r.config.cruise.gains.kd == Approx(d.cruise.gains.kd));
    CHECK(r.config.steering.lateral.lookaheadBase == Approx(d.steering.lateral.lookaheadBase));
    CHECK(r.config.steering.shaper.maxRate == Approx(d.steering.shaper.maxRate));
    CHECK(r.config.steering.outputSign == Approx(d.steering.outputSign));
    CHECK(r.config.safety.steeringOverrideThreshold == Approx(d.safety.steeringOverrideThreshold));
    CHECK(r.config.safety.maxCrossTrack == Approx(d.safety.maxCrossTrack));
    CHECK(r.config.planner.maxLateralAccel == Approx(d.planner.maxLateralAccel));
    CHECK(r.config.controls.toggleAutopilot == d.controls.toggleAutopilot);
    CHECK(r.config.map.laneWidth == Approx(d.map.laneWidth));
    CHECK(r.config.debug.recordTelemetry == d.debug.recordTelemetry);
}

TEST_CASE("invalid config values fall back safely with warnings") {
    const ConfigLoadResult r = loadConfig(R"(
[speed]
max = 500
units = "furlongs"
[steering]
controller = "magic"
lookahead_min_m = 70
lookahead_max_m = 20
[cruise]
kp = "fast"
[safety]
driver_override = 1
[mystery]
key = 3
)");
    CHECK(r.warnings.size() >= 6);
    CHECK(r.config.speed.maxSpeed == Approx(130.0));  // clamped
    CHECK((r.config.speed.units == SpeedUnits::Mph));
    CHECK((r.config.steering.lateral.algorithm == LateralAlgorithm::PurePursuit));
    CHECK(r.config.steering.lateral.lookaheadMin <= r.config.steering.lateral.lookaheadMax);
    CHECK(r.config.cruise.gains.kp == Approx(Config{}.cruise.gains.kp));
    CHECK(r.config.safety.driverOverride);  // safety stays on when the value is invalid
}

TEST_CASE("garbage input never throws") {
    CHECK_NOTHROW(loadConfig(std::string("\0\xff\xfe[[[===\"\"\"", 12)));
    CHECK_NOTHROW(loadConfig(""));
}

TEST_CASE("speed unit conversion") {
    CHECK(speedToMps(65.0, SpeedUnits::Mph) == Approx(29.0576));
    CHECK(speedToMps(100.0, SpeedUnits::Kph) == Approx(27.7778).epsilon(1e-4));
    CHECK(mpsToSpeed(speedToMps(55.0, SpeedUnits::Mph), SpeedUnits::Mph) == Approx(55.0));
}

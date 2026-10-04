#pragma once

#include <string>
#include <vector>

#include "control/GameCruise.h"
#include "control/Lateral.h"
#include "control/Longitudinal.h"
#include "control/SpeedPlanner.h"
#include "control/SteeringShaper.h"

namespace atspilot {

enum class SpeedUnits { Mph, Kph };

struct SpeedConfig {
    SpeedUnits units = SpeedUnits::Mph;
    double maxSpeed = 65.0;        // in `units`; used until the player sets the game's cruise speed
    double limitOffset = 0.0;      // added to the posted limit, in `units`
    bool followSpeedLimit = true;
};

struct SteeringConfig {
    LateralParams lateral;
    SteeringShaperParams shaper;
    double initialMaxWheelAngleDeg = 35.0;
    bool learnSteeringRatio = true;
    // semantical.steering is subtracted in ATS's `steering` mix (controls.sii),
    // so a positive (left) command must be sent as a negative value.
    double outputSign = -1.0;
};

struct SafetyConfig {
    bool driverOverride = true;
    double steeringOverrideThreshold = 0.20;
    double brakeOverrideThreshold = 0.10;
    double throttleOverrideThreshold = 0.30;
    double telemetryTimeout = 0.5;   // s
    double pathTimeout = 1.5;        // s
    double commandTimeout = 0.25;    // s
    double warnCrossTrack = 1.2;     // m
    double maxCrossTrack = 3.0;      // m; beyond this the path is considered lost
    double maxHeadingErrorDeg = 35.0;
};

// ATSPilot has exactly one control: on/off. Speed is adjusted with the game's
// own cruise control keys, and braking or steering takes over at any time.
struct ControlsConfig {
    std::string toggle = "F9";
};

// Use of the game's built-in driving systems.
struct IngameConfig {
    // Hold speed with the game's cruise control, so the game's adaptive cruise
    // control and emergency brake assist (on trucks that have them) handle traffic.
    bool useCruiseControl = true;
    GameCruiseParams cruise;
    // Operate the turn signals for lane changes, exits and turns.
    bool useBlinkers = true;
    double blinkerDistance = 150.0;  // m before the manoeuvre
    // Ask the game to park the trailer once the depot entrance is reached.
    bool quickPark = true;
};

struct IntersectionConfig {
    bool stopAtSignals = true;      // traffic-light controlled junction lanes (state is unknown to ATSPilot)
    bool stopAtStopSigns = true;
    double yieldSpeed = 4.0;        // m/s through give-way and railway-crossing lanes
    double goTapMaxSeconds = 1.5;   // a throttle tap shorter than this while waiting means "go"
    double stopLineMargin = 1.5;    // m between the truck's front and the stop line
};

struct MapConfig {
    bool enabled = true;
    std::string gameDir;   // empty = derive from the plugin location
    double laneWidth = 4.5;
    double pathAhead = 600.0;    // m of driving path kept ahead of the truck
    double pathBehind = 30.0;
    double maxLocalizationDistance = 12.0;  // m from a lane centre
};

struct RouteConfig {
    bool enabled = true;
    double laneChangeCost = 60.0;  // m of driving a lane change is worth when routing
    double recalcAfter = 1.0;      // s off the route before recalculating
    // Keep the route consistent with the in-game GPS by comparing its remaining
    // distance with the game's navigation distance (see docs/map-parsing.md).
    bool matchGameGps = true;
    double gpsTolerance = 0.04;    // relative distance mismatch tolerated
};

enum class DrivingProfile { Auto, Comfort, Normal, Assertive, HeavyHaul };

const char* toString(DrivingProfile p);

struct HudConfig {
    bool enabled = true;
    std::string corner = "top_right";  // top_left, top_right, bottom_left, bottom_right
    double scale = 1.0;
    double opacity = 0.85;
};

struct DebugConfig {
    bool logging = true;
    std::string logLevel = "info";
    double logMaxMb = 5.0;
    int logFiles = 3;
    bool recordTelemetry = false;
    double recordMaxMb = 50.0;
    bool statusFile = true;
    bool showPath = false;
};

struct Config {
    bool autopilotEnabled = true;
    SpeedConfig speed;
    SteeringConfig steering;
    LongitudinalParams cruise;
    SpeedPlannerParams planner;
    double aggressiveness = 1.0;
    DrivingProfile profile = DrivingProfile::Auto;
    SafetyConfig safety;
    ControlsConfig controls;
    IngameConfig ingame;
    IntersectionConfig intersections;
    MapConfig map;
    RouteConfig route;
    bool audioEnabled = true;
    HudConfig hud;
    DebugConfig debug;
};

struct ConfigLoadResult {
    Config config;
    std::vector<std::string> warnings;
};

// Parses and validates. Never throws: invalid values fall back to defaults and
// produce a warning, so a bad edit cannot stop the plugin from loading.
ConfigLoadResult loadConfig(const std::string& text);

// The documented default configuration file, written when none exists.
std::string defaultConfigText();

// A driving profile scales the tuning on top of the configured values.
// `Auto` picks HeavyHaul for loads above 25 t and Normal otherwise.
DrivingProfile resolveProfile(DrivingProfile p, double cargoMassKg);
Config applyProfile(const Config& base, DrivingProfile resolved);

double speedToMps(double value, SpeedUnits units);
double mpsToSpeed(double mps, SpeedUnits units);
const char* unitLabel(SpeedUnits units);

}  // namespace atspilot

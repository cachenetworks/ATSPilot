#pragma once

#include <string>
#include <vector>

#include "control/Lateral.h"
#include "control/Longitudinal.h"
#include "control/SpeedPlanner.h"
#include "control/SteeringShaper.h"

namespace atspilot {

enum class SpeedUnits { Mph, Kph };

struct SpeedConfig {
    SpeedUnits units = SpeedUnits::Mph;
    double maxSpeed = 65.0;        // in `units`
    double limitOffset = 0.0;      // added to the posted limit, in `units`
    bool followSpeedLimit = true;
    double step = 5.0;             // set-speed change per key press, in `units`
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

struct ControlsConfig {
    std::string toggleAutopilot = "F9";
    std::string toggleLaneAssist = "F8";
    std::string toggleCruise = "Insert";
    std::string speedUp = "Equals";
    std::string speedDown = "Minus";
    std::string resume = "Shift+F9";
    std::string cancel = "Delete";
    std::string emergencyDisable = "Shift+Delete";
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
    SafetyConfig safety;
    ControlsConfig controls;
    MapConfig map;
    RouteConfig route;
    bool audioEnabled = true;
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

double speedToMps(double value, SpeedUnits units);
double mpsToSpeed(double mps, SpeedUnits units);
const char* unitLabel(SpeedUnits units);

}  // namespace atspilot

#include "config/Config.h"

#include <functional>
#include <set>

#include "config/Toml.h"
#include "math/MathUtil.h"

namespace atspilot {
namespace {

struct NumberBinding {
    const char* section;
    const char* key;
    double* target;
    double min;
    double max;
};

struct BoolBinding {
    const char* section;
    const char* key;
    bool* target;
};

struct StringBinding {
    const char* section;
    const char* key;
    std::string* target;
    std::function<bool(const std::string&)> validate;
};

bool isLogLevel(const std::string& s) {
    static const std::set<std::string> levels{"trace", "debug", "info", "warn", "error", "critical"};
    return levels.count(s) > 0;
}

bool nonEmpty(const std::string& s) { return !s.empty(); }

}  // namespace

double speedToMps(double value, SpeedUnits units) { return value * (units == SpeedUnits::Mph ? kMphToMps : kKphToMps); }
double mpsToSpeed(double mps, SpeedUnits units) { return mps / (units == SpeedUnits::Mph ? kMphToMps : kKphToMps); }
const char* unitLabel(SpeedUnits units) { return units == SpeedUnits::Mph ? "mph" : "km/h"; }

ConfigLoadResult loadConfig(const std::string& text) {
    ConfigLoadResult result;
    Config& c = result.config;
    const TomlDocument doc = TomlDocument::parse(text);
    for (const auto& e : doc.errors()) {
        result.warnings.push_back("config line " + std::to_string(e.line) + ": " + e.message);
    }

    double maxWheelAngle = c.steering.initialMaxWheelAngleDeg;
    double logFiles = c.debug.logFiles;
    std::string units = "mph";
    std::string controller = "pure_pursuit";

    const NumberBinding numbers[] = {
        {"speed", "max", &c.speed.maxSpeed, 5.0, 130.0},
        {"speed", "limit_offset", &c.speed.limitOffset, -30.0, 30.0},
        {"speed", "step", &c.speed.step, 0.5, 20.0},
        {"steering", "lookahead_base_m", &c.steering.lateral.lookaheadBase, 2.0, 80.0},
        {"steering", "lookahead_speed_factor", &c.steering.lateral.lookaheadSpeedFactor, 0.0, 3.0},
        {"steering", "lookahead_min_m", &c.steering.lateral.lookaheadMin, 2.0, 80.0},
        {"steering", "lookahead_max_m", &c.steering.lateral.lookaheadMax, 5.0, 200.0},
        {"steering", "stanley_gain", &c.steering.lateral.stanleyGain, 0.05, 10.0},
        {"steering", "stanley_softening", &c.steering.lateral.stanleySoftening, 0.1, 20.0},
        {"steering", "max_steering_rate", &c.steering.shaper.maxRate, 0.05, 10.0},
        {"steering", "high_speed_rate_scale", &c.steering.shaper.highSpeedRateScale, 0.05, 1.0},
        {"steering", "smoothing_time_s", &c.steering.shaper.smoothingTime, 0.0, 1.0},
        {"steering", "max_lateral_accel", &c.steering.shaper.maxLateralAccel, 0.5, 6.0},
        {"steering", "trailer_conservatism", &c.steering.shaper.trailerConservatism, 0.2, 1.0},
        {"steering", "max_wheel_angle_deg", &maxWheelAngle, 5.0, 70.0},
        {"steering", "output_sign", &c.steering.outputSign, -1.0, 1.0},
        {"cruise", "kp", &c.cruise.gains.kp, 0.0, 5.0},
        {"cruise", "ki", &c.cruise.gains.ki, 0.0, 2.0},
        {"cruise", "kd", &c.cruise.gains.kd, 0.0, 2.0},
        {"cruise", "brake_enter", &c.cruise.brakeEnter, 0.0, 0.5},
        {"cruise", "brake_exit", &c.cruise.brakeExit, 0.0, 0.5},
        {"cruise", "max_normal_brake", &c.cruise.maxNormalBrake, 0.05, 1.0},
        {"cruise", "max_strong_brake", &c.cruise.maxStrongBrake, 0.05, 1.0},
        {"cruise", "emergency_brake", &c.cruise.emergencyBrake, 0.1, 1.0},
        {"planner", "max_lateral_accel", &c.planner.maxLateralAccel, 0.3, 4.0},
        {"planner", "comfort_decel", &c.planner.comfortDecel, 0.3, 4.0},
        {"planner", "min_curve_speed_mps", &c.planner.minCurveSpeed, 1.0, 20.0},
        {"planner", "horizon_m", &c.planner.horizon, 50.0, 2000.0},
        {"planner", "aggressiveness", &c.aggressiveness, 0.5, 1.5},
        {"safety", "steering_override_threshold", &c.safety.steeringOverrideThreshold, 0.02, 1.0},
        {"safety", "brake_override_threshold", &c.safety.brakeOverrideThreshold, 0.01, 1.0},
        {"safety", "throttle_override_threshold", &c.safety.throttleOverrideThreshold, 0.01, 1.0},
        {"safety", "telemetry_timeout_s", &c.safety.telemetryTimeout, 0.05, 5.0},
        {"safety", "path_timeout_s", &c.safety.pathTimeout, 0.1, 10.0},
        {"safety", "command_timeout_s", &c.safety.commandTimeout, 0.05, 2.0},
        {"safety", "warn_cross_track_m", &c.safety.warnCrossTrack, 0.1, 10.0},
        {"safety", "max_cross_track_m", &c.safety.maxCrossTrack, 0.5, 15.0},
        {"safety", "max_heading_error_deg", &c.safety.maxHeadingErrorDeg, 5.0, 90.0},
        {"map", "lane_width_m", &c.map.laneWidth, 2.5, 6.0},
        {"map", "path_ahead_m", &c.map.pathAhead, 100.0, 3000.0},
        {"map", "max_localization_distance_m", &c.map.maxLocalizationDistance, 2.0, 50.0},
        {"debug", "log_max_mb", &c.debug.logMaxMb, 0.1, 100.0},
        {"debug", "log_files", &logFiles, 1.0, 20.0},
        {"debug", "record_max_mb", &c.debug.recordMaxMb, 1.0, 2000.0},
    };
    const BoolBinding bools[] = {
        {"autopilot", "enabled", &c.autopilotEnabled},
        {"speed", "follow_speed_limit", &c.speed.followSpeedLimit},
        {"steering", "learn_steering_ratio", &c.steering.learnSteeringRatio},
        {"safety", "driver_override", &c.safety.driverOverride},
        {"map", "enabled", &c.map.enabled},
        {"audio", "enabled", &c.audioEnabled},
        {"debug", "logging", &c.debug.logging},
        {"debug", "record_telemetry", &c.debug.recordTelemetry},
        {"debug", "status_file", &c.debug.statusFile},
        {"debug", "show_path", &c.debug.showPath},
    };
    const StringBinding strings[] = {
        {"speed", "units", &units, [](const std::string& s) { return s == "mph" || s == "kph"; }},
        {"steering", "controller", &controller,
         [](const std::string& s) { return s == "pure_pursuit" || s == "stanley"; }},
        {"controls", "toggle_autopilot", &c.controls.toggleAutopilot, nonEmpty},
        {"controls", "toggle_lane_assist", &c.controls.toggleLaneAssist, nonEmpty},
        {"controls", "toggle_cruise", &c.controls.toggleCruise, nonEmpty},
        {"controls", "speed_up", &c.controls.speedUp, nonEmpty},
        {"controls", "speed_down", &c.controls.speedDown, nonEmpty},
        {"controls", "resume", &c.controls.resume, nonEmpty},
        {"controls", "cancel", &c.controls.cancel, nonEmpty},
        {"controls", "emergency_disable", &c.controls.emergencyDisable, nonEmpty},
        {"map", "game_dir", &c.map.gameDir, [](const std::string&) { return true; }},
        {"debug", "log_level", &c.debug.logLevel, isLogLevel},
    };

    std::set<std::string> known;
    for (const auto& b : numbers) {
        const std::string full = std::string(b.section) + "." + b.key;
        known.insert(full);
        if (!doc.has(b.section, b.key)) continue;
        const auto v = doc.getNumber(b.section, b.key);
        if (!v || !std::isfinite(*v)) {
            result.warnings.push_back(full + " must be a number; using default");
        } else if (*v < b.min || *v > b.max) {
            result.warnings.push_back(full + " out of range [" + std::to_string(b.min) + ", " + std::to_string(b.max) +
                                      "]; clamped");
            *b.target = clamp(*v, b.min, b.max);
        } else {
            *b.target = *v;
        }
    }
    for (const auto& b : bools) {
        const std::string full = std::string(b.section) + "." + b.key;
        known.insert(full);
        if (!doc.has(b.section, b.key)) continue;
        if (const auto v = doc.getBool(b.section, b.key)) *b.target = *v;
        else result.warnings.push_back(full + " must be true or false; using default");
    }
    for (const auto& b : strings) {
        const std::string full = std::string(b.section) + "." + b.key;
        known.insert(full);
        if (!doc.has(b.section, b.key)) continue;
        const auto v = doc.getString(b.section, b.key);
        if (v && b.validate(*v)) *b.target = *v;
        else result.warnings.push_back(full + " has an invalid value; using default");
    }
    for (const auto& k : doc.keys()) {
        if (!known.count(k)) result.warnings.push_back("unknown config key '" + k + "' ignored");
    }

    c.speed.units = units == "kph" ? SpeedUnits::Kph : SpeedUnits::Mph;
    c.steering.lateral.algorithm = lateralAlgorithmFromString(controller);
    c.steering.initialMaxWheelAngleDeg = maxWheelAngle;
    c.steering.outputSign = c.steering.outputSign < 0.0 ? -1.0 : 1.0;
    c.debug.logFiles = static_cast<int>(logFiles);

    // Cross-field consistency.
    if (c.steering.lateral.lookaheadMin > c.steering.lateral.lookaheadMax) {
        result.warnings.push_back("steering.lookahead_min_m exceeds lookahead_max_m; swapped");
        std::swap(c.steering.lateral.lookaheadMin, c.steering.lateral.lookaheadMax);
    }
    if (c.cruise.brakeExit > c.cruise.brakeEnter) {
        result.warnings.push_back("cruise.brake_exit must not exceed brake_enter; using brake_enter");
        c.cruise.brakeExit = c.cruise.brakeEnter;
    }
    if (c.cruise.maxNormalBrake > c.cruise.maxStrongBrake) {
        result.warnings.push_back("cruise.max_normal_brake exceeds max_strong_brake; clamped");
        c.cruise.maxNormalBrake = c.cruise.maxStrongBrake;
    }
    if (c.safety.warnCrossTrack > c.safety.maxCrossTrack) c.safety.warnCrossTrack = c.safety.maxCrossTrack;
    return result;
}

std::string defaultConfigText() {
    return R"(# ATSPilot configuration
# Edit while the game is closed, or run "sdk reinit" in the ATS console to reload.
# Invalid values never stop ATS from loading: they fall back to defaults and are
# reported in atspilot.log.

[autopilot]
enabled = true

[speed]
units = "mph"               # "mph" or "kph"
max = 65                    # highest set speed ATSPilot will use
limit_offset = 0            # added to the navigation speed limit
follow_speed_limit = true   # never exceed the limit reported by the in-game navigation
step = 5                    # set-speed change per key press

[steering]
controller = "pure_pursuit" # "pure_pursuit" or "stanley"
lookahead_base_m = 15.0
lookahead_speed_factor = 0.5
lookahead_min_m = 8.0
lookahead_max_m = 60.0
stanley_gain = 1.0
stanley_softening = 2.0
max_steering_rate = 1.5     # normalized steering units per second at low speed
high_speed_rate_scale = 0.35
smoothing_time_s = 0.08
max_lateral_accel = 2.5     # bounds how hard steering corrections may turn the truck
trailer_conservatism = 0.75 # steering rate multiplier per trailer
max_wheel_angle_deg = 35.0  # starting estimate; refined from telemetry while driving
learn_steering_ratio = true
output_sign = -1            # flip to 1 only if ATSPilot steers the wrong way

[cruise]
kp = 0.30
ki = 0.04
kd = 0.08
brake_enter = 0.10
brake_exit = 0.03
max_normal_brake = 0.45
max_strong_brake = 0.75
emergency_brake = 0.85

[planner]
max_lateral_accel = 1.6     # m/s^2 in curves for a bobtail on a dry road
comfort_decel = 1.2         # m/s^2 used to slow down before curves
min_curve_speed_mps = 4.0
horizon_m = 400
aggressiveness = 1.0        # 0.5 (gentle) .. 1.5 (assertive)

[safety]
driver_override = true
steering_override_threshold = 0.20
brake_override_threshold = 0.10
throttle_override_threshold = 0.30
telemetry_timeout_s = 0.5
path_timeout_s = 1.5
command_timeout_s = 0.25
warn_cross_track_m = 1.2
max_cross_track_m = 3.0
max_heading_error_deg = 35

[controls]
# Key names: F1-F24, A-Z, 0-9, Insert, Delete, Home, End, PageUp, PageDown,
# Up, Down, Left, Right, Minus, Equals, Backspace, Pause, Num0-Num9, NumPlus,
# NumMinus. Modifiers: Shift+, Ctrl+, Alt+. Keys are only read while ATS has focus.
toggle_autopilot = "F9"
toggle_lane_assist = "F8"
toggle_cruise = "Insert"
speed_up = "Equals"
speed_down = "Minus"
resume = "Shift+F9"
cancel = "Delete"
emergency_disable = "Shift+Delete"

[map]
enabled = true
game_dir = ""               # empty = detect from the plugin location
lane_width_m = 4.5
path_ahead_m = 600
max_localization_distance_m = 12

[audio]
enabled = true

[debug]
logging = true
log_level = "info"          # trace, debug, info, warn, error, critical
log_max_mb = 5
log_files = 3
record_telemetry = false    # CSV recording for controller tuning
record_max_mb = 50
status_file = true          # writes status.json for optional external displays
show_path = false
)";
}

}  // namespace atspilot

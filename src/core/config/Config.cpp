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

const char* toString(DrivingProfile p) {
    switch (p) {
        case DrivingProfile::Auto: return "auto";
        case DrivingProfile::Comfort: return "comfort";
        case DrivingProfile::Normal: return "normal";
        case DrivingProfile::Assertive: return "assertive";
        case DrivingProfile::HeavyHaul: return "heavy_haul";
    }
    return "?";
}

DrivingProfile resolveProfile(DrivingProfile p, double cargoMassKg) {
    if (p != DrivingProfile::Auto) return p;
    return cargoMassKg > 25000.0 ? DrivingProfile::HeavyHaul : DrivingProfile::Normal;
}

Config applyProfile(const Config& base, DrivingProfile resolved) {
    Config c = base;
    // Multipliers on the configured tuning: curve lateral acceleration, braking,
    // steering rate and maximum speed.
    double lat = 1.0, decel = 1.0, rate = 1.0, speedCap = 1e9;
    switch (resolved) {
        case DrivingProfile::Comfort: lat = 0.8; decel = 0.8; rate = 0.8; break;
        case DrivingProfile::Assertive: lat = 1.2; decel = 1.25; rate = 1.2; break;
        case DrivingProfile::HeavyHaul:
            lat = 0.7; decel = 0.7; rate = 0.7;
            speedCap = speedToMps(55.0, SpeedUnits::Mph);
            break;
        case DrivingProfile::Auto:
        case DrivingProfile::Normal: break;
    }
    c.planner.maxLateralAccel *= lat;
    c.planner.comfortDecel *= decel;
    c.steering.shaper.maxRate *= rate;
    c.speed.maxSpeed = std::min(c.speed.maxSpeed, mpsToSpeed(speedCap, c.speed.units));
    c.profile = resolved;
    return c;
}

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
    std::string profile = "auto";

    const NumberBinding numbers[] = {
        {"speed", "max", &c.speed.maxSpeed, 5.0, 130.0},
        {"speed", "limit_offset", &c.speed.limitOffset, -30.0, 30.0},
        {"steering", "lookahead_base_m", &c.steering.lateral.lookaheadBase, 2.0, 80.0},
        {"steering", "lookahead_speed_factor", &c.steering.lateral.lookaheadSpeedFactor, 0.0, 3.0},
        {"steering", "lookahead_min_m", &c.steering.lateral.lookaheadMin, 2.0, 80.0},
        {"steering", "lookahead_max_m", &c.steering.lateral.lookaheadMax, 5.0, 200.0},
        {"steering", "curve_lookahead_factor", &c.steering.lateral.curveLookaheadFactor, 0.0, 2.0},
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
        {"route", "lane_change_cost_m", &c.route.laneChangeCost, 5.0, 1000.0},
        {"route", "recalculate_after_s", &c.route.recalcAfter, 0.2, 10.0},
        {"route", "gps_tolerance", &c.route.gpsTolerance, 0.005, 0.5},
        {"ingame", "blinker_distance_m", &c.ingame.blinkerDistance, 20.0, 1000.0},
        {"ingame", "cruise_min_speed_mps", &c.ingame.cruise.minSpeed, 2.0, 25.0},
        {"intersections", "yield_speed_mps", &c.intersections.yieldSpeed, 1.0, 15.0},
        {"intersections", "go_tap_max_s", &c.intersections.goTapMaxSeconds, 0.2, 5.0},
        {"intersections", "stop_line_margin_m", &c.intersections.stopLineMargin, 0.0, 10.0},
        {"hud", "scale", &c.hud.scale, 0.5, 3.0},
        {"hud", "opacity", &c.hud.opacity, 0.2, 1.0},
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
        {"route", "enabled", &c.route.enabled},
        {"route", "match_game_gps", &c.route.matchGameGps},
        {"ingame", "use_cruise_control", &c.ingame.useCruiseControl},
        {"ingame", "use_blinkers", &c.ingame.useBlinkers},
        {"ingame", "quick_park", &c.ingame.quickPark},
        {"intersections", "stop_at_signals", &c.intersections.stopAtSignals},
        {"intersections", "stop_at_stop_signs", &c.intersections.stopAtStopSigns},
        {"hud", "enabled", &c.hud.enabled},
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
        {"controls", "toggle", &c.controls.toggle, nonEmpty},
        {"profile", "name", &profile,
         [](const std::string& v) {
             return v == "auto" || v == "comfort" || v == "normal" || v == "assertive" || v == "heavy_haul";
         }},
        {"hud", "corner", &c.hud.corner,
         [](const std::string& v) {
             return v == "top_left" || v == "top_right" || v == "bottom_left" || v == "bottom_right";
         }},
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
    c.profile = profile == "comfort"      ? DrivingProfile::Comfort
                : profile == "normal"     ? DrivingProfile::Normal
                : profile == "assertive"  ? DrivingProfile::Assertive
                : profile == "heavy_haul" ? DrivingProfile::HeavyHaul
                                          : DrivingProfile::Auto;

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

[controls]
# ATSPilot has one key: on/off. Set the speed with the game's own cruise
# control +/- keys; braking, steering or the throttle take over at any time.
# Key names: F1-F24, A-Z, 0-9, Insert, Delete, Home, End, PageUp, PageDown,
# Up, Down, Left, Right, Minus, Equals, Backspace, Pause, Num0-Num9, NumPlus,
# NumMinus. Modifiers: Shift+, Ctrl+, Alt+. Read only while ATS has focus.
toggle = "F9"

[profile]
name = "auto"               # auto (heavy_haul above 25 t), comfort, normal, assertive, heavy_haul

[speed]
units = "mph"               # "mph" or "kph"
max = 65                    # used until you set the game's cruise control speed
limit_offset = 0            # added to the navigation speed limit
follow_speed_limit = true   # never exceed the limit reported by the in-game navigation

[ingame]
use_cruise_control = true   # hold speed with the game's cruise control (its adaptive cruise handles traffic)
cruise_min_speed_mps = 8.5  # starting guess for the lowest speed the game's cruise accepts; learned
use_blinkers = true         # indicate lane changes, exits and turns
blinker_distance_m = 150
quick_park = true           # ask the game to park once the depot entrance is reached

[intersections]
stop_at_signals = true      # stop at traffic lights and wait for a throttle tap
stop_at_stop_signs = true   # stop at stop signs and wait for a throttle tap
yield_speed_mps = 4.0       # through give-way and railway-crossing lanes
go_tap_max_s = 1.5          # a throttle tap shorter than this while waiting means "go"
stop_line_margin_m = 1.5

[steering]
controller = "pure_pursuit" # "pure_pursuit" or "stanley"
lookahead_base_m = 15.0
lookahead_speed_factor = 0.5
lookahead_min_m = 4.0
lookahead_max_m = 60.0
curve_lookahead_factor = 0.35 # limits lookahead to this fraction of the tightest radius ahead; 0 = off
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
# ATSPilot's own pedal control, used below cruise-control speed and for braking.
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
comfort_decel = 1.2         # m/s^2 used to slow down before curves and stops
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

[map]
enabled = true
game_dir = ""               # empty = detect from the plugin location
lane_width_m = 4.5
path_ahead_m = 600
max_localization_distance_m = 12

[route]
enabled = true              # follow a route to the job destination (otherwise follow the road)
match_game_gps = true       # keep the route consistent with the in-game GPS distance
gps_tolerance = 0.04
lane_change_cost_m = 60     # how strongly routing avoids lane changes
recalculate_after_s = 1.0   # time off the route before recalculating

[hud]
enabled = true              # in-game status panel (needs borderless or windowed display mode)
corner = "top_right"        # top_left, top_right, bottom_left, bottom_right
scale = 1.0
opacity = 0.85

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

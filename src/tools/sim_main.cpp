// Controller simulation harness: runs synthetic scenarios through the real
// Autopilot code and prints tracking metrics, optionally exporting a CSV trace
// in the same column layout as the in-game telemetry recorder.
//
//   atspilot_sim                       run all scenarios
//   atspilot_sim --config file.toml    use a specific configuration
//   atspilot_sim --csv out_dir         write one CSV per scenario

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "config/Config.h"
#include "math/Coordinates.h"
#include "math/MathUtil.h"
#include "sim/Scenario.h"

using namespace atspilot;
using namespace atspilot::sim;

int main(int argc, char** argv) {
    Config cfg;
    std::filesystem::path csvDir;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--config" && i + 1 < argc) {
            std::ifstream f(argv[++i]);
            std::stringstream ss;
            ss << f.rdbuf();
            const ConfigLoadResult r = loadConfig(ss.str());
            for (const auto& w : r.warnings) std::cerr << "config: " << w << "\n";
            cfg = r.config;
        } else if (a == "--csv" && i + 1 < argc) {
            csvDir = argv[++i];
            std::filesystem::create_directories(csvDir);
        } else {
            std::cerr << "usage: atspilot_sim [--config file.toml] [--csv out_dir]\n";
            return 2;
        }
    }

    struct Case {
        const char* name;
        Path path;
        double speed;
        SimParams params;
    };
    SimParams loaded;
    loaded.trailerCount = 1;
    loaded.massFactor = 2.5;
    loaded.steeringLag = 0.35;

    const Case cases[] = {
        {"straight", makeStraight(3000.0), 25.0, {}},
        {"gentle_curve", makeArc(300.0, 800.0, degToRad(60.0), 500.0), 29.0, {}},
        {"sharp_curve", makeArc(500.0, 60.0, degToRad(90.0), 400.0), 25.0, {}},
        {"s_curve", makeSCurve(300.0, 150.0, degToRad(50.0), 400.0), 20.0, {}},
        {"highway_ramp", makeHighwayRamp(), 27.0, {}},
        {"s_curve_loaded_trailer", makeSCurve(300.0, 200.0, degToRad(40.0), 400.0), 22.0, loaded},
    };

    int failures = 0;
    std::printf("%-24s %8s %8s %9s %9s %9s %s\n", "scenario", "maxXTE", "rmsXTE", "maxLatA", "reversals", "dist",
                "result");
    for (const auto& c : cases) {
        ScenarioOptions o;
        o.name = c.name;
        o.initialSpeed = c.speed;
        std::ofstream csv;
        if (!csvDir.empty()) {
            csv.open(csvDir / (std::string(c.name) + ".csv"));
            csv << "time,x,y,z,speed,target_speed,heading,target_heading,steer,throttle,brake,cross_track,"
                   "lookahead,curve_radius\n";
            o.observer = [&](const VehicleState& s, const ControlCommand& cmd, const ControllerDebug& d) {
                const double yaw = coords::sdkHeadingToYaw(s.headingUnit);
                csv << s.time << ',' << s.worldPosition.x << ',' << s.worldPosition.y << ',' << s.worldPosition.z
                    << ',' << s.speed << ',' << d.targetSpeed << ',' << yaw << ',' << yaw + d.headingError << ','
                    << cmd.steering << ',' << cmd.throttle << ',' << cmd.brake << ',' << d.crossTrackError << ','
                    << d.lookAheadDistance << ',' << d.curveRadius << '\n';
            };
        }
        const ScenarioResult r = runScenario(c.path, cfg, c.params, o);
        const bool ok = r.completed && !r.disengaged;
        failures += ok ? 0 : 1;
        std::printf("%-24s %8.3f %8.3f %9.3f %9d %9.0f %s%s\n", c.name, r.maxCrossTrack, r.rmsCrossTrack,
                    r.maxLateralAccel, r.steeringReversals, r.distance, ok ? "ok" : "FAILED ",
                    r.disengaged ? r.disengageReason.c_str() : "");
    }
    return failures == 0 ? 0 : 1;
}

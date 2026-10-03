#pragma once

#include <functional>
#include <string>

#include "config/Config.h"
#include "sim/VehicleSim.h"

namespace atspilot::sim {

struct ScenarioOptions {
    std::string name;
    double initialSpeed = 20.0;      // m/s
    double setSpeedOverride = 0.0;   // m/s; 0 = use config max
    double lateralOffset = 0.0;      // initial offset from the path, m (positive = left)
    double headingOffset = 0.0;      // initial yaw offset, rad
    double maxTime = 600.0;          // s
    double rate = 60.0;              // control rate, Hz
    PilotRequest engage = PilotRequest::ToggleAutopilot;
    // Optional per-step observer (time, state, command, debug), e.g. for CSV export.
    std::function<void(const VehicleState&, const ControlCommand&, const ControllerDebug&)> observer;
};

ScenarioResult runScenario(const Path& path, const Config& cfg, const SimParams& simParams,
                           const ScenarioOptions& options);

}  // namespace atspilot::sim

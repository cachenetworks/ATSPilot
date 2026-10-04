#pragma once

#include <optional>
#include <string>

#include "config/Config.h"
#include "pilot/PilotTypes.h"
#include "pilot/VehicleState.h"

namespace atspilot {

enum class OverrideKind { None, Steering, Brake, Throttle };

const char* toString(OverrideKind k);

// Whether the game's reported `input.*` values already contain ATSPilot's own
// semantical input. The SDK does not document this, so it is inferred from data.
enum class InputMixing { Unknown, IncludesCommand, ExcludesCommand };

// Detects the driver taking over by comparing what the game reports as input
// with what ATSPilot commanded on the previous frame.
class OverrideDetector {
public:
    explicit OverrideDetector(const SafetyConfig& cfg) : cfg_(cfg) {}

    void setConfig(const SafetyConfig& cfg) { cfg_ = cfg; }
    // `driverSteering` is the steering the driver already holds when ATSPilot takes
    // over (the game keeps keyboard steering where it was). It only counts as an
    // override once the driver moves beyond it.
    void reset(double driverSteering = 0.0);

    OverrideKind update(const VehicleState& s, const ControlCommand& previous);

    InputMixing steeringMixing() const { return steerMixing_; }
    double driverSteering() const { return driverSteer_; }
    double steeringBaseline() const { return steerBaseline_; }

private:
    double driverInput(double reported, double commanded, InputMixing mixing) const;
    void learnMixing(double reported, double commanded, InputMixing& mixing, int& includeVotes, int& excludeVotes);

    SafetyConfig cfg_;
    InputMixing steerMixing_ = InputMixing::Unknown;
    InputMixing pedalMixing_ = InputMixing::Unknown;
    int steerInclude_ = 0, steerExclude_ = 0;
    int pedalInclude_ = 0, pedalExclude_ = 0;
    int steerFrames_ = 0;
    int throttleFrames_ = 0;
    double driverSteer_ = 0.0;
    double steerBaseline_ = 0.0;
};

// Verifies that every input the control loop relies on is fresh.
class Watchdog {
public:
    explicit Watchdog(const SafetyConfig& cfg) : cfg_(cfg) {}
    void setConfig(const SafetyConfig& cfg) { cfg_ = cfg; }

    // Returns a reason when control must stop. `now` is wall-clock seconds.
    std::optional<std::string> check(double now, double lastTelemetryWall, double lastPathWall, bool pathRequired) const;

private:
    SafetyConfig cfg_;
};

}  // namespace atspilot

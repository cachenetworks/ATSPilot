#include "pilot/Safety.h"

#include <algorithm>
#include <cmath>

namespace atspilot {

const char* toString(OverrideKind k) {
    switch (k) {
        case OverrideKind::None: return "none";
        case OverrideKind::Steering: return "steering";
        case OverrideKind::Brake: return "brake";
        case OverrideKind::Throttle: return "throttle";
    }
    return "?";
}

void OverrideDetector::reset(double driverSteering) {
    steerFrames_ = 0;
    throttleFrames_ = 0;
    driverSteer_ = driverSteering;
    steerBaseline_ = std::abs(driverSteering);
}

void OverrideDetector::learnMixing(double reported, double commanded, InputMixing& mixing, int& includeVotes,
                                   int& excludeVotes) {
    if (mixing != InputMixing::Unknown || std::abs(commanded) < 0.08) return;
    // A frame only counts as evidence when it clearly matches one hypothesis.
    if (std::abs(reported - commanded) < 0.015) ++includeVotes;
    else if (std::abs(reported) < 0.015) ++excludeVotes;
    if (includeVotes >= 30) mixing = InputMixing::IncludesCommand;
    else if (excludeVotes >= 30) mixing = InputMixing::ExcludesCommand;
}

double OverrideDetector::driverInput(double reported, double commanded, InputMixing mixing) const {
    switch (mixing) {
        case InputMixing::ExcludesCommand: return reported;
        case InputMixing::IncludesCommand: return reported - commanded;
        case InputMixing::Unknown:
        default:
            // Until the game's behaviour is known, attribute to the driver only what
            // neither hypothesis can explain.
            return std::abs(reported - commanded) < std::abs(reported) ? reported - commanded : reported;
    }
}

OverrideKind OverrideDetector::update(const VehicleState& s, const ControlCommand& previous) {
    if (!cfg_.driverOverride) return OverrideKind::None;

    const double cmdSteer = previous.steerActive ? previous.steering : 0.0;
    const double cmdThrottle = previous.pedalsActive ? previous.throttle : 0.0;
    const double cmdBrake = previous.pedalsActive ? previous.brake : 0.0;

    if (previous.steerActive) learnMixing(s.inputSteering, cmdSteer, steerMixing_, steerInclude_, steerExclude_);
    if (previous.pedalsActive) {
        learnMixing(s.inputThrottle, cmdThrottle, pedalMixing_, pedalInclude_, pedalExclude_);
        learnMixing(s.inputBrake, cmdBrake, pedalMixing_, pedalInclude_, pedalExclude_);
    }

    driverSteer_ = driverInput(s.inputSteering, cmdSteer, steerMixing_);
    const double driverBrake = driverInput(s.inputBrake, cmdBrake, pedalMixing_);
    const double driverThrottle = driverInput(s.inputThrottle, cmdThrottle, pedalMixing_);

    // Brakes act immediately; steering and throttle need a few consecutive frames so
    // a single noisy sample cannot disengage the pilot.
    if (driverBrake > cfg_.brakeOverrideThreshold) return OverrideKind::Brake;

    // Steering held at engagement re-centres on its own; follow it down so that
    // only a fresh move by the driver counts.
    steerBaseline_ = std::min(steerBaseline_, std::abs(driverSteer_));
    const double freshSteer = std::abs(driverSteer_) - steerBaseline_;
    steerFrames_ = freshSteer > cfg_.steeringOverrideThreshold ? steerFrames_ + 1 : 0;
    if (steerFrames_ >= 3) return OverrideKind::Steering;

    throttleFrames_ = driverThrottle > cfg_.throttleOverrideThreshold ? throttleFrames_ + 1 : 0;
    if (throttleFrames_ >= 5) return OverrideKind::Throttle;

    return OverrideKind::None;
}

std::optional<std::string> Watchdog::check(double now, double lastTelemetryWall, double lastPathWall,
                                           bool pathRequired) const {
    if (now - lastTelemetryWall > cfg_.telemetryTimeout) return "Telemetry stale";
    if (pathRequired && now - lastPathWall > cfg_.pathTimeout) return "Path stale";
    return std::nullopt;
}

}  // namespace atspilot

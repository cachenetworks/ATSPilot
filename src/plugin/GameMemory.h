#pragma once

#include <atomic>
#include <cstdint>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "config/Config.h"
#include "math/Vec.h"
#include "util/Logger.h"
#include "world/World.h"

namespace atspilot::plugin {

// Reads what the SDK does not expose (AI traffic, traffic-light states, the GPS
// route) from the game's memory, and writes the truck's steering directly.
//
// Uses the game structure layouts and code patterns of ETS2LA's game plugin
// (third_party/ets2la_plugin, MIT). Layouts change between game versions, so it
// only runs on the version they were written for, finds every address by pattern
// scan in the background, and reads under structured exception handling: a bad
// read switches the affected feature off instead of crashing the game.
//
// All per-frame calls are made on the game's main thread from SDK callbacks.
class GameMemory {
public:
    GameMemory(Logger& log, const GameMemoryConfig& cfg);
    ~GameMemory();

    // Checks the game version and starts the background pattern scan.
    void start(const std::string& gameName);

    bool ready() const { return state_.load() == State::Ready; }
    bool steeringAvailable() const { return ready() && cfg_.steering && steeringOk_; }

    // Traffic and light states within `radius` metres of the truck.
    WorldSnapshotPtr readWorld(double time, const Vec3& truckWorld, double radius = 250.0);

    // The in-game GPS route as map node uids from the truck to the destination,
    // when it changed since the last call (nullopt when unchanged or unavailable;
    // an empty vector when the route was cleared).
    std::optional<std::vector<std::uint64_t>> readGpsRouteIfChanged();

    // Steering, in the SDK's effective-steering convention ([-1, 1], left positive).
    // While ATSPilot is not steering, `observeSteering` learns how the stored value
    // relates to the SDK's effective steering.
    void observeSteering(double effectiveSteering);
    bool writeSteering(double steering);

private:
    enum class State { Idle, Scanning, Ready, Unavailable };

    void scan();
    std::optional<double> readRawSteering();
    void fault(const char* what, bool& feature);

    Logger& log_;
    GameMemoryConfig cfg_;
    std::atomic<State> state_{State::Idle};
    std::thread scanner_;

    bool trafficOk_ = true;
    bool lightsOk_ = true;
    bool gpsOk_ = true;
    bool steeringOk_ = true;
    int faults_ = 0;

    // Steering calibration: stored value = sign * scale * effective steering.
    double steeringSign_ = 1.0;  // measured in game (1.61): stored steering = SDK effective steering
    double steeringScale_ = 1.0;
    int steeringSamples_ = 0;
    int steeringAgree_ = 0;
    double steeringRatioSum_ = 0.0;
    bool steeringCalibrated_ = false;

    // GPS route change detection.
    std::uint64_t routeFirstUid_ = 0;
    std::uint64_t routeLastUid_ = 0;
    std::uint64_t routeSize_ = 0;
};

}  // namespace atspilot::plugin

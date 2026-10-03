#pragma once

#include <atomic>
#include <condition_variable>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "config/Config.h"
#include "map/LanePlanner.h"
#include "map/RoadNetwork.h"
#include "pilot/PilotTypes.h"
#include "pilot/VehicleState.h"
#include "util/Logger.h"

namespace atspilot {

enum class MapState { Disabled, Loading, Ready, Failed };

// Owns the road network and the planning thread:
//
//   main thread ── submitVehicle() ──▶ [latest state] ──▶ planner thread (10 Hz)
//                                                           localize ▶ build path
//   main thread ◀── latestPath() ◀── [atomic snapshot] ◀───┘
//
// The game thread never waits on map work: it copies a small struct in and a
// shared_ptr out, each under a briefly held mutex.
class MapService {
public:
    MapService(Logger& log, const Config& cfg);
    ~MapService();
    MapService(const MapService&) = delete;
    MapService& operator=(const MapService&) = delete;

    void start(const std::filesystem::path& gameDir, const std::filesystem::path& cacheDir);
    void stop();
    void setConfig(const Config& cfg);

    void submitVehicle(const VehicleState& s, const VehicleConfig& vc, double wallTime);
    // Latest path and the wall-clock time it was computed for.
    PathSnapshotPtr latestPath(double* wallTime) const;
    // Drops the current plan, e.g. after a teleport, ferry or job change.
    void invalidate(const std::string& reason);

    MapState state() const { return state_.load(); }
    double progress() const { return progress_.load(); }
    std::string statusText() const;

private:
    void run();
    bool loadOrBuild();
    void planOnce();

    Logger& log_;
    mutable std::mutex cfgMutex_;
    Config cfg_;

    std::filesystem::path gameDir_;
    std::filesystem::path cacheDir_;
    std::unique_ptr<RoadNetwork> net_;

    std::thread thread_;
    std::atomic<bool> stop_{false};
    std::atomic<MapState> state_{MapState::Disabled};
    std::atomic<double> progress_{0.0};
    mutable std::mutex statusMutex_;
    std::string statusText_ = "Map not loaded";

    mutable std::mutex inMutex_;
    std::condition_variable inCv_;
    std::optional<VehicleState> vehicle_;
    VehicleConfig vehicleConfig_;
    double vehicleWall_ = 0.0;
    bool newVehicle_ = false;
    std::atomic<bool> invalidate_{false};

    mutable std::mutex outMutex_;
    PathSnapshotPtr path_;
    double pathWall_ = -1e9;

    // Planner-thread-only state.
    std::unique_ptr<Localizer> localizer_;
    std::vector<std::uint32_t> chain_;
    std::uint64_t generation_ = 0;
    std::optional<Vec2> lastPos_;
    std::string lastFailure_;
};

}  // namespace atspilot

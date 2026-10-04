#pragma once

#include <atomic>
#include <condition_variable>
#include <filesystem>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "config/Config.h"
#include "map/LanePlanner.h"
#include "map/RoadNetwork.h"
#include "map/RoutePlanner.h"
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
    // The in-game GPS route, read from the game's memory (plan coordinates from the
    // truck to the destination; empty when the route was cleared). Routing then
    // follows it exactly, and a GPS destination without a job is driven to as well.
    void setGameRoute(std::vector<Vec2> points);

    MapState state() const { return state_.load(); }
    double progress() const { return progress_.load(); }
    std::string statusText() const;

private:
    void run();
    bool loadOrBuild();
    void planOnce();
    void updateRoute(const VehicleState& s, const VehicleConfig& vc, const LocalizationResult& loc, const Config& cfg,
                     double wall, double step);
    void checkGpsAgreement(const VehicleState& s, const PlannedPath& planned, const Config& cfg, double wall);

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
    std::optional<std::vector<Vec2>> pendingGameRoute_;

    mutable std::mutex outMutex_;
    PathSnapshotPtr path_;
    double pathWall_ = -1e9;

    // Planner-thread-only state.
    std::unique_ptr<Localizer> localizer_;
    std::vector<std::uint32_t> chain_;
    std::uint64_t generation_ = 0;
    std::optional<Vec2> lastPos_;
    std::string lastFailure_;

    // Routing (planner thread). Declared after net_ so a running search is
    // waited for before the network it reads is destroyed.
    std::string destinationKey_;
    std::shared_ptr<const Route> route_;
    std::future<Route> routeFuture_;
    double routeRetryWall_ = 0.0;
    double offRouteSince_ = -1.0;
    bool destinationMissingLogged_ = false;
    std::shared_ptr<const GpsCorridor> gameRoute_;  // the in-game GPS route, when known

    // In-game GPS agreement: the navigation distance is compared with the route.
    double gpsScale_ = 0.0;        // navigation distance units per metre driven (learned)
    double gpsNavAtSample_ = -1.0;
    double gpsTravel_ = 0.0;
    bool gpsMatched_ = false;
    double gpsMismatchSince_ = -1.0;
    bool gpsReplan_ = false;
};

}  // namespace atspilot

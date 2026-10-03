#include "map/MapService.h"

#include <chrono>

#include "map/MapBuilder.h"
#include "math/Coordinates.h"
#include "math/MathUtil.h"

namespace atspilot {

MapService::MapService(Logger& log, const Config& cfg) : log_(log), cfg_(cfg) {}

MapService::~MapService() { stop(); }

void MapService::setConfig(const Config& cfg) {
    std::lock_guard lock(cfgMutex_);
    cfg_ = cfg;
}

void MapService::start(const std::filesystem::path& gameDir, const std::filesystem::path& cacheDir) {
    stop();
    gameDir_ = gameDir;
    cacheDir_ = cacheDir;
    stop_ = false;
    state_ = MapState::Loading;
    thread_ = std::thread([this] { run(); });
}

void MapService::stop() {
    stop_ = true;
    inCv_.notify_all();
    if (thread_.joinable()) thread_.join();
}

std::string MapService::statusText() const {
    std::lock_guard lock(statusMutex_);
    return statusText_;
}

void MapService::submitVehicle(const VehicleState& s, const VehicleConfig& vc, double wallTime) {
    {
        std::lock_guard lock(inMutex_);
        vehicle_ = s;
        vehicleConfig_ = vc;
        vehicleWall_ = wallTime;
        newVehicle_ = true;
    }
    inCv_.notify_one();
}

PathSnapshotPtr MapService::latestPath(double* wallTime) const {
    std::lock_guard lock(outMutex_);
    if (wallTime) *wallTime = pathWall_;
    return path_;
}

void MapService::invalidate(const std::string& reason) {
    invalidate_ = true;
    {
        std::lock_guard lock(outMutex_);
        path_.reset();
    }
    log_.info("Route invalidated: {}", reason);
}

bool MapService::loadOrBuild() {
    Config cfg;
    {
        std::lock_guard lock(cfgMutex_);
        cfg = cfg_;
    }
    MapBuildOptions opts;
    opts.gameDir = gameDir_;
    opts.laneWidth = cfg.map.laneWidth;
    opts.cancel = &stop_;
    opts.progress = [this](double p, const std::string& phase) {
        progress_ = p;
        std::lock_guard lock(statusMutex_);
        statusText_ = phase;
    };
    const std::string fingerprint = mapFingerprint(opts);
    const auto cacheFile = cacheDir_ / "map_usa.cache";

    std::string why;
    if (auto cached = RoadNetwork::load(cacheFile, fingerprint, &why)) {
        net_ = std::make_unique<RoadNetwork>(std::move(*cached));
        log_.info("Map loaded from cache: {} lane segments", net_->size());
        return true;
    }
    log_.info("Building map cache ({}) from {}", why, gameDir_.string());

    MapBuildStats stats;
    const auto t0 = std::chrono::steady_clock::now();
    auto built = buildRoadNetwork(opts, stats, &log_);
    if (!built) return false;
    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    log_.info("Map built in {:.1f} s: {} sectors ({} errors), {} roads, {} prefabs, {} lanes, {:.1f}% lane ends connected",
              secs, stats.sectors, stats.sectorErrors, stats.roads, stats.prefabs, stats.lanes,
              stats.laneEnds ? 100.0 * stats.laneEndsConnected / stats.laneEnds : 0.0);
    net_ = std::make_unique<RoadNetwork>(std::move(*built));
    if (!net_->save(cacheFile, fingerprint)) log_.warn("Could not write map cache to {}", cacheFile.string());
    return true;
}

void MapService::run() {
    try {
        if (!loadOrBuild()) {
            if (!stop_) {
                state_ = MapState::Failed;
                std::lock_guard lock(statusMutex_);
                statusText_ = "Map unavailable";
                log_.error("Map could not be loaded; steering modes are unavailable");
            }
            return;
        }
        localizer_ = std::make_unique<Localizer>(*net_);
        state_ = MapState::Ready;
        progress_ = 1.0;
        {
            std::lock_guard lock(statusMutex_);
            statusText_ = "Map ready";
        }

        while (!stop_) {
            {
                std::unique_lock lock(inMutex_);
                inCv_.wait_for(lock, std::chrono::milliseconds(100), [this] { return stop_.load() || newVehicle_; });
                if (stop_) break;
            }
            planOnce();
            // Planning at ~10 Hz is plenty: the controller interpolates along the path.
            std::this_thread::sleep_for(std::chrono::milliseconds(80));
        }
    } catch (const std::exception& e) {
        state_ = MapState::Failed;
        log_.error("Map service failed: {}", e.what());
    } catch (...) {
        state_ = MapState::Failed;
        log_.error("Map service failed with an unknown error");
    }
    std::lock_guard lock(outMutex_);
    path_.reset();
}

void MapService::planOnce() {
    VehicleState s;
    VehicleConfig vc;
    double wall = 0.0;
    {
        std::lock_guard lock(inMutex_);
        if (!vehicle_ || !newVehicle_) return;
        s = *vehicle_;
        vc = vehicleConfig_;
        wall = vehicleWall_;
        newVehicle_ = false;
    }
    if (!s.valid) return;
    Config cfg;
    {
        std::lock_guard lock(cfgMutex_);
        cfg = cfg_;
    }

    const double yaw = coords::sdkHeadingToYaw(s.headingUnit);
    const Vec2 origin = coords::worldToPlan(s.worldPosition);
    const Vec2 rear = origin + coords::yawToDirection(yaw) * (-vc.rearAxleZ);

    // A large jump between samples means a teleport, ferry, service or job reset.
    if (invalidate_.exchange(false) || (lastPos_ && distance(*lastPos_, rear) > 60.0)) {
        chain_.clear();
        localizer_->reset();
        ++generation_;
    }
    lastPos_ = rear;

    Localizer::Params lp;
    lp.maxDistance = cfg.map.maxLocalizationDistance;
    localizer_->setParams(lp);
    const LocalizationResult loc = localizer_->update(rear, yaw, chain_);
    if (!loc.valid) {
        if (loc.reason != lastFailure_) log_.info("Localization: {}", loc.reason);
        lastFailure_ = loc.reason;
        chain_.clear();
        std::lock_guard lock(outMutex_);
        path_.reset();
        return;
    }
    if (!lastFailure_.empty()) {
        log_.info("Vehicle localized to lane segment {} ({:.1f} m)", loc.match.segment, loc.match.distance);
        lastFailure_.clear();
    }

    PathBuildParams pp;
    pp.ahead = cfg.map.pathAhead;
    pp.behind = cfg.map.pathBehind;
    const bool continuing = std::find(chain_.begin(), chain_.end(), loc.match.segment) != chain_.end();
    PlannedPath planned = buildPlannedPath(*net_, loc.match, pp, chain_);
    if (!continuing) ++generation_;
    chain_ = planned.chain;

    auto snap = std::make_shared<PathSnapshot>();
    snap->path = std::move(planned.path);
    snap->time = s.time;
    snap->truckS = planned.truckS;
    snap->roadName = net_->segment(loc.match.segment).kind == LaneKind::Road ? "Road" : "Junction";
    snap->nextManeuver = planned.nextManeuver;
    snap->nextManeuverDistance = planned.nextManeuverDistance;
    snap->generation = generation_;
    std::lock_guard lock(outMutex_);
    path_ = std::move(snap);
    pathWall_ = wall;
}

}  // namespace atspilot

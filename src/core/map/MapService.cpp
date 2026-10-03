#include "map/MapService.h"

#include <chrono>

#include "map/MapBuilder.h"
#include "map/Token.h"
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

void MapService::updateRoute(const VehicleConfig& vc, const LocalizationResult& loc, const Config& cfg, double wall) {
    const std::string key = cfg.route.enabled && vc.hasJob && !vc.destinationCompanyId.empty()
                                ? vc.destinationCityId + "/" + vc.destinationCompanyId
                                : std::string();
    if (key != destinationKey_) {
        destinationKey_ = key;
        route_.reset();
        offRouteSince_ = -1.0;
        routeRetryWall_ = 0.0;
        destinationMissingLogged_ = false;
        if (!key.empty()) log_.info("Navigation destination: {}", key);
    }

    if (routeFuture_.valid() && routeFuture_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        Route r = routeFuture_.get();
        if (r.found) {
            log_.info("Route calculated: {:.1f} km, {} lane segments ({} expanded)", r.length / 1000.0, r.steps.size(),
                      r.expanded);
            route_ = std::make_shared<const Route>(std::move(r));
            offRouteSince_ = -1.0;
        } else {
            log_.warn("Route unavailable: {}", r.failure);
            routeRetryWall_ = wall + 30.0;
        }
    }
    if (key.empty()) return;

    // Off-route detection: the truck's lane is neither on the route nor beside it.
    if (route_) {
        bool onRoute = route_->find(loc.match.segment) >= 0;
        if (!onRoute) {
            for (auto n : net_->laneNeighbors(loc.match.segment)) onRoute = onRoute || route_->find(n) >= 0;
        }
        if (onRoute) {
            offRouteSince_ = -1.0;
        } else if (offRouteSince_ < 0.0) {
            offRouteSince_ = wall;
        } else if (wall - offRouteSince_ > cfg.route.recalcAfter) {
            log_.info("Left the route (wrong turn, missed exit or manual override); recalculating");
            route_.reset();
            offRouteSince_ = -1.0;
            routeRetryWall_ = 0.0;
        }
    }

    if (!route_ && !routeFuture_.valid() && wall >= routeRetryWall_) {
        const Destination* dest = net_->findDestination(tokenFromString(vc.destinationCityId),
                                                        tokenFromString(vc.destinationCompanyId));
        if (!dest) {
            if (!destinationMissingLogged_) {
                log_.warn("Destination {} not found in the map data; following the road instead", key);
                destinationMissingLogged_ = true;
            }
            routeRetryWall_ = wall + 60.0;
            return;
        }
        RouteOptions opts;
        opts.laneChangeCost = cfg.route.laneChangeCost;
        const RoadNetwork* net = net_.get();
        const std::uint32_t startSeg = loc.match.segment;
        const double startS = loc.match.s;
        const std::vector<std::uint32_t> goals = dest->lanes;
        routeFuture_ = std::async(std::launch::async,
                                  [net, startSeg, startS, goals, opts] { return planRoute(*net, startSeg, startS, goals, opts); });
    }
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

    updateRoute(vc, loc, cfg, wall);

    PathBuildParams pp;
    pp.ahead = cfg.map.pathAhead;
    pp.behind = cfg.map.pathBehind;
    const bool continuing = std::find(chain_.begin(), chain_.end(), loc.match.segment) != chain_.end();
    PlannedPath planned = buildPlannedPath(*net_, loc.match, pp, chain_, route_.get());
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
    snap->navigationActive = planned.onRoute;
    snap->routeRemaining = planned.routeRemaining;
    std::lock_guard lock(outMutex_);
    path_ = std::move(snap);
    pathWall_ = wall;
}

}  // namespace atspilot

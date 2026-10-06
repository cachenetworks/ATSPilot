#include "map/MapService.h"

#include <chrono>

#include "map/MapBuilder.h"
#include "map/ServicePlanner.h"
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

void MapService::start(const std::filesystem::path& gameDir, const std::filesystem::path& cacheDir, std::string mapName,
                       std::string cacheFile) {
    stop();
    gameDir_ = gameDir;
    cacheDir_ = cacheDir;
    mapName_ = std::move(mapName);
    cacheFile_ = std::move(cacheFile);
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
    opts.mapName = mapName_;
    opts.laneWidth = cfg.map.laneWidth;
    opts.cancel = &stop_;
    opts.progress = [this](double p, const std::string& phase) {
        progress_ = p;
        std::lock_guard lock(statusMutex_);
        statusText_ = phase;
    };
    const std::string fingerprint = mapFingerprint(opts);
    const auto cacheFile = cacheDir_ / cacheFile_;

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

void MapService::checkGpsAgreement(const VehicleState& s, const PlannedPath& planned, const Config& cfg, double wall) {
    if (gameRoute_) {
        // Routed along the GPS route itself: nothing to infer from its distance.
        gpsMatched_ = route_ != nullptr;
        gpsMismatchSince_ = -1.0;
        return;
    }
    if (!cfg.route.matchGameGps || !route_ || !planned.onRoute || gpsScale_ <= 0.0 || s.navigationDistance <= 1.0) {
        gpsMismatchSince_ = -1.0;
        return;
    }
    const double expected = gpsScale_ * planned.routeRemaining;
    const double error = std::abs(s.navigationDistance - expected);
    const bool matched = error <= std::max(150.0 * gpsScale_, cfg.route.gpsTolerance * s.navigationDistance);
    if (matched != gpsMatched_) {
        log_.info("Route {} the in-game GPS ({:.1f} km by GPS, {:.1f} km by route)", matched ? "agrees with" : "differs from",
                  s.navigationDistance / gpsScale_ / 1000.0, planned.routeRemaining / 1000.0);
    }
    gpsMatched_ = matched;
    if (matched) {
        gpsMismatchSince_ = -1.0;
    } else if (gpsMismatchSince_ < 0.0) {
        gpsMismatchSince_ = wall;
    } else if (wall - gpsMismatchSince_ > 5.0 && !gpsReplan_) {
        log_.info("Re-planning to match the in-game GPS route");
        gpsReplan_ = true;
        gpsMismatchSince_ = wall + 25.0;  // give the re-plan time before judging again
    }
}

void MapService::updateRoute(const VehicleState& s, const VehicleConfig& vc, const LocalizationResult& loc,
                             const Config& cfg, double wall, double step) {
    // Learn how navigation distance relates to metres driven: while the truck follows
    // the GPS route, the navigation distance falls by gpsScale_ per metre.
    if (s.navigationDistance > 1.0 && step < 60.0) {
        if (gpsNavAtSample_ <= 0.0) {
            gpsNavAtSample_ = s.navigationDistance;
            gpsTravel_ = 0.0;
        }
        gpsTravel_ += step;
        if (gpsTravel_ >= 200.0) {
            const double ratio = (gpsNavAtSample_ - s.navigationDistance) / gpsTravel_;
            if (ratio > 0.2 && ratio < 40.0) {
                const bool first = gpsScale_ <= 0.0;
                gpsScale_ = first ? ratio : 0.8 * gpsScale_ + 0.2 * ratio;
                if (first) log_.info("In-game GPS distance scale: {:.3f} per metre", gpsScale_);
            }
            gpsNavAtSample_ = s.navigationDistance;
            gpsTravel_ = 0.0;
        }
    } else {
        gpsNavAtSample_ = -1.0;
    }

    // A new in-game GPS route (the player changed the destination or the GPS
    // re-routed) replaces the current route.
    {
        std::optional<std::vector<std::uint64_t>> pending;
        {
            std::lock_guard lock(inMutex_);
            pending.swap(pendingGameRoute_);
        }
        if (pending) {
            std::vector<Vec2> points;
            points.reserve(pending->size());
            for (const auto uid : *pending) {
                if (const auto p = net_->nodePosition(uid)) points.push_back(*p);
            }
            if (!pending->empty()) {
                log_.info("In-game GPS route: {} of {} nodes found in the map data", points.size(), pending->size());
            }
            // Mostly unknown nodes (mods, version mismatch): do not trust it.
            if (points.size() < 2 || points.size() * 10 < pending->size() * 9) points.clear();
            gameRoute_ = points.size() >= 2 ? std::make_shared<const GpsCorridor>(std::move(points)) : nullptr;
            route_.reset();
            routeRetryWall_ = 0.0;
            offRouteSince_ = -1.0;
            log_.info("{}", gameRoute_ ? "Following the in-game GPS route" : "In-game GPS route cleared");
        }
    }

    if (updateServices(s, vc, loc, cfg, wall)) {
        route_.reset();
        routeRetryWall_ = 0.0;
        offRouteSince_ = -1.0;
    }

    std::string key;
    if (cfg.route.enabled && vc.hasJob && !vc.destinationCompanyId.empty()) {
        key = vc.destinationCityId + "/" + vc.destinationCompanyId;
    } else if (cfg.route.enabled && gameRoute_) {
        // No job, but the player set a GPS destination: drive there.
        const Vec2 end = gameRoute_->points().back();
        key = "gps:" + std::to_string(std::lround(end.x)) + "," + std::to_string(std::lround(end.y));
    }
    if (key != destinationKey_) {
        destinationKey_ = key;
        route_.reset();
        offRouteSince_ = -1.0;
        routeRetryWall_ = 0.0;
        destinationMissingLogged_ = false;
        gpsMatched_ = false;
        gpsReplan_ = false;
        if (!key.empty()) log_.info("Navigation destination: {}", key);
    }

    if (routeFuture_.valid() && routeFuture_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        Route r = routeFuture_.get();
        if (r.found) {
            log_.info("Route calculated: {:.1f} km, {} lane segments ({} expanded){}", r.length / 1000.0,
                      r.steps.size(), r.expanded, r.gpsMatched ? ", matches the in-game GPS" : "");
            const bool viaFuel = std::any_of(r.services.begin(), r.services.end(),
                                             [](const RouteService& v) { return v.kind == ServiceKind::Fuel; });
            if (needFuel_ && !viaFuel) {
                // Pumps in open lots cannot be driven to along mapped lanes.
                log_.warn("Fuel low, but no fuel station ATSPilot can drive into is near; refuel manually (will look "
                          "again in 10 minutes)");
                needFuel_ = false;
                fuelDisabledUntil_ = wall + 600.0;
            }
            for (const auto& svc : r.services) {
                const Vec2 at = net_->segment(svc.lane).points.front().plan();
                log_.info("Route stops at a {} (lane {}, {:.0f} m from here)",
                          svc.kind == ServiceKind::Fuel ? "fuel station" : "weigh station", svc.lane,
                          distance(at, net_->segment(loc.match.segment).points.front().plan()));
            }
            route_ = std::make_shared<const Route>(std::move(r));
            offRouteSince_ = -1.0;
        } else {
            log_.warn("Route unavailable: {}", r.failure);
            routeRetryWall_ = wall + 30.0;
        }
    }
    if (key.empty() && !needFuel_) return;

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

    if ((!route_ || gpsReplan_) && !routeFuture_.valid() && wall >= routeRetryWall_) {
        gpsReplan_ = false;
        std::vector<std::uint32_t> gpsGoals;
        if (key.rfind("gps:", 0) == 0) {
            for (const auto& m : net_->query(gameRoute_->points().back(), 40.0)) gpsGoals.push_back(m.segment);
        }
        const Destination* dest = gpsGoals.empty() && !key.empty()
                                      ? net_->findDestination(tokenFromString(vc.destinationCityId),
                                                              tokenFromString(vc.destinationCompanyId))
                                      : nullptr;
        if (!dest && gpsGoals.empty() && !needFuel_) {
            if (!destinationMissingLogged_) {
                log_.warn("Destination {} not found in the map data; following the road instead", key);
                destinationMissingLogged_ = true;
            }
            routeRetryWall_ = wall + 60.0;
            return;
        }
        RouteOptions opts;
        opts.laneChangeCost = cfg.route.laneChangeCost;
        opts.corridor = gameRoute_;
        const RoadNetwork* net = net_.get();
        const std::uint32_t startSeg = loc.match.segment;
        const double startS = loc.match.s;
        const std::vector<std::uint32_t> goals = dest ? dest->lanes : gpsGoals;
        // Without the GPS route itself, aim for the GPS's remaining distance
        // (with a learned scale) to pick the same branches it does.
        const double target = !gameRoute_ && cfg.route.matchGameGps && gpsScale_ > 0.0 && s.navigationDistance > 1.0
                                  ? s.navigationDistance / gpsScale_
                                  : 0.0;
        const double tolerance = cfg.route.gpsTolerance;
        ServicePlanOptions sp;
        sp.fuel = needFuel_;
        sp.weigh = cfg.services.weighStations;
        for (const auto& [lane, when] : visitedServices_) sp.skipLanes.push_back(lane);
        routeFuture_ = std::async(std::launch::async, [net, startSeg, startS, goals, opts, target, tolerance, sp] {
            return planRouteWithServices(*net, startSeg, startS, goals, opts, target, tolerance, sp);
        });
    }
}

bool MapService::updateServices(const VehicleState& s, const VehicleConfig& vc, const LocalizationResult& loc,
                                const Config& cfg, double wall) {
    bool replan = false;
    // Fuel: low below the configured fraction (or the dashboard warning), and
    // satisfied again once the tank is nearly full.
    if (vc.fuelCapacity > 1.0 && s.fuel >= 0.0) {
        const double frac = s.fuel / vc.fuelCapacity;
        if (!needFuel_ && cfg.services.refuel && wall >= fuelDisabledUntil_ &&
            (frac < cfg.services.refuelBelow || s.fuelWarning)) {
            needFuel_ = true;
            replan = true;
            log_.info("Fuel low ({:.0f}%): routing to a fuel station", frac * 100.0);
        } else if (needFuel_ && (frac > 0.9 || !cfg.services.refuel)) {
            needFuel_ = false;
            failedPumps_ = 0;
            replan = true;
            log_.info("Fuel {:.0f}%: continuing to the destination", frac * 100.0);
        }
    }
    visitedServices_.erase(std::remove_if(visitedServices_.begin(), visitedServices_.end(),
                                          [&](const auto& v) { return wall - v.second > 900.0; }),
                           visitedServices_.end());
    // A service stop the truck has driven past is done (or, for fuel that is
    // still low, failed: another pump is tried, and after two the search stops).
    if (route_ && !route_->services.empty()) {
        const int here = route_->find(loc.match.segment);
        for (const auto& svc : route_->services) {
            const int at = route_->find(svc.lane);
            const bool passed = here >= 0 && at >= 0 &&
                                (here > at || (here == at && loc.match.s > svc.s + 40.0));
            if (!passed) continue;
            visitedServices_.push_back({svc.lane, wall});
            replan = true;
            if (svc.kind == ServiceKind::Fuel && needFuel_) {
                log_.warn("Left the fuel station without refuelling");
                if (++failedPumps_ >= 2) {
                    needFuel_ = false;
                    fuelDisabledUntil_ = wall + 1800.0;
                    log_.warn("Refuelling did not work at two stations; not routing to fuel for 30 minutes");
                }
            }
        }
    }
    return replan;
}

void MapService::setGameRoute(std::vector<std::uint64_t> nodeUids) {
    std::lock_guard lock(inMutex_);
    pendingGameRoute_ = std::move(nodeUids);
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

    const double step = lastPos_ ? distance(*lastPos_, rear) : 0.0;
    // A large jump between samples means a teleport, ferry, service or job reset.
    if (invalidate_.exchange(false) || (lastPos_ && step > 60.0)) {
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

    updateRoute(s, vc, loc, cfg, wall, step);

    PathBuildParams pp;
    pp.ahead = cfg.map.pathAhead;
    pp.behind = cfg.map.pathBehind;
    const bool continuing = std::find(chain_.begin(), chain_.end(), loc.match.segment) != chain_.end();
    PlannedPath planned = buildPlannedPath(*net_, loc.match, pp, chain_, route_.get());
    if (!continuing) ++generation_;
    chain_ = planned.chain;
    checkGpsAgreement(s, planned, cfg, wall);

    auto snap = std::make_shared<PathSnapshot>();
    snap->path = std::move(planned.path);
    snap->time = s.time;
    snap->truckS = planned.truckS;
    snap->roadName = net_->segment(loc.match.segment).kind == LaneKind::Road ? "Road" : "Junction";
    snap->nextManeuver = planned.nextManeuver;
    snap->nextManeuverDistance = planned.nextManeuverDistance;
    snap->generation = generation_;
    // A route that only leads to a fuel pump (no destination) is not navigation:
    // its end is no arrival.
    snap->navigationActive = planned.onRoute && !destinationKey_.empty();
    snap->stops = planned.stops;
    snap->indications = planned.indications;
    snap->gpsMatched = planned.onRoute && gpsMatched_;
    snap->routeRemaining = planned.routeRemaining;
    std::lock_guard lock(outMutex_);
    path_ = std::move(snap);
    pathWall_ = wall;
}

}  // namespace atspilot

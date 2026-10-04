#include "GameMemory.h"

#include <windows.h>

#include <algorithm>
#include <cmath>
#include <unordered_set>

#include "math/Coordinates.h"
#include "math/MathUtil.h"

// Vendored game structures (third_party/ets2la_plugin, MIT), built as published.
#pragma warning(push, 0)
#include "core.hpp"
#include "prism/controllers/base_ctrl.hpp"
#include "prism/controllers/game_ctrl.hpp"
#include "prism/game_actor.hpp"
#include "prism/management/item/kdop_item.hpp"
#include "prism/management/item/node_item.hpp"
#include "prism/management/item/prefab_item.hpp"
#include "prism/management/item/segment.hpp"
#include "prism/management/item/semaphore_instance.hpp"
#include "prism/navigation/gps_manager.hpp"
#include "prism/navigation/route_task.hpp"
#include "prism/traffic/game_traffic.hpp"
#include "prism/traffic/objects/traffic_ai_vehicle.hpp"
#include "prism/traffic/objects/traffic_light.hpp"
#include "prism/traffic/objects/traffic_semaphore_actor.hpp"
#include "prism/vehicles/game_physics_vehicle.hpp"
#include "prism/vehicles/vehicle_shared.hpp"
#pragma warning(pop)

namespace ets2la_plugin {
CCore* CCore::g_instance = nullptr;
}

namespace atspilot::plugin {

namespace prism = ets2la_plugin::prism;

namespace {

// The game version the vendored layouts and patterns were written for.
constexpr const char* kSupportedVersion = "1.61.";

Logger* g_log = nullptr;
ets2la_plugin::CCore g_core;

void coreSink(int level, const std::string& message) {
    if (!g_log) return;
    if (level >= 3) g_log->warn("Game memory: {}", message);
    else g_log->debug("Game memory: {}", message);
}

// Runs `f`, turning any access violation or other hardware exception into `false`.
// No C++ objects may live in this frame (MSVC structured exception handling).
template <class F>
bool guarded(F& f) {
    __try {
        f();
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

bool scanStep(bool (*fn)()) {
    bool ok = false;
    auto call = [&] {
        try {
            ok = fn();
        } catch (...) {
            ok = false;
        }
    };
    return guarded(call) && ok;
}

LightState lightState(std::uint32_t raw) {
    using S = prism::traffic_light_t::ELightState;
    switch (raw) {
        case S::OFF: return LightState::Off;
        case S::ORANGE_TO_RED: return LightState::AmberToRed;
        case S::RED: return LightState::Red;
        case S::ORANGE_TO_GREEN: return LightState::AmberToGreen;
        case S::GREEN: return LightState::Green;
        case S::SLEEP: return LightState::Flashing;
        default: return LightState::Unknown;
    }
}

Vec2 planOf(const ets2la_plugin::float3_t& world) {
    return coords::worldToPlan(Vec3{world.x, world.y, world.z});
}

// Centre and heading of one traffic body; the placement is not at the centre,
// so the bounding box centre is rotated into world space (as ETS2LA does).
void addBody(const prism::traffic_actor_t* a, int id, double speed, const Vec2& truck, double radius,
             std::vector<WorldVehicle>& out) {
    const auto& pl = a->placement;
    const auto& bb = a->aabox;
    const ets2la_plugin::float3_t local{(bb.start.x + bb.end.x) * 0.5f, (bb.start.y + bb.end.y) * 0.5f,
                                        (bb.start.z + bb.end.z) * 0.5f};
    const ets2la_plugin::float3_t centre = pl.to_global_position() + local.rotate(pl.rot);
    const ets2la_plugin::float3_t forward = ets2la_plugin::float3_t{0.0f, 0.0f, -1.0f}.rotate(pl.rot);
    WorldVehicle v;
    v.id = id;
    v.position = planOf(centre);
    if (distance(v.position, truck) > radius) return;
    v.yaw = std::atan2(-static_cast<double>(forward.z), static_cast<double>(forward.x));
    v.length = std::abs(bb.end.z - bb.start.z);
    v.width = std::abs(bb.end.x - bb.start.x);
    v.speed = speed;
    if (!(v.length > 0.5 && v.length < 40.0 && v.width > 0.5 && v.width < 6.0)) return;  // implausible: skip
    out.push_back(v);
}

void addVehicle(const prism::traffic_ai_vehicle_t* veh, const Vec2& truck, double radius,
                std::unordered_set<const void*>& seen, std::vector<WorldVehicle>& out) {
    if (veh == nullptr || veh->physics == nullptr || !seen.insert(veh).second) return;
    const int id = static_cast<int>((reinterpret_cast<std::uintptr_t>(veh) >> 4) & 0x7FFFFFFF);
    const double speed = veh->physics->speed;
    const prism::traffic_actor_t* body = veh;
    for (int k = 0; body != nullptr && k < 5; ++k, body = body->slave) addBody(body, id, speed, truck, radius, out);
}

void collectTraffic(const Vec2& truck, double radius, std::vector<WorldVehicle>& out) {
    auto* traffic = prism::game_traffic_u::get();
    if (traffic == nullptr) return;
    std::unordered_set<const void*> seen;
    for (auto& sv : traffic->spawned_vehicles_1) addVehicle(sv.vehicle, truck, radius, seen, out);
    for (auto& sv : traffic->spawned_vehicles_2) addVehicle(sv.vehicle, truck, radius, seen, out);
    for (auto* obj : traffic->traffic_objects_1) {
        if (obj == nullptr || obj->get_type() != prism::ETrafficObjectType::traffic_ai_vehicle) continue;
        addVehicle(reinterpret_cast<const prism::traffic_ai_vehicle_t*>(obj), truck, radius, seen, out);
    }
}

void collectLights(const Vec2& truck, double radius, std::vector<WorldLight>& out) {
    auto* base = prism::base_ctrl_u::get();
    if (base == nullptr) return;
    auto* items = base->get_nearby_kdop_items();
    if (items == nullptr) return;
    for (auto* item : *items) {
        if (item == nullptr) continue;
        if (item->item_type != 4) {  // 4 = prefab; items are ordered by type
            if (item->item_type > 4) break;
            continue;
        }
        const auto* prefab = static_cast<const prism::prefab_item_t*>(item);
        if (prefab->segment == nullptr) continue;
        for (auto& inst : prefab->segment->semaphore_instances) {
            const auto* actor = inst.actor;
            if (actor == nullptr || actor->get_type() != prism::ETrafficObjectType::traffic_semaphore_actor) continue;
            const auto* sem = static_cast<const prism::traffic_semaphore_actor_t*>(actor);
            const auto* rule = sem->traffic_rule;
            if (rule == nullptr || rule->get_type() != prism::ETrafficObjectType::traffic_light) continue;
            WorldLight l;
            l.semaphoreId = static_cast<int>(inst.id);
            l.position = planOf(sem->placement.to_global_position());
            if (distance(l.position, truck) > radius) continue;
            l.state = lightState(rule->state);
            l.timeRemaining = rule->state_time_remaining;
            out.push_back(l);
        }
    }
}

struct RawRoute {
    std::uint64_t size = 0;
    std::uint64_t firstUid = 0;
    std::uint64_t lastUid = 0;
    bool active = false;
};

void routeHeader(RawRoute& r) {
    auto* gps = prism::gps_manager_t::get();
    if (gps == nullptr || gps->simple_route_source.route_task == nullptr) return;
    auto& items = gps->simple_route_source.route_task->physical_route_items;
    if (items.size == 0 || items.size > 50000) return;
    r.active = true;
    r.size = items.size;
    r.firstUid = items[0].node ? items[0].node->uid : 0;
    r.lastUid = items[items.size - 1].node ? items[items.size - 1].node->uid : 0;
}

struct RawNode {
    double x = 0.0, y = 0.0, z = 0.0;
    float toEnd = 0.0f;
};

void routeNodes(std::vector<RawNode>& out) {
    auto* gps = prism::gps_manager_t::get();
    if (gps == nullptr || gps->simple_route_source.route_task == nullptr) return;
    for (auto& item : gps->simple_route_source.route_task->physical_route_items) {
        if (item.node == nullptr) continue;
        out.push_back({static_cast<double>(item.node->coords.x), static_cast<double>(item.node->coords.y),
                       static_cast<double>(item.node->coords.z), item.total_distance_till_end});
    }
}

}  // namespace

GameMemory::GameMemory(Logger& log, const GameMemoryConfig& cfg) : log_(log), cfg_(cfg) {
    g_log = &log_;
    g_core.sink = &coreSink;
    ets2la_plugin::CCore::g_instance = &g_core;
}

GameMemory::~GameMemory() {
    if (scanner_.joinable()) scanner_.join();
    g_core.sink = nullptr;
    g_log = nullptr;
}

void GameMemory::start(const std::string& gameName) {
    if (state_.load() != State::Idle) return;
    if (!cfg_.enabled) {
        state_ = State::Unavailable;
        return;
    }
    if (gameName.find(kSupportedVersion) == std::string::npos) {
        state_ = State::Unavailable;
        log_.warn("Game memory features need game version {}x, found '{}'; driving on map data only",
                  kSupportedVersion, gameName);
        return;
    }
    state_ = State::Scanning;
    scanner_ = std::thread([this] { scan(); });
}

void GameMemory::scan() {
    const auto t0 = std::chrono::steady_clock::now();
    const bool base = scanStep(&prism::base_ctrl_u::scan_patterns);
    const bool game = base && scanStep(&prism::game_ctrl_u::scan_patterns);
    const bool traffic = scanStep(&prism::game_traffic_u::scan_patterns);
    const bool steering = base && scanStep(&prism::vehicle_shared_u::scan_patterns);
    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

    trafficOk_ = traffic;
    lightsOk_ = base;
    gpsOk_ = game;
    steeringOk_ = steering;
    if (!traffic && !base) {
        state_ = State::Unavailable;
        log_.warn("Game memory: no known structures found ({:.1f} s); driving on map data only", secs);
        return;
    }
    log_.info("Game memory ready in {:.1f} s: traffic {}, traffic lights {}, GPS route {}, steering {}", secs,
              traffic ? "yes" : "no", base ? "yes" : "no", game ? "yes" : "no", steering ? "yes" : "no");
    state_ = State::Ready;
}

void GameMemory::fault(const char* what, bool& feature) {
    feature = false;
    ++faults_;
    log_.error("Game memory: reading {} failed; {} switched off for this session", what, what);
}

WorldSnapshotPtr GameMemory::readWorld(double time, const Vec3& truckWorld, double radius) {
    auto snap = std::make_shared<WorldSnapshot>();
    snap->time = time;
    if (!ready()) return snap;
    const Vec2 truck = coords::worldToPlan(truckWorld);

    if (cfg_.traffic && trafficOk_) {
        snap->vehicles.reserve(256);
        auto read = [&] { collectTraffic(truck, radius, snap->vehicles); };
        if (guarded(read)) {
            snap->valid = true;
        } else {
            snap->vehicles.clear();
            fault("traffic", trafficOk_);
        }
    }
    if (cfg_.trafficLights && lightsOk_) {
        snap->lights.reserve(64);
        auto read = [&] { collectLights(truck, radius, snap->lights); };
        if (!guarded(read)) {
            snap->lights.clear();
            fault("traffic lights", lightsOk_);
        }
    }
    return snap;
}

std::optional<std::vector<Vec2>> GameMemory::readGpsRouteIfChanged(const Vec3& truckWorld) {
    if (!ready() || !cfg_.gpsRoute || !gpsOk_) return std::nullopt;
    RawRoute header;
    auto readHeader = [&] { routeHeader(header); };
    if (!guarded(readHeader)) {
        fault("GPS route", gpsOk_);
        return std::nullopt;
    }
    if (!header.active) {
        if (routeSize_ == 0) return std::nullopt;
        routeSize_ = routeFirstUid_ = routeLastUid_ = 0;
        return std::vector<Vec2>{};
    }
    if (header.size == routeSize_ && header.firstUid == routeFirstUid_ && header.lastUid == routeLastUid_) {
        return std::nullopt;
    }

    std::vector<RawNode> nodes;
    nodes.reserve(static_cast<std::size_t>(header.size));
    auto readNodes = [&] { routeNodes(nodes); };
    if (!guarded(readNodes)) {
        fault("GPS route", gpsOk_);
        return std::nullopt;
    }
    if (nodes.size() < 2) return std::nullopt;
    routeSize_ = header.size;
    routeFirstUid_ = header.firstUid;
    routeLastUid_ = header.lastUid;

    // Order from the truck towards the destination.
    std::stable_sort(nodes.begin(), nodes.end(), [](const RawNode& a, const RawNode& b) { return a.toEnd > b.toEnd; });

    // Node coordinates are fixed point. Pick the scale that puts the route near
    // the truck (it starts where the truck is).
    const Vec2 truck = coords::worldToPlan(truckWorld);
    auto nearest = [&](double scale) {
        double best = 1e18;
        for (const auto& n : nodes) {
            best = std::min(best, distance(coords::worldToPlan(Vec3{n.x * scale, n.y * scale, n.z * scale}), truck));
        }
        return best;
    };
    if (routeCoordScale_ == 0.0) {
        for (const double scale : {1.0 / 256.0, 1.0}) {
            const double d = nearest(scale);
            if (d < 3000.0) {
                routeCoordScale_ = scale;
                log_.info("Game memory: GPS route node scale 1/{:.0f} (nearest node {:.0f} m)", 1.0 / scale, d);
                break;
            }
        }
        if (routeCoordScale_ == 0.0) {
            log_.warn("Game memory: GPS route nodes are not near the truck; GPS route following off");
            gpsOk_ = false;
            return std::nullopt;
        }
    }
    std::vector<Vec2> pts;
    pts.reserve(nodes.size());
    for (const auto& n : nodes) {
        pts.push_back(coords::worldToPlan(Vec3{n.x * routeCoordScale_, n.y * routeCoordScale_, n.z * routeCoordScale_}));
    }
    log_.info("In-game GPS route: {} nodes, {:.1f} km", pts.size(), nodes.front().toEnd / 1000.0);
    return pts;
}

std::optional<double> GameMemory::readRawSteering() {
    double value = 0.0;
    bool found = false;
    auto read = [&] {
        auto* actor = prism::game_actor_u::get();
        if (actor == nullptr || actor->game_physics_vehicle == nullptr) return;
        value = actor->game_physics_vehicle->get_steering_angle();
        found = true;
    };
    if (!guarded(read)) {
        fault("steering", steeringOk_);
        return std::nullopt;
    }
    if (!found || !std::isfinite(value)) return std::nullopt;
    return value;
}

void GameMemory::observeSteering(double effective) {
    if (!steeringAvailable() || steeringCalibrated_) return;
    if (std::abs(effective) < 0.08) return;
    const auto raw = readRawSteering();
    if (!raw || std::abs(*raw) < 1e-3) return;
    ++steeringSamples_;
    if ((*raw > 0.0) == (effective > 0.0)) ++steeringAgree_;
    steeringRatioSum_ += std::abs(*raw / effective);
    if (steeringSamples_ < 40) return;

    steeringCalibrated_ = true;
    const double agree = static_cast<double>(steeringAgree_) / steeringSamples_;
    const double scale = steeringRatioSum_ / steeringSamples_;
    if (agree > 0.9) steeringSign_ = 1.0;
    else if (agree < 0.1) steeringSign_ = -1.0;
    if (scale > 0.2 && scale < 5.0) steeringScale_ = scale;
    log_.info("Game memory: steering calibrated (sign {:+.0f}, scale {:.2f}, {:.0f}% sign agreement)", steeringSign_,
              steeringScale_, agree * 100.0);
    if (agree >= 0.1 && agree <= 0.9) {
        log_.warn("Game memory: stored steering does not follow the SDK's consistently; direct steering off");
        steeringOk_ = false;
    }
}

bool GameMemory::writeSteering(double steering) {
    if (!steeringAvailable()) return false;
    const float raw = static_cast<float>(steeringSign_ * steeringScale_ * clamp(steering, -1.0, 1.0));
    bool written = false;
    auto write = [&] {
        auto* actor = prism::game_actor_u::get();
        if (actor == nullptr || actor->game_physics_vehicle == nullptr) return;
        written = actor->game_physics_vehicle->set_steering_angle(raw);
    };
    if (!guarded(write)) {
        fault("steering", steeringOk_);
        return false;
    }
    return written;
}

}  // namespace atspilot::plugin

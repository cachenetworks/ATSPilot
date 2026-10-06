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
#include "prism/traffic/objects/traffic_parked_actor.hpp"
#include "prism/traffic/objects/traffic_parked_trailer.hpp"
#include "prism/traffic/objects/traffic_player_trailer.hpp"
#include "prism/traffic/objects/traffic_player_vehicle.hpp"
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
    v.height = centre.y - 0.5 * std::abs(bb.end.y - bb.start.y);
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

void addPlayerVehicle(const prism::traffic_player_vehicle_t* veh, const Vec2& truck, double radius,
                      std::unordered_set<const void*>& seen, std::vector<WorldVehicle>& out) {
    if (veh == nullptr || !seen.insert(veh).second) return;
    const int id = static_cast<int>((reinterpret_cast<std::uintptr_t>(veh) >> 4) & 0x7FFFFFFF);
    const double speed = std::isfinite(veh->speed) ? std::max(0.0, static_cast<double>(veh->speed)) : 0.0;
    const prism::traffic_actor_t* explicitTrailer = static_cast<const prism::traffic_actor_t*>(veh->trailer);
    const prism::traffic_actor_t* body = veh;
    bool trailerInChain = false;
    for (int k = 0; body != nullptr && k < 6; ++k, body = body->slave) {
        if (body == explicitTrailer) trailerInChain = true;
        addBody(body, id, speed, truck, radius, out);
    }

    // Convoy trailers are also exposed directly from the player vehicle. Some
    // game builds link them anywhere in actor::slave as well, so only add the
    // explicit trailer when the primary actor chain did not already include it.
    if (explicitTrailer != nullptr && !trailerInChain) {
        body = explicitTrailer;
        for (int k = 0; body != nullptr && k < 5; ++k, body = body->slave) addBody(body, id, speed, truck, radius, out);
    }
}

void addParkedActor(const prism::traffic_actor_t* actor, const Vec2& truck, double radius,
                    std::unordered_set<const void*>& seen, std::vector<WorldVehicle>& out) {
    if (actor == nullptr || !seen.insert(actor).second) return;
    const int id = static_cast<int>((reinterpret_cast<std::uintptr_t>(actor) >> 4) & 0x7FFFFFFF);
    const prism::traffic_actor_t* body = actor;
    for (int k = 0; body != nullptr && k < 6; ++k, body = body->slave) addBody(body, id, 0.0, truck, radius, out);
}

void addTrafficObject(const prism::traffic_object_t* obj, const Vec2& truck, double radius,
                      std::unordered_set<const void*>& seen, std::vector<WorldVehicle>& out) {
    if (obj == nullptr) return;
    switch (obj->get_type()) {
        case prism::ETrafficObjectType::traffic_ai_vehicle:
            addVehicle(static_cast<const prism::traffic_ai_vehicle_t*>(obj), truck, radius, seen, out);
            break;
        case prism::ETrafficObjectType::traffic_player_vehicle:
            addPlayerVehicle(static_cast<const prism::traffic_player_vehicle_t*>(obj), truck, radius, seen, out);
            break;
        case prism::ETrafficObjectType::traffic_parked_vehicle:
            // Random-road-event vehicles use the actor-backed parked layout.
            addParkedActor(static_cast<const prism::traffic_parked_actor_t*>(obj), truck, radius, seen, out);
            break;
        case prism::ETrafficObjectType::traffic_parked_trailer: {
            const auto* trailer = static_cast<const prism::traffic_parked_trailer_t*>(obj);
            addParkedActor(static_cast<const prism::traffic_parked_actor_t*>(trailer), truck, radius, seen, out);
            break;
        }
        default:
            break;
    }
}

struct TrafficPoolStats {
    std::uint64_t size = 0;
    std::uint64_t ai = 0;
    std::uint64_t player = 0;
    std::uint64_t parked = 0;
    std::uint64_t parkedTrailer = 0;
    std::uint64_t semaphore = 0;
    std::uint64_t roadBlock = 0;
    std::uint64_t roadworkLight = 0;
    std::uint64_t other = 0;
    std::uint64_t firstOther = 0;
};

TrafficPoolStats trafficPoolStats(const prism::array_dyn_t<prism::traffic_object_t*>& pool) {
    TrafficPoolStats stats;
    stats.size = pool.size;
    for (std::uint64_t i = 0; i < pool.size; ++i) {
        auto* obj = pool.value[i];
        if (obj == nullptr) continue;
        const auto type = obj->get_type();
        switch (type) {
            case prism::ETrafficObjectType::traffic_ai_vehicle: ++stats.ai; break;
            case prism::ETrafficObjectType::traffic_player_vehicle: ++stats.player; break;
            case prism::ETrafficObjectType::traffic_parked_vehicle: ++stats.parked; break;
            case prism::ETrafficObjectType::traffic_parked_trailer: ++stats.parkedTrailer; break;
            case prism::ETrafficObjectType::traffic_semaphore_actor: ++stats.semaphore; break;
            case prism::ETrafficObjectType::road_block: ++stats.roadBlock; break;
            case prism::ETrafficObjectType::traffic_light_roadwork: ++stats.roadworkLight; break;
            default:
                ++stats.other;
                if (stats.firstOther == 0) stats.firstOther = static_cast<std::uint64_t>(type);
                break;
        }
    }
    return stats;
}

void collectTraffic(const Vec2& truck, double radius, std::vector<WorldVehicle>& out) {
    auto* traffic = prism::game_traffic_u::get();
    if (traffic == nullptr) return;
    std::unordered_set<const void*> seen;
    for (auto& sv : traffic->spawned_vehicles_1) addVehicle(sv.vehicle, truck, radius, seen, out);
    for (auto& sv : traffic->spawned_vehicles_2) addVehicle(sv.vehicle, truck, radius, seen, out);
    for (auto* veh : traffic->traffic_player_vehicles_1) addPlayerVehicle(veh, truck, radius, seen, out);
    for (auto* veh : traffic->traffic_player_vehicles_2) addPlayerVehicle(veh, truck, radius, seen, out);
    for (auto* obj : traffic->traffic_objects_1) addTrafficObject(obj, truck, radius, seen, out);
    // SCS keeps additional traffic-object pools after the primary actor list.
    // Random road events can live in these pools, so consume only the object
    // types whose actor layouts are known and leave rule-only objects untouched.
    for (auto* obj : traffic->traffic_objects_2) addTrafficObject(obj, truck, radius, seen, out);
    for (auto* obj : traffic->traffic_objects_3) addTrafficObject(obj, truck, radius, seen, out);

    // Keep enough live evidence to identify roadwork rule/object placement if
    // the actor-backed blockers still are not sufficient. Rate-limit this to
    // avoid turning the normal driving log into a frame-by-frame trace.
    static ULONGLONG lastPoolLog = 0;
    const ULONGLONG now = GetTickCount64();
    if (g_log != nullptr && now - lastPoolLog >= 5000) {
        const auto p1 = trafficPoolStats(traffic->traffic_objects_1);
        const auto p2 = trafficPoolStats(traffic->traffic_objects_2);
        const auto p3 = trafficPoolStats(traffic->traffic_objects_3);
        const bool interesting = p2.size != 0 || p3.size != 0 || p1.parked != 0 || p1.parkedTrailer != 0 ||
                                 p1.roadBlock != 0 || p1.roadworkLight != 0;
        if (interesting) {
            g_log->info(
                "Game memory traffic pools: o1={} ai={} player={} parked={}/{} sem={} block={} roadwork={} other={} first=0x{:x}; "
                "o2={} ai={} player={} parked={}/{} sem={} block={} roadwork={} other={} first=0x{:x}; "
                "o3={} ai={} player={} parked={}/{} sem={} block={} roadwork={} other={} first=0x{:x}",
                p1.size, p1.ai, p1.player, p1.parked, p1.parkedTrailer, p1.semaphore, p1.roadBlock,
                p1.roadworkLight, p1.other, p1.firstOther,
                p2.size, p2.ai, p2.player, p2.parked, p2.parkedTrailer, p2.semaphore, p2.roadBlock,
                p2.roadworkLight, p2.other, p2.firstOther,
                p3.size, p3.ai, p3.player, p3.parked, p3.parkedTrailer, p3.semaphore, p3.roadBlock,
                p3.roadworkLight, p3.other, p3.firstOther);
        }
        lastPoolLog = now;
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
            if (rule == nullptr) continue;
            const auto ruleType = rule->get_type();
            if (ruleType != prism::ETrafficObjectType::traffic_light &&
                ruleType != prism::ETrafficObjectType::traffic_light_roadwork) {
                continue;
            }
            WorldLight l;
            l.semaphoreId = static_cast<int>(inst.id);
            l.position = planOf(sem->placement.to_global_position());
            if (distance(l.position, truck) > radius) continue;
            const auto forward = ets2la_plugin::float3_t{0.0f, 0.0f, -1.0f}.rotate(sem->placement.rot);
            l.yaw = std::atan2(-static_cast<double>(forward.z), static_cast<double>(forward.x));
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
    std::uint64_t uid = 0;
    float toEnd = 0.0f;
};

void routeNodes(std::vector<RawNode>& out) {
    auto* gps = prism::gps_manager_t::get();
    if (gps == nullptr || gps->simple_route_source.route_task == nullptr) return;
    for (auto& item : gps->simple_route_source.route_task->physical_route_items) {
        if (item.node == nullptr) continue;
        out.push_back({item.node->uid, item.total_distance_till_end});
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

void GameMemory::start(const std::string& gameVersion) {
    if (state_.load() != State::Idle) return;
    if (!cfg_.enabled) {
        state_ = State::Unavailable;
        return;
    }
    if (gameVersion.empty()) {
        state_ = State::Unavailable;
        log_.warn("Game memory features need game version {}x, but the executable version could not be read; driving on map data only",
                  kSupportedVersion);
        return;
    }
    if (gameVersion.find(kSupportedVersion) == std::string::npos) {
        state_ = State::Unavailable;
        log_.warn("Game memory features need game version {}x, found '{}'; driving on map data only",
                  kSupportedVersion, gameVersion);
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

std::optional<std::vector<std::uint64_t>> GameMemory::readGpsRouteIfChanged() {
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
        return std::vector<std::uint64_t>{};
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
    std::vector<std::uint64_t> uids;
    uids.reserve(nodes.size());
    for (const auto& n : nodes) uids.push_back(n.uid);
    log_.info("In-game GPS route changed: {} nodes, {:.1f} km", uids.size(), nodes.front().toEnd / 1000.0);
    return uids;
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

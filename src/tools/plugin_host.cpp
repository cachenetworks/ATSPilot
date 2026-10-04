// Plugin host: loads the real atspilot.dll and plays the game's side of the SCS
// SDK (telemetry events/channels and the input device), with the kinematic
// simulator driving on real ATS road geometry from the map cache. It exercises
// the whole in-game chain except ATS's own vehicle physics:
//
//   simulated truck ─▶ SDK telemetry callbacks ─▶ ATSPilot ─▶ semantical input ─▶ simulated truck
//
//   atspilot_plugin_host <atspilot.dll> <game_dir> <map.cache> [scenarios] [seconds] [speedup]

#include <windows.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "map/LanePlanner.h"
#include "map/RoutePlanner.h"
#include "map/Token.h"
#include "map/RoadNetwork.h"
#include "math/Coordinates.h"
#include "math/MathUtil.h"
#include "scssdk_input.h"
#include "scssdk_telemetry.h"
#include "sim/VehicleSim.h"
#include "common/scssdk_telemetry_common_configs.h"
#include "common/scssdk_telemetry_truck_common_channels.h"
#include "amtrucks/scssdk_ats.h"
#include "amtrucks/scssdk_telemetry_ats.h"
#include "amtrucks/scssdk_input_ats.h"

using namespace atspilot;

namespace {

struct ChannelReg {
    scs_telemetry_channel_callback_t cb = nullptr;
    scs_context_t ctx = nullptr;
    scs_value_type_t type = 0;
};

std::map<scs_event_t, std::pair<scs_telemetry_event_callback_t, scs_context_t>> g_events;
std::map<std::pair<std::string, scs_u32_t>, ChannelReg> g_channels;
scs_input_device_t g_device{};
std::vector<scs_input_device_input_t> g_deviceInputs;
std::vector<std::string> g_inputNames;

SCSAPI_VOID hostLog(const scs_log_type_t type, const scs_string_t msg) {
    std::printf("  [game log %d] %s\n", type, msg);
}

SCSAPI_RESULT regEvent(const scs_event_t e, const scs_telemetry_event_callback_t cb, const scs_context_t ctx) {
    g_events[e] = {cb, ctx};
    return SCS_RESULT_ok;
}
SCSAPI_RESULT unregEvent(const scs_event_t e) {
    g_events.erase(e);
    return SCS_RESULT_ok;
}
SCSAPI_RESULT regChannel(const scs_string_t name, const scs_u32_t index, const scs_value_type_t type, const scs_u32_t,
                         const scs_telemetry_channel_callback_t cb, const scs_context_t ctx) {
    g_channels[{name, index}] = {cb, ctx, type};
    return SCS_RESULT_ok;
}
SCSAPI_RESULT unregChannel(const scs_string_t name, const scs_u32_t index, const scs_value_type_t) {
    g_channels.erase({name, index});
    return SCS_RESULT_ok;
}
SCSAPI_RESULT regDevice(const scs_input_device_t* const d) {
    g_device = *d;
    g_deviceInputs.assign(d->inputs, d->inputs + d->input_count);
    for (const auto& i : g_deviceInputs) g_inputNames.push_back(i.name);
    g_device.inputs = g_deviceInputs.data();
    return SCS_RESULT_ok;
}

void fireEvent(scs_event_t e, const void* info) {
    const auto it = g_events.find(e);
    if (it != g_events.end()) it->second.first(e, info, it->second.second);
}

void sendFloat(const char* name, float v, scs_u32_t index = SCS_U32_NIL) {
    const auto it = g_channels.find({name, index});
    if (it == g_channels.end()) return;
    scs_value_t val{};
    val.type = SCS_VALUE_TYPE_float;
    val.value_float.value = v;
    it->second.cb(name, index, &val, it->second.ctx);
}

void sendBool(const char* name, bool v) {
    const auto it = g_channels.find({name, SCS_U32_NIL});
    if (it == g_channels.end()) return;
    scs_value_t val{};
    val.type = SCS_VALUE_TYPE_bool;
    val.value_bool.value = v ? 1 : 0;
    it->second.cb(name, SCS_U32_NIL, &val, it->second.ctx);
}

void sendS32(const char* name, int v) {
    const auto it = g_channels.find({name, SCS_U32_NIL});
    if (it == g_channels.end()) return;
    scs_value_t val{};
    val.type = SCS_VALUE_TYPE_s32;
    val.value_s32.value = v;
    it->second.cb(name, SCS_U32_NIL, &val, it->second.ctx);
}

void sendVector(const char* name, const Vec3& v) {
    const auto it = g_channels.find({name, SCS_U32_NIL});
    if (it == g_channels.end()) return;
    scs_value_t val{};
    val.type = SCS_VALUE_TYPE_fvector;
    val.value_fvector = {static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)};
    it->second.cb(name, SCS_U32_NIL, &val, it->second.ctx);
}

void sendPlacement(const Vec3& pos, double headingUnit) {
    const char* name = SCS_TELEMETRY_TRUCK_CHANNEL_world_placement;
    const auto it = g_channels.find({name, SCS_U32_NIL});
    if (it == g_channels.end()) return;
    scs_value_t val{};
    val.type = SCS_VALUE_TYPE_dplacement;
    val.value_dplacement.position = {pos.x, pos.y, pos.z};
    val.value_dplacement.orientation = {static_cast<float>(headingUnit), 0.0f, 0.0f};
    it->second.cb(name, SCS_U32_NIL, &val, it->second.ctx);
}

void sendTruckConfig(const sim::SimParams& p) {
    // 6 wheels: steer axle at frontAxleZ, two driven axles around rearAxleZ.
    struct W {
        Vec3 pos;
        bool steer;
        bool powered;
    };
    const W wheels[] = {{{-1.0, 0.5, p.frontAxleZ}, true, false},  {{1.0, 0.5, p.frontAxleZ}, true, false},
                        {{-1.0, 0.5, p.rearAxleZ - 0.65}, false, true}, {{1.0, 0.5, p.rearAxleZ - 0.65}, false, true},
                        {{-1.0, 0.5, p.rearAxleZ + 0.65}, false, true}, {{1.0, 0.5, p.rearAxleZ + 0.65}, false, true}};
    std::vector<scs_named_value_t> attrs;
    auto add = [&](const char* name, scs_u32_t index, scs_value_t v) {
        scs_named_value_t a{};
        a.name = name;
        a.index = index;
        a.value = v;
        attrs.push_back(a);
    };
    scs_value_t v{};
    v.type = SCS_VALUE_TYPE_string;
    v.value_string.value = "Host Test Truck";
    add(SCS_TELEMETRY_CONFIG_ATTRIBUTE_name, SCS_U32_NIL, v);
    v = {};
    v.type = SCS_VALUE_TYPE_u32;
    v.value_u32.value = 6;
    add(SCS_TELEMETRY_CONFIG_ATTRIBUTE_wheel_count, SCS_U32_NIL, v);
    for (scs_u32_t i = 0; i < 6; ++i) {
        v = {};
        v.type = SCS_VALUE_TYPE_fvector;
        v.value_fvector = {static_cast<float>(wheels[i].pos.x), static_cast<float>(wheels[i].pos.y),
                           static_cast<float>(wheels[i].pos.z)};
        add(SCS_TELEMETRY_CONFIG_ATTRIBUTE_wheel_position, i, v);
        v = {};
        v.type = SCS_VALUE_TYPE_bool;
        v.value_bool.value = wheels[i].steer;
        add(SCS_TELEMETRY_CONFIG_ATTRIBUTE_wheel_steerable, i, v);
        v.value_bool.value = wheels[i].powered;
        add(SCS_TELEMETRY_CONFIG_ATTRIBUTE_wheel_powered, i, v);
    }
    attrs.push_back(scs_named_value_t{});  // terminator: name == nullptr
    scs_telemetry_configuration_t cfg{};
    cfg.id = SCS_TELEMETRY_CONFIG_truck;
    cfg.attributes = attrs.data();
    fireEvent(SCS_TELEMETRY_EVENT_configuration, &cfg);
}

void sendJobConfig(const std::string& cityId, const std::string& companyId) {
    std::vector<scs_named_value_t> attrs;
    auto addString = [&](const char* name, const std::string* value) {
        scs_named_value_t a{};
        a.name = name;
        a.index = SCS_U32_NIL;
        a.value.type = SCS_VALUE_TYPE_string;
        a.value.value_string.value = value->c_str();
        attrs.push_back(a);
    };
    static std::string city, company, cityName, cargo;
    city = cityId;
    company = companyId;
    cityName = cityId;
    cargo = "test_cargo";
    addString(SCS_TELEMETRY_CONFIG_ATTRIBUTE_cargo_id, &cargo);
    addString(SCS_TELEMETRY_CONFIG_ATTRIBUTE_destination_city_id, &city);
    addString(SCS_TELEMETRY_CONFIG_ATTRIBUTE_destination_city, &cityName);
    addString(SCS_TELEMETRY_CONFIG_ATTRIBUTE_destination_company_id, &company);
    scs_named_value_t mass{};
    mass.name = SCS_TELEMETRY_CONFIG_ATTRIBUTE_cargo_mass;
    mass.index = SCS_U32_NIL;
    mass.value.type = SCS_VALUE_TYPE_float;
    mass.value.value_float.value = 18000.0f;
    attrs.push_back(mass);
    attrs.push_back(scs_named_value_t{});
    scs_telemetry_configuration_t cfg{};
    cfg.id = SCS_TELEMETRY_CONFIG_job;
    cfg.attributes = attrs.data();
    fireEvent(SCS_TELEMETRY_EVENT_configuration, &cfg);
}

// Applies one input event from the plugin's device to the analogue values and
// this frame's button presses (indices follow the device's input list).
void applyInputEvent(const scs_input_event_t& ev, float* analog, GameButtons& buttons) {
    if (ev.input_index < 3) {
        analog[ev.input_index] = ev.value_float.value;
        return;
    }
    if (!ev.value_bool.value || ev.input_index >= g_inputNames.size()) return;
    const std::string& n = g_inputNames[ev.input_index];
    if (n == "cruiectrl") buttons.cruiseToggle = true;
    else if (n == "cruiectrlinc") buttons.cruiseInc = true;
    else if (n == "cruiectrldec") buttons.cruiseDec = true;
    else if (n == "cruiectrlres") buttons.cruiseResume = true;
    else if (n == "lblinker") buttons.leftBlinker = true;
    else if (n == "rblinker") buttons.rightBlinker = true;
    else if (n == "quickpark") buttons.quickPark = true;
}

std::string devNavigation(HMODULE dll) {
    using Fn = int (*)(char*, int);
    auto fn = reinterpret_cast<Fn>(GetProcAddress(dll, "atspilot_dev_navigation"));
    char buf[512] = {};
    if (fn) fn(buf, sizeof(buf));
    return buf;
}

std::string devState(HMODULE dll) {
    using Fn = int (*)(char*, int);
    auto fn = reinterpret_cast<Fn>(GetProcAddress(dll, "atspilot_dev_state"));
    char buf[512] = {};
    if (fn) fn(buf, sizeof(buf));
    return buf;
}

std::optional<RoadNetwork> loadCache(const std::string& file) {
    std::ifstream in(file, std::ios::binary);
    char magic[8];
    std::uint32_t version = 0, fpLen = 0;
    in.read(magic, 8);
    in.read(reinterpret_cast<char*>(&version), 4);
    in.read(reinterpret_cast<char*>(&fpLen), 4);
    std::string fp(fpLen, '\0');
    in.read(fp.data(), fpLen);
    in.close();
    return RoadNetwork::load(file, fp);
}

// Picks start lanes on long, connected road stretches (highway-like).
std::vector<std::uint32_t> pickStarts(const RoadNetwork& net, int count, std::uint32_t seed) {
    std::mt19937 rng(seed);
    std::uniform_int_distribution<std::size_t> pick(0, net.size() - 1);
    std::vector<std::uint32_t> out;
    for (int attempts = 0; attempts < 200000 && static_cast<int>(out.size()) < count; ++attempts) {
        const auto id = static_cast<std::uint32_t>(pick(rng));
        const auto& s = net.segment(id);
        if (s.kind != LaneKind::Road || s.length < 150.0f || s.next.empty()) continue;
        LaneMatch m;
        m.segment = id;
        PathBuildParams pp;
        pp.ahead = 3000.0;
        const PlannedPath plan = buildPlannedPath(net, m, pp, {});
        if (plan.path.length() < 2500.0) continue;
        out.push_back(id);
    }
    return out;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 4) {
        std::fprintf(stderr, "usage: atspilot_plugin_host <atspilot.dll> <game_dir> <map.cache> [scenarios] [seconds] [speedup] [nav_seconds]\n");
        return 2;
    }
    const std::filesystem::path dllPath = std::filesystem::absolute(argv[1]);
    const std::string gameDir = argv[2];
    const std::string cacheFile = argv[3];
    const int scenarios = argc > 4 ? std::atoi(argv[4]) : 5;
    const double seconds = argc > 5 ? std::atof(argv[5]) : 90.0;
    const double speedup = argc > 6 ? std::atof(argv[6]) : 4.0;
    const double navSeconds = argc > 7 ? std::atof(argv[7]) : 0.0;

    const auto net = loadCache(cacheFile);
    if (!net) {
        std::fprintf(stderr, "cannot load map cache %s\n", cacheFile.c_str());
        return 1;
    }

    // Isolated data directory with a config pointing at the game data.
    const auto dataDir = std::filesystem::temp_directory_path() / "atspilot_plugin_host";
    std::filesystem::create_directories(dataDir / "cache");
    std::filesystem::copy_file(cacheFile, dataDir / "cache" / "map_usa.cache",
                               std::filesystem::copy_options::overwrite_existing);
    {
        std::string gd = gameDir;
        for (auto& c : gd) if (c == '\\') c = '/';
        std::ofstream cfg(dataDir / "atspilot.toml");
        cfg << "[map]\ngame_dir = \"" << gd << "\"\n[audio]\nenabled = false\n[debug]\nlog_level = \"info\"\n"
            << "record_telemetry = true\n";
    }
    SetEnvironmentVariableW(L"ATSPILOT_DATA_DIR", dataDir.wstring().c_str());

    HMODULE dll = LoadLibraryW(dllPath.wstring().c_str());
    if (!dll) {
        std::fprintf(stderr, "LoadLibrary failed (%lu)\n", GetLastError());
        return 1;
    }
    using TelInit = scs_result_t(SCSAPIFUNC*)(scs_u32_t, const scs_telemetry_init_params_t*);
    using InInit = scs_result_t(SCSAPIFUNC*)(scs_u32_t, const scs_input_init_params_t*);
    using Shutdown = void(SCSAPIFUNC*)();
    auto telInit = reinterpret_cast<TelInit>(GetProcAddress(dll, "scs_telemetry_init"));
    auto telShutdown = reinterpret_cast<Shutdown>(GetProcAddress(dll, "scs_telemetry_shutdown"));
    auto inInit = reinterpret_cast<InInit>(GetProcAddress(dll, "scs_input_init"));
    auto inShutdown = reinterpret_cast<Shutdown>(GetProcAddress(dll, "scs_input_shutdown"));
    auto devRequest = reinterpret_cast<int (*)(int)>(GetProcAddress(dll, "atspilot_dev_request"));
    if (!telInit || !telShutdown || !inInit || !inShutdown || !devRequest) {
        std::fprintf(stderr, "missing exports\n");
        return 1;
    }

    scs_telemetry_init_params_v101_t tp{};
    tp.common.game_name = "American Truck Simulator";
    tp.common.game_id = SCS_GAME_ID_ATS;
    tp.common.game_version = SCS_TELEMETRY_ATS_GAME_VERSION_CURRENT;
    tp.common.log = hostLog;
    tp.register_for_event = regEvent;
    tp.unregister_from_event = unregEvent;
    tp.register_for_channel = regChannel;
    tp.unregister_from_channel = unregChannel;
    // Version negotiation: the game offers the newest first.
    if (telInit(SCS_TELEMETRY_VERSION_1_01, &tp) != SCS_RESULT_ok) {
        std::fprintf(stderr, "telemetry init failed\n");
        return 1;
    }
    scs_input_init_params_v100_t ip{};
    ip.common = tp.common;
    ip.common.game_version = SCS_INPUT_ATS_GAME_VERSION_CURRENT;
    ip.register_device = regDevice;
    if (inInit(SCS_INPUT_VERSION_1_00, &ip) != SCS_RESULT_ok) {
        std::fprintf(stderr, "input init failed\n");
        return 1;
    }
    std::printf("plugin loaded: %zu events, %zu channels, device '%s' (%s) with %u inputs:",
                g_events.size(), g_channels.size(), g_device.name, g_device.type == SCS_INPUT_DEVICE_TYPE_semantical ? "semantical" : "generic",
                g_device.input_count);
    for (const auto& n : g_inputNames) std::printf(" %s", n.c_str());
    std::printf("\n");
    if (g_device.input_active_callback) g_device.input_active_callback(1, g_device.callback_context);

    sim::SimParams sp;
    sendTruckConfig(sp);
    fireEvent(SCS_TELEMETRY_EVENT_started, nullptr);

    // Wait for the plugin's map service to load the cache.
    for (int i = 0; i < 600; ++i) {
        const std::string st = devState(dll);
        if (st.find("|ready|") != std::string::npos) break;
        if (st.find("|failed|") != std::string::npos) {
            std::fprintf(stderr, "plugin map failed: %s\n", st.c_str());
            return 1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    std::printf("plugin state: %s\n", devState(dll).c_str());

    std::map<std::string, int> sem;
    for (const auto& n : g_inputNames) sem[n] = 0;
    float semValues[3] = {0, 0, 0};

    const auto starts = pickStarts(*net, scenarios, 1234);
    double simClock = 0.0;
    int failures = 0;
    for (std::size_t sc = 0; sc < starts.size(); ++sc) {
        const auto& seg = net->segment(starts[sc]);
        const Vec2 a = seg.points[0].plan();
        const Vec2 b = seg.points[1].plan();
        const double yaw = std::atan2(b.y - a.y, b.x - a.x);

        sim::VehicleSim truck(sp);
        truck.reset(a, yaw, 22.0);
        // Hold the truck still in time for the plugin to localize before engaging.
        double maxDev = 0.0, sumSq = 0.0, distance = 0.0;
        int samples = 0;
        bool engaged = false;
        std::string endState;
        const double dt = 1.0 / 60.0;
        const auto wallStart = std::chrono::steady_clock::now();
        Vec2 lastPos = truck.rearAxle();
        std::size_t frame = 0;
        Localizer metric(*net);
        for (double t = 0.0; t < seconds; t += dt, ++frame) {
            simClock += dt;
            // Pace against wall time so the plugin's background planner keeps up.
            const double wallElapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - wallStart).count();
            if (t / speedup > wallElapsed) std::this_thread::sleep_for(std::chrono::duration<double>(t / speedup - wallElapsed));

            const VehicleState s = truck.state();
            scs_telemetry_frame_start_t fs{};
            fs.simulation_time = static_cast<scs_timestamp_t>(simClock * 1e6);
            fs.render_time = fs.simulation_time;
            fs.paused_simulation_time = fs.simulation_time;
            fireEvent(SCS_TELEMETRY_EVENT_frame_start, &fs);
            sendPlacement(s.worldPosition, s.headingUnit);
            sendVector(SCS_TELEMETRY_TRUCK_CHANNEL_local_linear_velocity, s.localVelocity);
            sendVector(SCS_TELEMETRY_TRUCK_CHANNEL_local_angular_velocity, s.localAngularVelocity);
            sendFloat(SCS_TELEMETRY_TRUCK_CHANNEL_speed, static_cast<float>(s.speed));
            // The game's steering mix subtracts semantical.steering (see Input.cpp).
            sendFloat(SCS_TELEMETRY_TRUCK_CHANNEL_input_steering, -semValues[0]);
            sendFloat(SCS_TELEMETRY_TRUCK_CHANNEL_input_throttle, semValues[1]);
            sendFloat(SCS_TELEMETRY_TRUCK_CHANNEL_input_brake, semValues[2]);
            sendFloat(SCS_TELEMETRY_TRUCK_CHANNEL_effective_steering, static_cast<float>(s.effectiveSteering));
            sendFloat(SCS_TELEMETRY_TRUCK_CHANNEL_effective_throttle, semValues[1]);
            sendFloat(SCS_TELEMETRY_TRUCK_CHANNEL_effective_brake, semValues[2]);
            for (scs_u32_t w = 0; w < 2; ++w) {
                sendFloat(SCS_TELEMETRY_TRUCK_CHANNEL_wheel_steering, static_cast<float>(s.steerableWheelAngle / kTwoPi), w);
            }
            sendBool(SCS_TELEMETRY_TRUCK_CHANNEL_parking_brake, false);
            sendBool(SCS_TELEMETRY_TRUCK_CHANNEL_engine_enabled, true);
            sendBool(SCS_TELEMETRY_TRUCK_CHANNEL_electric_enabled, true);
            sendS32(SCS_TELEMETRY_TRUCK_CHANNEL_engine_gear, 8);
            sendFloat(SCS_TELEMETRY_TRUCK_CHANNEL_cruise_control, static_cast<float>(truck.cruiseSet()));
            sendBool(SCS_TELEMETRY_TRUCK_CHANNEL_lblinker, truck.blinkerLeft());
            sendBool(SCS_TELEMETRY_TRUCK_CHANNEL_rblinker, truck.blinkerRight());
            sendS32(SCS_TELEMETRY_TRUCK_CHANNEL_displayed_gear, 8);
            fireEvent(SCS_TELEMETRY_EVENT_frame_end, nullptr);

            // Input phase of the next render frame.
            GameButtons frameButtons;
            scs_u32_t flags = SCS_INPUT_EVENT_CALLBACK_FLAG_first_in_frame;
            for (int guard = 0; guard < 32; ++guard) {
                scs_input_event_t ev{};
                if (g_device.input_event_callback(&ev, flags, g_device.callback_context) != SCS_RESULT_ok) break;
                applyInputEvent(ev, semValues, frameButtons);
                flags = 0;
            }

            ControlCommand cmd;
            cmd.active = true;
            cmd.steerActive = true;
            cmd.pedalsActive = true;
            cmd.steering = -semValues[0];
            cmd.throttle = semValues[1];
            cmd.brake = semValues[2];
            cmd.buttons = frameButtons;
            truck.step(cmd, dt);

            if (!engaged && t > 1.0) {
                devRequest(static_cast<int>(PilotRequest::Toggle));
                engaged = true;
            }
            const std::string st = devState(dll);
            if (engaged && t > 1.5 && st.rfind("AUTOPILOT", 0) != 0) {
                endState = st;
                break;
            }
            if (t > 3.0) {
                const auto loc = metric.update(truck.rearAxle(), truck.yaw(), {});
                if (loc.valid) {
                    maxDev = std::max(maxDev, loc.match.distance);
                    sumSq += loc.match.distance * loc.match.distance;
                    ++samples;
                }
            }
            distance += atspilot::distance(lastPos, truck.rearAxle());
            lastPos = truck.rearAxle();
        }
        if (endState.empty()) endState = devState(dll);
        const bool ok = endState.rfind("AUTOPILOT", 0) == 0;
        failures += ok ? 0 : 1;
        std::printf("scenario %zu (lane %u): drove %.0f m, max lane deviation %.2f m, rms %.2f m, final speed %.1f m/s -> %s\n",
                    sc + 1, starts[sc], distance, maxDev, samples ? std::sqrt(sumSq / samples) : 0.0, truck.speed(),
                    endState.c_str());
        // Disengage between scenarios; the next start is a relocation.
        devRequest(static_cast<int>(PilotRequest::Cancel));
    }

    // --- Navigation scenario: a job to a depot a few km away ---------------------
    int navResult = -1;
    if (navSeconds > 0.0) {
        // Find a depot 2.5-8 km away (straight line) that is reachable by route.
        std::mt19937 rng(99);
        std::uniform_int_distribution<std::size_t> pickLane(0, net->size() - 1);
        std::uint32_t startLane = 0;
        const Destination* dest = nullptr;
        double routeLen = 0.0;
        for (int attempt = 0; attempt < 50 && !dest; ++attempt) {
            const auto id = static_cast<std::uint32_t>(pickLane(rng));
            const auto& s = net->segment(id);
            if (s.kind != LaneKind::Road || s.length < 100.0f || s.next.empty()) continue;
            for (const auto& d : net->destinations()) {
                const double crow = atspilot::distance(s.points.front().plan(), d.position);
                if (crow < 2500.0 || crow > 8000.0) continue;
                const Route r = planRoute(*net, id, 0.0, d.lanes);
                if (!r.found || r.length > 15000.0) continue;
                startLane = id;
                dest = &d;
                routeLen = r.length;
                break;
            }
        }
        if (!dest) {
            std::printf("navigation scenario: no suitable depot found\n");
        } else {
            const std::string city = tokenToString(dest->city), company = tokenToString(dest->company);
            std::printf("navigation scenario: lane %u -> %s/%s, planned %.1f km\n", startLane, city.c_str(),
                        company.c_str(), routeLen / 1000.0);
            sendJobConfig(city, company);
            const auto& seg = net->segment(startLane);
            const Vec2 a = seg.points[0].plan(), b = seg.points[1].plan();
            sim::VehicleSim truck(sp);
            truck.reset(a, std::atan2(b.y - a.y, b.x - a.x), 18.0);
            float navSem[3] = {0, 0, 0};
            const double dt = 1.0 / 60.0;
            const auto wallStart = std::chrono::steady_clock::now();
            bool engaged = false, sawNav = false;
            double gpsDistance = routeLen;
            int cruiseOnFrames = 0, blinkerFrames = 0, stopsReleased = 0, tapLeft = 0;
            double waitSince = -1.0;
            bool tapped = false;
            double driven = 0.0, minRemaining = 1e12;
            Vec2 last = truck.rearAxle();
            std::string endState, lastNav;
            for (double t = 0.0; t < navSeconds; t += dt) {
                simClock += dt;
                const double wallElapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - wallStart).count();
                if (t / speedup > wallElapsed) std::this_thread::sleep_for(std::chrono::duration<double>(t / speedup - wallElapsed));
                const VehicleState s = truck.state();
                scs_telemetry_frame_start_t fs{};
                fs.simulation_time = static_cast<scs_timestamp_t>(simClock * 1e6);
                fireEvent(SCS_TELEMETRY_EVENT_frame_start, &fs);
                sendPlacement(s.worldPosition, s.headingUnit);
                sendFloat(SCS_TELEMETRY_TRUCK_CHANNEL_speed, static_cast<float>(s.speed));
                sendFloat(SCS_TELEMETRY_TRUCK_CHANNEL_input_steering, -navSem[0]);
                // A driver tapping the throttle to release a stop line.
                const float driverThrottle = tapLeft > 0 ? 0.6f : 0.0f;
                if (tapLeft > 0) --tapLeft;
                sendFloat(SCS_TELEMETRY_TRUCK_CHANNEL_input_throttle, navSem[1] + driverThrottle);
                sendFloat(SCS_TELEMETRY_TRUCK_CHANNEL_input_brake, navSem[2]);
                sendFloat(SCS_TELEMETRY_TRUCK_CHANNEL_effective_steering, static_cast<float>(s.effectiveSteering));
                for (scs_u32_t w = 0; w < 2; ++w) {
                    sendFloat(SCS_TELEMETRY_TRUCK_CHANNEL_wheel_steering, static_cast<float>(s.steerableWheelAngle / kTwoPi), w);
                }
                sendBool(SCS_TELEMETRY_TRUCK_CHANNEL_parking_brake, false);
                sendBool(SCS_TELEMETRY_TRUCK_CHANNEL_engine_enabled, true);
                sendS32(SCS_TELEMETRY_TRUCK_CHANNEL_engine_gear, 8);
                sendFloat(SCS_TELEMETRY_TRUCK_CHANNEL_cruise_control, static_cast<float>(truck.cruiseSet()));
                sendBool(SCS_TELEMETRY_TRUCK_CHANNEL_lblinker, truck.blinkerLeft());
                sendBool(SCS_TELEMETRY_TRUCK_CHANNEL_rblinker, truck.blinkerRight());
                // The game's GPS: shortest-route distance to the depot, refreshed twice a second.
                if (std::fmod(t, 0.5) < dt) {
                    const auto here = net->query(truck.rearAxle(), 15.0);
                    for (const auto& m : here) {
                        const Route g = planRoute(*net, m.segment, m.s, dest->lanes);
                        if (g.found) {
                            gpsDistance = g.length;
                            break;
                        }
                    }
                }
                sendFloat(SCS_TELEMETRY_TRUCK_CHANNEL_navigation_distance, static_cast<float>(gpsDistance));
                fireEvent(SCS_TELEMETRY_EVENT_frame_end, nullptr);
                GameButtons frameButtons;
                scs_u32_t flags = SCS_INPUT_EVENT_CALLBACK_FLAG_first_in_frame;
                for (int guard = 0; guard < 32; ++guard) {
                    scs_input_event_t ev{};
                    if (g_device.input_event_callback(&ev, flags, g_device.callback_context) != SCS_RESULT_ok) break;
                    applyInputEvent(ev, navSem, frameButtons);
                    flags = 0;
                }
                ControlCommand cmd;
                cmd.active = cmd.steerActive = cmd.pedalsActive = true;
                cmd.steering = -navSem[0];
                cmd.throttle = navSem[1];
                cmd.brake = navSem[2];
                cmd.buttons = frameButtons;
                truck.step(cmd, dt);
                driven += atspilot::distance(last, truck.rearAxle());
                cruiseOnFrames += truck.cruiseSet() > 0.0 ? 1 : 0;
                blinkerFrames += truck.blinkerLeft() || truck.blinkerRight() ? 1 : 0;
                last = truck.rearAxle();

                if (!engaged && t > 2.0) {
                    devRequest(static_cast<int>(PilotRequest::Toggle));
                    engaged = true;
                }
                const std::string nav = devNavigation(dll);
                if (nav.rfind("nav|", 0) == 0) {
                    sawNav = true;
                    minRemaining = std::min(minRemaining, std::atof(nav.c_str() + 4));
                }
                if (nav != lastNav && static_cast<int>(t) % 20 == 0) lastNav = nav;
                const std::string st = devState(dll);
                if (st.find("|Waiting at") != std::string::npos) {
                    if (waitSince < 0.0) waitSince = t;
                    if (!tapped && t - waitSince > 1.0) {
                        tapLeft = 18;
                        tapped = true;
                        ++stopsReleased;
                    }
                } else {
                    waitSince = -1.0;
                    tapped = false;
                }
                if (engaged && t > 2.5 && st.rfind("AUTOPILOT", 0) != 0) {
                    endState = st;
                    break;
                }
            }
            if (endState.empty()) endState = devState(dll);
            std::printf("navigation scenario: drove %.0f m, navigation %s, closest remaining %.0f m, last nav '%s' -> %s\n",
                        driven, sawNav ? "active" : "never active", minRemaining, devNavigation(dll).c_str(),
                        endState.c_str());
            std::printf("navigation scenario: game cruise control on for %.0f s, blinkers on for %.0f s, %d stop lines "
                        "released with a throttle tap, quick-park presses %d\n",
                        cruiseOnFrames * dt, blinkerFrames * dt, stopsReleased, truck.quickParkPresses());
            // Success: navigation engaged and the truck either arrived (stopped at the end
            // of the route) or was still driving the route when time ran out.
            navResult = sawNav && endState.find("Destination Reached") != std::string::npos ? 0 : 1;
            devRequest(static_cast<int>(PilotRequest::Cancel));
        }
    }

    if (g_device.input_active_callback) g_device.input_active_callback(0, g_device.callback_context);
    inShutdown();
    telShutdown();
    FreeLibrary(dll);
    std::printf("%d/%zu scenarios kept autopilot engaged; log: %s\n", static_cast<int>(starts.size()) - failures,
                starts.size(), (dataDir / "logs" / "atspilot.log").string().c_str());
    if (navResult >= 0) std::printf("navigation scenario %s\n", navResult == 0 ? "passed" : "FAILED");
    return failures == 0 && navResult <= 0 ? 0 : 1;
}

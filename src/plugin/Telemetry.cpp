// Telemetry API integration: channel/event registration against SCS SDK 1.15
// headers (scssdk_telemetry*.h). All callbacks run on the game's main thread.

#include <cstring>
#include <string>

#include "Runtime.h"
#include "amtrucks/scssdk_ats.h"
#include "amtrucks/scssdk_telemetry_ats.h"
#include "common/scssdk_telemetry_common_configs.h"
#include "common/scssdk_telemetry_common_gameplay_events.h"
#include "common/scssdk_telemetry_truck_common_channels.h"
#include "scssdk_telemetry.h"

using namespace atspilot;
using namespace atspilot::plugin;

namespace {

scs_telemetry_register_for_channel_t g_registerChannel = nullptr;

VehicleState& state() { return Runtime::instance()->pending(); }

SCSAPI_VOID onFloat(const scs_string_t, const scs_u32_t, const scs_value_t* const value, const scs_context_t ctx) {
    if (!value || !Runtime::instance()) return;
    *static_cast<double*>(ctx) = value->value_float.value;
}

SCSAPI_VOID onBool(const scs_string_t, const scs_u32_t, const scs_value_t* const value, const scs_context_t ctx) {
    if (!Runtime::instance()) return;
    *static_cast<bool*>(ctx) = value && value->value_bool.value != 0;
}

SCSAPI_VOID onS32(const scs_string_t, const scs_u32_t, const scs_value_t* const value, const scs_context_t ctx) {
    if (!Runtime::instance()) return;
    *static_cast<int*>(ctx) = value ? value->value_s32.value : 0;
}

SCSAPI_VOID onPlacement(const scs_string_t, const scs_u32_t, const scs_value_t* const value, const scs_context_t) {
    if (!value || !Runtime::instance()) return;
    const auto& p = value->value_dplacement;
    auto& s = state();
    s.worldPosition = {p.position.x, p.position.y, p.position.z};
    s.headingUnit = p.orientation.heading;
    s.pitchUnit = p.orientation.pitch;
    s.rollUnit = p.orientation.roll;
}

SCSAPI_VOID onLinearVelocity(const scs_string_t, const scs_u32_t, const scs_value_t* const value, const scs_context_t) {
    if (!value || !Runtime::instance()) return;
    const auto& v = value->value_fvector;
    state().localVelocity = {v.x, v.y, v.z};
}

SCSAPI_VOID onAngularVelocity(const scs_string_t, const scs_u32_t, const scs_value_t* const value, const scs_context_t) {
    if (!value || !Runtime::instance()) return;
    const auto& v = value->value_fvector;
    state().localAngularVelocity = {v.x, v.y, v.z};
}

SCSAPI_VOID onWheelSteering(const scs_string_t, const scs_u32_t index, const scs_value_t* const value, const scs_context_t) {
    if (!value || !Runtime::instance()) return;
    Runtime::instance()->onWheelSteering(index, value->value_float.value);
}

void registerChannels() {
    auto& s = state();
    struct FloatChannel {
        const char* name;
        double* target;
    };
    const FloatChannel floats[] = {
        {SCS_TELEMETRY_TRUCK_CHANNEL_speed, &s.speed},
        {SCS_TELEMETRY_TRUCK_CHANNEL_input_steering, &s.inputSteering},
        {SCS_TELEMETRY_TRUCK_CHANNEL_input_throttle, &s.inputThrottle},
        {SCS_TELEMETRY_TRUCK_CHANNEL_input_brake, &s.inputBrake},
        {SCS_TELEMETRY_TRUCK_CHANNEL_effective_steering, &s.effectiveSteering},
        {SCS_TELEMETRY_TRUCK_CHANNEL_effective_throttle, &s.effectiveThrottle},
        {SCS_TELEMETRY_TRUCK_CHANNEL_effective_brake, &s.effectiveBrake},
        {SCS_TELEMETRY_TRUCK_CHANNEL_cruise_control, &s.cruiseControlSpeed},
        {SCS_TELEMETRY_TRUCK_CHANNEL_navigation_distance, &s.navigationDistance},
        {SCS_TELEMETRY_TRUCK_CHANNEL_navigation_time, &s.navigationTime},
        {SCS_TELEMETRY_TRUCK_CHANNEL_navigation_speed_limit, &s.navigationSpeedLimit},
    };
    auto& log = Runtime::instance()->log();
    for (const auto& c : floats) {
        if (g_registerChannel(c.name, SCS_U32_NIL, SCS_VALUE_TYPE_float, SCS_TELEMETRY_CHANNEL_FLAG_none, onFloat,
                              c.target) != SCS_RESULT_ok) {
            log.warn("Telemetry channel {} unavailable", c.name);
        }
    }
    const std::pair<const char*, bool*> bools[] = {
        {SCS_TELEMETRY_TRUCK_CHANNEL_parking_brake, &s.parkingBrake},
        {SCS_TELEMETRY_TRUCK_CHANNEL_engine_enabled, &s.engineEnabled},
        {SCS_TELEMETRY_TRUCK_CHANNEL_electric_enabled, &s.electricEnabled},
        {SCS_TELEMETRY_TRUCK_CHANNEL_wipers, &s.wipers},
    };
    for (const auto& [name, target] : bools) {
        if (g_registerChannel(name, SCS_U32_NIL, SCS_VALUE_TYPE_bool, SCS_TELEMETRY_CHANNEL_FLAG_none, onBool, target) !=
            SCS_RESULT_ok) {
            log.warn("Telemetry channel {} unavailable", name);
        }
    }
    g_registerChannel(SCS_TELEMETRY_TRUCK_CHANNEL_engine_gear, SCS_U32_NIL, SCS_VALUE_TYPE_s32,
                      SCS_TELEMETRY_CHANNEL_FLAG_none, onS32, &s.gear);
    g_registerChannel(SCS_TELEMETRY_TRUCK_CHANNEL_displayed_gear, SCS_U32_NIL, SCS_VALUE_TYPE_s32,
                      SCS_TELEMETRY_CHANNEL_FLAG_none, onS32, &s.displayedGear);
    if (g_registerChannel(SCS_TELEMETRY_TRUCK_CHANNEL_world_placement, SCS_U32_NIL, SCS_VALUE_TYPE_dplacement,
                          SCS_TELEMETRY_CHANNEL_FLAG_none, onPlacement, nullptr) != SCS_RESULT_ok) {
        log.error("Telemetry channel {} unavailable; autopilot cannot localize",
                  SCS_TELEMETRY_TRUCK_CHANNEL_world_placement);
    }
    g_registerChannel(SCS_TELEMETRY_TRUCK_CHANNEL_local_linear_velocity, SCS_U32_NIL, SCS_VALUE_TYPE_fvector,
                      SCS_TELEMETRY_CHANNEL_FLAG_none, onLinearVelocity, nullptr);
    g_registerChannel(SCS_TELEMETRY_TRUCK_CHANNEL_local_angular_velocity, SCS_U32_NIL, SCS_VALUE_TYPE_fvector,
                      SCS_TELEMETRY_CHANNEL_FLAG_none, onAngularVelocity, nullptr);
    for (scs_u32_t i = 0; i < 16; ++i) {
        // Indices beyond the truck's wheel count are simply never updated.
        g_registerChannel(SCS_TELEMETRY_TRUCK_CHANNEL_wheel_steering, i, SCS_VALUE_TYPE_float,
                          SCS_TELEMETRY_CHANNEL_FLAG_none, onWheelSteering, nullptr);
    }
}

const scs_named_value_t* findAttribute(const scs_named_value_t* attrs, const char* name, scs_u32_t index) {
    for (auto* a = attrs; a && a->name; ++a) {
        if (std::strcmp(a->name, name) == 0 && (index == SCS_U32_NIL || a->index == index)) return a;
    }
    return nullptr;
}

void handleTruckConfig(const scs_named_value_t* attrs) {
    Runtime& rt = *Runtime::instance();
    VehicleConfig& vc = rt.vehicleConfig();
    auto& steerable = rt.wheelSteerable();
    steerable.fill(false);

    if (const auto* id = findAttribute(attrs, SCS_TELEMETRY_CONFIG_ATTRIBUTE_id, SCS_U32_NIL)) {
        vc.truckId = id->value.value_string.value ? id->value.value_string.value : "";
    }
    if (const auto* nm = findAttribute(attrs, SCS_TELEMETRY_CONFIG_ATTRIBUTE_name, SCS_U32_NIL)) {
        vc.truckName = nm->value.value_string.value ? nm->value.value_string.value : "";
    }
    const auto* countAttr = findAttribute(attrs, SCS_TELEMETRY_CONFIG_ATTRIBUTE_wheel_count, SCS_U32_NIL);
    const scs_u32_t wheels = countAttr ? countAttr->value.value_u32.value : 0;

    // Wheelbase for the bicycle model: steer axle to the mean of the driven axles.
    double frontSum = 0.0, rearSum = 0.0;
    int front = 0, rear = 0;
    for (scs_u32_t i = 0; i < wheels && i < 16; ++i) {
        const auto* pos = findAttribute(attrs, SCS_TELEMETRY_CONFIG_ATTRIBUTE_wheel_position, i);
        const auto* st = findAttribute(attrs, SCS_TELEMETRY_CONFIG_ATTRIBUTE_wheel_steerable, i);
        const auto* pw = findAttribute(attrs, SCS_TELEMETRY_CONFIG_ATTRIBUTE_wheel_powered, i);
        if (!pos) continue;
        const double z = pos->value.value_fvector.z;
        const bool isSteer = st && st->value.value_bool.value;
        const bool isPowered = pw && pw->value.value_bool.value;
        steerable[i] = isSteer;
        if (isSteer) {
            frontSum += z;
            ++front;
        } else if (isPowered) {
            rearSum += z;
            ++rear;
        }
    }
    if (front > 0 && rear > 0) {
        vc.frontAxleZ = frontSum / front;
        vc.rearAxleZ = rearSum / rear;
        vc.wheelbase = std::max(2.0, vc.rearAxleZ - vc.frontAxleZ);
        vc.valid = true;
    }
    rt.onTruckConfiguration();
}

SCSAPI_VOID onEvent(const scs_event_t event, const void* const info, const scs_context_t) {
    Runtime* rt = Runtime::instance();
    if (!rt) return;
    try {
        switch (event) {
            case SCS_TELEMETRY_EVENT_frame_start: {
                const auto* fs = static_cast<const scs_telemetry_frame_start_t*>(info);
                rt->onFrameStart(static_cast<double>(fs->simulation_time) / 1e6,
                                 (fs->flags & SCS_TELEMETRY_FRAME_START_FLAG_timer_restart) != 0);
                break;
            }
            case SCS_TELEMETRY_EVENT_frame_end:
                rt->onFrameEnd();
                break;
            case SCS_TELEMETRY_EVENT_paused:
                rt->onPaused(true);
                break;
            case SCS_TELEMETRY_EVENT_started:
                rt->onPaused(false);
                break;
            case SCS_TELEMETRY_EVENT_configuration: {
                const auto* cfg = static_cast<const scs_telemetry_configuration_t*>(info);
                if (!cfg || !cfg->id) break;
                const std::string id = cfg->id;
                if (id == SCS_TELEMETRY_CONFIG_truck) {
                    handleTruckConfig(cfg->attributes);
                } else if (id.rfind("trailer.", 0) == 0) {
                    // trailer.N configurations with an empty attribute set mean "no trailer N".
                    const unsigned index = static_cast<unsigned>(std::stoul(id.substr(8)));
                    const bool present = cfg->attributes && cfg->attributes->name != nullptr;
                    auto& vc = rt->vehicleConfig();
                    if (present) vc.trailerCount = std::max(vc.trailerCount, static_cast<int>(index) + 1);
                    else if (static_cast<int>(index) < vc.trailerCount) vc.trailerCount = static_cast<int>(index);
                } else if (id == SCS_TELEMETRY_CONFIG_job) {
                    auto& vc = rt->vehicleConfig();
                    const auto* mass = findAttribute(cfg->attributes, SCS_TELEMETRY_CONFIG_ATTRIBUTE_cargo_mass, SCS_U32_NIL);
                    vc.cargoMassKg = mass ? mass->value.value_float.value : 0.0;
                    vc.hasJob = cfg->attributes && cfg->attributes->name != nullptr;
                }
                break;
            }
            case SCS_TELEMETRY_EVENT_gameplay: {
                const auto* g = static_cast<const scs_telemetry_gameplay_event_t*>(info);
                if (g && g->id) rt->onGameplayEvent(g->id);
                break;
            }
            default:
                break;
        }
    } catch (...) {
        // Never propagate into the game.
    }
}

}  // namespace

extern "C" SCSAPI_RESULT scs_telemetry_init(const scs_u32_t version, const scs_telemetry_init_params_t* const params) {
    if (version != SCS_TELEMETRY_VERSION_1_01 && version != SCS_TELEMETRY_VERSION_1_00) return SCS_RESULT_unsupported;
    const auto* p = static_cast<const scs_telemetry_init_params_v100_t*>(params);
    try {
        Runtime& rt = Runtime::acquire(p->common.log);
        if (std::strcmp(p->common.game_id, SCS_GAME_ID_ATS) != 0) {
            rt.log().warn("Game '{}' is not ATS; ATSPilot's map support targets ATS", p->common.game_name);
        }
        rt.log().info("Telemetry API {}.{} on {} (game telemetry version {}.{})", SCS_GET_MAJOR_VERSION(version),
                      SCS_GET_MINOR_VERSION(version), p->common.game_name,
                      SCS_GET_MAJOR_VERSION(p->common.game_version), SCS_GET_MINOR_VERSION(p->common.game_version));
        if (SCS_GET_MAJOR_VERSION(p->common.game_version) != SCS_GET_MAJOR_VERSION(SCS_TELEMETRY_ATS_GAME_VERSION_CURRENT)) {
            rt.log().warn("Unexpected ATS telemetry major version; values may be misinterpreted");
        }
        g_registerChannel = p->register_for_channel;
        const scs_event_t events[] = {SCS_TELEMETRY_EVENT_frame_start, SCS_TELEMETRY_EVENT_frame_end,
                                      SCS_TELEMETRY_EVENT_paused,      SCS_TELEMETRY_EVENT_started,
                                      SCS_TELEMETRY_EVENT_configuration, SCS_TELEMETRY_EVENT_gameplay};
        for (auto e : events) {
            if (p->register_for_event(e, onEvent, nullptr) != SCS_RESULT_ok && e <= SCS_TELEMETRY_EVENT_started) {
                rt.log().error("Could not register telemetry event {}", e);
                Runtime::release();
                return SCS_RESULT_generic_error;
            }
        }
        registerChannels();
        rt.setTelemetryActive(true);
        rt.flushGameLog();
        return SCS_RESULT_ok;
    } catch (...) {
        return SCS_RESULT_generic_error;
    }
}

extern "C" SCSAPI_VOID scs_telemetry_shutdown(void) {
    try {
        if (Runtime* rt = Runtime::instance()) rt->setTelemetryActive(false);
        g_registerChannel = nullptr;
        Runtime::release();
    } catch (...) {
    }
}

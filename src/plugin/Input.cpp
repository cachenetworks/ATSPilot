// Input API integration: a "semantical" SDK input device (scssdk_input_device.h).
// Its inputs feed ATS mixes of the same name directly, which controls.sii
// combines with the player's own devices:
//
//   mix steering     `... - semantical.steering?0`
//   mix aforward     `... + semantical.aforward?0`
//   mix abackward    `... + semantical.abackward?0`
//   mix cruiectrl    `keyboard.c?0 || ... | semantical.cruiectrl?0`     (and cruiectrlinc/dec/res)
//   mix lblinker     `keyboard.lbracket?0 || ... | semantical.lblinker?0` (and rblinker)
//   mix quickpark    `keyboard.q?0 | semantical.quickpark?0`
//   mix lblinkerh    `semantical.lblinkerh?0`   (and rblinkerh: signal while held)
//
// so ATSPilot adds to, and never replaces, the player's input. Analogue values of
// zero are neutral; buttons are pressed for exactly one input frame. Verified
// against a 1.61 profile's controls.sii; see docs/sdk.md.

#include <cmath>
#include <cstring>

#include "Runtime.h"
#include "amtrucks/scssdk_input_ats.h"
#include "scssdk_input.h"

using namespace atspilot;
using namespace atspilot::plugin;

namespace {

enum InputIndex : scs_u32_t {
    kSteering = 0,
    kThrottle,
    kBrake,
    kCruiseToggle,
    kCruiseInc,
    kCruiseDec,
    kCruiseResume,
    kLeftBlinker,
    kRightBlinker,
    kQuickPark,
    kLeftHold,
    kRightHold,
    kInputCount
};

// Display names: the game rejects the whole device if any name has characters
// other than letters, digits and spaces (seen: "(" in 1.61).
const scs_input_device_input_t g_inputs[kInputCount] = {
    {"steering", "ATSPilot Steering", SCS_VALUE_TYPE_float},
    {"aforward", "ATSPilot Throttle", SCS_VALUE_TYPE_float},
    {"abackward", "ATSPilot Brake", SCS_VALUE_TYPE_float},
    {"cruiectrl", "ATSPilot Cruise Control", SCS_VALUE_TYPE_bool},
    {"cruiectrlinc", "ATSPilot Cruise Faster", SCS_VALUE_TYPE_bool},
    {"cruiectrldec", "ATSPilot Cruise Slower", SCS_VALUE_TYPE_bool},
    {"cruiectrlres", "ATSPilot Cruise Resume", SCS_VALUE_TYPE_bool},
    {"lblinker", "ATSPilot Left Blinker", SCS_VALUE_TYPE_bool},
    {"rblinker", "ATSPilot Right Blinker", SCS_VALUE_TYPE_bool},
    {"quickpark", "ATSPilot Quick Park", SCS_VALUE_TYPE_bool},
    {"lblinkerh", "ATSPilot Left Signal Hold", SCS_VALUE_TYPE_bool},
    {"rblinkerh", "ATSPilot Right Signal Hold", SCS_VALUE_TYPE_bool},
};

constexpr bool isButton(scs_u32_t i) { return i >= kCruiseToggle; }

float g_sent[kInputCount];
float g_target[kInputCount];
bool g_inputAcquired = false;

void forgetSent() {
    for (auto& v : g_sent) v = NAN;
}

SCSAPI_VOID onActive(const scs_u8_t active, const scs_context_t) {
    Runtime* rt = Runtime::instance();
    forgetSent();
    if (rt) rt->setInputDeviceActive(active != 0);
}

SCSAPI_RESULT onInputEvent(scs_input_event_t* const event, const scs_u32_t flags, const scs_context_t) {
    try {
        if (flags & SCS_INPUT_EVENT_CALLBACK_FLAG_first_after_activation) forgetSent();
        if (flags & SCS_INPUT_EVENT_CALLBACK_FLAG_first_in_frame) {
            OutputValues out;
            GameButtons buttons;
            if (Runtime* rt = Runtime::instance()) {
                out = rt->currentOutput();
                buttons = rt->takeButtons();
            }
            g_target[kSteering] = out.steering;
            g_target[kThrottle] = out.throttle;
            g_target[kBrake] = out.brake;
            g_target[kCruiseToggle] = buttons.cruiseToggle ? 1.0f : 0.0f;
            g_target[kCruiseInc] = buttons.cruiseInc ? 1.0f : 0.0f;
            g_target[kCruiseDec] = buttons.cruiseDec ? 1.0f : 0.0f;
            g_target[kCruiseResume] = buttons.cruiseResume ? 1.0f : 0.0f;
            g_target[kLeftBlinker] = buttons.leftBlinker ? 1.0f : 0.0f;
            g_target[kRightBlinker] = buttons.rightBlinker ? 1.0f : 0.0f;
            g_target[kQuickPark] = buttons.quickPark ? 1.0f : 0.0f;
            g_target[kLeftHold] = out.indicator > 0 ? 1.0f : 0.0f;
            g_target[kRightHold] = out.indicator < 0 ? 1.0f : 0.0f;
        }
        // One event per changed input; the game calls again until not_found.
        for (scs_u32_t i = 0; i < kInputCount; ++i) {
            if (!(g_sent[i] == g_target[i])) {
                event->input_index = i;
                if (isButton(i)) event->value_bool.value = g_target[i] > 0.5f ? 1 : 0;
                else event->value_float.value = g_target[i];
                g_sent[i] = g_target[i];
                return SCS_RESULT_ok;
            }
        }
        return SCS_RESULT_not_found;
    } catch (...) {
        return SCS_RESULT_not_found;
    }
}

}  // namespace

extern "C" SCSAPI_RESULT scs_input_init(const scs_u32_t version, const scs_input_init_params_t* const params) {
    if (version != SCS_INPUT_VERSION_1_00) return SCS_RESULT_unsupported;
    const auto* p = static_cast<const scs_input_init_params_v100_t*>(params);
    try {
        forgetSent();
        for (auto& v : g_target) v = 0.0f;
        Runtime& rt = Runtime::acquire(p->common.log);
        g_inputAcquired = true;
        scs_input_device_t device;
        std::memset(&device, 0, sizeof(device));
        device.name = "atspilot";
        device.display_name = "ATSPilot";
        device.type = SCS_INPUT_DEVICE_TYPE_semantical;
        device.input_count = kInputCount;
        device.inputs = g_inputs;
        device.callback_context = nullptr;
        device.input_active_callback = onActive;
        device.input_event_callback = onInputEvent;
        if (p->register_device(&device) != SCS_RESULT_ok) {
            rt.log().error("Could not register the ATSPilot input device; ATSPilot cannot drive");
            rt.flushGameLog();
            g_inputAcquired = false;
            Runtime::release();
            return SCS_RESULT_generic_error;
        }
        rt.log().info("Input device registered (semantical: steering, pedals, cruise control, blinkers, quick park)");
        return SCS_RESULT_ok;
    } catch (...) {
        return SCS_RESULT_generic_error;
    }
}

extern "C" SCSAPI_VOID scs_input_shutdown(void) {
    try {
        for (auto& v : g_target) v = 0.0f;
        if (g_inputAcquired) {
            g_inputAcquired = false;
            Runtime::release();
        }
    } catch (...) {
    }
}

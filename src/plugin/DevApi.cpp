// Development-only exports used by tools/plugin_host to drive the real plugin
// DLL without the game. ATS only calls the scs_* exports and ignores these.

#include <cstring>
#include <string>

#include "Runtime.h"

using namespace atspilot;
using namespace atspilot::plugin;

extern "C" int atspilot_dev_request(int request) {
    Runtime* rt = Runtime::instance();
    if (!rt || request < 0 || request > static_cast<int>(PilotRequest::EmergencyDisable)) return -1;
    rt->queueRequest(static_cast<PilotRequest>(request));
    return 0;
}

// Writes "<nav|nonav>|<route m>|<next maneuver>|<maneuver m>" into buf.
extern "C" int atspilot_dev_navigation(char* buf, int size) {
    Runtime* rt = Runtime::instance();
    if (!rt || !buf || size <= 0) return -1;
    const PilotStatus st = rt->pilotStatus();
    const std::string s = std::string(st.navigationActive ? "nav" : "nonav") + "|" +
                          std::to_string(static_cast<long long>(st.routeDistance)) + "|" + st.nextManeuver + "|" +
                          std::to_string(static_cast<long long>(st.nextManeuverDistance));
    std::strncpy(buf, s.c_str(), static_cast<std::size_t>(size) - 1);
    buf[size - 1] = '\0';
    return 0;
}

// Writes "<mode>|<map state>|<status message>" into buf.
extern "C" int atspilot_dev_state(char* buf, int size) {
    Runtime* rt = Runtime::instance();
    if (!rt || !buf || size <= 0) return -1;
    static const char* mapStates[] = {"disabled", "loading", "ready", "failed"};
    const std::string s = std::string(toString(rt->mode())) + "|" + mapStates[static_cast<int>(rt->mapState())] + "|" +
                          rt->statusMessage();
    std::strncpy(buf, s.c_str(), static_cast<std::size_t>(size) - 1);
    buf[size - 1] = '\0';
    return 0;
}

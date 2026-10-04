#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

#include "config/Config.h"
#include "pilot/PilotTypes.h"

namespace atspilot::plugin {

// In-game status panel. The SCS SDK has no UI extension, so this is a
// transparent, click-through, topmost layered window kept over the game's
// window and shown only while the game has focus. It draws from a PilotStatus
// snapshot and never touches controllers. It is visible in the game's
// borderless/windowed display modes; exclusive fullscreen may hide it.
class Hud {
public:
    Hud(const HudConfig& cfg, SpeedUnits units);
    ~Hud();
    Hud(const Hud&) = delete;
    Hud& operator=(const Hud&) = delete;

    void update(const PilotStatus& status);

private:
    void run();

    HudConfig cfg_;
    SpeedUnits units_;
    std::thread thread_;
    std::atomic<bool> stop_{false};
    std::atomic<void*> window_{nullptr};
    std::mutex mutex_;
    PilotStatus status_;
};

}  // namespace atspilot::plugin

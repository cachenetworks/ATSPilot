#include "control/GameCruise.h"

#include <algorithm>
#include <cmath>

namespace atspilot {

void GameCruiseManager::reset() {
    wasOn_ = false;
    lastPress_ = -100.0;
    lastOwnBrake_ = -100.0;
    toggleSentAt_ = -100.0;
    expectedSet_ = 0.0;
    lastSet_ = 0.0;
    initialized_ = false;
}

GameCruiseManager::Output GameCruiseManager::update(double time, double speed, double cruiseSet, double target,
                                                    bool wantBrake, bool ownBraking) {
    Output out;
    const bool on = cruiseSet > 0.1;
    if (ownBraking) lastOwnBrake_ = time;
    const bool canPress = time - lastPress_ >= p_.pressInterval;
    auto press = [&](bool GameButtons::*button) {
        out.buttons.*button = true;
        lastPress_ = time;
    };

    if (!initialized_) {
        // Engaging with the cruise control already on: its set speed is the player's choice.
        initialized_ = true;
        wasOn_ = on;
        lastSet_ = expectedSet_ = cruiseSet;
        if (on) {
            out.playerSetSpeed = true;
            out.playerSpeed = cruiseSet;
        }
    }

    // Switched off since the last frame.
    if (wasOn_ && !on) {
        const bool weCaused = time - lastOwnBrake_ < 1.0 || time - toggleSentAt_ < p_.confirmTime;
        if (!weCaused) out.cancelledExternally = true;
    }

    if (!on) {
        // A press of the toggle that did not take: the speed was below what the game accepts.
        if (toggleSentAt_ > -50.0 && time - toggleSentAt_ > p_.confirmTime && time - toggleSentAt_ < p_.confirmTime + 0.1) {
            p_.minSpeed = std::min(25.0, std::max(p_.minSpeed, speed + 1.5));
        }
        out.ownPedals = true;
        // Hand over as soon as the game accepts it; the game then accelerates to the
        // set speed, which is raised to the target with the + control.
        const bool readyForCruise = speed >= p_.minSpeed && target >= p_.minSpeed && !wantBrake && !ownBraking &&
                                    time - lastOwnBrake_ > 0.5;
        if (readyForCruise && canPress && time - toggleSentAt_ > p_.confirmTime + 0.2) {
            press(&GameButtons::cruiseToggle);
            toggleSentAt_ = time;
            expectedSet_ = speed;
        }
    } else {
        out.ownPedals = wantBrake;  // only to brake harder than the cruise control can
        // Set speed moved without a recent press of ours: the player used the game's +/-.
        const bool ourChange = time - lastPress_ < p_.confirmTime;
        if (wasOn_ && !ourChange && std::abs(cruiseSet - lastSet_) > 0.2 && std::abs(cruiseSet - expectedSet_) > 0.2) {
            out.playerSetSpeed = true;
            out.playerSpeed = cruiseSet;
        }
        if (!wasOn_ || ourChange) expectedSet_ = cruiseSet;
        if (!wantBrake && canPress) {
            if (cruiseSet < target - p_.tolerance) {
                press(&GameButtons::cruiseInc);
            } else if (cruiseSet > target + p_.tolerance) {
                press(&GameButtons::cruiseDec);
            }
        }
    }

    wasOn_ = on;
    lastSet_ = cruiseSet;
    return out;
}

}  // namespace atspilot

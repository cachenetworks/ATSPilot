#pragma once

#include <string>

namespace atspilot {

// Momentary presses of the game's own controls, sent through the input device
// for one frame each. Names match the controls.sii mixes they drive.
struct GameButtons {
    bool cruiseToggle = false;  // cruiectrl
    bool cruiseInc = false;     // cruiectrlinc
    bool cruiseDec = false;     // cruiectrldec
    bool cruiseResume = false;  // cruiectrlres
    bool leftBlinker = false;   // lblinker
    bool rightBlinker = false;  // rblinker
    bool quickPark = false;     // quickpark

    bool any() const {
        return cruiseToggle || cruiseInc || cruiseDec || cruiseResume || leftBlinker || rightBlinker || quickPark;
    }
    void merge(const GameButtons& o) {
        cruiseToggle |= o.cruiseToggle;
        cruiseInc |= o.cruiseInc;
        cruiseDec |= o.cruiseDec;
        cruiseResume |= o.cruiseResume;
        leftBlinker |= o.leftBlinker;
        rightBlinker |= o.rightBlinker;
        quickPark |= o.quickPark;
    }
};

struct GameCruiseParams {
    double minSpeed = 8.5;          // m/s; below this the game's cruise control will not hold (learned upwards)
    double pressInterval = 0.25;    // s between presses, so the game sees separate key presses
    double confirmTime = 0.8;       // s to wait for the game to confirm a change
    double tolerance = 0.45;        // m/s set-speed dead band
    double brakeMargin = 1.5;       // m/s over target before ATSPilot brakes itself
};

// Drives the game's cruise control (and with it the game's adaptive cruise /
// emergency braking, which are what make speed control traffic-aware).
//
//   - Below the cruise minimum, or when it must brake harder than the cruise
//     control can, ATSPilot uses its own pedals; braking cancels the game's
//     cruise control just as it would for a driver.
//   - Otherwise it switches the cruise control on and nudges its set speed with
//     the game's +/- controls towards ATSPilot's target, confirmed through the
//     truck.cruise_control telemetry channel.
//   - Set-speed changes it did not make are the player's: they become the new
//     maximum speed.
//   - If the cruise control switches off without ATSPilot causing it (driver,
//     emergency brake assist), that is reported so the autopilot can hand over.
class GameCruiseManager {
public:
    explicit GameCruiseManager(GameCruiseParams p = {}) : p_(p) {}

    struct Output {
        bool ownPedals = true;            // ATSPilot's pedal controller drives this frame
        GameButtons buttons;
        bool cancelledExternally = false;
        bool playerSetSpeed = false;      // the player changed the set speed this frame
        double playerSpeed = 0.0;         // m/s, valid when playerSetSpeed
    };

    void reset();
    // `cruiseSet`: game cruise set speed from telemetry (m/s, 0 = off).
    // `wantBrake`: the speed plan needs braking beyond what cruise control does.
    Output update(double time, double speed, double cruiseSet, double target, bool wantBrake, bool ownBraking);

    bool active() const { return wasOn_; }
    double learnedMinSpeed() const { return p_.minSpeed; }

private:
    GameCruiseParams p_;
    bool wasOn_ = false;
    double lastPress_ = -100.0;
    double lastOwnBrake_ = -100.0;
    double toggleSentAt_ = -100.0;
    double expectedSet_ = 0.0;   // set speed we expect after our presses
    double lastSet_ = 0.0;
    bool initialized_ = false;
};

}  // namespace atspilot

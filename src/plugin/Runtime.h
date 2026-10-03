#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "config/Config.h"
#include "map/MapService.h"
#include "pilot/Autopilot.h"
#include "util/KeyBinding.h"
#include "util/Logger.h"
#include "util/TelemetryRecorder.h"

#include "scssdk.h"

namespace atspilot::plugin {

struct Paths {
    std::filesystem::path dataDir;   // Documents/American Truck Simulator/atspilot
    std::filesystem::path configFile;
    std::filesystem::path logFile;
    std::filesystem::path cacheDir;
    std::filesystem::path recordingsDir;
    std::filesystem::path statusFile;
    std::filesystem::path gameDir;
};

// The values sent through the semantical input device each frame.
struct OutputValues {
    float steering = 0.0f;
    float throttle = 0.0f;
    float brake = 0.0f;
};

// Everything ATSPilot keeps alive inside the game process. Created by the first
// SDK API that initialises and destroyed when the last one shuts down.
//
// Threads: the game's main thread runs all SDK callbacks and the control loop;
// MapService, Logger, TelemetryRecorder and the status/audio worker run in the
// background and never call back into the SDK.
class Runtime {
public:
    static Runtime* instance();
    static Runtime& acquire(scs_log_t gameLog);  // +1 reference
    static void release();                       // -1 reference; destroys at zero

    // --- Telemetry side (main thread) ---
    VehicleState& pending() { return pending_; }
    VehicleConfig& vehicleConfig() { return vehicleConfig_; }
    std::array<bool, 16>& wheelSteerable() { return wheelSteerable_; }
    void onFrameStart(double simulationTime, bool timerRestart);
    void onFrameEnd();
    void onPaused(bool paused);
    void onTruckConfiguration();
    void onGameplayEvent(const std::string& id);
    void setTelemetryActive(bool active);
    void onWheelSteering(unsigned index, float rotations);

    // --- Input side (main thread) ---
    void setInputDeviceActive(bool active);
    OutputValues currentOutput();

    // Requests from outside the frame loop (development harness); applied on the next frame.
    void queueRequest(PilotRequest r);
    PilotMode mode() const { return pilot_ ? pilot_->mode() : PilotMode::Off; }
    std::string statusMessage() const { return pilot_ ? pilot_->status().statusMessage : std::string(); }
    MapState mapState() const { return map_ ? map_->state() : MapState::Disabled; }

    Logger& log() { return log_; }
    const Config& config() const { return config_; }
    void flushGameLog();

private:
    explicit Runtime(scs_log_t gameLog);
    ~Runtime();

    void initialise();
    void loadConfiguration();
    void pollHotkeys();
    void processRequest(PilotRequest r);
    void handleEvent(PilotEvent e, const std::string& msg);
    void publishStatus();
    void workerLoop();
    void fail(const std::string& where, const std::string& what);
    double wallNow() const;

    static Runtime* instance_;
    static int refs_;

    scs_log_t gameLog_ = nullptr;
    Paths paths_;
    Config config_;
    Logger log_;
    std::unique_ptr<MapService> map_;
    std::unique_ptr<Autopilot> pilot_;
    TelemetryRecorder recorder_;

    VehicleState pending_;
    VehicleState current_;
    VehicleConfig vehicleConfig_;
    std::array<float, 16> wheelSteering_{};
    std::array<bool, 16> wheelSteerable_{};
    double lastTelemetryWall_ = -1e9;
    bool telemetryActive_ = false;
    bool inputActive_ = false;
    bool paused_ = true;
    Vec3 lastPosition_;
    bool havePosition_ = false;

    ControlCommand command_;
    double commandWall_ = -1e9;

    struct Hotkey {
        KeyBinding binding;
        PilotRequest request;
        bool wasDown = false;
    };
    std::vector<Hotkey> hotkeys_;
    std::mutex requestMutex_;
    std::vector<PilotRequest> queuedRequests_;

    // Status / audio worker.
    std::thread worker_;
    std::mutex workerMutex_;
    std::condition_variable workerCv_;
    bool workerStop_ = false;
    PilotStatus statusSnapshot_;
    bool statusDirty_ = false;
    std::deque<std::string> sounds_;
    double lastStatusPublish_ = 0.0;

    std::chrono::steady_clock::time_point epoch_ = std::chrono::steady_clock::now();
};

}  // namespace atspilot::plugin

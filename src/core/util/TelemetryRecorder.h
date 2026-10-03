#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

#include "pilot/PilotTypes.h"
#include "pilot/VehicleState.h"

namespace atspilot {

// Development CSV recorder. Rows are formatted on the caller's thread (cheap)
// and written by a background thread; recording stops at `maxBytes`.
class TelemetryRecorder {
public:
    static constexpr const char* kHeader =
        "time,x,y,z,speed,target_speed,heading,target_heading,steer,throttle,brake,"
        "input_steer,effective_steer,cross_track,heading_error,lookahead,curve_radius,max_wheel_angle,mode";

    ~TelemetryRecorder();

    bool start(const std::filesystem::path& file, std::uint64_t maxBytes);
    void stop();
    bool active() const { return active_.load(); }

    void record(const VehicleState& s, const ControlCommand& c, const ControllerDebug& d, PilotMode mode);

private:
    void run();

    std::atomic<bool> active_{false};
    std::uint64_t maxBytes_ = 0;
    std::uint64_t written_ = 0;
    std::ofstream file_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<std::string> queue_;
    bool running_ = false;
    std::thread thread_;
};

}  // namespace atspilot

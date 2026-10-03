#include "util/TelemetryRecorder.h"

#include <cstdio>

#include "math/Coordinates.h"

namespace atspilot {

TelemetryRecorder::~TelemetryRecorder() { stop(); }

bool TelemetryRecorder::start(const std::filesystem::path& file, std::uint64_t maxBytes) {
    stop();
    std::error_code ec;
    std::filesystem::create_directories(file.parent_path(), ec);
    file_.open(file, std::ios::out | std::ios::trunc);
    if (!file_) return false;
    file_ << kHeader << '\n';
    maxBytes_ = maxBytes;
    written_ = 0;
    {
        std::lock_guard lock(mutex_);
        running_ = true;
    }
    active_ = true;
    thread_ = std::thread([this] { run(); });
    return true;
}

void TelemetryRecorder::stop() {
    {
        std::lock_guard lock(mutex_);
        if (!running_) return;
        running_ = false;
    }
    cv_.notify_all();
    if (thread_.joinable()) thread_.join();
    active_ = false;
    file_.close();
}

void TelemetryRecorder::record(const VehicleState& s, const ControlCommand& c, const ControllerDebug& d,
                               PilotMode mode) {
    if (!active_) return;
    const double yaw = coords::sdkHeadingToYaw(s.headingUnit);
    char line[512];
    const int n = std::snprintf(
        line, sizeof(line),
        "%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.5f,%.5f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.5f,%.2f,%.1f,%.4f,%s", s.time,
        s.worldPosition.x, s.worldPosition.y, s.worldPosition.z, s.speed, d.targetSpeed, yaw, yaw + d.headingError,
        c.steering, c.throttle, c.brake, s.inputSteering, s.effectiveSteering, d.crossTrackError, d.headingError,
        d.lookAheadDistance, d.curveRadius, d.maxWheelAngle, toString(mode));
    if (n <= 0) return;
    {
        std::lock_guard lock(mutex_);
        if (queue_.size() < 10000) queue_.emplace_back(line, static_cast<std::size_t>(n));
    }
    cv_.notify_one();
}

void TelemetryRecorder::run() {
    std::unique_lock lock(mutex_);
    while (true) {
        cv_.wait(lock, [this] { return !queue_.empty() || !running_; });
        std::deque<std::string> batch;
        batch.swap(queue_);
        const bool stopping = !running_;
        lock.unlock();
        for (const auto& l : batch) {
            if (written_ >= maxBytes_) {
                active_ = false;
                break;
            }
            file_ << l << '\n';
            written_ += l.size() + 1;
        }
        file_.flush();
        lock.lock();
        if (stopping && queue_.empty()) break;
    }
}

}  // namespace atspilot

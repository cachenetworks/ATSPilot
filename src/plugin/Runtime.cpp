#include "Runtime.h"

#include <windows.h>
#include <mmsystem.h>
#include <shlobj.h>

#include <cmath>
#include <fstream>
#include <sstream>

#include "math/MathUtil.h"

namespace atspilot::plugin {

Runtime* Runtime::instance_ = nullptr;
int Runtime::refs_ = 0;

namespace {

std::filesystem::path documentsDir() {
    PWSTR raw = nullptr;
    std::filesystem::path out;
    if (SUCCEEDED(SHGetKnownFolderPath(FOLDERID_Documents, 0, nullptr, &raw))) out = raw;
    CoTaskMemFree(raw);
    return out;
}

std::filesystem::path thisModuleDir() {
    HMODULE module = nullptr;
    GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                       reinterpret_cast<LPCWSTR>(&documentsDir), &module);
    wchar_t buf[MAX_PATH * 2] = {};
    GetModuleFileNameW(module, buf, static_cast<DWORD>(std::size(buf)));
    return std::filesystem::path(buf).parent_path();
}

bool gameHasFocus() {
    const HWND fg = GetForegroundWindow();
    if (!fg) return false;
    DWORD pid = 0;
    GetWindowThreadProcessId(fg, &pid);
    return pid == GetCurrentProcessId();
}

bool keyDown(int vk) { return (GetAsyncKeyState(vk) & 0x8000) != 0; }

std::string jsonEscape(const std::string& s) {
    std::string o;
    for (char c : s) {
        if (c == '"' || c == '\\') o += '\\';
        if (static_cast<unsigned char>(c) < 0x20) continue;
        o += c;
    }
    return o;
}

}  // namespace

Runtime* Runtime::instance() { return instance_; }

Runtime& Runtime::acquire(scs_log_t gameLog) {
    if (!instance_) instance_ = new Runtime(gameLog);
    if (gameLog && !instance_->gameLog_) instance_->gameLog_ = gameLog;
    ++refs_;
    return *instance_;
}

void Runtime::release() {
    if (refs_ > 0 && --refs_ == 0) {
        delete instance_;
        instance_ = nullptr;
    }
}

Runtime::Runtime(scs_log_t gameLog) : gameLog_(gameLog) { initialise(); }

Runtime::~Runtime() {
    try {
        if (pilot_) pilot_->disengage("Plugin shutdown");
        log_.info("ATSPilot shutting down");
        {
            std::lock_guard lock(workerMutex_);
            workerStop_ = true;
        }
        workerCv_.notify_all();
        if (worker_.joinable()) worker_.join();
        hud_.reset();
        memory_.reset();
        if (map_) map_->stop();
        recorder_.stop();
        log_.stop();
    } catch (...) {
        // Never let shutdown throw into the game.
    }
}

double Runtime::wallNow() const {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - epoch_).count();
}

void Runtime::loadConfiguration() {
    std::error_code ec;
    if (!std::filesystem::exists(paths_.configFile, ec)) {
        std::ofstream(paths_.configFile) << defaultConfigText();
    }
    std::ifstream in(paths_.configFile);
    std::stringstream ss;
    ss << in.rdbuf();
    ConfigLoadResult r = loadConfig(ss.str());
    config_ = r.config;
    log_.setLevel(logLevelFromString(config_.debug.logLevel));
    for (const auto& w : r.warnings) log_.warn("Config: {}", w);

    hotkeys_.clear();
    // ATSPilot has a single key; speed is set with the game's own cruise control keys.
    if (const auto k = parseKeyBinding(config_.controls.toggle)) {
        hotkeys_.push_back({*k, PilotRequest::Toggle, false});
        log_.info("ATSPilot on/off key: {}", toString(*k));
    } else {
        log_.warn("Config: cannot parse key binding '{}'; using F9", config_.controls.toggle);
        hotkeys_.push_back({*parseKeyBinding("F9"), PilotRequest::Toggle, false});
    }
}

void Runtime::initialise() {
    // ATSPILOT_DATA_DIR redirects all files; used by the plugin host test harness.
    wchar_t overrideDir[MAX_PATH] = {};
    if (GetEnvironmentVariableW(L"ATSPILOT_DATA_DIR", overrideDir, MAX_PATH) > 0) {
        paths_.dataDir = overrideDir;
    } else {
        paths_.dataDir = documentsDir() / "American Truck Simulator" / "atspilot";
    }
    paths_.configFile = paths_.dataDir / "atspilot.toml";
    paths_.logFile = paths_.dataDir / "logs" / "atspilot.log";
    paths_.cacheDir = paths_.dataDir / "cache";
    paths_.recordingsDir = paths_.dataDir / "recordings";
    paths_.statusFile = paths_.dataDir / "status.json";
    std::error_code ec;
    for (const auto& d : {paths_.dataDir, paths_.logFile.parent_path(), paths_.cacheDir, paths_.recordingsDir}) {
        std::filesystem::create_directories(d, ec);
    }

    Logger::Options lo;
    lo.file = paths_.logFile;
    log_.start(lo);
    log_.setGameForwardLevel(LogLevel::Warn);

    try {
        loadConfiguration();
    } catch (const std::exception& e) {
        config_ = Config{};
        log_.error("Config could not be loaded ({}); using defaults", e.what());
    }
    if (!config_.debug.logging) log_.setLevel(LogLevel::Warn);
    Logger::Options lo2 = lo;
    lo2.level = log_.level();
    lo2.maxBytes = static_cast<std::uint64_t>(config_.debug.logMaxMb * 1024 * 1024);
    lo2.maxFiles = config_.debug.logFiles;
    log_.start(lo2);

    log_.info("ATSPilot {} plugin initialized (data: {})", "0.3.0", paths_.dataDir.string());

    // <game>/bin/win_x64/plugins/atspilot.dll -> <game>
    paths_.gameDir = config_.map.gameDir.empty() ? thisModuleDir().parent_path().parent_path().parent_path()
                                                 : std::filesystem::path(config_.map.gameDir);

    pilot_ = std::make_unique<Autopilot>(config_, &log_);
    pilot_->setEventCallback([this](PilotEvent e, const std::string& msg) { handleEvent(e, msg); });

    if (config_.map.enabled) {
        if (std::filesystem::exists(paths_.gameDir / "base_map.scs", ec)) {
            map_ = std::make_unique<MapService>(log_, config_);
            map_->start(paths_.gameDir, paths_.cacheDir);
        } else {
            log_.error("Game data not found in '{}'; set map.game_dir. Steering modes unavailable.",
                       paths_.gameDir.string());
        }
    } else {
        log_.info("Map disabled in config; only cruise mode is available");
    }

    if (config_.debug.recordTelemetry) {
        const auto stamp = std::chrono::system_clock::now().time_since_epoch().count();
        const auto file = paths_.recordingsDir / ("telemetry_" + std::to_string(stamp) + ".csv");
        if (recorder_.start(file, static_cast<std::uint64_t>(config_.debug.recordMaxMb * 1024 * 1024))) {
            log_.info("Recording telemetry to {}", file.string());
        }
    }

    if (config_.hud.enabled) hud_ = std::make_unique<Hud>(config_.hud, config_.speed.units);

    worker_ = std::thread([this] { workerLoop(); });
}

void Runtime::flushGameLog() {
    if (!gameLog_) return;
    for (const auto& [level, msg] : log_.drainGameMessages()) {
        const std::string line = "[ATSPilot] " + msg;
        gameLog_(level >= LogLevel::Error ? SCS_LOG_TYPE_error : SCS_LOG_TYPE_warning, line.c_str());
    }
}

void Runtime::setTelemetryActive(bool active) {
    telemetryActive_ = active;
    if (!active && pilot_) pilot_->disengage("Telemetry Unavailable");
    log_.info("ATS telemetry {}", active ? "connected" : "disconnected");
}

void Runtime::setGameName(const std::string& name) {
    if (!config_.memory.enabled || memory_) return;
    memory_ = std::make_unique<GameMemory>(log_, config_.memory);
    memory_->start(name);
}

void Runtime::updateFromGameMemory() {
    if (!memory_ || !memory_->ready()) return;
    WorldSnapshotPtr world = memory_->readWorld(current_.time, current_.worldPosition);
    if (!worldLogged_ && (!world->vehicles.empty() || !world->lights.empty())) {
        worldLogged_ = true;
        log_.info("Game memory: {} traffic bodies and {} traffic lights around the truck", world->vehicles.size(),
                  world->lights.size());
    }
    pilot_->setWorld(std::move(world));
    // The steering path only changes while ATSPilot is off; losing it while
    // driving hands control back.
    const bool direct = memory_->steeringAvailable();
    if (pilot_->mode() == PilotMode::Off) {
        pilot_->setDirectSteering(direct);
    } else if (pilot_->directSteering() && !direct) {
        pilot_->disengage("Direct steering unavailable - take over");
    }
    if (map_) {
        if (auto route = memory_->readGpsRouteIfChanged()) map_->setGameRoute(std::move(*route));
    }
}

void Runtime::applyDirectSteering(bool frameStart) {
    if (!memory_ || !pilot_ || !pilot_->directSteering()) return;
    const bool steering = pilot_->mode() != PilotMode::Off && command_.active && command_.steerActive &&
                          wallNow() - commandWall_ <= config_.safety.commandTimeout;
    if (steering) {
        if (!memory_->writeSteering(command_.steering)) pilot_->disengage("Direct steering failed - take over");
    } else if (!frameStart) {
        memory_->observeSteering(current_.effectiveSteering);
    }
}

void Runtime::setInputDeviceActive(bool active) {
    inputActive_ = active;
    log_.info("Input device {}", active ? "active" : "inactive");
    if (!active && pilot_) pilot_->disengage("Input device inactive");
}

void Runtime::onWheelSteering(unsigned index, float rotations) {
    if (index < wheelSteering_.size()) wheelSteering_[index] = rotations;
}

void Runtime::onPaused(bool paused) {
    paused_ = paused;
    pending_.paused = paused;
    if (paused && pilot_ && pilot_->mode() != PilotMode::Off) pilot_->disengage("Game paused");
}

void Runtime::onTruckConfiguration() {
    const auto& vc = vehicleConfig_;
    log_.info("Truck configuration: '{}' wheelbase {:.2f} m, trailers {}, cargo {:.0f} kg", vc.truckName, vc.wheelbase,
              vc.trailerCount, vc.cargoMassKg);
}

void Runtime::onGameplayEvent(const std::string& id) {
    log_.info("Gameplay event: {}", id);
    if (id == "player.use.ferry" || id == "player.use.train" || id == "job.delivered" || id == "job.cancelled") {
        if (pilot_) pilot_->disengage("World transition (" + id + ")");
        if (map_) map_->invalidate(id);
    }
}

void Runtime::onFrameStart(double simulationTime, bool timerRestart) {
    pending_.time = simulationTime;
    if (timerRestart && pilot_ && pilot_->mode() != PilotMode::Off) pilot_->disengage("Timer restart");
    // Re-apply the steering before the game simulates this frame.
    try {
        applyDirectSteering(true);
    } catch (...) {
    }
}

void Runtime::queueRequest(PilotRequest r) {
    std::lock_guard lock(requestMutex_);
    queuedRequests_.push_back(r);
}

void Runtime::processRequest(PilotRequest r) {
    double pathWall = -1e9;
    const PathSnapshotPtr path = map_ ? map_->latestPath(&pathWall) : nullptr;
    pilot_->request(r, current_, vehicleConfig_, path, wallNow(), pathWall);
}

void Runtime::pollHotkeys() {
    std::vector<PilotRequest> queued;
    {
        std::lock_guard lock(requestMutex_);
        queued.swap(queuedRequests_);
    }
    for (auto r : queued) processRequest(r);

    if (!gameHasFocus()) {
        for (auto& h : hotkeys_) h.wasDown = false;
        return;
    }
    const bool shift = keyDown(VK_SHIFT), ctrl = keyDown(VK_CONTROL), alt = keyDown(VK_MENU);
    for (auto& h : hotkeys_) {
        const bool down = keyDown(h.binding.virtualKey) && shift == h.binding.shift && ctrl == h.binding.ctrl &&
                          alt == h.binding.alt;
        if (down && !h.wasDown) {
            log_.debug("Hotkey {} -> {}", toString(h.binding), toString(h.request));
            processRequest(h.request);
        }
        h.wasDown = down;
    }
}

void Runtime::onFrameEnd() {
    try {
        const double now = wallNow();
        // Mean steering angle of steerable wheels; the SDK reports rotations.
        double sum = 0.0;
        int n = 0;
        for (std::size_t i = 0; i < wheelSteering_.size(); ++i) {
            if (wheelSteerable_[i]) {
                sum += wheelSteering_[i];
                ++n;
            }
        }
        pending_.steerableWheelAngle = n ? sum / n * kTwoPi : 0.0;
        pending_.valid = telemetryActive_;
        pending_.paused = paused_;
        current_ = pending_;
        lastTelemetryWall_ = now;

        // Teleports, garage moves and job resets show up as a position jump.
        if (havePosition_ && distance(current_.worldPosition, lastPosition_) > 50.0) {
            if (pilot_->mode() != PilotMode::Off) pilot_->disengage("Vehicle relocated");
            if (map_) map_->invalidate("vehicle relocated");
        }
        lastPosition_ = current_.worldPosition;
        havePosition_ = true;

        if (map_) map_->submitVehicle(current_, vehicleConfig_, now);
        updateFromGameMemory();
        pollHotkeys();

        double pathWall = -1e9;
        const PathSnapshotPtr path = map_ ? map_->latestPath(&pathWall) : nullptr;
        command_ = pilot_->update(current_, vehicleConfig_, path, now, lastTelemetryWall_, pathWall);
        commandWall_ = now;
        // Game-control presses are queued until the input device sends them, since
        // several physics frames can pass between two input callbacks.
        pendingButtons_.merge(command_.buttons);
        applyDirectSteering(false);
        recorder_.record(current_, command_, pilot_->debug(), pilot_->mode());
        publishStatus();
        flushGameLog();
    } catch (const std::exception& e) {
        fail("frame", e.what());
    } catch (...) {
        fail("frame", "unknown error");
    }
}

GameButtons Runtime::takeButtons() {
    GameButtons b = pendingButtons_;
    pendingButtons_ = GameButtons{};
    return b;
}

OutputValues Runtime::currentOutput() {
    OutputValues out;
    if (!pilot_ || !command_.active) return out;
    // Commands expire: a stalled control loop must never leave pedals or steering applied.
    if (wallNow() - commandWall_ > config_.safety.commandTimeout) {
        if (pilot_->mode() != PilotMode::Off) pilot_->disengage("Control loop stalled");
        command_ = ControlCommand{};
        return out;
    }
    // Direct steering bypasses the input mix entirely.
    if (command_.steerActive && !pilot_->directSteering()) {
        out.steering = static_cast<float>(clamp(command_.steering * config_.steering.outputSign, -1.0, 1.0));
    }
    out.indicator = command_.indicator;
    if (command_.pedalsActive) {
        out.throttle = static_cast<float>(clamp(command_.throttle, 0.0, 1.0));
        out.brake = static_cast<float>(clamp(command_.brake, 0.0, 1.0));
    }
    return out;
}

void Runtime::fail(const std::string& where, const std::string& what) {
    command_ = ControlCommand{};
    try {
        if (pilot_) pilot_->disengage("Internal error");
        log_.error("Error in {}: {}", where, what);
    } catch (...) {
    }
}

void Runtime::handleEvent(PilotEvent e, const std::string& msg) {
    if (!config_.audioEnabled) return;
    const char* alias = nullptr;
    switch (e) {
        case PilotEvent::Engaged: alias = "SystemAsterisk"; break;
        case PilotEvent::Disengaged: alias = "SystemExclamation"; break;
        case PilotEvent::DriverOverride: alias = "SystemExclamation"; break;
        case PilotEvent::Unavailable: alias = "SystemHand"; break;
        case PilotEvent::EmergencyBraking: alias = "SystemHand"; break;
        case PilotEvent::SetSpeedChanged: break;
        case PilotEvent::WaitingAtIntersection: alias = "SystemNotification"; break;
        case PilotEvent::Arrived: alias = "SystemAsterisk"; break;
    }
    (void)msg;
    if (!alias) return;
    {
        std::lock_guard lock(workerMutex_);
        if (sounds_.size() < 4) sounds_.push_back(alias);
    }
    workerCv_.notify_one();
}

void Runtime::publishStatus() {
    const double now = wallNow();
    if (now - lastStatusPublish_ < 0.25) return;
    lastStatusPublish_ = now;
    PilotStatus st = pilot_->status();
    if (map_) {
        st.mapLoaded = map_->state() == MapState::Ready;
        if (!st.mapLoaded && pilot_->mode() == PilotMode::Off && map_->state() == MapState::Loading) {
            st.statusMessage = "Loading map: " + map_->statusText();
        }
    }
    if (hud_) hud_->update(st);
    {
        std::lock_guard lock(workerMutex_);
        statusSnapshot_ = st;
        statusDirty_ = true;
    }
    workerCv_.notify_one();
}

void Runtime::workerLoop() {
    std::unique_lock lock(workerMutex_);
    double lastSound = -10.0;
    while (!workerStop_) {
        workerCv_.wait_for(lock, std::chrono::milliseconds(500),
                           [this] { return workerStop_ || statusDirty_ || !sounds_.empty(); });
        if (workerStop_) break;
        std::deque<std::string> sounds;
        sounds.swap(sounds_);
        const bool writeStatus = statusDirty_ && config_.debug.statusFile;
        const PilotStatus st = statusSnapshot_;
        statusDirty_ = false;
        lock.unlock();

        for (const auto& s : sounds) {
            if (wallNow() - lastSound < 1.0) continue;  // never spam
            lastSound = wallNow();
            PlaySoundA(s.c_str(), nullptr, SND_ALIAS | SND_ASYNC | SND_NODEFAULT);
        }
        if (writeStatus) {
            const auto units = config_.speed.units;
            std::ostringstream j;
            j << "{\"mode\":\"" << toString(st.mode) << "\",\"available\":" << (st.available ? "true" : "false")
              << ",\"telemetry\":" << (st.telemetryConnected ? "true" : "false")
              << ",\"map\":" << (st.mapLoaded ? "true" : "false")
              << ",\"navigation\":" << (st.navigationActive ? "true" : "false")
              << ",\"gps_matched\":" << (st.gpsMatched ? "true" : "false")
              << ",\"game_cruise\":" << (st.gameCruiseActive ? "true" : "false")
              << ",\"cruise_set_speed\":" << std::lround(mpsToSpeed(st.cruiseSetSpeed, units))
              << ",\"waiting_at_intersection\":" << (st.waitingAtIntersection ? "true" : "false")
              << ",\"traffic_aware\":" << (st.trafficAware ? "true" : "false")
              << ",\"direct_steering\":" << (st.directSteering ? "true" : "false")
              << ",\"lead_distance_m\":" << std::lround(st.leadDistance)
              << ",\"signal\":\"" << jsonEscape(st.signalState) << "\""
              << ",\"profile\":\"" << jsonEscape(st.profile) << "\"" << ",\"units\":\"" << unitLabel(units)
              << "\",\"speed\":" << std::lround(mpsToSpeed(st.speed, units))
              << ",\"set_speed\":" << std::lround(mpsToSpeed(st.setSpeed, units))
              << ",\"target_speed\":" << std::lround(mpsToSpeed(st.targetSpeed, units))
              << ",\"cross_track_m\":" << std::round(st.crossTrackError * 100.0) / 100.0 << ",\"road\":\""
              << jsonEscape(st.road) << "\",\"next\":\"" << jsonEscape(st.nextManeuver)
              << "\",\"next_distance_m\":" << std::lround(st.nextManeuverDistance)
              << ",\"route_distance_m\":" << std::lround(st.routeDistance) << ",\"message\":\""
              << jsonEscape(st.statusMessage) << "\"}\n";
            const auto tmp = paths_.statusFile.string() + ".tmp";
            {
                std::ofstream f(tmp, std::ios::trunc);
                f << j.str();
            }
            std::error_code ec;
            std::filesystem::rename(tmp, paths_.statusFile, ec);
        }
        lock.lock();
    }
}

}  // namespace atspilot::plugin

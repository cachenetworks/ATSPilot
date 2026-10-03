#include "util/Logger.h"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace atspilot {

const char* toString(LogLevel l) {
    switch (l) {
        case LogLevel::Trace: return "TRACE";
        case LogLevel::Debug: return "DEBUG";
        case LogLevel::Info: return "INFO";
        case LogLevel::Warn: return "WARN";
        case LogLevel::Error: return "ERROR";
        case LogLevel::Critical: return "CRITICAL";
    }
    return "?";
}

LogLevel logLevelFromString(const std::string& s) {
    if (s == "trace") return LogLevel::Trace;
    if (s == "debug") return LogLevel::Debug;
    if (s == "warn") return LogLevel::Warn;
    if (s == "error") return LogLevel::Error;
    if (s == "critical") return LogLevel::Critical;
    return LogLevel::Info;
}

Logger::~Logger() { stop(); }

void Logger::start(const Options& options) {
    stop();
    options_ = options;
    level_.store(options.level);
    {
        std::lock_guard lock(mutex_);
        running_ = true;
    }
    if (!options_.file.empty()) {
        try {
            std::filesystem::create_directories(options_.file.parent_path());
            openFile();
        } catch (...) {
            // Logging must never take the plugin down; continue without a file.
        }
    }
    thread_ = std::thread([this] { run(); });
}

void Logger::stop() {
    {
        std::lock_guard lock(mutex_);
        if (!running_) return;
        running_ = false;
    }
    cv_.notify_all();
    if (thread_.joinable()) thread_.join();
    if (file_.is_open()) file_.close();
}

void Logger::setSink(std::function<void(LogLevel, const std::string&)> sink) {
    std::lock_guard lock(mutex_);
    sink_ = std::move(sink);
}

void Logger::log(LogLevel level, std::string message) {
    if (!enabled(level)) return;

    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    std::ostringstream line;
    line << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << '.' << std::setw(3) << std::setfill('0') << ms << " ["
         << toString(level) << "] " << message;

    std::function<void(LogLevel, const std::string&)> sink;
    {
        std::lock_guard lock(mutex_);
        sink = sink_;
        if (level >= gameLevel_.load() && gameQueue_.size() < 256) gameQueue_.emplace_back(level, message);
        if (running_) {
            if (queue_.size() >= options_.maxQueue) {
                dropped_.fetch_add(1);
            } else {
                queue_.push_back(line.str());
            }
        }
    }
    cv_.notify_one();
    if (sink) sink(level, message);
}

std::vector<std::pair<LogLevel, std::string>> Logger::drainGameMessages() {
    std::lock_guard lock(mutex_);
    std::vector<std::pair<LogLevel, std::string>> out;
    out.swap(gameQueue_);
    return out;
}

void Logger::openFile() {
    file_.open(options_.file, std::ios::out | std::ios::app);
    std::error_code ec;
    fileBytes_ = std::filesystem::exists(options_.file, ec) ? std::filesystem::file_size(options_.file, ec) : 0;
}

void Logger::rotate() {
    file_.close();
    std::error_code ec;
    const auto base = options_.file;
    for (int i = options_.maxFiles - 1; i >= 1; --i) {
        auto from = base;
        from += "." + std::to_string(i);
        auto to = base;
        to += "." + std::to_string(i + 1);
        if (std::filesystem::exists(from, ec)) {
            std::filesystem::remove(to, ec);
            std::filesystem::rename(from, to, ec);
        }
    }
    auto first = base;
    first += ".1";
    std::filesystem::remove(first, ec);
    std::filesystem::rename(base, first, ec);
    openFile();
}

void Logger::run() {
    std::unique_lock lock(mutex_);
    while (true) {
        cv_.wait(lock, [this] { return !queue_.empty() || !running_; });
        std::deque<std::string> batch;
        batch.swap(queue_);
        const bool stopping = !running_;
        lock.unlock();

        if (file_.is_open()) {
            for (const auto& l : batch) {
                file_ << l << '\n';
                fileBytes_ += l.size() + 1;
            }
            file_.flush();
            if (options_.maxBytes > 0 && fileBytes_ > options_.maxBytes) rotate();
        }

        lock.lock();
        if (stopping && queue_.empty()) break;
    }
}

}  // namespace atspilot

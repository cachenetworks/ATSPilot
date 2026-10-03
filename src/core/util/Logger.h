#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <format>
#include <fstream>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace atspilot {

enum class LogLevel { Trace, Debug, Info, Warn, Error, Critical };

const char* toString(LogLevel l);
LogLevel logLevelFromString(const std::string& s);

// Thread-safe logger. Callers only format and enqueue; a background thread does
// the file I/O and rotation, so logging never blocks the game's main thread on disk.
class Logger {
public:
    struct Options {
        std::filesystem::path file;  // empty = no file output
        LogLevel level = LogLevel::Info;
        std::uint64_t maxBytes = 5ull * 1024 * 1024;
        int maxFiles = 3;
        std::size_t maxQueue = 4096;
    };

    Logger() = default;
    ~Logger();
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    void start(const Options& options);
    void stop();

    void setLevel(LogLevel level) { level_.store(level); }
    LogLevel level() const { return level_.load(); }
    bool enabled(LogLevel l) const { return l >= level_.load(); }

    void log(LogLevel level, std::string message);

    template <typename... A> void trace(std::format_string<A...> f, A&&... a) { fmt(LogLevel::Trace, f, std::forward<A>(a)...); }
    template <typename... A> void debug(std::format_string<A...> f, A&&... a) { fmt(LogLevel::Debug, f, std::forward<A>(a)...); }
    template <typename... A> void info(std::format_string<A...> f, A&&... a) { fmt(LogLevel::Info, f, std::forward<A>(a)...); }
    template <typename... A> void warn(std::format_string<A...> f, A&&... a) { fmt(LogLevel::Warn, f, std::forward<A>(a)...); }
    template <typename... A> void error(std::format_string<A...> f, A&&... a) { fmt(LogLevel::Error, f, std::forward<A>(a)...); }
    template <typename... A> void critical(std::format_string<A...> f, A&&... a) { fmt(LogLevel::Critical, f, std::forward<A>(a)...); }

    // Messages at or above this level are also kept for the game console, which
    // may only be written from the game's main thread (see drainGameMessages).
    void setGameForwardLevel(LogLevel level) { gameLevel_.store(level); }
    std::vector<std::pair<LogLevel, std::string>> drainGameMessages();

    // In-memory sink used by tests and the simulator.
    void setSink(std::function<void(LogLevel, const std::string&)> sink);

    std::uint64_t dropped() const { return dropped_.load(); }

private:
    template <typename... A>
    void fmt(LogLevel l, std::format_string<A...> f, A&&... a) {
        if (enabled(l)) log(l, std::format(f, std::forward<A>(a)...));
    }

    void run();
    void rotate();
    void openFile();

    Options options_;
    std::atomic<LogLevel> level_{LogLevel::Info};
    std::atomic<LogLevel> gameLevel_{LogLevel::Info};
    std::atomic<std::uint64_t> dropped_{0};

    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<std::string> queue_;
    std::vector<std::pair<LogLevel, std::string>> gameQueue_;
    std::function<void(LogLevel, const std::string&)> sink_;
    bool running_ = false;
    std::thread thread_;

    std::ofstream file_;
    std::uint64_t fileBytes_ = 0;
};

}  // namespace atspilot

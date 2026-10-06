#pragma once

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <string>
#include <string_view>

namespace atspilot {

enum class GameKind { Unknown, Ats, Ets2 };

struct GameDefinition {
    GameKind kind;
    const char* id;
    const char* selector;
    const char* displayName;
    const char* executable;
    const char* steamDirectory;
    const char* documentsDirectory;
    const char* mapName;
    const char* cacheFile;
};

inline constexpr GameDefinition kUnknownGame{GameKind::Unknown, "", "", "Unknown SCS game", "", "", "", "usa",
                                             "map_usa.cache"};
inline constexpr GameDefinition kAtsGame{GameKind::Ats,
                                        "ats",
                                        "ats",
                                        "American Truck Simulator",
                                        "amtrucks.exe",
                                        "American Truck Simulator",
                                        "American Truck Simulator",
                                        "usa",
                                        "map_usa.cache"};
inline constexpr GameDefinition kEts2Game{GameKind::Ets2,
                                         "eut2",
                                         "ets2",
                                         "Euro Truck Simulator 2",
                                         "eurotrucks2.exe",
                                         "Euro Truck Simulator 2",
                                         "Euro Truck Simulator 2",
                                         "europe",
                                         "map_europe.cache"};

inline constexpr const GameDefinition& gameDefinition(GameKind kind) {
    switch (kind) {
        case GameKind::Ats:
            return kAtsGame;
        case GameKind::Ets2:
            return kEts2Game;
        default:
            return kUnknownGame;
    }
}

inline std::string lowerAscii(std::string_view value) {
    std::string out(value);
    std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return out;
}

inline GameKind gameKindFromId(std::string_view id) {
    if (id == kAtsGame.id) return GameKind::Ats;
    if (id == kEts2Game.id) return GameKind::Ets2;
    return GameKind::Unknown;
}

inline GameKind gameKindFromSelector(std::string_view selector) {
    const std::string value = lowerAscii(selector);
    if (value == "ats" || value == "amtrucks" || value == "amtrucks.exe" || value == "american truck simulator") {
        return GameKind::Ats;
    }
    if (value == "ets2" || value == "eut2" || value == "eurotrucks2" || value == "eurotrucks2.exe" ||
        value == "euro truck simulator 2") {
        return GameKind::Ets2;
    }
    return GameKind::Unknown;
}

inline GameKind gameKindFromExecutable(const std::filesystem::path& executable) {
    return gameKindFromSelector(executable.filename().string());
}

inline GameKind gameKindFromDirectory(const std::filesystem::path& gameDir) {
    std::error_code ec;
    if (std::filesystem::exists(gameDir / "bin" / "win_x64" / kAtsGame.executable, ec)) return GameKind::Ats;
    ec.clear();
    if (std::filesystem::exists(gameDir / "bin" / "win_x64" / kEts2Game.executable, ec)) return GameKind::Ets2;
    return GameKind::Unknown;
}

}  // namespace atspilot

#include "util/KeyBinding.h"

#include <algorithm>
#include <cctype>
#include <map>
#include <sstream>

namespace atspilot {
namespace {

std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

const std::map<std::string, int>& namedKeys() {
    static const std::map<std::string, int> keys = [] {
        std::map<std::string, int> m{
            {"insert", 0x2D}, {"delete", 0x2E},   {"home", 0x24},     {"end", 0x23},       {"pageup", 0x21},
            {"pagedown", 0x22}, {"up", 0x26},     {"down", 0x28},     {"left", 0x25},      {"right", 0x27},
            {"minus", 0xBD},  {"equals", 0xBB},   {"backspace", 0x08}, {"pause", 0x13},    {"numplus", 0x6B},
            {"numminus", 0x6D}, {"space", 0x20},  {"tab", 0x09},      {"comma", 0xBC},     {"period", 0xBE},
        };
        for (int i = 1; i <= 24; ++i) m["f" + std::to_string(i)] = 0x70 + i - 1;
        for (int i = 0; i <= 9; ++i) m["num" + std::to_string(i)] = 0x60 + i;
        return m;
    }();
    return keys;
}

}  // namespace

std::optional<KeyBinding> parseKeyBinding(const std::string& text) {
    KeyBinding k;
    std::stringstream ss(text);
    std::string part;
    std::string keyName;
    while (std::getline(ss, part, '+')) {
        part.erase(std::remove_if(part.begin(), part.end(), [](unsigned char c) { return std::isspace(c); }),
                   part.end());
        const std::string p = lower(part);
        if (p.empty()) return std::nullopt;
        if (p == "shift") k.shift = true;
        else if (p == "ctrl" || p == "control") k.ctrl = true;
        else if (p == "alt") k.alt = true;
        else if (!keyName.empty()) return std::nullopt;  // two non-modifier keys
        else keyName = p;
    }
    if (keyName.empty()) return std::nullopt;
    if (keyName.size() == 1 && std::isalnum(static_cast<unsigned char>(keyName[0]))) {
        k.virtualKey = std::toupper(static_cast<unsigned char>(keyName[0]));
        return k;
    }
    const auto it = namedKeys().find(keyName);
    if (it == namedKeys().end()) return std::nullopt;
    k.virtualKey = it->second;
    return k;
}

std::string toString(const KeyBinding& k) {
    std::string name;
    for (const auto& [n, vk] : namedKeys()) {
        if (vk == k.virtualKey) {
            name = n;
            break;
        }
    }
    if (name.empty() && k.virtualKey > 0 && k.virtualKey < 128 && std::isalnum(k.virtualKey)) {
        name = std::string(1, static_cast<char>(k.virtualKey));
    }
    if (!name.empty()) name[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(name[0])));
    std::string out;
    if (k.ctrl) out += "Ctrl+";
    if (k.alt) out += "Alt+";
    if (k.shift) out += "Shift+";
    return out + name;
}

}  // namespace atspilot

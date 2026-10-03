#include "map/Sii.h"

#include <cctype>
#include <charconv>
#include <cstdint>
#include <cstring>
#include <sstream>

namespace atspilot {
namespace {

std::string trim(const std::string& s) {
    std::size_t b = 0, e = s.size();
    while (b < e && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
    while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
    return s.substr(b, e - b);
}

std::string stripComment(const std::string& line) {
    bool inString = false;
    for (std::size_t i = 0; i < line.size(); ++i) {
        if (line[i] == '"') inString = !inString;
        if (inString) continue;
        if (line[i] == '#') return line.substr(0, i);
        if (line[i] == '/' && i + 1 < line.size() && line[i + 1] == '/') return line.substr(0, i);
    }
    return line;
}

}  // namespace

const std::string* SiiUnit::first(const std::string& key) const {
    const auto it = attributes.find(key);
    return it == attributes.end() || it->second.empty() ? nullptr : &it->second.front();
}

std::size_t SiiUnit::count(const std::string& key) const {
    const auto it = attributes.find(key);
    return it == attributes.end() ? 0 : it->second.size();
}

std::vector<SiiUnit> parseSiiText(const std::string& text) {
    std::vector<SiiUnit> units;
    std::istringstream in(text);
    std::string raw;
    SiiUnit current;
    bool inUnit = false;
    bool pendingHeader = false;
    bool inBlockComment = false;

    while (std::getline(in, raw)) {
        std::string line = raw;
        if (inBlockComment) {
            const auto end = line.find("*/");
            if (end == std::string::npos) continue;
            line = line.substr(end + 2);
            inBlockComment = false;
        }
        const auto blockStart = line.find("/*");
        if (blockStart != std::string::npos && line.find("*/", blockStart) == std::string::npos) {
            line = line.substr(0, blockStart);
            inBlockComment = true;
        }
        line = trim(stripComment(line));
        if (line.empty() || line == "SiiNunit") continue;

        if (!inUnit) {
            if (line == "{") {
                if (pendingHeader) {
                    inUnit = true;
                    pendingHeader = false;
                }
                continue;  // the file-level brace
            }
            if (line == "}") continue;
            const auto colon = line.find(':');
            if (colon == std::string::npos) continue;
            current = SiiUnit{};
            current.className = trim(line.substr(0, colon));
            std::string rest = trim(line.substr(colon + 1));
            if (!rest.empty() && rest.back() == '{') {
                rest = trim(rest.substr(0, rest.size() - 1));
                inUnit = true;
            } else {
                pendingHeader = true;
            }
            current.name = rest;
            continue;
        }

        if (line == "}") {
            units.push_back(std::move(current));
            current = SiiUnit{};
            inUnit = false;
            continue;
        }
        const auto colon = line.find(':');
        if (colon == std::string::npos) continue;
        std::string key = trim(line.substr(0, colon));
        const std::string value = trim(line.substr(colon + 1));
        const auto bracket = key.find('[');
        if (bracket != std::string::npos) key = key.substr(0, bracket);
        current.attributes[key].push_back(value);
    }
    return units;
}

double siiNumber(const std::string& value, double fallback) {
    std::string v = value;
    if (v.empty()) return fallback;
    // Hex-encoded IEEE floats ("&40900000") are used by some exporters.
    if (v.front() == '&' && v.size() == 9) {
        std::uint32_t bits = 0;
        if (std::from_chars(v.data() + 1, v.data() + v.size(), bits, 16).ec != std::errc()) return fallback;
        float f;
        std::memcpy(&f, &bits, sizeof(f));
        return f;
    }
    double d = fallback;
    const auto r = std::from_chars(v.data(), v.data() + v.size(), d);
    return r.ec == std::errc() ? d : fallback;
}

std::string siiUnquote(const std::string& value) {
    if (value.size() >= 2 && value.front() == '"' && value.back() == '"') return value.substr(1, value.size() - 2);
    return value;
}

}  // namespace atspilot

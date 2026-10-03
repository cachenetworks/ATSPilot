#include "config/Toml.h"

#include <cctype>
#include <charconv>
#include <sstream>

namespace atspilot {
namespace {

std::string trim(const std::string& s) {
    std::size_t b = 0;
    std::size_t e = s.size();
    while (b < e && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
    while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
    return s.substr(b, e - b);
}

bool validKey(const std::string& k) {
    if (k.empty()) return false;
    for (char c : k) {
        if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-')) return false;
    }
    return true;
}

// Removes a trailing comment that is not inside a string.
std::string stripComment(const std::string& line) {
    bool inString = false;
    for (std::size_t i = 0; i < line.size(); ++i) {
        const char c = line[i];
        if (c == '"' && (i == 0 || line[i - 1] != '\\')) inString = !inString;
        if (c == '#' && !inString) return line.substr(0, i);
    }
    return line;
}

}  // namespace

TomlDocument TomlDocument::parse(const std::string& text) {
    TomlDocument doc;
    std::istringstream in(text);
    std::string raw;
    std::string section;
    int lineNo = 0;

    while (std::getline(in, raw)) {
        ++lineNo;
        if (!raw.empty() && raw.back() == '\r') raw.pop_back();
        if (lineNo == 1 && raw.size() >= 3 && static_cast<unsigned char>(raw[0]) == 0xEF) raw = raw.substr(3);
        const std::string line = trim(stripComment(raw));
        if (line.empty()) continue;

        if (line.front() == '[') {
            if (line.back() != ']' || line.size() < 3) {
                doc.errors_.push_back({lineNo, "malformed section header"});
                continue;
            }
            section = trim(line.substr(1, line.size() - 2));
            if (!validKey(section)) doc.errors_.push_back({lineNo, "invalid section name '" + section + "'"});
            continue;
        }

        const auto eq = line.find('=');
        if (eq == std::string::npos) {
            doc.errors_.push_back({lineNo, "expected key = value"});
            continue;
        }
        const std::string key = trim(line.substr(0, eq));
        const std::string val = trim(line.substr(eq + 1));
        if (!validKey(key)) {
            doc.errors_.push_back({lineNo, "invalid key '" + key + "'"});
            continue;
        }
        const std::string full = section.empty() ? key : section + "." + key;

        if (val == "true" || val == "false") {
            doc.values_[full] = (val == "true");
        } else if (val.size() >= 2 && val.front() == '"' && val.back() == '"') {
            std::string s;
            for (std::size_t i = 1; i + 1 < val.size(); ++i) {
                if (val[i] == '\\' && i + 2 < val.size()) {
                    const char n = val[++i];
                    s += (n == 'n') ? '\n' : (n == 't') ? '\t' : n;
                } else {
                    s += val[i];
                }
            }
            doc.values_[full] = s;
        } else {
            std::string num;
            for (char c : val) {
                if (c != '_') num += c;
            }
            double d = 0.0;
            const auto res = std::from_chars(num.data(), num.data() + num.size(), d);
            if (res.ec != std::errc() || res.ptr != num.data() + num.size()) {
                doc.errors_.push_back({lineNo, "unsupported value for '" + full + "'"});
                continue;
            }
            doc.values_[full] = d;
        }
    }
    return doc;
}

const TomlDocument::Value* TomlDocument::find(const std::string& section, const std::string& key) const {
    const auto it = values_.find(section.empty() ? key : section + "." + key);
    return it == values_.end() ? nullptr : &it->second;
}

std::optional<double> TomlDocument::getNumber(const std::string& section, const std::string& key) const {
    const Value* v = find(section, key);
    if (v && std::holds_alternative<double>(*v)) return std::get<double>(*v);
    return std::nullopt;
}

std::optional<bool> TomlDocument::getBool(const std::string& section, const std::string& key) const {
    const Value* v = find(section, key);
    if (v && std::holds_alternative<bool>(*v)) return std::get<bool>(*v);
    return std::nullopt;
}

std::optional<std::string> TomlDocument::getString(const std::string& section, const std::string& key) const {
    const Value* v = find(section, key);
    if (v && std::holds_alternative<std::string>(*v)) return std::get<std::string>(*v);
    return std::nullopt;
}

bool TomlDocument::has(const std::string& section, const std::string& key) const { return find(section, key) != nullptr; }

std::vector<std::string> TomlDocument::keys() const {
    std::vector<std::string> out;
    out.reserve(values_.size());
    for (const auto& [k, v] : values_) out.push_back(k);
    return out;
}

}  // namespace atspilot

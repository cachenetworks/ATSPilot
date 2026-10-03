#pragma once

#include <map>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace atspilot {

// Minimal TOML subset: [section] headers, key = value pairs where value is a
// number, boolean or basic "string", and # comments. Enough for ATSPilot's
// configuration without pulling in a dependency; anything else is reported as
// an error with its line number rather than guessed at.
class TomlDocument {
public:
    using Value = std::variant<double, bool, std::string>;

    struct Error {
        int line = 0;
        std::string message;
    };

    static TomlDocument parse(const std::string& text);

    const std::vector<Error>& errors() const { return errors_; }

    std::optional<double> getNumber(const std::string& section, const std::string& key) const;
    std::optional<bool> getBool(const std::string& section, const std::string& key) const;
    std::optional<std::string> getString(const std::string& section, const std::string& key) const;
    bool has(const std::string& section, const std::string& key) const;

    // Keys present in the document, as "section.key".
    std::vector<std::string> keys() const;

private:
    const Value* find(const std::string& section, const std::string& key) const;

    std::map<std::string, Value> values_;  // "section.key" -> value
    std::vector<Error> errors_;
};

}  // namespace atspilot

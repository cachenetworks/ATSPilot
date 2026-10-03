#pragma once

#include <map>
#include <string>
#include <vector>

namespace atspilot {

// One unit from a plain-text SII file, e.g.
//   road_look : road.us_tmpl0 { lanes_left[]: traffic_lane.road.freeway ... }
// Values are kept as raw strings; array attributes ("key[]") accumulate.
struct SiiUnit {
    std::string className;
    std::string name;
    std::map<std::string, std::vector<std::string>> attributes;

    const std::string* first(const std::string& key) const;
    std::size_t count(const std::string& key) const;
};

// Parses plain-text SII ("SiiNunit"). Binary/encrypted SII (BSII, ScsC) is not
// handled and yields no units; ATS ships the definitions ATSPilot needs as text.
std::vector<SiiUnit> parseSiiText(const std::string& text);

double siiNumber(const std::string& value, double fallback = 0.0);
std::string siiUnquote(const std::string& value);

}  // namespace atspilot

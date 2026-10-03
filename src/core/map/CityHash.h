#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace atspilot {

// CityHash64 v1.0.3 (Google, MIT licence). HashFS v2 archives identify entries by
// the CityHash64 of their path without a leading slash.
std::uint64_t cityHash64(const char* data, std::size_t len);
inline std::uint64_t cityHash64(std::string_view s) { return cityHash64(s.data(), s.size()); }

}  // namespace atspilot

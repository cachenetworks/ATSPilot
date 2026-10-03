#pragma once

#include <cstdint>
#include <string>

namespace atspilot {

// SCS "tokens": up to 12 characters from [0-9a-z_] packed base-38 into a u64,
// least significant digit first, with 0 meaning "no character".
std::uint64_t tokenFromString(const std::string& s);
std::string tokenToString(std::uint64_t token);

}  // namespace atspilot

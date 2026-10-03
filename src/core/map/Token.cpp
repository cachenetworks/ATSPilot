#include "map/Token.h"

namespace atspilot {
namespace {

constexpr char kLetters[] = "\0" "0123456789abcdefghijklmnopqrstuvwxyz_";

int letterIndex(char c) {
    if (c >= '0' && c <= '9') return 1 + (c - '0');
    if (c >= 'a' && c <= 'z') return 11 + (c - 'a');
    if (c >= 'A' && c <= 'Z') return 11 + (c - 'A');
    if (c == '_') return 37;
    return -1;
}

}  // namespace

std::uint64_t tokenFromString(const std::string& s) {
    if (s.size() > 12) return 0;
    std::uint64_t value = 0;
    std::uint64_t mul = 1;
    for (char c : s) {
        const int idx = letterIndex(c);
        if (idx < 0) return 0;
        value += static_cast<std::uint64_t>(idx) * mul;
        mul *= 38;
    }
    return value;
}

std::string tokenToString(std::uint64_t token) {
    std::string out;
    while (token != 0) {
        const auto idx = static_cast<std::size_t>(token % 38);
        if (idx != 0) out += kLetters[idx];
        token /= 38;
    }
    return out;
}

}  // namespace atspilot

// CityHash64 v1.0.3 (the variant SCS uses for HashFS path hashes).
// Copyright (c) 2011 Google, Inc. Licensed under the MIT licence; see
// THIRD_PARTY_NOTICES.md. Re-expressed here for a single-function use.

#include "map/CityHash.h"

#include <cstring>
#include <utility>

namespace atspilot {
namespace {

using u64 = std::uint64_t;
using u32 = std::uint32_t;

constexpr u64 k0 = 0xc3a5c85c97cb3127ULL;
constexpr u64 k1 = 0xb492b66fbe98f273ULL;
constexpr u64 k2 = 0x9ae16a3b2f90404fULL;
constexpr u64 k3 = 0xc949d7c7509e6557ULL;

u64 fetch64(const char* p) {
    u64 r;
    std::memcpy(&r, p, sizeof(r));
    return r;  // little-endian hosts only (x86-64)
}

u32 fetch32(const char* p) {
    u32 r;
    std::memcpy(&r, p, sizeof(r));
    return r;
}

u64 rotate(u64 val, int shift) { return shift == 0 ? val : ((val >> shift) | (val << (64 - shift))); }
u64 rotateByAtLeast1(u64 val, int shift) { return (val >> shift) | (val << (64 - shift)); }
u64 shiftMix(u64 val) { return val ^ (val >> 47); }

u64 hashLen16(u64 u, u64 v) {
    constexpr u64 kMul = 0x9ddfea08eb382d69ULL;
    u64 a = (u ^ v) * kMul;
    a ^= (a >> 47);
    u64 b = (v ^ a) * kMul;
    b ^= (b >> 47);
    b *= kMul;
    return b;
}

u64 hashLen0to16(const char* s, std::size_t len) {
    if (len > 8) {
        const u64 a = fetch64(s);
        const u64 b = fetch64(s + len - 8);
        return hashLen16(a, rotateByAtLeast1(b + len, static_cast<int>(len))) ^ b;
    }
    if (len >= 4) {
        const u64 a = fetch32(s);
        return hashLen16(len + (a << 3), fetch32(s + len - 4));
    }
    if (len > 0) {
        const std::uint8_t a = static_cast<std::uint8_t>(s[0]);
        const std::uint8_t b = static_cast<std::uint8_t>(s[len >> 1]);
        const std::uint8_t c = static_cast<std::uint8_t>(s[len - 1]);
        const u32 y = static_cast<u32>(a) + (static_cast<u32>(b) << 8);
        const u32 z = static_cast<u32>(len) + (static_cast<u32>(c) << 2);
        return shiftMix(y * k2 ^ z * k3) * k2;
    }
    return k2;
}

u64 hashLen17to32(const char* s, std::size_t len) {
    const u64 a = fetch64(s) * k1;
    const u64 b = fetch64(s + 8);
    const u64 c = fetch64(s + len - 8) * k2;
    const u64 d = fetch64(s + len - 16) * k0;
    return hashLen16(rotate(a - b, 43) + rotate(c, 30) + d, a + rotate(b ^ k3, 20) - c + len);
}

std::pair<u64, u64> weakHashLen32WithSeeds(u64 w, u64 x, u64 y, u64 z, u64 a, u64 b) {
    a += w;
    b = rotate(b + a + z, 21);
    const u64 c = a;
    a += x;
    a += y;
    b += rotate(a, 44);
    return {a + z, b + c};
}

std::pair<u64, u64> weakHashLen32WithSeeds(const char* s, u64 a, u64 b) {
    return weakHashLen32WithSeeds(fetch64(s), fetch64(s + 8), fetch64(s + 16), fetch64(s + 24), a, b);
}

u64 hashLen33to64(const char* s, std::size_t len) {
    u64 z = fetch64(s + 24);
    u64 a = fetch64(s) + (len + fetch64(s + len - 16)) * k0;
    u64 b = rotate(a + z, 52);
    u64 c = rotate(a, 37);
    a += fetch64(s + 8);
    c += rotate(a, 7);
    a += fetch64(s + 16);
    const u64 vf = a + z;
    const u64 vs = b + rotate(a, 31) + c;
    a = fetch64(s + 16) + fetch64(s + len - 32);
    z = fetch64(s + len - 8);
    b = rotate(a + z, 52);
    c = rotate(a, 37);
    a += fetch64(s + len - 24);
    c += rotate(a, 7);
    a += fetch64(s + len - 16);
    const u64 wf = a + z;
    const u64 ws = b + rotate(a, 31) + c;
    const u64 r = shiftMix((vf + ws) * k2 + (wf + vs) * k0);
    return shiftMix(r * k0 + vs) * k2;
}

}  // namespace

std::uint64_t cityHash64(const char* s, std::size_t len) {
    if (len <= 32) return len <= 16 ? hashLen0to16(s, len) : hashLen17to32(s, len);
    if (len <= 64) return hashLen33to64(s, len);

    u64 x = fetch64(s + len - 40);
    u64 y = fetch64(s + len - 16) + fetch64(s + len - 56);
    u64 z = hashLen16(fetch64(s + len - 48) + len, fetch64(s + len - 24));
    auto v = weakHashLen32WithSeeds(s + len - 64, len, z);
    auto w = weakHashLen32WithSeeds(s + len - 32, y + k1, x);
    x = x * k1 + fetch64(s);

    len = (len - 1) & ~static_cast<std::size_t>(63);
    do {
        x = rotate(x + y + v.first + fetch64(s + 8), 37) * k1;
        y = rotate(y + v.second + fetch64(s + 48), 42) * k1;
        x ^= w.second;
        y += v.first + fetch64(s + 40);
        z = rotate(z + w.first, 33) * k1;
        v = weakHashLen32WithSeeds(s, v.second * k1, x + w.first);
        w = weakHashLen32WithSeeds(s + 32, z + w.second, y + fetch64(s + 16));
        std::swap(z, x);
        s += 64;
        len -= 64;
    } while (len != 0);
    return hashLen16(hashLen16(v.first, w.first) + shiftMix(y) * k1 + z, hashLen16(v.second, w.second) + x);
}

}  // namespace atspilot

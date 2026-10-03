#pragma once

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

#include "math/Vec.h"

namespace atspilot {

struct ParseError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

// Bounds-checked little-endian reader. Any overrun throws ParseError, so a
// format change in a game update fails one file cleanly instead of reading garbage.
class BinaryReader {
public:
    BinaryReader(const char* data, std::size_t size) : data_(data), size_(size) {}

    std::size_t pos() const { return pos_; }
    std::size_t size() const { return size_; }
    std::size_t remaining() const { return size_ - pos_; }
    void seek(std::size_t p) {
        if (p > size_) throw ParseError("seek beyond end");
        pos_ = p;
    }
    void skip(std::size_t n) {
        need(n);
        pos_ += n;
    }

    template <typename T>
    T read() {
        need(sizeof(T));
        T v;
        std::memcpy(&v, data_ + pos_, sizeof(T));
        pos_ += sizeof(T);
        return v;
    }

    std::uint8_t u8() { return read<std::uint8_t>(); }
    std::uint16_t u16() { return read<std::uint16_t>(); }
    std::uint32_t u32() { return read<std::uint32_t>(); }
    std::int32_t i32() { return read<std::int32_t>(); }
    std::uint64_t u64() { return read<std::uint64_t>(); }
    float f32() { return read<float>(); }
    Vec3 vec3f() {
        const float x = f32(), y = f32(), z = f32();
        return {x, y, z};
    }

    // u32 element count followed by `count * elementSize` bytes.
    std::uint32_t skipArray32(std::size_t elementSize) {
        const std::uint32_t n = u32();
        skip(std::size_t{n} * elementSize);
        return n;
    }

    // u64 length-prefixed string.
    std::string string64() {
        const std::uint64_t n = u64();
        if (n > remaining()) throw ParseError("string length beyond end");
        std::string s(data_ + pos_, static_cast<std::size_t>(n));
        pos_ += static_cast<std::size_t>(n);
        return s;
    }

private:
    void need(std::size_t n) const {
        if (n > size_ - pos_) throw ParseError("unexpected end of data at offset " + std::to_string(pos_));
    }

    const char* data_;
    std::size_t size_;
    std::size_t pos_ = 0;
};

}  // namespace atspilot

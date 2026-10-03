#include "map/HashFs.h"

#include <algorithm>
#include <cstring>
#include <set>

#include <zlib.h>

#include "map/CityHash.h"

namespace atspilot {
namespace {

template <typename T>
T readLe(const char* p) {
    T v;
    std::memcpy(&v, p, sizeof(T));
    return v;
}

constexpr std::uint8_t kMetaPlain = 0x80;
constexpr std::uint8_t kMetaDirectory = 0x81;

}  // namespace

bool inflateZlib(const char* data, std::size_t size, std::size_t expected, std::vector<char>& out) {
    out.resize(expected);
    z_stream zs{};
    zs.next_in = reinterpret_cast<Bytef*>(const_cast<char*>(data));
    zs.avail_in = static_cast<uInt>(size);
    zs.next_out = reinterpret_cast<Bytef*>(out.data());
    zs.avail_out = static_cast<uInt>(expected);
    if (inflateInit(&zs) != Z_OK) return false;
    const int rc = inflate(&zs, Z_FINISH);
    inflateEnd(&zs);
    if (rc != Z_STREAM_END) return false;
    out.resize(zs.total_out);
    return true;
}

std::string HashFsArchive::normalize(const std::string& path) {
    std::string p = path;
    std::replace(p.begin(), p.end(), '\\', '/');
    while (!p.empty() && p.front() == '/') p.erase(p.begin());
    while (!p.empty() && p.back() == '/') p.pop_back();
    return p;
}

std::unique_ptr<HashFsArchive> HashFsArchive::open(const std::filesystem::path& file, std::string* error) {
    auto a = std::unique_ptr<HashFsArchive>(new HashFsArchive());
    a->path_ = file;
    a->file_.open(file, std::ios::binary);
    if (!a->file_) {
        if (error) *error = "cannot open " + file.string();
        return nullptr;
    }
    char hdr[12];
    if (!a->file_.read(hdr, sizeof(hdr))) {
        if (error) *error = "truncated header";
        return nullptr;
    }
    if (std::memcmp(hdr, "SCS#", 4) != 0) {
        if (error) *error = "not a HashFS archive (zip mods are not supported)";
        return nullptr;
    }
    a->version_ = readLe<std::uint16_t>(hdr + 4);
    a->salt_ = readLe<std::uint16_t>(hdr + 6);
    if (std::memcmp(hdr + 8, "CITY", 4) != 0) {
        if (error) *error = "unsupported hash method";
        return nullptr;
    }
    const bool ok = a->version_ == 1 ? a->loadV1(error) : a->version_ == 2 ? a->loadV2(error) : false;
    if (!ok) {
        if (error && error->empty()) *error = "unsupported HashFS version " + std::to_string(a->version_);
        return nullptr;
    }
    return a;
}

bool HashFsArchive::readRaw(std::uint64_t offset, std::size_t size, std::vector<char>& out) const {
    std::lock_guard lock(mutex_);
    out.resize(size);
    file_.clear();
    file_.seekg(static_cast<std::streamoff>(offset));
    return static_cast<bool>(file_.read(out.data(), static_cast<std::streamsize>(size)));
}

bool HashFsArchive::loadV1(std::string* error) {
    // Header: magic, u16 version, u16 salt, "CITY", u32 entry count, u32 table offset.
    std::vector<char> hdr;
    if (!readRaw(0, 20, hdr)) return false;
    const auto count = readLe<std::uint32_t>(hdr.data() + 12);
    const auto tableOffset = readLe<std::uint32_t>(hdr.data() + 16);
    std::vector<char> table;
    if (!readRaw(tableOffset, std::size_t{count} * 32, table)) {
        if (error) *error = "truncated v1 entry table";
        return false;
    }
    for (std::uint32_t i = 0; i < count; ++i) {
        const char* e = table.data() + std::size_t{i} * 32;
        Entry entry;
        const auto hash = readLe<std::uint64_t>(e);
        entry.offset = readLe<std::uint64_t>(e + 8);
        const auto flags = readLe<std::uint32_t>(e + 16);
        entry.size = readLe<std::uint32_t>(e + 24);
        entry.compressedSize = readLe<std::uint32_t>(e + 28);
        entry.directory = (flags & 1u) != 0;
        entry.compression = (flags & 2u) ? 1 : 0;
        entries_[hash] = entry;
    }
    return true;
}

bool HashFsArchive::loadV2(std::string* error) {
    std::vector<char> hdr;
    if (!readRaw(0, 52, hdr)) return false;
    const auto entryCount = readLe<std::uint32_t>(hdr.data() + 12);
    const auto entryCompressed = readLe<std::uint32_t>(hdr.data() + 16);
    // Counted in 32-bit words, not bytes (verified against ATS 1.61 base_map.scs).
    const auto metaWords = readLe<std::uint32_t>(hdr.data() + 20);
    const auto metaCompressed = readLe<std::uint32_t>(hdr.data() + 24);
    const auto entryOffset = readLe<std::uint64_t>(hdr.data() + 28);
    const auto metaOffset = readLe<std::uint64_t>(hdr.data() + 36);

    auto loadTable = [&](std::uint64_t off, std::uint32_t compressed, std::size_t size, std::vector<char>& out) {
        std::vector<char> raw;
        if (!readRaw(off, compressed, raw)) return false;
        if (compressed == size) {
            out = std::move(raw);
            return true;
        }
        return inflateZlib(raw.data(), raw.size(), size, out);
    };

    std::vector<char> entryTable;
    std::vector<char> metaTable;
    if (!loadTable(entryOffset, entryCompressed, std::size_t{entryCount} * 16, entryTable) ||
        !loadTable(metaOffset, metaCompressed, std::size_t{metaWords} * 4, metaTable)) {
        if (error) *error = "cannot read v2 tables";
        return false;
    }

    for (std::uint32_t i = 0; i < entryCount; ++i) {
        const char* e = entryTable.data() + std::size_t{i} * 16;
        const auto hash = readLe<std::uint64_t>(e);
        const auto metaIndex = readLe<std::uint32_t>(e + 8);
        const auto metaCount = readLe<std::uint16_t>(e + 12);
        const auto flags = static_cast<std::uint8_t>(e[14]);

        for (std::uint32_t m = 0; m < metaCount; ++m) {
            const std::size_t hdrOff = std::size_t{metaIndex + m} * 4;
            if (hdrOff + 4 > metaTable.size()) break;
            const auto word = readLe<std::uint32_t>(metaTable.data() + hdrOff);
            const std::uint32_t index = word & 0x00FFFFFFu;
            const auto type = static_cast<std::uint8_t>(word >> 24);
            if (type != kMetaPlain && type != kMetaDirectory) continue;
            const std::size_t off = std::size_t{index} * 4;
            if (off + 16 > metaTable.size()) break;
            const auto packed = readLe<std::uint32_t>(metaTable.data() + off);
            Entry entry;
            entry.compressedSize = packed & 0x0FFFFFFFu;
            entry.compression = static_cast<std::uint8_t>(packed >> 28);
            entry.size = readLe<std::uint32_t>(metaTable.data() + off + 4) & 0x0FFFFFFFu;
            entry.offset = std::uint64_t{readLe<std::uint32_t>(metaTable.data() + off + 12)} * 16;
            entry.directory = (flags & 1u) != 0 || type == kMetaDirectory;
            entries_[hash] = entry;
            break;
        }
    }
    return true;
}

bool HashFsArchive::contains(const std::string& path) const {
    std::string p = normalize(path);
    if (salt_ != 0) p = std::to_string(salt_) + p;
    return entries_.count(cityHash64(p)) > 0;
}

std::optional<std::vector<char>> HashFsArchive::readEntry(const Entry& e, std::string* error) const {
    std::vector<char> raw;
    if (!readRaw(e.offset, e.compressedSize, raw)) {
        if (error) *error = "read failed";
        return std::nullopt;
    }
    if (e.compression == 0) {
        raw.resize(e.size);
        return raw;
    }
    if (e.compression == 1) {
        std::vector<char> out;
        if (!inflateZlib(raw.data(), raw.size(), e.size, out)) {
            if (error) *error = "zlib inflate failed";
            return std::nullopt;
        }
        return out;
    }
    if (error) *error = "unsupported compression " + std::to_string(e.compression);
    return std::nullopt;
}

std::optional<std::vector<char>> HashFsArchive::read(const std::string& path, std::string* error) const {
    std::string p = normalize(path);
    if (salt_ != 0) p = std::to_string(salt_) + p;
    const auto it = entries_.find(cityHash64(p));
    if (it == entries_.end()) {
        if (error) *error = "not found";
        return std::nullopt;
    }
    return readEntry(it->second, error);
}

std::optional<DirectoryListing> HashFsArchive::list(const std::string& dir) const {
    std::string p = normalize(dir);
    if (salt_ != 0) p = std::to_string(salt_) + p;
    const auto it = entries_.find(cityHash64(p));
    if (it == entries_.end() || !it->second.directory) return std::nullopt;
    const auto data = readEntry(it->second, nullptr);
    if (!data) return std::nullopt;

    DirectoryListing out;
    if (version_ == 1) {
        // Newline-separated names; subdirectories are prefixed with '*'.
        std::string text(data->begin(), data->end());
        std::size_t start = 0;
        while (start < text.size()) {
            std::size_t end = text.find('\n', start);
            if (end == std::string::npos) end = text.size();
            std::string name = text.substr(start, end - start);
            if (!name.empty() && name.back() == '\r') name.pop_back();
            if (!name.empty()) {
                if (name.front() == '*') out.subdirectories.push_back(name.substr(1));
                else out.files.push_back(name);
            }
            start = end + 1;
        }
        return out;
    }

    // v2: u32 count, count u8 lengths, then the strings; '/'-prefixed names are directories.
    const char* d = data->data();
    const std::size_t n = data->size();
    if (n < 4) return out;
    const auto count = readLe<std::uint32_t>(d);
    std::size_t pos = 4 + count;
    if (pos > n) return out;
    for (std::uint32_t i = 0; i < count; ++i) {
        const auto len = static_cast<std::uint8_t>(d[4 + i]);
        if (pos + len > n) break;
        std::string name(d + pos, len);
        pos += len;
        if (!name.empty() && name.front() == '/') out.subdirectories.push_back(name.substr(1));
        else out.files.push_back(name);
    }
    return out;
}

std::optional<std::vector<char>> GameFileSystem::read(const std::string& path) const {
    for (auto it = archives_.rbegin(); it != archives_.rend(); ++it) {
        if ((*it)->contains(path)) {
            if (auto d = (*it)->read(path)) return d;
        }
    }
    return std::nullopt;
}

bool GameFileSystem::contains(const std::string& path) const {
    return std::any_of(archives_.begin(), archives_.end(), [&](const auto& a) { return a->contains(path); });
}

DirectoryListing GameFileSystem::list(const std::string& dir) const {
    std::set<std::string> dirs;
    std::set<std::string> files;
    for (const auto& a : archives_) {
        if (const auto l = a->list(dir)) {
            dirs.insert(l->subdirectories.begin(), l->subdirectories.end());
            files.insert(l->files.begin(), l->files.end());
        }
    }
    return {{dirs.begin(), dirs.end()}, {files.begin(), files.end()}};
}

}  // namespace atspilot

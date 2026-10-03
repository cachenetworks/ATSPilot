#pragma once

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace atspilot {

struct DirectoryListing {
    std::vector<std::string> subdirectories;
    std::vector<std::string> files;
};

// Reader for SCS HashFS archives (.scs), versions 1 and 2. Format notes and
// sources are in docs/map-parsing.md. Only plain and zlib-compressed entries are
// supported; GDeflate is used by SCS for texture data, which ATSPilot never reads.
class HashFsArchive {
public:
    static std::unique_ptr<HashFsArchive> open(const std::filesystem::path& file, std::string* error = nullptr);

    const std::filesystem::path& path() const { return path_; }
    int version() const { return version_; }
    std::size_t entryCount() const { return entries_.size(); }

    bool contains(const std::string& path) const;
    std::optional<std::vector<char>> read(const std::string& path, std::string* error = nullptr) const;
    std::optional<DirectoryListing> list(const std::string& dir) const;

    static std::string normalize(const std::string& path);

private:
    struct Entry {
        std::uint64_t offset = 0;
        std::uint32_t size = 0;
        std::uint32_t compressedSize = 0;
        std::uint8_t compression = 0;  // 0 none, 1 zlib, 3 gdeflate
        bool directory = false;
    };

    bool loadV1(std::string* error);
    bool loadV2(std::string* error);
    bool readRaw(std::uint64_t offset, std::size_t size, std::vector<char>& out) const;
    std::optional<std::vector<char>> readEntry(const Entry& e, std::string* error) const;

    std::filesystem::path path_;
    int version_ = 0;
    std::uint16_t salt_ = 0;
    mutable std::ifstream file_;
    mutable std::mutex mutex_;
    std::unordered_map<std::uint64_t, Entry> entries_;
};

// Layered view over several archives: later archives override earlier ones,
// matching how the game applies DLC and mods on top of base data.
class GameFileSystem {
public:
    void add(std::unique_ptr<HashFsArchive> archive) { archives_.push_back(std::move(archive)); }

    std::optional<std::vector<char>> read(const std::string& path) const;
    // Union of all layers' listings.
    DirectoryListing list(const std::string& dir) const;
    bool contains(const std::string& path) const;

    const std::vector<std::unique_ptr<HashFsArchive>>& archives() const { return archives_; }

private:
    std::vector<std::unique_ptr<HashFsArchive>> archives_;
};

bool inflateZlib(const char* data, std::size_t size, std::size_t expected, std::vector<char>& out);

}  // namespace atspilot

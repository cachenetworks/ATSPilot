#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "math/Vec.h"

namespace atspilot {

struct MapNode {
    std::uint64_t uid = 0;
    Vec3 position;        // world space, metres
    Quat rotation;
    std::uint64_t backwardItem = 0;
    std::uint64_t forwardItem = 0;

    // The node's forward direction: vehicle-style -Z rotated by the quaternion.
    Vec3 forward() const { return rotation.rotate({0.0, 0.0, -1.0}); }
};

struct MapRoad {
    std::uint64_t uid = 0;
    std::uint64_t roadLook = 0;  // token, matches road_look unit "road.<token>"
    std::uint64_t startNode = 0;
    std::uint64_t endNode = 0;
    float length = 0.0f;
    std::uint32_t flags = 0;
};

struct MapPrefab {
    std::uint64_t uid = 0;
    std::uint64_t model = 0;    // token, matches prefab_model unit "prefab.<token>"
    std::uint64_t variant = 0;
    std::vector<std::uint64_t> nodes;
    std::uint16_t originIndex = 0;
    std::uint32_t flags = 0;
    std::uint64_t ferryLink = 0;
};

struct SectorData {
    std::uint32_t version = 0;
    std::vector<MapNode> nodes;
    std::vector<MapRoad> roads;
    std::vector<MapPrefab> prefabs;
    std::size_t itemCount = 0;
};

inline constexpr std::uint32_t kSupportedSectorVersion = 907;

// Parses a map sector (.base). Throws ParseError on malformed or unsupported data.
// Every item type must be decoded to find the next one, because items carry no
// size prefix; see docs/map-parsing.md for the layouts.
SectorData parseSector(const char* data, std::size_t size);

}  // namespace atspilot

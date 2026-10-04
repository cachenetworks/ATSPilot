#pragma once

#include <cstdint>
#include <vector>

#include "math/Vec.h"

namespace atspilot {

struct PrefabNodeDesc {
    Vec3 position;   // prefab-local
    Vec3 direction;  // prefab-local direction vector
    std::vector<int> inputLanes;   // nav curve indices entering the prefab here
    std::vector<int> outputLanes;  // nav curve indices leaving the prefab here
};

struct PrefabNavCurve {
    std::uint32_t flags = 0;
    Vec3 startPos;
    Vec3 endPos;
    Quat startRot;
    Quat endRot;
    float length = 0.0f;
    std::vector<int> next;
    std::vector<int> prev;
    std::int32_t semaphoreId = -1;    // >= 0: the curve starts at a traffic-light stop line
    std::uint64_t trafficRule = 0;    // token of a traffic_rule unit, e.g. "stop", "give_way"
};

// Spawn point types used by ATSPilot (the ppd's spawn point type field).
namespace SpawnType {
inline constexpr std::uint32_t Gas = 3;            // a fuel pump stand
inline constexpr std::uint32_t WeighStation = 6;   // the scale of a weigh station
}  // namespace SpawnType

struct PrefabSpawnPoint {
    Vec3 position;  // prefab-local
    Quat rotation;
    std::uint32_t type = 0;
};

struct PrefabDescription {
    std::uint32_t version = 0;
    std::vector<PrefabNodeDesc> nodes;
    std::vector<PrefabNavCurve> curves;
    std::vector<PrefabSpawnPoint> spawnPoints;
};

inline constexpr std::uint32_t kSupportedPpdVersion = 25;

// Parses the parts of a compiled prefab descriptor (.ppd) needed for driving:
// nodes, AI navigation curves and spawn points. Throws ParseError on malformed data.
PrefabDescription parsePrefabDescription(const char* data, std::size_t size);

}  // namespace atspilot

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
};

struct PrefabDescription {
    std::uint32_t version = 0;
    std::vector<PrefabNodeDesc> nodes;
    std::vector<PrefabNavCurve> curves;
};

inline constexpr std::uint32_t kSupportedPpdVersion = 25;

// Parses the parts of a compiled prefab descriptor (.ppd) needed for driving:
// nodes and AI navigation curves. Throws ParseError on malformed data.
PrefabDescription parsePrefabDescription(const char* data, std::size_t size);

}  // namespace atspilot

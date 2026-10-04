#include "map/PrefabDescription.h"

#include <string>

#include "map/BinaryReader.h"

namespace atspilot {
namespace {

Quat readQuat(BinaryReader& r) {
    const float w = r.f32(), x = r.f32(), y = r.f32(), z = r.f32();
    return {w, x, y, z};
}

}  // namespace

PrefabDescription parsePrefabDescription(const char* data, std::size_t size) {
    BinaryReader r(data, size);
    PrefabDescription d;
    d.version = r.u32();
    if (d.version != kSupportedPpdVersion) {
        throw ParseError("unsupported ppd version " + std::to_string(d.version));
    }
    const std::uint32_t nodeCount = r.u32();
    const std::uint32_t curveCount = r.u32();
    r.skip(2 * 4);  // signs, semaphores
    const std::uint32_t spawnCount = r.u32();
    r.skip(6 * 4);  // terrain points/variants, map points, triggers, intersections, nav nodes
    const std::uint32_t nodeOffset = r.u32();
    const std::uint32_t curveOffset = r.u32();
    r.skip(2 * 4);  // sign, semaphore offsets
    const std::uint32_t spawnOffset = r.u32();

    constexpr std::size_t kNodeSize = 104;
    constexpr std::size_t kCurveSize = 132;
    constexpr std::size_t kSpawnSize = 36;
    if (std::size_t{nodeOffset} + std::size_t{nodeCount} * kNodeSize > size ||
        std::size_t{curveOffset} + std::size_t{curveCount} * kCurveSize > size ||
        std::size_t{spawnOffset} + std::size_t{spawnCount} * kSpawnSize > size) {
        throw ParseError("ppd tables exceed file size");
    }

    r.seek(nodeOffset);
    d.nodes.resize(nodeCount);
    for (auto& n : d.nodes) {
        r.skip(16);  // terrain point/variant indices
        n.position = r.vec3f();
        n.direction = r.vec3f();
        for (int i = 0; i < 8; ++i) {
            const std::int32_t v = r.i32();
            if (v >= 0) n.inputLanes.push_back(v);
        }
        for (int i = 0; i < 8; ++i) {
            const std::int32_t v = r.i32();
            if (v >= 0) n.outputLanes.push_back(v);
        }
    }

    r.seek(curveOffset);
    d.curves.resize(curveCount);
    for (auto& c : d.curves) {
        r.skip(8);  // name token
        c.flags = r.u32();
        r.skip(4);  // leads-to-nodes bitfield
        c.startPos = r.vec3f();
        c.endPos = r.vec3f();
        c.startRot = readQuat(r);
        c.endRot = readQuat(r);
        c.length = r.f32();
        std::int32_t next[4], prev[4];
        for (auto& v : next) v = r.i32();
        for (auto& v : prev) v = r.i32();
        const std::uint32_t countNext = r.u32();
        const std::uint32_t countPrev = r.u32();
        for (std::uint32_t i = 0; i < countNext && i < 4; ++i) {
            if (next[i] >= 0 && static_cast<std::uint32_t>(next[i]) < curveCount) c.next.push_back(next[i]);
        }
        for (std::uint32_t i = 0; i < countPrev && i < 4; ++i) {
            if (prev[i] >= 0 && static_cast<std::uint32_t>(prev[i]) < curveCount) c.prev.push_back(prev[i]);
        }
        c.semaphoreId = r.i32();
        c.trafficRule = r.u64();
        r.skip(4);  // nav node index
    }

    r.seek(spawnOffset);
    d.spawnPoints.resize(spawnCount);
    for (auto& sp : d.spawnPoints) {
        sp.position = r.vec3f();
        sp.rotation = readQuat(r);
        sp.type = r.u32();
        r.skip(4);  // flags
    }
    return d;
}

}  // namespace atspilot

#include "map/SectorParser.h"

#include <bit>

#include "map/BinaryReader.h"

namespace atspilot {
namespace {

// Item type ids as stored in .base files (sector version 907, ATS 1.5x-1.61).
enum ItemType : std::uint32_t {
    Terrain = 1,
    Building = 2,
    Road = 3,
    Prefab = 4,
    Model = 5,
    Company = 6,
    Service = 7,
    CutPlane = 8,
    Mover = 9,
    NoWeather = 11,
    City = 12,
    Hinge = 13,
    MapOverlay = 18,
    Ferry = 19,
    Garage = 22,
    CameraPoint = 23,
    Trigger = 34,
    FuelPump = 35,
    Sign = 36,
    BusStop = 37,
    TrafficRule = 38,
    BezierPatch = 39,
    Compound = 40,
    TrajectoryItem = 41,
    MapArea = 42,
    FarModel = 43,
    Curve = 44,
    CameraPath = 45,
    Cutscene = 46,
    Hookup = 47,
    VisibilityArea = 48,
    Gate = 49,
};

constexpr std::size_t kTok = 8;
constexpr std::size_t kU64 = 8;
constexpr std::size_t kNodeSize = 56;

struct ItemHeader {
    std::uint64_t uid = 0;
    std::uint32_t flags = 0;
};

ItemHeader readHeader(BinaryReader& r) {
    ItemHeader h;
    h.uid = r.u64();
    r.skip(12 + 12 + 12 + 4);  // bounding data and padding
    h.flags = r.u32();
    r.skip(1);  // view distance
    return h;
}

void skipQuadInfo(BinaryReader& r) {
    const std::uint16_t materials = r.u16();
    r.skip(std::size_t{materials} * (kTok + 2));
    const std::uint16_t colors = r.u16();
    r.skip(std::size_t{colors} * 4);
    r.skip(4);  // size x, size y
    r.skipArray32(4);   // storage
    r.skipArray32(16);  // offsets
    r.skipArray32(16);  // normals
}

void skipVegetationSpheres(BinaryReader& r) { r.skipArray32(20); }

MapNode readNode(BinaryReader& r) {
    MapNode n;
    n.uid = r.u64();
    const std::int32_t x = r.i32(), y = r.i32(), z = r.i32();
    // Node positions are fixed point with 8 fractional bits.
    n.position = {x / 256.0, y / 256.0, z / 256.0};
    const float qw = r.f32(), qx = r.f32(), qy = r.f32(), qz = r.f32();
    n.rotation = {qw, qx, qy, qz};
    n.backwardItem = r.u64();
    n.forwardItem = r.u64();
    r.skip(4);  // flags
    return n;
}

void skipTrigger(BinaryReader& r) {
    r.skipArray32(kTok);                       // tags
    const std::uint32_t nodes = r.skipArray32(kU64);
    const std::uint32_t actions = r.u32();
    for (std::uint32_t i = 0; i < actions; ++i) {
        r.skip(kTok);
        const std::int32_t hasOverride = r.i32();
        if (hasOverride >= 0) {
            r.skip(std::size_t(hasOverride) * 4);
            const std::uint32_t params = r.u32();
            for (std::uint32_t p = 0; p < params; ++p) r.string64();
            r.skipArray32(kU64);
            r.skip(4 + 4);
        }
    }
    if (nodes == 1) r.skip(4);  // radius
}

void skipSign(BinaryReader& r) {
    r.skip(kTok + kU64 + kTok + kTok);
    const std::uint8_t boards = r.u8();
    r.skip(std::size_t{boards} * 3 * kTok);
    const std::string overrideTemplate = r.string64();
    if (overrideTemplate.empty()) return;
    const std::uint32_t overrideBoards = r.u32();
    for (std::uint32_t i = 0; i < overrideBoards; ++i) {
        r.skip(kTok);
        const std::uint8_t flags = r.u8();
        if (flags & 0x01) r.skip(2);
        if (flags & 0x02) r.skip(kTok);
    }
    const std::uint32_t items = r.u32();
    for (std::uint32_t i = 0; i < items; ++i) {
        r.skip(4 + kTok);
        const std::uint32_t attrs = r.u32();
        for (std::uint32_t a = 0; a < attrs; ++a) {
            const std::uint16_t type = r.u16();
            r.skip(4);  // index
            switch (type) {
                case 1: r.skip(1); break;
                case 2:
                case 3:
                case 4: r.skip(4); break;
                case 5: r.string64(); break;
                case 6: r.skip(8); break;
                default: throw ParseError("unknown sign attribute type " + std::to_string(type));
            }
        }
    }
}

void skipCurve(BinaryReader& r) {
    r.skip(kU64 * 4);  // start, end, two padding uids
    r.skip(4);          // length
    const std::uint32_t mask = r.u32();
    const int subcurves = std::popcount(mask);
    for (int i = 0; i < subcurves; ++i) {
        r.skip(kTok + 4 + 4 + 4 + 4 + 4 + kTok + 4 + 4 + kTok + kTok + kTok + kTok);
        r.skipArray32(4);
        r.skip(4 * 5);
    }
}

void skipCutscene(BinaryReader& r) {
    r.skipArray32(kTok);
    r.skip(kU64);
    const std::uint32_t actions = r.u32();
    for (std::uint32_t i = 0; i < actions; ++i) {
        r.skipArray32(4);
        const std::uint32_t strings = r.u32();
        for (std::uint32_t s = 0; s < strings; ++s) r.string64();
        r.skipArray32(kU64);
        r.skip(4 + 4);
    }
}

// Reads one item payload. Roads and prefabs are kept; everything else is skipped.
void readItem(BinaryReader& r, std::uint32_t type, SectorData& out, bool allowCompound);

void readCompound(BinaryReader& r, SectorData& out) {
    r.skip(kU64);
    const std::uint32_t children = r.u32();
    for (std::uint32_t i = 0; i < children; ++i) {
        const std::uint32_t type = r.u32();
        readItem(r, type, out, false);
    }
    const std::uint32_t nodes = r.u32();
    for (std::uint32_t i = 0; i < nodes; ++i) out.nodes.push_back(readNode(r));
}

void readItem(BinaryReader& r, std::uint32_t type, SectorData& out, bool allowCompound) {
    const ItemHeader h = readHeader(r);
    switch (type) {
        case Terrain:
            r.skip(kU64 * 2 + 12 * 2 + 4 * 2 + 4);
            r.skip(3 * (kTok + 2));
            r.skip(2 * (2 + kTok + 4 + kTok + 4 + 3 * (kTok + 2 + 1 + 1 + 2 + 2) + 2 + 2));
            skipVegetationSpheres(r);
            skipQuadInfo(r);
            skipQuadInfo(r);
            r.skip(4 * kTok);
            break;
        case Building:
            r.skip(kTok * 2 + kU64 * 2 + 4 * 3);
            r.skipArray32(4);
            break;
        case Road: {
            MapRoad road;
            road.uid = h.uid;
            road.flags = h.flags;
            r.skip(4);  // flags1, dlc guard, flags2
            road.roadLook = r.u64();
            r.skip(8 * kTok);                 // lane variants, template variants, edges
            r.skip(2 * (kTok + 4));           // profiles + coefficients
            r.skip(3 * kTok);                 // template looks, material
            r.skip(3 * (kTok + 2 + kTok + 2));  // railings
            r.skip(4 + 4);                    // road heights
            road.startNode = r.u64();
            road.endNode = r.u64();
            road.length = r.f32();
            out.roads.push_back(road);
            break;
        }
        case Prefab: {
            MapPrefab p;
            p.uid = h.uid;
            p.flags = h.flags;
            p.model = r.u64();
            p.variant = r.u64();
            r.skipArray32(kTok);  // additional parts
            const std::uint32_t nodeCount = r.u32();
            p.nodes.reserve(nodeCount);
            for (std::uint32_t i = 0; i < nodeCount; ++i) p.nodes.push_back(r.u64());
            r.skipArray32(kU64);  // connected items
            p.ferryLink = r.u64();
            p.originIndex = r.u16();
            r.skip(std::size_t{nodeCount} * (kTok + 4));  // terrain info per node
            r.skip(kTok);                                 // semaphore profile
            out.prefabs.push_back(std::move(p));
            break;
        }
        case Model:
            r.skip(3 * kTok);
            r.skipArray32(kTok);
            r.skip(kU64 + 12 + kTok + 4 + 4);
            break;
        case Company:
            r.skip(kTok + kU64 + kTok + kU64);
            r.skipArray32(kU64 + 4);
            break;
        case Service:
        case FuelPump:
            r.skip(kU64 * 2);
            r.skipArray32(kU64);
            break;
        case CutPlane:
        case MapArea:
            r.skipArray32(kU64);
            if (type == MapArea) r.skip(4);
            break;
        case Mover:
            r.skipArray32(kTok);
            r.skip(3 * kTok + 4 * 4);
            r.skipArray32(4);
            r.skipArray32(kU64);
            break;
        case NoWeather:
            r.skip(4 * 3 + 16 + kU64);
            break;
        case City:
            r.skip(kTok + 4 + 4 + kU64);
            break;
        case Hinge:
            r.skip(kTok * 2 + kU64 + 4 + 4);
            break;
        case MapOverlay:
            r.skip(kTok + kU64);
            break;
        case Ferry:
            r.skip(kTok + kU64 * 2 + 12);
            break;
        case Garage:
            r.skip(kTok + 4 + kU64 * 2);
            r.skipArray32(kU64);
            break;
        case CameraPoint:
            r.skipArray32(kTok);
            r.skip(kU64);
            break;
        case Trigger:
            skipTrigger(r);
            break;
        case Sign:
            skipSign(r);
            break;
        case BusStop:
            r.skip(kTok + kU64 * 2);
            break;
        case TrafficRule:
            r.skipArray32(kTok);
            r.skipArray32(kU64);
            r.skip(kTok + 4);
            break;
        case BezierPatch:
            r.skip(16 * 12 + 2 + 2 + kU64 + 4);
            r.skip(3 * (kTok + 2 + 1));
            skipVegetationSpheres(r);
            skipQuadInfo(r);
            break;
        case Compound:
            if (!allowCompound) throw ParseError("nested compound item");
            readCompound(r, out);
            break;
        case TrajectoryItem:
            r.skipArray32(kU64);
            r.skip(8);
            r.skipArray32(4 + kTok + 4 + 12);
            r.skipArray32(kTok * 2);
            r.skipArray32(kTok);
            break;
        case FarModel:
            r.skip(4 + 4);
            r.skipArray32(kTok + 12);
            r.skipArray32(kU64);
            r.skipArray32(kU64);
            break;
        case Curve:
            skipCurve(r);
            break;
        case CameraPath:
            r.skipArray32(kTok);
            r.skipArray32(kU64);
            r.skipArray32(kU64);
            r.skipArray32(kU64);
            r.skipArray32(40);
            r.skip(4);
            break;
        case Cutscene:
            skipCutscene(r);
            break;
        case Hookup:
            r.string64();
            r.skip(kU64);
            break;
        case VisibilityArea:
            r.skip(kU64 + 4 + 4);
            r.skipArray32(kU64);
            break;
        case Gate:
            r.skip(kTok);
            r.skipArray32(kU64);
            for (int i = 0; i < 2; ++i) {
                r.string64();
                r.skip(4);
            }
            break;
        default:
            throw ParseError("unknown item type " + std::to_string(type));
    }
}

}  // namespace

SectorData parseSector(const char* data, std::size_t size) {
    BinaryReader r(data, size);
    SectorData out;
    out.version = r.u32();
    if (out.version != kSupportedSectorVersion) {
        throw ParseError("unsupported sector version " + std::to_string(out.version));
    }
    r.skip(kTok + 4);  // game id, padding
    const std::uint32_t items = r.u32();
    out.itemCount = items;
    for (std::uint32_t i = 0; i < items; ++i) {
        const std::uint32_t type = r.u32();
        readItem(r, type, out, true);
    }
    const std::uint32_t nodes = r.u32();
    out.nodes.reserve(out.nodes.size() + nodes);
    for (std::uint32_t i = 0; i < nodes; ++i) out.nodes.push_back(readNode(r));
    r.skipArray32(kU64);  // visible-area child uids
    if (r.remaining() != 0) throw ParseError("trailing bytes after sector data");
    return out;
}

}  // namespace atspilot

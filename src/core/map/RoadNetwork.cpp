#include "map/RoadNetwork.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>

#include "math/MathUtil.h"

namespace atspilot {

std::uint32_t RoadNetwork::add(LaneSegment seg) {
    float len = 0.0f;
    for (std::size_t i = 1; i < seg.points.size(); ++i) {
        len += static_cast<float>(distance(seg.points[i - 1].plan(), seg.points[i].plan()));
    }
    seg.length = len;
    segments_.push_back(std::move(seg));
    return static_cast<std::uint32_t>(segments_.size() - 1);
}

std::vector<std::uint32_t> RoadNetwork::laneNeighbors(std::uint32_t id) const {
    std::vector<std::uint32_t> out;
    const auto& seg = segments_[id];
    if (seg.kind != LaneKind::Road) return out;
    const auto it = roadLanes_.find(seg.itemUid);
    if (it == roadLanes_.end()) return out;
    for (auto other : it->second) {
        const auto& o = segments_[other];
        if (other == id || o.leftSide != seg.leftSide) continue;
        if (o.laneIndex + 1 == seg.laneIndex || seg.laneIndex + 1 == o.laneIndex) out.push_back(other);
    }
    return out;
}

void RoadNetwork::finalize() {
    grid_.clear();
    roadLanes_.clear();
    for (std::uint32_t id = 0; id < segments_.size(); ++id) {
        if (segments_[id].kind == LaneKind::Road) roadLanes_[segments_[id].itemUid].push_back(id);
    }
    for (auto& s : segments_) {
        s.prev.clear();
        float len = 0.0f;
        for (std::size_t i = 1; i < s.points.size(); ++i) {
            len += static_cast<float>(distance(s.points[i - 1].plan(), s.points[i].plan()));
        }
        s.length = len;
    }
    for (std::uint32_t id = 0; id < segments_.size(); ++id) {
        auto& seg = segments_[id];
        for (std::uint32_t n : seg.next) {
            if (n < segments_.size()) segments_[n].prev.push_back(id);
        }
        // Register every cell the polyline passes through, sampling each
        // edge finely enough that no crossed cell is skipped.
        std::uint64_t lastKey = std::numeric_limits<std::uint64_t>::max();
        for (std::size_t i = 0; i + 1 < seg.points.size() || (i == 0 && seg.points.size() == 1); ++i) {
            const Vec2 a = seg.points[i].plan();
            const Vec2 b = seg.points.size() > 1 ? seg.points[i + 1].plan() : a;
            const int steps = std::max(1, static_cast<int>(std::ceil(distance(a, b) / (kCellSize * 0.5))));
            for (int k = 0; k <= steps; ++k) {
                const Vec2 p = lerp(a, b, static_cast<double>(k) / steps);
                const auto key = cellKey(static_cast<std::int32_t>(std::floor(p.x / kCellSize)),
                                         static_cast<std::int32_t>(std::floor(p.y / kCellSize)));
                if (key == lastKey) continue;
                lastKey = key;
                auto& cell = grid_[key];
                if (cell.empty() || cell.back() != id) cell.push_back(id);
            }
        }
    }
    for (auto& [k, v] : grid_) {
        std::sort(v.begin(), v.end());
        v.erase(std::unique(v.begin(), v.end()), v.end());
    }
}

LaneMatch RoadNetwork::project(std::uint32_t id, const Vec2& p) const {
    const auto& seg = segments_[id];
    LaneMatch best;
    best.segment = id;
    best.distance = std::numeric_limits<double>::max();
    double s = 0.0;
    for (std::size_t i = 0; i + 1 < seg.points.size(); ++i) {
        const Vec2 a = seg.points[i].plan();
        const Vec2 b = seg.points[i + 1].plan();
        const double t = projectOntoSegment(p, a, b);
        const Vec2 q = lerp(a, b, t);
        const double d = distance(p, q);
        const double segLen = distance(a, b);
        if (d < best.distance) {
            const Vec2 dir = (b - a).normalized();
            best.pointIndex = i;
            best.t = t;
            best.distance = d;
            best.crossTrack = cross(dir, p - q);
            best.yaw = std::atan2(dir.y, dir.x);
            best.s = s + t * segLen;
        }
        s += segLen;
    }
    return best;
}

std::vector<LaneMatch> RoadNetwork::query(const Vec2& p, double radius) const {
    std::vector<std::uint32_t> candidates;
    const auto x0 = static_cast<std::int32_t>(std::floor((p.x - radius) / kCellSize));
    const auto x1 = static_cast<std::int32_t>(std::floor((p.x + radius) / kCellSize));
    const auto y0 = static_cast<std::int32_t>(std::floor((p.y - radius) / kCellSize));
    const auto y1 = static_cast<std::int32_t>(std::floor((p.y + radius) / kCellSize));
    for (std::int32_t ix = x0; ix <= x1; ++ix) {
        for (std::int32_t iy = y0; iy <= y1; ++iy) {
            const auto it = grid_.find(cellKey(ix, iy));
            if (it != grid_.end()) candidates.insert(candidates.end(), it->second.begin(), it->second.end());
        }
    }
    std::sort(candidates.begin(), candidates.end());
    candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());

    std::vector<LaneMatch> out;
    for (std::uint32_t id : candidates) {
        if (segments_[id].points.size() < 2) continue;
        const LaneMatch m = project(id, p);
        if (m.distance <= radius) out.push_back(m);
    }
    std::sort(out.begin(), out.end(), [](const LaneMatch& a, const LaneMatch& b) { return a.distance < b.distance; });
    return out;
}

const Destination* RoadNetwork::findDestination(std::uint64_t city, std::uint64_t company) const {
    for (const auto& d : destinations_) {
        if (d.city == city && d.company == company) return &d;
    }
    return nullptr;
}

namespace {

constexpr char kMagic[8] = {'A', 'T', 'S', 'P', 'M', 'A', 'P', '\0'};

template <typename T>
void put(std::ofstream& f, const T& v) {
    f.write(reinterpret_cast<const char*>(&v), sizeof(T));
}

template <typename T>
bool get(std::ifstream& f, T& v) {
    return static_cast<bool>(f.read(reinterpret_cast<char*>(&v), sizeof(T)));
}

}  // namespace

void RoadNetwork::setNodes(std::vector<NodePoint> nodes) {
    std::sort(nodes.begin(), nodes.end(), [](const NodePoint& a, const NodePoint& b) { return a.uid < b.uid; });
    nodes_ = std::move(nodes);
}

std::optional<Vec2> RoadNetwork::nodePosition(std::uint64_t uid) const {
    const auto it = std::lower_bound(nodes_.begin(), nodes_.end(), uid,
                                     [](const NodePoint& n, std::uint64_t u) { return n.uid < u; });
    if (it == nodes_.end() || it->uid != uid) return std::nullopt;
    return Vec2{it->x, it->y};
}

bool RoadNetwork::save(const std::filesystem::path& file, const std::string& fingerprint) const {
    std::error_code ec;
    std::filesystem::create_directories(file.parent_path(), ec);
    const auto tmp = file.string() + ".tmp";
    {
        std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
        if (!f) return false;
        f.write(kMagic, sizeof(kMagic));
        put(f, kFormatVersion);
        put(f, static_cast<std::uint32_t>(fingerprint.size()));
        f.write(fingerprint.data(), static_cast<std::streamsize>(fingerprint.size()));
        put(f, static_cast<std::uint32_t>(segments_.size()));
        for (const auto& s : segments_) {
            put(f, s.itemUid);
            put(f, static_cast<std::uint8_t>(s.kind));
            put(f, s.laneIndex);
            put(f, static_cast<std::uint8_t>((s.leftSide ? 1 : 0) | (s.rules << 1)));
            put(f, s.semaphoreId);
            put(f, static_cast<std::uint32_t>(s.points.size()));
            f.write(reinterpret_cast<const char*>(s.points.data()),
                    static_cast<std::streamsize>(s.points.size() * sizeof(LanePoint)));
            put(f, static_cast<std::uint32_t>(s.next.size()));
            f.write(reinterpret_cast<const char*>(s.next.data()),
                    static_cast<std::streamsize>(s.next.size() * sizeof(std::uint32_t)));
        }
        put(f, static_cast<std::uint32_t>(destinations_.size()));
        for (const auto& d : destinations_) {
            put(f, d.city);
            put(f, d.company);
            put(f, d.position.x);
            put(f, d.position.y);
            put(f, static_cast<std::uint32_t>(d.lanes.size()));
            f.write(reinterpret_cast<const char*>(d.lanes.data()),
                    static_cast<std::streamsize>(d.lanes.size() * sizeof(std::uint32_t)));
        }
        put(f, static_cast<std::uint32_t>(nodes_.size()));
        f.write(reinterpret_cast<const char*>(nodes_.data()),
                static_cast<std::streamsize>(nodes_.size() * sizeof(NodePoint)));
        if (!f) return false;
    }
    std::filesystem::rename(tmp, file, ec);
    return !ec;
}

std::optional<RoadNetwork> RoadNetwork::load(const std::filesystem::path& file, const std::string& expectedFingerprint,
                                             std::string* error) {
    std::ifstream f(file, std::ios::binary);
    if (!f) {
        if (error) *error = "no cache";
        return std::nullopt;
    }
    char magic[8];
    std::uint32_t version = 0, fpLen = 0, count = 0;
    if (!f.read(magic, sizeof(magic)) || std::memcmp(magic, kMagic, sizeof(kMagic)) != 0 || !get(f, version) ||
        version != kFormatVersion || !get(f, fpLen) || fpLen > 1 << 20) {
        if (error) *error = "cache format mismatch";
        return std::nullopt;
    }
    std::string fp(fpLen, '\0');
    if (!f.read(fp.data(), fpLen) || fp != expectedFingerprint) {
        if (error) *error = "cache built for a different game/map configuration";
        return std::nullopt;
    }
    if (!get(f, count)) return std::nullopt;

    RoadNetwork net;
    net.segments_.resize(count);
    for (auto& s : net.segments_) {
        std::uint8_t kind = 0, left = 0;
        std::uint32_t pts = 0, nexts = 0;
        if (!get(f, s.itemUid) || !get(f, kind) || !get(f, s.laneIndex) || !get(f, left) || !get(f, s.semaphoreId) || !get(f, pts) ||
            pts > 100000) {
            if (error) *error = "truncated cache";
            return std::nullopt;
        }
        s.kind = static_cast<LaneKind>(kind);
        s.leftSide = (left & 1) != 0;
        s.rules = static_cast<std::uint8_t>(left >> 1);
        s.points.resize(pts);
        if (!f.read(reinterpret_cast<char*>(s.points.data()), static_cast<std::streamsize>(pts * sizeof(LanePoint))) ||
            !get(f, nexts) || nexts > 64) {
            if (error) *error = "truncated cache";
            return std::nullopt;
        }
        s.next.resize(nexts);
        if (!f.read(reinterpret_cast<char*>(s.next.data()), static_cast<std::streamsize>(nexts * sizeof(std::uint32_t)))) {
            if (error) *error = "truncated cache";
            return std::nullopt;
        }
        float len = 0.0f;
        for (std::size_t i = 1; i < s.points.size(); ++i) {
            len += static_cast<float>(distance(s.points[i - 1].plan(), s.points[i].plan()));
        }
        s.length = len;
    }
    std::uint32_t destCount = 0;
    if (!get(f, destCount) || destCount > 1000000) {
        if (error) *error = "truncated cache";
        return std::nullopt;
    }
    net.destinations_.resize(destCount);
    for (auto& d : net.destinations_) {
        std::uint32_t lanes = 0;
        if (!get(f, d.city) || !get(f, d.company) || !get(f, d.position.x) || !get(f, d.position.y) ||
            !get(f, lanes) || lanes > 100000) {
            if (error) *error = "truncated cache";
            return std::nullopt;
        }
        d.lanes.resize(lanes);
        if (!f.read(reinterpret_cast<char*>(d.lanes.data()), static_cast<std::streamsize>(lanes * sizeof(std::uint32_t)))) {
            if (error) *error = "truncated cache";
            return std::nullopt;
        }
        for (auto l : d.lanes) {
            if (l >= count) {
                if (error) *error = "corrupt cache";
                return std::nullopt;
            }
        }
    }
    std::uint32_t nodeCount = 0;
    if (!get(f, nodeCount) || nodeCount > 50000000) {
        if (error) *error = "truncated cache";
        return std::nullopt;
    }
    net.nodes_.resize(nodeCount);
    if (!f.read(reinterpret_cast<char*>(net.nodes_.data()), static_cast<std::streamsize>(nodeCount * sizeof(NodePoint)))) {
        if (error) *error = "truncated cache";
        return std::nullopt;
    }
    for (const auto& s : net.segments_) {
        for (auto n : s.next) {
            if (n >= count) {
                if (error) *error = "corrupt cache";
                return std::nullopt;
            }
        }
    }
    net.finalize();
    return net;
}

}  // namespace atspilot

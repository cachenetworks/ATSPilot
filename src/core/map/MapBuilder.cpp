#include "map/MapBuilder.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <unordered_map>

#include "map/BinaryReader.h"
#include "map/HashFs.h"
#include "map/PrefabDescription.h"
#include "map/SectorParser.h"
#include "map/Sii.h"
#include "map/Token.h"
#include "math/Coordinates.h"
#include "math/MathUtil.h"

namespace atspilot {
namespace {

struct RoadLookInfo {
    int lanesLeft = 1;
    int lanesRight = 1;
    double roadOffset = 0.0;
    std::vector<double> laneOffsetsLeft;
    std::vector<double> laneOffsetsRight;
};

struct CurveSample {
    Vec3 pos;
    Vec3 dir;
};

bool cancelled(const MapBuildOptions& o) { return o.cancel && o.cancel->load(); }

void report(const MapBuildOptions& o, double p, const std::string& phase) {
    if (o.progress) o.progress(p, phase);
}

double planAngle(const Vec3& v) { return std::atan2(v.z, v.x); }

// Samples a cubic Hermite curve whose tangents have the chord's length, which is
// how SCS defines road and nav-curve splines. Sample density follows curvature.
std::vector<CurveSample> sampleHermite(const Vec3& p0, Vec3 d0, const Vec3& p1, Vec3 d1) {
    const double chord = distance(p0, p1);
    d0 = d0.normalized();
    d1 = d1.normalized();
    const Vec3 m0 = d0 * chord;
    const Vec3 m1 = d1 * chord;
    const double turn = std::acos(clamp(dot(d0, d1), -1.0, 1.0));
    const int steps = clamp(static_cast<int>(std::ceil(std::max(chord / 25.0, turn / degToRad(3.0)))), 1, 200);
    std::vector<CurveSample> out;
    out.reserve(static_cast<std::size_t>(steps) + 1);
    for (int i = 0; i <= steps; ++i) {
        const double t = static_cast<double>(i) / steps;
        Vec3 tangent = hermiteTangent(p0, m0, p1, m1, t);
        if (tangent.lengthSq() < 1e-9) tangent = d0;
        out.push_back({hermite(p0, m0, p1, m1, t), tangent.normalized()});
    }
    return out;
}

LanePoint toLanePoint(const Vec3& world) {
    const Vec2 p = coords::worldToPlan(world);
    return {static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(world.y)};
}

double firstTupleNumber(const std::string& v) {
    // "(1.75, 0)" -> 1.75
    const auto open = v.find('(');
    const auto comma = v.find(',');
    if (open == std::string::npos || comma == std::string::npos || comma < open) return 0.0;
    std::string n = v.substr(open + 1, comma - open - 1);
    n.erase(std::remove_if(n.begin(), n.end(), [](char c) { return std::isspace(static_cast<unsigned char>(c)); }),
            n.end());
    return siiNumber(n, 0.0);
}

std::string textOf(const std::vector<char>& d) { return std::string(d.begin(), d.end()); }

std::unordered_map<std::uint64_t, RoadLookInfo> loadRoadLooks(const GameFileSystem& fs, MapBuildStats& stats) {
    std::unordered_map<std::uint64_t, RoadLookInfo> looks;
    const DirectoryListing dir = fs.list("def/world");
    for (const auto& file : dir.files) {
        if (file.rfind("road_look", 0) != 0 || file.size() < 4 || file.substr(file.size() - 4) != ".sii") continue;
        const auto data = fs.read("def/world/" + file);
        if (!data) continue;
        for (const auto& u : parseSiiText(textOf(*data))) {
            if (u.className != "road_look" || u.name.rfind("road.", 0) != 0) continue;
            RoadLookInfo info;
            info.lanesLeft = static_cast<int>(u.count("lanes_left"));
            info.lanesRight = static_cast<int>(u.count("lanes_right"));
            if (const auto* o = u.first("road_offset")) info.roadOffset = siiNumber(*o);
            if (const auto it = u.attributes.find("lane_offsets_left"); it != u.attributes.end()) {
                for (const auto& v : it->second) info.laneOffsetsLeft.push_back(firstTupleNumber(v));
            }
            if (const auto it = u.attributes.find("lane_offsets_right"); it != u.attributes.end()) {
                for (const auto& v : it->second) info.laneOffsetsRight.push_back(firstTupleNumber(v));
            }
            looks[tokenFromString(u.name.substr(5))] = info;
        }
    }
    stats.roadLooks = looks.size();
    return looks;
}

std::unordered_map<std::uint64_t, std::string> loadPrefabPaths(const GameFileSystem& fs) {
    std::unordered_map<std::uint64_t, std::string> paths;
    const DirectoryListing dir = fs.list("def/world");
    for (const auto& file : dir.files) {
        if (file.rfind("prefab", 0) != 0 || file.size() < 4 || file.substr(file.size() - 4) != ".sii") continue;
        const auto data = fs.read("def/world/" + file);
        if (!data) continue;
        for (const auto& u : parseSiiText(textOf(*data))) {
            if (u.className != "prefab_model" || u.name.rfind("prefab.", 0) != 0) continue;
            if (const auto* d = u.first("prefab_desc")) paths[tokenFromString(u.name.substr(7))] = siiUnquote(*d);
        }
    }
    return paths;
}

// Spatial hash of lane start points for geometric connection matching.
class EndpointIndex {
public:
    explicit EndpointIndex(double cell) : cell_(cell) {}

    void add(const Vec2& p, std::uint32_t id) { map_[key(p)].push_back({p, id}); }

    template <typename F>
    void near(const Vec2& p, double radius, F&& f) const {
        const auto cx = static_cast<std::int64_t>(std::floor(p.x / cell_));
        const auto cy = static_cast<std::int64_t>(std::floor(p.y / cell_));
        const int r = static_cast<int>(std::ceil(radius / cell_));
        for (int dx = -r; dx <= r; ++dx) {
            for (int dy = -r; dy <= r; ++dy) {
                const auto it = map_.find(pack(cx + dx, cy + dy));
                if (it == map_.end()) continue;
                for (const auto& e : it->second) {
                    if (distance(e.first, p) <= radius) f(e.second, distance(e.first, p));
                }
            }
        }
    }

    template <typename F>
    void nearPoints(const Vec2& p, double radius, F&& f) const {
        const auto cx = static_cast<std::int64_t>(std::floor(p.x / cell_));
        const auto cy = static_cast<std::int64_t>(std::floor(p.y / cell_));
        const int r = static_cast<int>(std::ceil(radius / cell_));
        for (int dx = -r; dx <= r; ++dx) {
            for (int dy = -r; dy <= r; ++dy) {
                const auto it = map_.find(pack(cx + dx, cy + dy));
                if (it == map_.end()) continue;
                for (const auto& e : it->second) {
                    if (distance(e.first, p) <= radius) f(e.first, e.second);
                }
            }
        }
    }

private:
    static std::uint64_t pack(std::int64_t x, std::int64_t y) {
        return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32) | static_cast<std::uint32_t>(y);
    }
    std::uint64_t key(const Vec2& p) const {
        return pack(static_cast<std::int64_t>(std::floor(p.x / cell_)), static_cast<std::int64_t>(std::floor(p.y / cell_)));
    }

    double cell_;
    std::unordered_map<std::uint64_t, std::vector<std::pair<Vec2, std::uint32_t>>> map_;
};

double segmentStartYaw(const LaneSegment& s) {
    const Vec2 d = s.points[1].plan() - s.points[0].plan();
    return std::atan2(d.y, d.x);
}

double segmentEndYaw(const LaneSegment& s) {
    const std::size_t n = s.points.size();
    const Vec2 d = s.points[n - 1].plan() - s.points[n - 2].plan();
    return std::atan2(d.y, d.x);
}

}  // namespace

std::vector<std::filesystem::path> gameArchives(const std::filesystem::path& gameDir) {
    std::vector<std::filesystem::path> out;
    std::error_code ec;
    for (const auto& e : std::filesystem::directory_iterator(gameDir, ec)) {
        if (e.is_regular_file() && e.path().extension() == ".scs") out.push_back(e.path());
    }
    // Base data first, DLC afterwards so DLC content overrides base content.
    std::sort(out.begin(), out.end(), [](const auto& a, const auto& b) {
        const std::string na = a.filename().string(), nb = b.filename().string();
        const bool da = na.rfind("dlc_", 0) == 0, db = nb.rfind("dlc_", 0) == 0;
        if (da != db) return !da;
        return na < nb;
    });
    return out;
}

std::string mapFingerprint(const MapBuildOptions& options) {
    std::ostringstream fp;
    fp << "atspilot-map-v" << RoadNetwork::kFormatVersion << ";map=" << options.mapName
       << ";lane=" << options.laneWidth;
    auto add = [&](const std::filesystem::path& p) {
        std::error_code ec;
        const auto size = std::filesystem::file_size(p, ec);
        const auto time = std::filesystem::last_write_time(p, ec).time_since_epoch().count();
        fp << ';' << p.filename().string() << ':' << size << ':' << time;
    };
    for (const auto& a : gameArchives(options.gameDir)) add(a);
    for (const auto& a : options.extraArchives) add(a);
    return fp.str();
}

std::optional<RoadNetwork> buildRoadNetwork(const MapBuildOptions& options, MapBuildStats& stats, Logger* log) {
    auto error = [&](const std::string& e) {
        if (stats.errors.size() < 20) stats.errors.push_back(e);
        if (log) log->warn("map: {}", e);
    };

    GameFileSystem fs;
    std::vector<std::filesystem::path> archives = gameArchives(options.gameDir);
    archives.insert(archives.end(), options.extraArchives.begin(), options.extraArchives.end());
    for (const auto& path : archives) {
        std::string err;
        auto a = HashFsArchive::open(path, &err);
        if (a) fs.add(std::move(a));
        else error(path.filename().string() + ": " + err);
    }
    stats.archives = fs.archives().size();
    if (fs.archives().empty()) {
        error("no readable archives in " + options.gameDir.string());
        return std::nullopt;
    }

    report(options, 0.01, "Reading road definitions");
    const auto looks = loadRoadLooks(fs, stats);
    const auto prefabPaths = loadPrefabPaths(fs);

    // --- Sectors -------------------------------------------------------------
    const std::string mapDir = "map/" + options.mapName;
    std::vector<std::string> sectorFiles;
    for (const auto& f : fs.list(mapDir).files) {
        if (f.size() > 5 && f.substr(f.size() - 5) == ".base") sectorFiles.push_back(f);
    }
    if (sectorFiles.empty()) {
        error("no sectors found in " + mapDir);
        return std::nullopt;
    }

    std::unordered_map<std::uint64_t, MapNode> nodes;
    std::vector<MapRoad> roads;
    std::vector<MapPrefab> prefabs;
    for (std::size_t i = 0; i < sectorFiles.size(); ++i) {
        if (cancelled(options)) return std::nullopt;
        if (i % 50 == 0) report(options, 0.05 + 0.6 * i / sectorFiles.size(), "Parsing map sectors");
        const auto data = fs.read(mapDir + "/" + sectorFiles[i]);
        if (!data) {
            ++stats.sectorErrors;
            error(sectorFiles[i] + ": unreadable");
            continue;
        }
        try {
            SectorData sector = parseSector(data->data(), data->size());
            for (auto& n : sector.nodes) nodes[n.uid] = n;
            roads.insert(roads.end(), sector.roads.begin(), sector.roads.end());
            for (auto& p : sector.prefabs) prefabs.push_back(std::move(p));
            ++stats.sectors;
        } catch (const std::exception& e) {
            ++stats.sectorErrors;
            error(sectorFiles[i] + ": " + e.what());
        }
    }
    stats.roads = roads.size();
    stats.prefabs = prefabs.size();

    RoadNetwork net;

    // --- Prefab nav curves -----------------------------------------------------
    report(options, 0.7, "Building junction curves");
    std::unordered_map<std::uint64_t, std::shared_ptr<const PrefabDescription>> descCache;
    for (const auto& prefab : prefabs) {
        if (cancelled(options)) return std::nullopt;
        if (prefab.nodes.empty()) continue;
        auto cached = descCache.find(prefab.model);
        if (cached == descCache.end()) {
            std::shared_ptr<const PrefabDescription> loaded;
            const auto path = prefabPaths.find(prefab.model);
            if (path != prefabPaths.end()) {
                if (const auto data = fs.read(path->second)) {
                    try {
                        loaded = std::make_shared<PrefabDescription>(parsePrefabDescription(data->data(), data->size()));
                        ++stats.prefabDescs;
                    } catch (const std::exception& ex) {
                        error(path->second + ": " + ex.what());
                    }
                }
            }
            cached = descCache.emplace(prefab.model, std::move(loaded)).first;
        }
        const PrefabDescription* desc = cached->second.get();
        if (!desc || desc->nodes.size() != prefab.nodes.size() || prefab.originIndex >= desc->nodes.size()) {
            ++stats.prefabsWithoutDesc;
            continue;
        }
        const auto origin = nodes.find(prefab.nodes[0]);
        if (origin == nodes.end()) {
            ++stats.prefabsWithoutDesc;
            continue;
        }

        // Place the prefab so its origin node coincides with the map node.
        const PrefabNodeDesc& localOrigin = desc->nodes[prefab.originIndex];
        const double delta = planAngle(origin->second.forward()) - planAngle(localOrigin.direction);
        const double c = std::cos(delta), sn = std::sin(delta);
        auto rotate = [&](const Vec3& v) { return Vec3{v.x * c - v.z * sn, v.y, v.x * sn + v.z * c}; };
        auto place = [&](const Vec3& local) { return origin->second.position + rotate(local - localOrigin.position); };

        std::vector<std::uint32_t> curveIds(desc->curves.size());
        for (std::size_t ci = 0; ci < desc->curves.size(); ++ci) {
            const PrefabNavCurve& curve = desc->curves[ci];
            const Vec3 a = place(curve.startPos);
            const Vec3 b = place(curve.endPos);
            const Vec3 da = rotate(curve.startRot.rotate({0.0, 0.0, -1.0}));
            const Vec3 db = rotate(curve.endRot.rotate({0.0, 0.0, -1.0}));
            LaneSegment seg;
            seg.itemUid = prefab.uid;
            seg.kind = LaneKind::Prefab;
            seg.laneIndex = static_cast<std::uint8_t>(ci & 0xFF);
            for (const auto& smp : sampleHermite(a, da, b, db)) seg.points.push_back(toLanePoint(smp.pos));
            curveIds[ci] = net.add(std::move(seg));
        }
        for (std::size_t ci = 0; ci < desc->curves.size(); ++ci) {
            for (int n : desc->curves[ci].next) net.mutableSegments()[curveIds[ci]].next.push_back(curveIds[n]);
        }
    }

    // --- Lane offset calibration -----------------------------------------------
    // Templated road looks do not state their lane width, so each look's lane
    // offsets are measured from the junction (prefab) lanes its ends attach to.
    // The game's own prefab geometry is the reference; see docs/map-parsing.md.
    report(options, 0.85, "Calibrating lane offsets");
    std::vector<double> prefabStartYaw, prefabEndYaw;
    EndpointIndex prefabStarts(4.0), prefabEnds(4.0);
    for (const auto& seg : net.segments()) {
        if (seg.points.size() < 2) continue;
        prefabStarts.add(seg.points.front().plan(), static_cast<std::uint32_t>(prefabStartYaw.size()));
        prefabStartYaw.push_back(segmentStartYaw(seg));
        prefabEnds.add(seg.points.back().plan(), static_cast<std::uint32_t>(prefabEndYaw.size()));
        prefabEndYaw.push_back(segmentEndYaw(seg));
    }

    struct LookSamples {
        std::vector<std::vector<float>> side[2];
    };
    std::unordered_map<std::uint64_t, LookSamples> samples;

    auto measure = [&](const Vec2& centre, double roadYaw, bool atStart, bool leftSide, int laneCount,
                       std::vector<std::vector<float>>& out) {
        if (laneCount <= 0) return;
        // Right lanes leave the start node and enter the end node; left lanes the reverse.
        const bool usePrefabEnds = (atStart && !leftSide) || (!atStart && leftSide);
        const double travelYaw = leftSide ? normalizeAngle(roadYaw + kPi) : roadYaw;
        const Vec2 along = coords::yawToDirection(roadYaw);
        const Vec2 right{along.y, -along.x};
        const auto& index = usePrefabEnds ? prefabEnds : prefabStarts;
        const auto& yaws = usePrefabEnds ? prefabEndYaw : prefabStartYaw;
        std::vector<double> mags;
        index.nearPoints(centre, 25.0, [&](const Vec2& p, std::uint32_t id) {
            if (std::abs(headingDifference(travelYaw, yaws[id])) > degToRad(20.0)) return;
            const Vec2 d = p - centre;
            if (std::abs(dot(d, along)) > 1.5) return;
            const double m = (leftSide ? -1.0 : 1.0) * dot(d, right);
            if (m > -0.5) mags.push_back(m);
        });
        std::sort(mags.begin(), mags.end());
        // Several curves (straight, turn) often start at the same lane point.
        std::vector<double> lanes;
        for (double m : mags) {
            if (lanes.empty() || m - lanes.back() > 0.6) lanes.push_back(m);
        }
        if (static_cast<int>(lanes.size()) != laneCount) return;
        if (out.size() < lanes.size()) out.resize(lanes.size());
        for (std::size_t i = 0; i < lanes.size(); ++i) out[i].push_back(static_cast<float>(lanes[i]));
    };

    for (const auto& road : roads) {
        const auto s = nodes.find(road.startNode);
        const auto e = nodes.find(road.endNode);
        const auto lk = looks.find(road.roadLook);
        if (s == nodes.end() || e == nodes.end() || lk == looks.end()) continue;
        const Vec3 p0 = s->second.position, p1 = e->second.position;
        if (distance(p0, p1) < 0.1) continue;
        Vec3 d0 = s->second.forward(), d1 = e->second.forward();
        if (dot(d0, p1 - p0) < 0.0) d0 = -d0;
        if (dot(d1, p1 - p0) < 0.0) d1 = -d1;
        auto& ls = samples[road.roadLook];
        for (int end = 0; end < 2; ++end) {
            const bool atStart = end == 0;
            const Vec2 pos = coords::worldToPlan(atStart ? p0 : p1);
            const double yaw = coords::worldDirectionToYaw(atStart ? d0 : d1);
            measure(pos, yaw, atStart, false, lk->second.lanesRight, ls.side[0]);
            measure(pos, yaw, atStart, true, lk->second.lanesLeft, ls.side[1]);
        }
    }

    std::unordered_map<std::uint64_t, std::vector<double>> calibrated[2];
    for (auto& [token, ls] : samples) {
        const auto& look = looks.at(token);
        for (int side = 0; side < 2; ++side) {
            const int count = side == 0 ? look.lanesRight : look.lanesLeft;
            auto& lanes = ls.side[side];
            if (count == 0 || static_cast<int>(lanes.size()) != count) continue;
            bool enough = true;
            for (const auto& v : lanes) enough = enough && v.size() >= 3;
            if (!enough) continue;
            std::vector<double> offsets;
            for (auto& v : lanes) {
                std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
                offsets.push_back(v[v.size() / 2]);
            }
            calibrated[side][token] = std::move(offsets);
            ++stats.lookSidesCalibrated;
        }
    }

    // --- Road lanes ----------------------------------------------------------
    report(options, 0.88, "Building road lanes");
    for (const auto& road : roads) {
        const auto s = nodes.find(road.startNode);
        const auto e = nodes.find(road.endNode);
        if (s == nodes.end() || e == nodes.end()) {
            ++stats.roadsMissingNodes;
            continue;
        }
        RoadLookInfo look;
        if (const auto it = looks.find(road.roadLook); it != looks.end()) {
            look = it->second;
        } else {
            ++stats.roadsWithoutLook;
        }
        const Vec3 p0 = s->second.position;
        const Vec3 p1 = e->second.position;
        if (distance(p0, p1) < 0.1) continue;
        Vec3 d0 = s->second.forward();
        Vec3 d1 = e->second.forward();
        // Node orientation follows the node's own forward item; when a road is
        // attached "backwards" the stored direction points the other way.
        if (dot(d0, p1 - p0) < 0.0) d0 = -d0;
        if (dot(d1, p1 - p0) < 0.0) d1 = -d1;
        const auto centre = sampleHermite(p0, d0, p1, d1);

        auto laneOffset = [&](int i, bool left) {
            const auto& cal = calibrated[left ? 1 : 0];
            if (const auto it = cal.find(road.roadLook); it != cal.end() && i < static_cast<int>(it->second.size())) {
                return it->second[i];
            }
            // Model for uncalibrated looks: lanes of laneWidth outward from half the median.
            const auto& extra = left ? look.laneOffsetsLeft : look.laneOffsetsRight;
            const double shift = i < static_cast<int>(extra.size()) ? extra[i] : 0.0;
            return look.roadOffset / 2.0 + shift + (i + 0.5) * options.laneWidth;
        };

        for (int side = 0; side < 2; ++side) {
            const bool left = side == 1;
            const int count = left ? look.lanesLeft : look.lanesRight;
            for (int i = 0; i < count; ++i) {
                const double off = laneOffset(i, left);
                LaneSegment seg;
                seg.itemUid = road.uid;
                seg.kind = LaneKind::Road;
                seg.laneIndex = static_cast<std::uint8_t>(i);
                seg.leftSide = left;
                seg.points.reserve(centre.size());
                for (const auto& c : centre) {
                    // Right of travel direction in the horizontal plane (world y is up).
                    const Vec3 right = Vec3{-c.dir.z, 0.0, c.dir.x}.normalized();
                    seg.points.push_back(toLanePoint(c.pos + right * (left ? -off : off)));
                }
                if (left) std::reverse(seg.points.begin(), seg.points.end());
                net.add(std::move(seg));
            }
        }
    }

    // --- Connections between items ----------------------------------------------
    report(options, 0.9, "Connecting lanes");
    auto& segs = net.mutableSegments();
    std::unordered_map<std::uint64_t, const MapRoad*> roadByUid;
    for (const auto& r : roads) roadByUid[r.uid] = &r;
    EndpointIndex starts(4.0);
    for (std::uint32_t id = 0; id < segs.size(); ++id) {
        if (segs[id].points.size() >= 2) starts.add(segs[id].points.front().plan(), id);
    }
    constexpr double kJoinTolerance = 1.6;     // m
    const double kJoinAngle = degToRad(30.0);
    for (std::uint32_t id = 0; id < segs.size(); ++id) {
        auto& seg = segs[id];
        if (seg.points.size() < 2) continue;
        ++stats.laneEnds;
        const Vec2 end = seg.points.back().plan();
        const double endYaw = segmentEndYaw(seg);
        bool connected = !seg.next.empty();
        starts.near(end, kJoinTolerance, [&](std::uint32_t other, double gap) {
            if (other == id || segs[other].itemUid == seg.itemUid) return;
            if (std::abs(headingDifference(endYaw, segmentStartYaw(segs[other]))) > kJoinAngle) return;
            if (std::find(seg.next.begin(), seg.next.end(), other) != seg.next.end()) return;
            seg.next.push_back(other);
            connected = true;
            if (seg.kind != segs[other].kind) {
                stats.roadPrefabGapSum += gap;
                ++stats.roadPrefabJoins;
                ++stats.joinGapHistogram[std::min<std::size_t>(31, static_cast<std::size_t>(gap / 0.25))];
                if (gap > 0.75 && stats.gapSamples.size() < 5000) {
                    const bool roadFirst = seg.kind == LaneKind::Road;
                    const LaneSegment& road = roadFirst ? seg : segs[other];
                    const auto rit = roadByUid.find(road.itemUid);
                    if (rit != roadByUid.end()) {
                        MapBuildStats::GapSample g;
                        g.look = tokenToString(rit->second->roadLook);
                        g.left = road.leftSide;
                        g.lane = road.laneIndex;
                        const auto lk = looks.find(rit->second->roadLook);
                        g.laneCount = lk == looks.end() ? 0 : (road.leftSide ? lk->second.lanesLeft : lk->second.lanesRight);
                        const Vec2 dir = coords::yawToDirection(endYaw);
                        const Vec2 d = segs[other].points.front().plan() - end;
                        g.lateral = dot(d, Vec2{dir.y, -dir.x});
                        g.longitudinal = dot(d, dir);
                        g.roadEndsIntoPrefab = roadFirst;
                        stats.gapSamples.push_back(g);
                    }
                }
            }
        });
        if (connected) {
            ++stats.laneEndsConnected;
        } else if (seg.kind == LaneKind::Road) {
            ++stats.roadEndsUnconnected;
            double nearest = 1e9;
            starts.near(end, 8.0, [&](std::uint32_t other, double gap) {
                if (segs[other].kind != LaneKind::Prefab) return;
                if (std::abs(headingDifference(endYaw, segmentStartYaw(segs[other]))) > kJoinAngle) return;
                nearest = std::min(nearest, gap);
            });
            if (nearest < 8.0) ++stats.missGapHistogram[std::min<std::size_t>(31, static_cast<std::size_t>(nearest / 0.25))];
        }
    }

    // Road lanes come from a lane-offset model while junction curves are authored
    // geometry, so the residual gap at a join is closed by shifting the road lane's
    // ends onto the junction curve, blending the correction along the road.
    std::vector<std::vector<std::uint32_t>> preds(segs.size());
    for (std::uint32_t id = 0; id < segs.size(); ++id) {
        for (auto n : segs[id].next) preds[n].push_back(id);
    }
    for (std::uint32_t id = 0; id < segs.size(); ++id) {
        auto& seg = segs[id];
        if (seg.kind != LaneKind::Road || seg.points.size() < 2) continue;
        Vec2 startShift, endShift;
        for (auto p : preds[id]) {
            if (segs[p].kind == LaneKind::Prefab) {
                startShift = segs[p].points.back().plan() - seg.points.front().plan();
                break;
            }
        }
        for (auto n : seg.next) {
            if (segs[n].kind == LaneKind::Prefab) {
                endShift = segs[n].points.front().plan() - seg.points.back().plan();
                break;
            }
        }
        if (startShift.lengthSq() < 1e-6 && endShift.lengthSq() < 1e-6) continue;
        double total = 0.0;
        for (std::size_t i = 1; i < seg.points.size(); ++i) total += distance(seg.points[i - 1].plan(), seg.points[i].plan());
        double s = 0.0;
        for (std::size_t i = 0; i < seg.points.size(); ++i) {
            if (i > 0) s += distance(seg.points[i - 1].plan(), seg.points[i].plan());
            const Vec2 shift = lerp(startShift, endShift, total > 0.0 ? s / total : 0.0);
            seg.points[i].x += static_cast<float>(shift.x);
            seg.points[i].y += static_cast<float>(shift.y);
        }
    }

    net.finalize();
    stats.lanes = net.size();
    for (const auto& s : net.segments()) stats.points += s.points.size();
    report(options, 1.0, "Map ready");
    return net;
}

}  // namespace atspilot

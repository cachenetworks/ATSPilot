#include "map/MapBuilder.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
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

std::uint8_t laneRules(const PrefabNavCurve& c) {
    static const std::uint64_t kStop = tokenFromString("stop");
    static const std::uint64_t kGiveWay = tokenFromString("give_way");
    static const std::uint64_t kRail = tokenFromString("rail_cross");
    static const std::uint64_t kNoTrucks[] = {tokenFromString("no_trucks"), tokenFromString("car_only"),
                                              tokenFromString("bus_only"), tokenFromString("tram_only")};
    std::uint8_t r = 0;
    if (c.semaphoreId >= 0) r |= LaneRule::Signal;
    if (c.trafficRule == kStop) r |= LaneRule::Stop;
    if (c.trafficRule == kGiveWay) r |= LaneRule::Yield;
    if (c.trafficRule == kRail) r |= LaneRule::RailCrossing;
    for (auto t : kNoTrucks) {
        if (c.trafficRule == t) r |= LaneRule::NoTrucks;
    }
    return r;
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
    std::vector<MapCompany> companies;
    std::vector<MapServiceItem> serviceItems;
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
            companies.insert(companies.end(), sector.companies.begin(), sector.companies.end());
            for (auto& s : sector.services) serviceItems.push_back(std::move(s));
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
    struct RawService {
        ServiceKind kind = ServiceKind::Fuel;
        Vec2 position;
        double yaw = 0.0;
        std::uint64_t prefab = 0;
    };
    std::vector<RawService> rawServices;
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
            seg.rules = laneRules(curve);
            if (curve.semaphoreId >= 0 && curve.semaphoreId < 32768)
                seg.semaphoreId = static_cast<std::int16_t>(curve.semaphoreId);
            if (seg.rules & (LaneRule::Signal | LaneRule::Stop | LaneRule::Yield)) ++stats.controlledLanes;
            for (const auto& smp : sampleHermite(a, da, b, db)) seg.points.push_back(toLanePoint(smp.pos));
            curveIds[ci] = net.add(std::move(seg));
        }
        for (std::size_t ci = 0; ci < desc->curves.size(); ++ci) {
            for (int n : desc->curves[ci].next) net.mutableSegments()[curveIds[ci]].next.push_back(curveIds[n]);
        }
        for (const auto& sp : desc->spawnPoints) {
            if (sp.type != SpawnType::Gas && sp.type != SpawnType::WeighStation) continue;
            const Vec3 dir = rotate(sp.rotation.rotate({0.0, 0.0, -1.0}));
            RawService rs;
            rs.kind = sp.type == SpawnType::Gas ? ServiceKind::Fuel : ServiceKind::Weigh;
            rs.position = coords::worldToPlan(place(sp.position));
            rs.yaw = std::atan2(-dir.z, dir.x);
            rs.prefab = prefab.uid;
            rawServices.push_back(rs);
        }

        // Placement check: every descriptor node must land on its map node.
        // Descriptor node j corresponds to map node nodes[(j - originIndex) mod n].
        const std::size_t n = prefab.nodes.size();
        for (std::size_t j = 0; j < n; ++j) {
            const auto mapNode = nodes.find(prefab.nodes[(j + n - prefab.originIndex) % n]);
            if (mapNode == nodes.end()) continue;
            ++stats.prefabNodeChecks;
            const Vec3 placed = place(desc->nodes[j].position);
            const double err = distance(coords::worldToPlan(placed), coords::worldToPlan(mapNode->second.position));
            if (err > 1.0) {
                ++stats.prefabNodeMismatches;
                if (stats.prefabMismatchSamples.size() < 10) {
                    stats.prefabMismatchSamples.push_back(tokenToString(prefab.model) + " node " + std::to_string(j) +
                                                          "/" + std::to_string(n) + " origin " +
                                                          std::to_string(prefab.originIndex) + " error " +
                                                          std::to_string(err) + " m");
                }
            }
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
    // Development aid: ATSPILOT_DEBUG_LOOK=<token> records the raw measurements for one look.
    const char* debugEnv = std::getenv("ATSPILOT_DEBUG_LOOK");
    const std::uint64_t debugLook = debugEnv ? tokenFromString(debugEnv) : 0;
    std::uint64_t currentLook = 0;

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
            // Signed offset towards this side's outer edge. It can be negative: on a
            // one-way carriageway the node line is the carriageway centre, so the
            // inner lane sits on the other side of it.
            mags.push_back((leftSide ? -1.0 : 1.0) * dot(d, right));
        });
        std::sort(mags.begin(), mags.end());
        // Several curves (straight, turn) often start at the same lane point.
        std::vector<double> points;
        for (double m : mags) {
            if (points.empty() || m - points.back() > 0.6) points.push_back(m);
        }
        // Find laneCount consecutive points with lane-like spacing. A road end is
        // only used when exactly one such window exists; extra aligned points
        // (ramps, parallel roads) make it ambiguous and it is skipped. Inner lanes
        // may lie well across the node line (e.g. 3-lane carriageways at -6.75,
        // -2.25, +2.25 m in ATS 1.61).
        const auto n = static_cast<std::size_t>(laneCount);
        std::size_t bestStart = points.size();
        int windows = 0;
        for (std::size_t s0 = 0; s0 + n <= points.size(); ++s0) {
            bool spaced = true;
            for (std::size_t k = 1; k < n; ++k) {
                const double gap = points[s0 + k] - points[s0 + k - 1];
                spaced = spaced && gap >= 2.5 && gap <= 6.0;
            }
            if (!spaced || points[s0] < -12.0 || points[s0 + n - 1] > 25.0) continue;
            ++windows;
            bestStart = s0;
        }
        if (windows != 1) bestStart = points.size();
        if (debugLook != 0 && currentLook == debugLook && stats.calibrationDebug.size() < 30) {
            std::string line = std::string(leftSide ? "L" : "R") + (atStart ? " start:" : " end:");
            for (double p : points) line += " " + std::to_string(p).substr(0, 6);
            line += bestStart == points.size() ? "  -> rejected" : "  -> window " + std::to_string(bestStart);
            stats.calibrationDebug.push_back(line);
        }
        if (bestStart == points.size()) return;
        if (out.size() < n) out.resize(n);
        for (std::size_t i = 0; i < n; ++i) out[i].push_back(static_cast<float>(points[bestStart + i]));
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
        currentLook = road.roadLook;
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
            // Model for uncalibrated looks: lanes of laneWidth outward from half the
            // median; one-way roads are centred on the node line.
            const auto& extra = left ? look.laneOffsetsLeft : look.laneOffsetsRight;
            const double shift = i < static_cast<int>(extra.size()) ? extra[i] : 0.0;
            if (look.lanesLeft == 0 || look.lanesRight == 0) {
                const int count = left ? look.lanesLeft : look.lanesRight;
                return shift + (i - (count - 1) / 2.0) * options.laneWidth;
            }
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
            std::uint32_t nearestId = 0;
            starts.near(end, 8.0, [&](std::uint32_t other, double gap) {
                if (segs[other].kind != LaneKind::Prefab) return;
                if (std::abs(headingDifference(endYaw, segmentStartYaw(segs[other]))) > kJoinAngle) return;
                if (gap < nearest) {
                    nearest = gap;
                    nearestId = other;
                }
            });
            if (nearest < 8.0) {
                ++stats.missGapHistogram[std::min<std::size_t>(31, static_cast<std::size_t>(nearest / 0.25))];
                const auto rit = roadByUid.find(seg.itemUid);
                if (rit != roadByUid.end() && stats.missSamples.size() < 20000) {
                    MapBuildStats::GapSample g;
                    g.look = tokenToString(rit->second->roadLook);
                    g.left = seg.leftSide;
                    g.lane = seg.laneIndex;
                    const auto lk = looks.find(rit->second->roadLook);
                    g.laneCount = lk == looks.end() ? 0 : (seg.leftSide ? lk->second.lanesLeft : lk->second.lanesRight);
                    const Vec2 dir = coords::yawToDirection(endYaw);
                    const Vec2 d = segs[nearestId].points.front().plan() - end;
                    g.lateral = dot(d, Vec2{dir.y, -dir.x});
                    g.longitudinal = dot(d, dir);
                    stats.missSamples.push_back(g);
                }
            }
        }
    }

    report(options, 0.92, "Linking merges");
    // Merges and lane drops: a lane that ends where the next item has fewer (or
    // shifted) lanes has a same-direction lane start about one lane width to the
    // side. Linking it keeps the graph routable; the path builder turns the
    // lateral offset into a smooth lane change.
    for (std::uint32_t id = 0; id < segs.size(); ++id) {
        auto& seg = segs[id];
        if (!seg.next.empty() || seg.points.size() < 2) continue;
        const Vec2 end = seg.points.back().plan();
        const double endYaw = segmentEndYaw(seg);
        const Vec2 dir = coords::yawToDirection(endYaw);
        // Prefer a continuation straight ahead (some junction curves begin metres
        // past the node), else a sideways merge into a neighbouring lane.
        std::uint32_t ahead = 0, merge = 0;
        double bestAhead = 1e9, bestMerge = 1e9;
        starts.near(end, 25.0, [&](std::uint32_t other, double gap) {
            if (other == id || segs[other].itemUid == seg.itemUid) return;
            if (std::abs(headingDifference(endYaw, segmentStartYaw(segs[other]))) > degToRad(20.0)) return;
            const Vec2 d = segs[other].points.front().plan() - end;
            const double lon = dot(d, dir);
            const double lat = std::abs(cross(dir, d));
            if (lat <= 1.6 && lon >= -1.0 && lon < bestAhead) {
                bestAhead = lon;
                ahead = other;
            } else if (gap <= 5.0 && gap < bestMerge) {
                bestMerge = gap;
                merge = other;
            }
        });
        if (bestAhead < 1e9) {
            seg.next.push_back(ahead);
            ++stats.gapConnections;
        } else if (bestMerge < 1e9) {
            seg.next.push_back(merge);
            ++stats.mergeConnections;
        }
    }

    // Diagnostics: road lanes that still end without a successor although the map
    // node at their end has another item attached are links the builder missed.
    {
        std::unordered_map<std::uint64_t, const MapRoad*> roadIndex;
        for (const auto& r : roads) roadIndex[r.uid] = &r;
        for (std::uint32_t id = 0; id < segs.size(); ++id) {
            const auto& seg = segs[id];
            if (seg.kind != LaneKind::Road || !seg.next.empty() || seg.points.size() < 2) continue;
            const auto rit = roadIndex.find(seg.itemUid);
            if (rit == roadIndex.end()) continue;
            const std::uint64_t endNode = seg.leftSide ? rit->second->startNode : rit->second->endNode;
            const auto n = nodes.find(endNode);
            if (n == nodes.end()) continue;
            const std::uint64_t other =
                n->second.forwardItem == seg.itemUid ? n->second.backwardItem : n->second.forwardItem;
            if (other == 0) {
                ++stats.roadDeadEndsInMap;
                continue;
            }
            ++stats.roadDeadEndsMissedLink;
            if (stats.missedLinkSamples.size() < 12) {
                char buf[160];
                const Vec3 w = coords::planToWorld(seg.points.back().plan(), 0.0);
                std::snprintf(buf, sizeof(buf), "road %016llx lane %u%s at (%.1f, %.1f) -> item %016llx",
                              static_cast<unsigned long long>(seg.itemUid), seg.laneIndex, seg.leftSide ? "L" : "R",
                              w.x, w.z, static_cast<unsigned long long>(other));
                stats.missedLinkSamples.push_back(buf);
            }
        }
    }

    report(options, 0.94, "Snapping lanes");
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
        // Only regular joins are snapped; merge and gap links are bridged by the path builder.
        for (auto p : preds[id]) {
            const Vec2 shift = segs[p].points.back().plan() - seg.points.front().plan();
            if (segs[p].kind == LaneKind::Prefab && shift.length() <= kJoinTolerance) {
                startShift = shift;
                break;
            }
        }
        for (auto n : seg.next) {
            const Vec2 shift = segs[n].points.front().plan() - seg.points.back().plan();
            if (segs[n].kind == LaneKind::Prefab && shift.length() <= kJoinTolerance) {
                endShift = shift;
                break;
            }
        }
        if (startShift.lengthSq() < 1e-6 && endShift.lengthSq() < 1e-6) continue;
        // Blend factors come from the unmodified geometry, so shifting a point can
        // never feed back into the factor of the next one.
        std::vector<double> along(seg.points.size(), 0.0);
        for (std::size_t i = 1; i < seg.points.size(); ++i) {
            along[i] = along[i - 1] + distance(seg.points[i - 1].plan(), seg.points[i].plan());
        }
        const double total = along.back();
        for (std::size_t i = 0; i < seg.points.size(); ++i) {
            const double t = total > 1e-6 ? clamp(along[i] / total, 0.0, 1.0) : 0.0;
            const Vec2 shift = lerp(startShift, endShift, t);
            seg.points[i].x += static_cast<float>(shift.x);
            seg.points[i].y += static_cast<float>(shift.y);
        }
    }

    report(options, 0.96, "Resolving destinations");
    // Destinations: each company item with the lanes of its depot prefab.
    std::unordered_map<std::uint64_t, std::vector<std::uint32_t>> lanesByItem;
    for (std::uint32_t id = 0; id < segs.size(); ++id) {
        if (segs[id].kind == LaneKind::Prefab) lanesByItem[segs[id].itemUid].push_back(id);
    }
    std::unordered_map<std::uint64_t, const MapPrefab*> prefabByUid;
    for (const auto& p : prefabs) prefabByUid[p.uid] = &p;
    // Most depot prefabs have no drivable nav curves of their own; they attach to
    // the road network at their prefab nodes. A depot is therefore reached by any
    // lane that ends at one of those nodes (the entrance), plus its own lanes if any.
    EndpointIndex laneEnds(4.0);
    for (std::uint32_t id = 0; id < segs.size(); ++id) {
        if (segs[id].points.size() >= 2) laneEnds.add(segs[id].points.back().plan(), id);
    }
    for (const auto& c : companies) {
        const auto prefab = prefabByUid.find(c.prefab);
        const auto node = nodes.find(c.node);
        if (prefab == prefabByUid.end()) {
            ++stats.companiesPrefabMissing;
            continue;
        }
        if (node == nodes.end()) {
            ++stats.companiesNodeMissing;
            continue;
        }
        Destination d;
        d.city = c.city;
        d.company = c.company;
        d.position = coords::worldToPlan(node->second.position);
        if (const auto own = lanesByItem.find(c.prefab); own != lanesByItem.end()) d.lanes = own->second;
        for (auto nodeUid : prefab->second->nodes) {
            const auto n = nodes.find(nodeUid);
            if (n == nodes.end()) continue;
            laneEnds.nearPoints(coords::worldToPlan(n->second.position), 8.0, [&](const Vec2&, std::uint32_t id) {
                if (segs[id].itemUid != c.prefab && std::find(d.lanes.begin(), d.lanes.end(), id) == d.lanes.end()) {
                    d.lanes.push_back(id);
                }
            });
        }
        if (d.lanes.empty()) {
            ++stats.companiesWithoutLanes;
            continue;
        }
        net.addDestination(std::move(d));
    }
    stats.destinations = net.destinations().size();
    stats.companies = companies.size();

    if (std::getenv("ATSPILOT_DUMP_SERVICES")) {
        std::unordered_map<std::uint32_t, int> byKey;
        int shown = 0;
        for (const auto& si : serviceItems) {
            ++byKey[si.type * 1000 + (si.flags & 0xFF)];
            const auto n = nodes.find(si.node);
            if (n == nodes.end() || shown >= 60) continue;
            ++shown;
            const Vec2 p = coords::worldToPlan(n->second.position);
            double gas = 1e9;
            for (const auto& rs : rawServices) gas = std::min(gas, distance(rs.position, p));
            const auto pf = prefabByUid.find(si.prefab);
            std::fprintf(stderr, "service type %u flags %08x at (%.0f, %.0f) prefab %s, %zu nodes, nearest spawn %.1f m\n",
                         si.type, si.flags, n->second.position.x, n->second.position.z,
                         pf != prefabByUid.end() ? tokenToString(pf->second->model).c_str() : "-", si.nodes.size(), gas);
        }
        for (const auto& [k, c] : byKey) std::fprintf(stderr, "service type %u flags-low %02x: %d\n", k / 1000, k % 1000, c);
    }

    // Services: the prefab lane passing nearest the pump or scale, driven in the
    // direction the stand faces where that is clear (weigh station scales).
    for (const auto& rs : rawServices) {
        const auto own = lanesByItem.find(rs.prefab);
        if (own == lanesByItem.end()) continue;
        double best = 1e18;
        ServicePoint sp;
        for (const std::uint32_t id : own->second) {
            if (segs[id].points.size() < 2) continue;
            const LaneMatch m = net.project(id, rs.position);
            double score = m.distance;
            if (rs.kind == ServiceKind::Weigh && std::abs(headingDifference(m.yaw, rs.yaw)) > degToRad(60.0)) score += 50.0;
            if (segs[id].rules & LaneRule::NoTrucks) score += 20.0;
            if (score < best) {
                best = score;
                sp.lane = id;
                sp.s = static_cast<float>(m.s);
                sp.offset = static_cast<float>(m.distance);
            }
        }
        if (best > 15.0) continue;
        sp.kind = rs.kind;
        sp.position = rs.position;
        sp.yaw = static_cast<float>(rs.yaw);
        net.addService(sp);
    }

    // Geometry sanity: a non-finite or out-of-map coordinate is dropped with its
    // lane rather than indexed (it would also be meaningless to steer along).
    for (auto& seg : segs) {
        const bool bad = std::any_of(seg.points.begin(), seg.points.end(), [](const LanePoint& p) {
            return !std::isfinite(p.x) || !std::isfinite(p.y) || std::abs(p.x) > 1e6f || std::abs(p.y) > 1e6f;
        });
        if (!bad) continue;
        ++stats.badSegments;
        if (stats.badSegmentSamples.size() < 5) {
            stats.badSegmentSamples.push_back(std::string(seg.kind == LaneKind::Road ? "road " : "prefab ") +
                                              std::to_string(seg.itemUid));
        }
        seg.points.clear();
        seg.next.clear();
    }
    report(options, 0.98, "Indexing");
    std::vector<NodePoint> nodePoints;
    nodePoints.reserve(nodes.size());
    for (const auto& [uid, n] : nodes) {
        const Vec2 p = coords::worldToPlan(n.position);
        nodePoints.push_back({uid, static_cast<float>(p.x), static_cast<float>(p.y)});
    }
    net.setNodes(std::move(nodePoints));
    net.finalize();
    stats.lanes = net.size();
    for (const auto& s : net.segments()) stats.points += s.points.size();
    report(options, 1.0, "Map ready");
    return net;
}

}  // namespace atspilot

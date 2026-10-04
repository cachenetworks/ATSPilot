#include <chrono>
#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <algorithm>
#include <iostream>
#include <map>
#include <optional>
#include <vector>
#include <string>
#include <unordered_map>

#include "map/HashFs.h"
#include "map/LanePlanner.h"
#include "map/MapBuilder.h"
#include "map/RoutePlanner.h"
#include "map/SectorParser.h"
#include "map/Token.h"
#include "map/RoadNetwork.h"
#include "math/Coordinates.h"
#include "math/MathUtil.h"

using namespace atspilot;

namespace {

int build(int argc, char** argv) {
    MapBuildOptions o;
    o.gameDir = argv[2];
    const std::string cache = argc > 3 ? argv[3] : "atspilot_map.cache";
    double last = -1.0;
    std::string lastPhase;
    o.progress = [&](double p, const std::string& phase) {
        if (p - last >= 0.05 || p >= 1.0 || phase != lastPhase) {
            lastPhase = phase;
            std::fprintf(stderr, "[%3.0f%%] %s\n", p * 100.0, phase.c_str());
            last = p;
        }
    };
    MapBuildStats st;
    const auto t0 = std::chrono::steady_clock::now();
    const auto net = buildRoadNetwork(o, st, nullptr);
    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

    std::printf("archives            %zu\n", st.archives);
    std::printf("road looks          %zu (calibrated sides %zu)\n", st.roadLooks, st.lookSidesCalibrated);
    std::printf("sectors             %zu (errors %zu)\n", st.sectors, st.sectorErrors);
    std::printf("roads               %zu (no look %zu, missing nodes %zu)\n", st.roads, st.roadsWithoutLook,
                st.roadsMissingNodes);
    std::printf("prefabs             %zu (descs %zu, unplaced %zu)\n", st.prefabs, st.prefabDescs,
                st.prefabsWithoutDesc);
    std::printf("lane segments       %zu, points %zu\n", st.lanes, st.points);
    std::printf("prefab node checks  %zu, misplaced %zu\n", st.prefabNodeChecks, st.prefabNodeMismatches);
    for (const auto& m : st.prefabMismatchSamples) std::printf("  misplaced: %s\n", m.c_str());
    std::printf("merge connections   %zu, gap connections %zu\n", st.mergeConnections, st.gapConnections);
    std::printf("controlled lanes    %zu (signals, stop signs, give way)\n", st.controlledLanes);
    std::printf("road dead ends      %zu genuine (nothing attached in the map), %zu missed links\n",
                st.roadDeadEndsInMap, st.roadDeadEndsMissedLink);
    for (const auto& m : st.missedLinkSamples) std::printf("  missed: %s\n", m.c_str());
    std::printf("dropped segments    %zu (invalid geometry)\n", st.badSegments);
    for (const auto& b : st.badSegmentSamples) std::printf("  dropped: %s\n", b.c_str());
    std::printf("companies           %zu (destinations with lanes %zu; prefab missing %zu, node missing %zu, no lanes %zu)\n",
                st.companies, st.destinations, st.companiesPrefabMissing, st.companiesNodeMissing,
                st.companiesWithoutLanes);
    std::printf("lane ends connected %zu / %zu (%.1f%%)\n", st.laneEndsConnected, st.laneEnds,
                st.laneEnds ? 100.0 * st.laneEndsConnected / st.laneEnds : 0.0);
    std::printf("road<->prefab joins %zu, mean gap %.3f m\n", st.roadPrefabJoins,
                st.roadPrefabJoins ? st.roadPrefabGapSum / st.roadPrefabJoins : 0.0);
    std::printf("build time          %.1f s\n", secs);
    std::printf("join gap histogram (0.25 m buckets):");
    for (auto v : st.joinGapHistogram) std::printf(" %zu", v);
    std::printf("\nunconnected road ends %zu, nearest prefab start (0.25 m buckets):", st.roadEndsUnconnected);
    for (auto v : st.missGapHistogram) std::printf(" %zu", v);
    std::printf("\n");
    {
        struct Agg { int n = 0; double lat = 0.0; double lon = 0.0; int laneCount = 0; };
        std::map<std::string, Agg> agg;
        for (const auto& g : (std::getenv("ATSPILOT_MISS") ? st.missSamples : st.gapSamples)) {
            const std::string key = g.look + (g.left ? " L" : " R") + std::to_string(g.lane) + "/" +
                                    std::to_string(g.laneCount) + (g.roadEndsIntoPrefab ? " road->prefab" : " prefab->road");
            auto& a = agg[key];
            ++a.n;
            a.lat += g.lateral;
            a.lon += g.longitudinal;
        }
        std::vector<std::pair<std::string, Agg>> v(agg.begin(), agg.end());
        std::sort(v.begin(), v.end(), [](const auto& a, const auto& b) { return a.second.n > b.second.n; });
        std::printf("largest gap groups (look side lane/count direction: n, mean lateral, mean longitudinal):\n");
        for (std::size_t i = 0; i < v.size() && i < 25; ++i) {
            std::printf("  %-40s n=%4d lat=%+.2f lon=%+.2f\n", v[i].first.c_str(), v[i].second.n,
                        v[i].second.lat / v[i].second.n, v[i].second.lon / v[i].second.n);
        }
    }
    for (const auto& c : st.calibrationDebug) std::printf("calib: %s\n", c.c_str());
    for (const auto& e : st.errors) std::printf("error: %s\n", e.c_str());
    if (!net) return 1;
    if (!net->save(cache, mapFingerprint(o))) {
        std::fprintf(stderr, "could not write cache %s\n", cache.c_str());
        return 1;
    }
    std::printf("cache written to    %s\n", cache.c_str());
    return 0;
}

// The fingerprint is checked by the plugin; the research tool accepts any cache.
std::optional<RoadNetwork> loadAnyCache(const char* file) {
    std::ifstream in(file, std::ios::binary);
    char magic[8];
    std::uint32_t version = 0, fpLen = 0;
    in.read(magic, 8);
    in.read(reinterpret_cast<char*>(&version), 4);
    in.read(reinterpret_cast<char*>(&fpLen), 4);
    if (!in || fpLen > (1u << 20)) return std::nullopt;
    std::string fp(fpLen, '\0');
    in.read(fp.data(), fpLen);
    in.close();
    std::string err;
    auto net = RoadNetwork::load(file, fp, &err);
    if (!net) std::cerr << "load failed: " << err << "\n";
    return net;
}

int locate(int argc, char** argv) {
    if (argc < 5) return 2;
    const auto net = loadAnyCache(argv[2]);
    if (!net) return 1;
    const Vec3 world{std::stod(argv[3]), 0.0, std::stod(argv[4])};
    const Vec2 p = coords::worldToPlan(world);
    const auto matches = net->query(p, 25.0);
    std::printf("%zu lanes within 25 m of world (%.1f, %.1f)\n", matches.size(), world.x, world.z);
    for (std::size_t i = 0; i < matches.size() && i < 12; ++i) {
        const auto& m = matches[i];
        const auto& s = net->segment(m.segment);
        std::printf("  seg %u %s item %016llx lane %u%s dist %.2f xte %+.2f heading %.3f (sdk) len %.1f next %zu\n",
                    m.segment, s.kind == LaneKind::Road ? "road  " : "prefab",
                    static_cast<unsigned long long>(s.itemUid), s.laneIndex, s.leftSide ? "L" : "R", m.distance,
                    m.crossTrack, coords::yawToSdkHeading(m.yaw), s.length, s.next.size());
    }
    return 0;
}

int companies(int argc, char** argv) {
    const auto net = loadAnyCache(argv[2]);
    if (!net) return 1;
    const std::string city = argc > 3 ? argv[3] : "";
    std::size_t shown = 0;
    for (const auto& d : net->destinations()) {
        if (!city.empty() && tokenToString(d.city) != city) continue;
        if (++shown > 40) break;
        const Vec3 w = coords::planToWorld(d.position, 0.0);
        std::printf("%-14s %-14s world (%.0f, %.0f)  %zu lanes\n", tokenToString(d.city).c_str(),
                    tokenToString(d.company).c_str(), w.x, w.z, d.lanes.size());
    }
    std::printf("%zu destinations in total\n", net->destinations().size());
    return 0;
}

// route <cache> <from city> <from company> <to city> <to company>: plans between two depots.
int route(int argc, char** argv) {
    if (argc < 7) return 2;
    const auto net = loadAnyCache(argv[2]);
    if (!net) return 1;
    const Destination* from = net->findDestination(tokenFromString(argv[3]), tokenFromString(argv[4]));
    const Destination* to = net->findDestination(tokenFromString(argv[5]), tokenFromString(argv[6]));
    if (!from || !to) {
        std::fprintf(stderr, "unknown depot\n");
        return 1;
    }
    // Leave the source depot on one of the nearest road lanes, as a truck pulling out would.
    const auto t0 = std::chrono::steady_clock::now();
    Route r;
    int tried = 0;
    for (const auto& m : net->query(from->position, 150.0)) {
        if (net->segment(m.segment).kind != LaneKind::Road || net->segment(m.segment).next.empty()) continue;
        r = planRoute(*net, m.segment, m.s, to->lanes);
        if (r.found || ++tried >= 8) break;
    }
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    if (!r.found) {
        std::printf("no route: %s (%zu expanded, %.0f ms)\n", r.failure.c_str(), r.expanded, ms);
        return 1;
    }
    std::size_t laneChanges = 0;
    for (const auto& st : r.steps) laneChanges += st.laneChange ? 1 : 0;
    const double crow = distance(from->position, to->position);
    std::printf("route %.1f km (straight line %.1f km), %zu lanes, %zu lane changes, %zu expanded, %.0f ms\n",
                r.length / 1000.0, crow / 1000.0, r.steps.size(), laneChanges, r.expanded, ms);
    return 0;
}

// routefrom <cache> <x> <z> <city> <company>: route from a world position.
int routeFrom(int argc, char** argv) {
    if (argc < 7) return 2;
    const auto net = loadAnyCache(argv[2]);
    if (!net) return 1;
    const Vec2 p = coords::worldToPlan({std::stod(argv[3]), 0.0, std::stod(argv[4])});
    const Destination* to = net->findDestination(tokenFromString(argv[5]), tokenFromString(argv[6]));
    if (!to) {
        std::fprintf(stderr, "unknown depot\n");
        return 1;
    }
    for (const auto& m : net->query(p, 30.0)) {
        const auto t0 = std::chrono::steady_clock::now();
        const Route r = planRoute(*net, m.segment, m.s, to->lanes);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        std::size_t laneChanges = 0;
        for (const auto& st : r.steps) laneChanges += st.laneChange ? 1 : 0;
        std::printf("start seg %u: %s %.1f km, %zu lane changes, %zu expanded, %.0f ms %s\n", m.segment,
                    r.found ? "route" : "no route", r.length / 1000.0, laneChanges, r.expanded, ms, r.failure.c_str());
    }
    return 0;
}

// reach <cache> <x> <z>: forward reachability from the nearest road lane, with a
// sample of the dead ends that bound it.
int reach(int argc, char** argv) {
    if (argc < 5) return 2;
    const auto net = loadAnyCache(argv[2]);
    if (!net) return 1;
    const Vec2 p = coords::worldToPlan({std::stod(argv[3]), 0.0, std::stod(argv[4])});
    const auto matches = net->query(p, 100.0);
    if (matches.empty()) return 1;
    std::vector<std::uint8_t> seen(net->size(), 0);
    std::vector<std::uint32_t> stack{matches.front().segment};
    std::vector<std::uint32_t> deadEnds;
    std::size_t count = 0;
    while (!stack.empty() && count < 3000000) {
        const auto id = stack.back();
        stack.pop_back();
        if (seen[id]) continue;
        seen[id] = 1;
        ++count;
        const auto& s = net->segment(id);
        if (s.next.empty()) deadEnds.push_back(id);
        for (auto n : s.next) stack.push_back(n);
    }
    std::printf("reachable lanes: %zu, dead ends: %zu\n", count, deadEnds.size());
    // Nearest same-direction (< 30 deg) lane start for every dead end, 1 m buckets up to 30 m.
    std::vector<int> hist(31, 0);
    int roadEnds = 0, prefabEnds = 0;
    for (auto id : deadEnds) {
        const auto& s = net->segment(id);
        (s.kind == LaneKind::Road ? roadEnds : prefabEnds)++;
        const Vec2 end = s.points.back().plan();
        const Vec2 d = end - s.points[s.points.size() - 2].plan();
        const double yaw = std::atan2(d.y, d.x);
        double best = 1e9;
        for (const auto& m : net->query(end, 30.0)) {
            const auto& o = net->segment(m.segment);
            if (m.segment == id || o.points.size() < 2) continue;
            const Vec2 od = o.points[1].plan() - o.points[0].plan();
            if (std::abs(headingDifference(yaw, std::atan2(od.y, od.x))) > degToRad(30.0)) continue;
            best = std::min(best, distance(o.points.front().plan(), end));
        }
        ++hist[best < 30.0 ? static_cast<int>(best) : 30];
        static int shown = 0;
        if (best >= 30.0 && s.kind == LaneKind::Road && shown++ < 6) {
            const Vec3 w = coords::planToWorld(end, 0.0);
            std::printf("  isolated road end: item %016llx lane %u%s at (%.1f, %.1f)\n",
                        static_cast<unsigned long long>(s.itemUid), s.laneIndex, s.leftSide ? "L" : "R", w.x, w.z);
        }
    }
    std::printf("dead ends: %d road, %d prefab; nearest aligned start (1 m buckets, last = none within 30 m):\n ", roadEnds,
                prefabEnds);
    for (int h : hist) std::printf(" %d", h);
    std::printf("\n");
    for (std::size_t i = 0; i < deadEnds.size() && i < 15; ++i) {
        const auto& s = net->segment(deadEnds[i]);
        const Vec2 end = s.points.back().plan();
        const Vec2 d = end - s.points[s.points.size() - 2].plan();
        const double yaw = std::atan2(d.y, d.x);
        // Nearest lane start of any kind, and its heading difference.
        double best = 1e9, bestTurn = 0.0;
        std::uint32_t bestId = 0;
        for (const auto& m : net->query(end, 10.0)) {
            const auto& o = net->segment(m.segment);
            const double gap = distance(o.points.front().plan(), end);
            if (m.segment == deadEnds[i] || gap > best) continue;
            const Vec2 od = o.points[1].plan() - o.points[0].plan();
            best = gap;
            bestTurn = radToDeg(headingDifference(yaw, std::atan2(od.y, od.x)));
            bestId = m.segment;
        }
        const Vec3 w = coords::planToWorld(end, 0.0);
        std::printf("  dead end seg %u %s lane %u%s at (%.1f, %.1f): nearest start seg %u %s gap %.2f m turn %.0f deg\n",
                    deadEnds[i], s.kind == LaneKind::Road ? "road" : "prefab", s.laneIndex, s.leftSide ? "L" : "R", w.x,
                    w.z, bestId, best < 1e8 && net->segment(bestId).kind == LaneKind::Road ? "road" : "prefab", best,
                    bestTurn);
    }
    return 0;
}

// item <game_dir> <uid hex>: what a road/prefab is attached to, straight from the sectors.
int item(int argc, char** argv) {
    if (argc < 4) return 2;
    const std::uint64_t uid = std::stoull(argv[3], nullptr, 16);
    GameFileSystem fs;
    for (const auto& a : gameArchives(argv[2])) {
        if (auto ar = HashFsArchive::open(a)) fs.add(std::move(ar));
    }
    std::unordered_map<std::uint64_t, MapNode> nodes;
    std::unordered_map<std::uint64_t, MapRoad> roads;
    std::unordered_map<std::uint64_t, MapPrefab> prefabs;
    for (const auto& f : fs.list("map/usa").files) {
        if (f.size() < 5 || f.substr(f.size() - 5) != ".base") continue;
        const auto data = fs.read("map/usa/" + f);
        if (!data) continue;
        try {
            const SectorData s = parseSector(data->data(), data->size());
            for (const auto& n : s.nodes) nodes[n.uid] = n;
            for (const auto& r : s.roads) roads[r.uid] = r;
            for (const auto& p : s.prefabs) prefabs[p.uid] = p;
        } catch (...) {
        }
    }
    auto describe = [&](std::uint64_t id) -> std::string {
        if (id == 0) return "none";
        if (roads.count(id)) return "road " + tokenToString(roads[id].roadLook);
        if (prefabs.count(id)) return "prefab " + tokenToString(prefabs[id].model);
        return "other item";
    };
    auto showNode = [&](const char* label, std::uint64_t nodeUid) {
        const auto n = nodes.find(nodeUid);
        if (n == nodes.end()) {
            std::printf("  %s node %016llx missing\n", label, static_cast<unsigned long long>(nodeUid));
            return;
        }
        std::printf("  %s node %016llx at (%.1f, %.1f): backward %016llx (%s), forward %016llx (%s)\n", label,
                    static_cast<unsigned long long>(nodeUid), n->second.position.x, n->second.position.z,
                    static_cast<unsigned long long>(n->second.backwardItem), describe(n->second.backwardItem).c_str(),
                    static_cast<unsigned long long>(n->second.forwardItem), describe(n->second.forwardItem).c_str());
    };
    if (roads.count(uid)) {
        const auto& r = roads[uid];
        std::printf("road %016llx look %s length %.1f flags %08x\n", static_cast<unsigned long long>(uid),
                    tokenToString(r.roadLook).c_str(), r.length, r.flags);
        showNode("start", r.startNode);
        showNode("end", r.endNode);
    } else if (prefabs.count(uid)) {
        const auto& p = prefabs[uid];
        std::printf("prefab %016llx model %s origin %u\n", static_cast<unsigned long long>(uid),
                    tokenToString(p.model).c_str(), p.originIndex);
        for (auto n : p.nodes) showNode("node", n);
    } else {
        std::printf("item not found\n");
    }
    return 0;
}

}  // namespace

int runMapCommands(int argc, char** argv) {
    const std::string cmd = argv[1];
    if (cmd == "build") return build(argc, argv);
    if (cmd == "locate") return locate(argc, argv);
    if (cmd == "companies") return companies(argc, argv);
    if (cmd == "route") return route(argc, argv);
    if (cmd == "reach") return reach(argc, argv);
    if (cmd == "item") return item(argc, argv);
    if (cmd == "routefrom") return routeFrom(argc, argv);
    std::cerr << "unknown command " << cmd << "\n";
    return 2;
}

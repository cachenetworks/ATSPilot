#include <chrono>
#include <cstdio>
#include <fstream>
#include <algorithm>
#include <iostream>
#include <map>
#include <vector>
#include <string>

#include "map/MapBuilder.h"
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
    o.progress = [&](double p, const std::string& phase) {
        if (p - last >= 0.05 || p >= 1.0) {
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
        for (const auto& g : st.gapSamples) {
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
    for (const auto& e : st.errors) std::printf("error: %s\n", e.c_str());
    if (!net) return 1;
    if (!net->save(cache, mapFingerprint(o))) {
        std::fprintf(stderr, "could not write cache %s\n", cache.c_str());
        return 1;
    }
    std::printf("cache written to    %s\n", cache.c_str());
    return 0;
}

int locate(int argc, char** argv) {
    if (argc < 5) return 2;
    std::FILE* f = std::fopen(argv[2], "rb");
    if (!f) return 1;
    std::fclose(f);
    // The fingerprint is checked by the plugin; the research tool accepts any cache.
    std::string err;
    std::ifstream in(argv[2], std::ios::binary);
    char magic[8];
    std::uint32_t version = 0, fpLen = 0;
    in.read(magic, 8);
    in.read(reinterpret_cast<char*>(&version), 4);
    in.read(reinterpret_cast<char*>(&fpLen), 4);
    std::string fp(fpLen, '\0');
    in.read(fp.data(), fpLen);
    in.close();
    const auto net = RoadNetwork::load(argv[2], fp, &err);
    if (!net) {
        std::cerr << "load failed: " << err << "\n";
        return 1;
    }
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

}  // namespace

int runMapCommands(int argc, char** argv) {
    const std::string cmd = argv[1];
    if (cmd == "build") return build(argc, argv);
    if (cmd == "locate") return locate(argc, argv);
    std::cerr << "unknown command " << cmd << "\n";
    return 2;
}

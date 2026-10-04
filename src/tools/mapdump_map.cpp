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

#include "control/SpeedPlanner.h"
#include "map/HashFs.h"
#include "map/LanePlanner.h"
#include "map/MapBuilder.h"
#include "map/RoutePlanner.h"
#include "map/SectorParser.h"
#include "map/ServicePlanner.h"
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

// services <cache>: fuel pumps and weigh-station scales and the lanes passing them.
int services(int argc, char** argv) {
    (void)argc;
    const auto net = loadAnyCache(argv[2]);
    if (!net) return 1;
    std::size_t counts[2] = {0, 0}, noTrucks[2] = {0, 0}, far[2] = {0, 0}, shown = 0;
    for (const auto& sp : net->services()) {
        const int k = static_cast<int>(sp.kind);
        ++counts[k];
        const auto& seg = net->segment(sp.lane);
        if (seg.rules & LaneRule::NoTrucks) ++noTrucks[k];
        if (sp.offset > 5.0f) ++far[k];
        if (shown++ < 30 || sp.offset > 8.0f) {
            const Vec3 w = coords::planToWorld(sp.position, 0.0);
            std::printf("%-5s world (%.0f, %.0f) lane %u (%s, len %.0f m) s %.1f offset %.1f m yaw diff %.0f deg\n",
                        sp.kind == ServiceKind::Fuel ? "fuel" : "weigh", w.x, w.z, sp.lane,
                        seg.kind == LaneKind::Road ? "road" : "prefab", seg.length, sp.s, sp.offset,
                        radToDeg(headingDifference(net->project(sp.lane, sp.position).yaw, sp.yaw)));
        }
    }
    // Fuel stands by lane shape: through lanes a truck can drive past the pump on.
    std::map<std::string, int> shapes;
    for (const auto& sp : net->services()) {
        if (sp.kind != ServiceKind::Fuel) continue;
        const auto& seg = net->segment(sp.lane);
        std::string k = std::string(seg.next.empty() ? "deadend" : "through") + (seg.length < 15.0f ? " short" : " long") +
                        (sp.offset < 1.5f ? " on-lane" : sp.offset < 7.5f ? " beside" : " far");
        ++shapes[k];
    }
    for (const auto& [k, c] : shapes) std::printf("  fuel %-28s %d\n", k.c_str(), c);
    // Whether pump lanes connect to roads both ways (searching up to 60 lanes).
    auto reaches = [&](std::uint32_t from, bool forward) {
        std::vector<std::uint32_t> open{from};
        std::vector<std::uint32_t> seen{from};
        const std::uint64_t item = net->segment(from).itemUid;
        for (std::size_t i = 0; i < open.size() && i < 3000; ++i) {
            const auto& sg = net->segment(open[i]);
            if (i > 0 && sg.itemUid != item) return true;
            for (auto n : forward ? sg.next : sg.prev) {
                if (std::find(seen.begin(), seen.end(), n) != seen.end()) continue;
                seen.push_back(n);
                open.push_back(n);
            }
        }
        return false;
    };
    int both = 0, inOnly = 0, outOnly = 0, none = 0;
    for (const auto& sp : net->services()) {
        if (sp.kind != ServiceKind::Fuel) continue;
        const bool in = reaches(sp.lane, false), out = reaches(sp.lane, true);
        (in && out ? both : in ? inOnly : out ? outOnly : none)++;
        if (!in && !out && none <= 4) {
            const auto& sg = net->segment(sp.lane);
            int lanes = 0, external = 0;
            for (std::uint32_t id = 0; id < net->size(); ++id) {
                const auto& o = net->segment(id);
                if (o.itemUid != sg.itemUid) continue;
                ++lanes;
                for (auto n : o.next) external += net->segment(n).itemUid != sg.itemUid ? 1 : 0;
                for (auto n : o.prev) external += net->segment(n).itemUid != sg.itemUid ? 1 : 0;
            }
            const Vec3 w = coords::planToWorld(sp.position, 0.0);
            std::printf("  isolated pump at world (%.0f, %.0f): lane %u of item %016llx (%d lanes, %d external links), "
                        "lane prev %zu next %zu\n",
                        w.x, w.z, sp.lane, static_cast<unsigned long long>(sg.itemUid), lanes, external, sg.prev.size(),
                        sg.next.size());
        }
    }
    std::printf("fuel lanes reachable from a road and back: %d, in only %d, out only %d, neither %d\n", both, inOnly,
                outOnly, none);
    // How far isolated pumps are from a connected pump, and from any truck-usable road lane.
    std::vector<Vec2> connected;
    for (const auto& sp : net->services()) {
        if (sp.kind == ServiceKind::Fuel && reaches(sp.lane, false) && reaches(sp.lane, true)) {
            connected.push_back(sp.position);
            if (connected.size() % 40 == 1) {
                const Destination* nearest = nullptr;
                for (const auto& d : net->destinations()) {
                    if (!nearest || distance(d.position, sp.position) < distance(nearest->position, sp.position)) nearest = &d;
                }
                const Vec3 w = coords::planToWorld(sp.position, 0.0);
                std::printf("  drivable pump at world (%.0f, %.0f), nearest depot %s %s\n", w.x, w.z,
                            nearest ? tokenToString(nearest->city).c_str() : "-",
                            nearest ? tokenToString(nearest->company).c_str() : "-");
            }
        }
    }
    int near100 = 0, near500 = 0, roadNear40 = 0, isolated = 0;
    for (const auto& sp : net->services()) {
        if (sp.kind != ServiceKind::Fuel || (reaches(sp.lane, false) && reaches(sp.lane, true))) continue;
        ++isolated;
        double best = 1e18;
        for (const auto& c : connected) best = std::min(best, distance(c, sp.position));
        near100 += best < 100.0;
        near500 += best < 500.0;
        for (const auto& m : net->query(sp.position, 40.0)) {
            if (net->segment(m.segment).kind == LaneKind::Road) {
                ++roadNear40;
                break;
            }
        }
    }
    std::printf("isolated pumps %d: connected pump within 100 m %d, within 500 m %d; a road lane within 40 m %d; "
                "%zu connected pumps\n",
                isolated, near100, near500, roadNear40, connected.size());
    std::printf("fuel %zu (%zu car-only lanes, %zu over 5 m), weigh %zu (%zu car-only, %zu over 5 m)\n", counts[0],
                noTrucks[0], far[0], counts[1], noTrucks[1], far[1]);
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
        // ATSPILOT_DUMP_SERVICES=weigh|fuel plans via weigh stations / a fuel pump as the plugin does.
        if (const char* sv = std::getenv("ATSPILOT_DUMP_SERVICES")) {
            ServicePlanOptions so;
            so.weigh = true;
            so.fuel = std::string(sv) == "fuel";
            r = planRouteWithServices(*net, m.segment, m.s, to->lanes, RouteOptions{}, 0.0, 0.0, so);
        } else {
            r = planRoute(*net, m.segment, m.s, to->lanes);
        }
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
    for (const auto& svc : r.services) {
        double along = 0.0;
        for (const auto& st : r.steps) {
            if (st.segment == svc.lane) break;
            if (!st.laneChange) along += net->segment(st.segment).length;
        }
        const Vec3 w = coords::planToWorld(net->segment(svc.lane).points.front().plan(), 0.0);
        std::printf("  %s stop after %.1f km at world (%.0f, %.0f)\n", svc.kind == ServiceKind::Fuel ? "fuel" : "weigh",
                    along / 1000.0, w.x, w.z);
    }

    // Optional: list the turn signals along the route, as the planner would drive it.
    if (argc > 7 && std::string(argv[7]) == "--signals") {
        LaneMatch at = net->project(r.steps.front().segment, net->segment(r.steps.front().segment).points.front().plan());
        double offset = 0.0;
        std::vector<std::uint32_t> chain;
        std::vector<std::pair<double, Indication>> seen;
        for (int window = 0; window < 400; ++window) {
            PathBuildParams pp;
            const PlannedPath p = buildPlannedPath(*net, at, pp, chain, &r);
            if (!p.path.valid()) break;
            if (std::getenv("ATSPILOT_DUMP_CHAIN") && window == 0) {
                double acc = 0.0;
                for (auto id : p.chain) {
                    const auto& sg = net->segment(id);
                    std::printf("    chain seg %u %s item %016llx len %.0f prev %zu next %zu at %.0f m\n", id,
                                sg.kind == LaneKind::Road ? "road  " : "prefab", static_cast<unsigned long long>(sg.itemUid),
                                sg.length, sg.prev.size(), sg.next.size(), acc);
                    acc += sg.length;
                }
            }
            for (const auto& ind : p.indications) {
                const double absStart = offset + (ind.sStart - p.truckS);
                bool dup = false;
                for (const auto& [s0, i0] : seen) {
                    dup = dup || (std::abs(s0 - absStart) < 25.0 && i0.side == ind.side && i0.kind == ind.kind);
                }
                if (dup) continue;
                seen.push_back({absStart, ind});
                // Heading change over the 150 m after the manoeuvre point, as a sanity check.
                const double lead = ind.kind == IndicationKind::Exit || ind.kind == IndicationKind::Merge ? 150.0
                                    : ind.kind == IndicationKind::Turn                                    ? 60.0
                                                                                                          : 40.0;
                const double at0 = std::clamp(ind.sStart + lead, 0.0, p.path.length());
                const double at1 = std::clamp(at0 + 150.0, 0.0, p.path.length());
                std::printf("      heading %+4.0f deg over the next 150 m, at world (%.0f, %.0f)\n",
                            radToDeg(headingDifference(p.path.yawAt(at0), p.path.yawAt(at1))),
                            p.path.positionAt(at0).x, -p.path.positionAt(at0).y);
                std::printf("  %7.0f m  %-5s %-11s for %.0f m\n", absStart, ind.side > 0 ? "left" : "right",
                            toString(ind.kind), ind.sEnd - ind.sStart);
            }
            if (const char* at = std::getenv("ATSPILOT_DUMP_AT")) {
                // Path points within 30 m of a route distance, with lateral offset from
                // the chord, to see what a kink is made of.
                const double target = std::atof(at) - offset + p.truckS;
                if (target > p.truckS && target < p.truckS + 400.0) {
                    const Vec2 a = p.path.positionAt(target - 30.0), b = p.path.positionAt(target + 30.0);
                    const Vec2 d = (b - a).normalized();
                    for (std::size_t i = 0; i < p.path.size(); ++i) {
                        const auto& pt = p.path[i];
                        if (std::abs(pt.s - target) > 30.0) continue;
                        std::printf("    pt s %7.1f seg %8llu lateral %+6.2f\n", offset + pt.s - p.truckS,
                                    static_cast<unsigned long long>(pt.segmentId), cross(d, pt.pos - a));
                    }
                }
            }
            if (std::getenv("ATSPILOT_DUMP_CURVES")) {
                // Curve speed (1.6 m/s^2) where it is low, and the real heading change
                // around that point: a tight radius with little heading change is a kink.
                const double end = std::min(p.path.length() - 20.0, p.truckS + 400.0);
                for (double s = p.truckS; s < end; s += 5.0) {
                    const double k = p.path.curvatureAt(s, 12.0);
                    const double v = curveSpeed(k, 1.6, 4.0);
                    if (v > 20.0) continue;
                    const double turn = radToDeg(headingDifference(p.path.yawAt(std::max(0.0, s - 40.0)),
                                                                   p.path.yawAt(std::min(p.path.length(), s + 40.0))));
                    const double impliedTurn = radToDeg(80.0 * std::abs(k));
                    std::printf("    curve at %7.0f m: R %5.0f m -> %4.1f m/s; heading change over 80 m %+5.0f deg (R implies %3.0f)%s\n",
                                offset + s - p.truckS, 1.0 / std::max(1e-6, std::abs(k)), v, turn, impliedTurn,
                                std::abs(turn) < 0.4 * impliedTurn ? "  <-- KINK" : "");
                }
            }
            const double step = std::min(400.0, p.path.length() - p.truckS - 50.0);
            if (step < 20.0) break;
            const double s = p.truckS + step;
            const auto idx = p.path.segmentIndexAt(s);
            const std::uint32_t seg = static_cast<std::uint32_t>(p.path[idx].segmentId);
            at = net->project(seg, p.path.positionAt(s));
            chain = p.chain;
            offset += step;
        }
        std::printf("%zu signals\n", seen.size());
        if (std::getenv("ATSPILOT_DUMP_JOINS")) {
            double acc = 0.0;
            for (std::size_t i = 1; i < r.steps.size(); ++i) {
                const auto& st = r.steps[i];
                if (!st.laneChange && net->segment(st.segment).prev.size() > 1) {
                    std::printf("  join at %6.0f m (%s, %zu lanes in):", acc,
                                net->segment(st.segment).kind == LaneKind::Road ? "road" : "prefab",
                                net->segment(st.segment).prev.size());
                    for (const auto& j : joinApproaches(*net, st.segment)) {
                        std::printf("  %s%+.1f m%s", j.lane == r.steps[i - 1].segment ? "*" : "", j.offset,
                                    j.parallel ? " parallel" : "");
                    }
                    std::printf("\n");
                }
                if (!st.laneChange) acc += net->segment(st.segment).length;
            }
        }
    }
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
    if (cmd == "services") return services(argc, argv);
    if (cmd == "route") return route(argc, argv);
    if (cmd == "reach") return reach(argc, argv);
    if (cmd == "item") return item(argc, argv);
    if (cmd == "routefrom") return routeFrom(argc, argv);
    std::cerr << "unknown command " << cmd << "\n";
    return 2;
}

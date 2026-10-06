#include "map/RoutePlanner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_set>

namespace atspilot {

GpsCorridor::GpsCorridor(std::vector<Vec2> points) : points_(std::move(points)) {
    for (std::size_t i = 0; i + 1 < points_.size(); ++i) {
        const Vec2& a = points_[i];
        const Vec2& b = points_[i + 1];
        const auto x0 = static_cast<std::int64_t>(std::floor(std::min(a.x, b.x) / kCell));
        const auto x1 = static_cast<std::int64_t>(std::floor(std::max(a.x, b.x) / kCell));
        const auto y0 = static_cast<std::int64_t>(std::floor(std::min(a.y, b.y) / kCell));
        const auto y1 = static_cast<std::int64_t>(std::floor(std::max(a.y, b.y) / kCell));
        if ((x1 - x0 + 1) * (y1 - y0 + 1) > 4096) continue;  // a teleport-like jump: not a road
        for (auto ix = x0; ix <= x1; ++ix) {
            for (auto iy = y0; iy <= y1; ++iy) grid_[key(ix, iy)].push_back(static_cast<std::uint32_t>(i));
        }
    }
}

double GpsCorridor::distanceTo(const Vec2& p) const {
    const auto cx = static_cast<std::int64_t>(std::floor(p.x / kCell));
    const auto cy = static_cast<std::int64_t>(std::floor(p.y / kCell));
    double best = kFar;
    for (std::int64_t dx = -1; dx <= 1; ++dx) {
        for (std::int64_t dy = -1; dy <= 1; ++dy) {
            const auto it = grid_.find(key(cx + dx, cy + dy));
            if (it == grid_.end()) continue;
            for (const auto i : it->second) {
                const Vec2& a = points_[i];
                const Vec2& b = points_[i + 1];
                const Vec2 ab = b - a;
                const double len2 = dot(ab, ab);
                const double t = len2 > 1e-9 ? std::clamp(dot(p - a, ab) / len2, 0.0, 1.0) : 0.0;
                best = std::min(best, distance(p, a + ab * t));
            }
        }
    }
    return best;
}

int Route::find(std::uint32_t segment, int from) const {
    for (int i = std::max(0, from); i < static_cast<int>(steps.size()); ++i) {
        if (steps[static_cast<std::size_t>(i)].segment == segment) return i;
    }
    return -1;
}

Route planRoute(const RoadNetwork& net, std::uint32_t startSegment, double startS,
                const std::vector<std::uint32_t>& goals, const RouteOptions& options) {
    Route route;
    route.startS = startS;
    if (goals.empty() || startSegment >= net.size()) {
        route.failure = "no destination lanes";
        return route;
    }

    // Goal area: centroid and radius of all goal lane points, so the heuristic
    // stays a lower bound for every goal lane.
    Vec2 centre;
    std::size_t pts = 0;
    for (auto g : goals) {
        for (const auto& p : net.segment(g).points) {
            centre += p.plan();
            ++pts;
        }
    }
    if (pts == 0) {
        route.failure = "destination lanes have no geometry";
        return route;
    }
    centre = centre / static_cast<double>(pts);
    double radius = 0.0;
    for (auto g : goals) {
        for (const auto& p : net.segment(g).points) radius = std::max(radius, distance(p.plan(), centre));
    }
    const std::unordered_set<std::uint32_t> goalSet(goals.begin(), goals.end());

    auto heuristic = [&](std::uint32_t id) {
        const auto& s = net.segment(id);
        return std::max(0.0, distance(s.points.back().plan(), centre) - radius);
    };

    constexpr float kInf = std::numeric_limits<float>::infinity();
    constexpr std::uint32_t kNone = std::numeric_limits<std::uint32_t>::max();
    std::vector<float> g(net.size(), kInf);
    std::vector<std::uint32_t> parent(net.size(), kNone);
    std::vector<std::uint8_t> viaLaneChange(net.size(), 0);
    std::vector<std::uint8_t> closed(net.size(), 0);

    using Entry = std::pair<double, std::uint32_t>;  // f, segment
    std::priority_queue<Entry, std::vector<Entry>, std::greater<>> open;

    // Cost to the end of the start segment.
    g[startSegment] = static_cast<float>(std::max(0.0, net.segment(startSegment).length - startS));
    open.push({g[startSegment] + heuristic(startSegment), startSegment});

    std::uint32_t reached = kNone;
    while (!open.empty()) {
        const auto [f, id] = open.top();
        open.pop();
        if (closed[id]) continue;
        closed[id] = 1;
        if (++route.expanded > options.maxExpansions) {
            route.failure = "search limit reached";
            return route;
        }
        if (goalSet.count(id)) {
            reached = id;
            break;
        }
        auto relax = [&](std::uint32_t next, double cost, bool laneChange) {
            if (closed[next] || net.segment(next).points.size() < 2) return;
            if (net.segment(next).rules & LaneRule::NoTrucks) return;
            const double ng = g[id] + cost;
            if (ng < g[next]) {
                g[next] = static_cast<float>(ng);
                parent[next] = id;
                viaLaneChange[next] = laneChange ? 1 : 0;
                open.push({ng + heuristic(next), next});
            }
        };
        auto laneCost = [&](std::uint32_t n) {
            const auto& seg = net.segment(n);
            double cost = seg.length;
            if (options.corridor && seg.points.size() >= 2) {
                const Vec2 mid = seg.points[seg.points.size() / 2].plan();
                const Vec2 end = seg.points.back().plan();
                if (options.corridor->distanceTo(mid) > options.corridorWidth ||
                    options.corridor->distanceTo(end) > options.corridorWidth) {
                    cost *= options.offCorridorFactor;
                }
            }
            return cost;
        };
        for (auto n : net.segment(id).next) relax(n, laneCost(n), false);
        // A lane change keeps the position along the road, so it only costs the
        // penalty. It still needs real longitudinal room to happen. Also do not
        // stack lane-change edges at the same longitudinal position: moving two
        // lanes over requires another road segment between the changes.
        const double laneChangeRoom =
            id == startSegment ? std::max(0.0, static_cast<double>(net.segment(id).length) - startS)
                               : static_cast<double>(net.segment(id).length);
        if (!viaLaneChange[id] && laneChangeRoom >= kMinLaneChangeRoom) {
            for (auto n : net.laneNeighbors(id)) relax(n, options.laneChangeCost, true);
        }
    }

    if (reached == kNone) {
        route.failure = "destination not reachable from the current lane";
        return route;
    }
    for (std::uint32_t cur = reached; cur != kNone; cur = parent[cur]) {
        route.steps.push_back({cur, viaLaneChange[cur] != 0});
        if (cur == startSegment) break;
    }
    std::reverse(route.steps.begin(), route.steps.end());
    for (const auto& s : route.steps) {
        if (!s.laneChange) route.length += net.segment(s.segment).length;
    }
    route.length -= std::min(route.length, startS);
    route.found = true;
    return route;
}

Route planRouteMatching(const RoadNetwork& net, std::uint32_t startSegment, double startS,
                        const std::vector<std::uint32_t>& goals, double targetLength, double tolerance,
                        const RouteOptions& options, int maxBranches) {
    Route best = planRoute(net, startSegment, startS, goals, options);
    if (!best.found || targetLength <= 0.0) return best;
    const double allowed = std::max(150.0, tolerance * targetLength);
    double bestError = std::abs(best.length - targetLength);
    if (bestError <= allowed) {
        best.gpsMatched = true;
        return best;
    }

    const Route shortest = best;
    double prefix = net.segment(startSegment).length - startS;
    int branches = 0;
    for (std::size_t i = 0; i + 1 < shortest.steps.size() && branches < maxBranches; ++i) {
        const std::uint32_t at = shortest.steps[i].segment;
        const RouteStep& taken = shortest.steps[i + 1];
        if (i > 0 && !shortest.steps[i].laneChange) prefix += net.segment(at).length;
        if (taken.laneChange || net.segment(at).next.size() < 2) continue;
        ++branches;
        for (auto alt : net.segment(at).next) {
            if (alt == taken.segment || (net.segment(alt).rules & LaneRule::NoTrucks)) continue;
            Route tail = planRoute(net, alt, 0.0, goals, options);
            if (!tail.found) continue;
            const double total = prefix + tail.length;
            const double error = std::abs(total - targetLength);
            if (error < bestError) {
                Route r;
                r.found = true;
                r.startS = startS;
                r.steps.assign(shortest.steps.begin(), shortest.steps.begin() + static_cast<std::ptrdiff_t>(i) + 1);
                r.steps.insert(r.steps.end(), tail.steps.begin(), tail.steps.end());
                r.length = total;
                r.expanded = shortest.expanded + tail.expanded;
                bestError = error;
                best = std::move(r);
            }
        }
    }
    best.gpsMatched = bestError <= allowed;
    return best;
}

}  // namespace atspilot

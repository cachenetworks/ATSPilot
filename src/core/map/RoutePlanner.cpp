#include "map/RoutePlanner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_set>

namespace atspilot {

int Route::find(std::uint32_t segment, int from) const {
    for (int i = std::max(0, from); i < static_cast<int>(steps.size()); ++i) {
        if (steps[static_cast<std::size_t>(i)].segment == segment) return i;
    }
    return -1;
}

Route planRoute(const RoadNetwork& net, std::uint32_t startSegment, double startS,
                const std::vector<std::uint32_t>& goals, const RouteOptions& options) {
    Route route;
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
        for (auto n : net.segment(id).next) relax(n, net.segment(n).length, false);
        // A lane change keeps the position along the road, so it only costs the penalty.
        for (auto n : net.laneNeighbors(id)) relax(n, options.laneChangeCost, true);
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

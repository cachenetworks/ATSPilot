#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "map/RoadNetwork.h"

namespace atspilot {

struct RouteStep {
    std::uint32_t segment = 0;
    bool laneChange = false;  // reached by moving sideways from the previous step's lane
};

// A stop on the route for a service: a fuel pump or a weigh station scale.
struct RouteService {
    std::uint32_t lane = 0;
    double s = 0.0;  // along the lane
    ServiceKind kind = ServiceKind::Fuel;
};

struct Route {
    bool found = false;
    std::vector<RouteStep> steps;
    double startS = 0.0;        // longitudinal position where this route was calculated
    double length = 0.0;       // metres along lanes, excluding lane-change penalties
    std::size_t expanded = 0;  // search effort, for diagnostics
    std::string failure;
    bool gpsMatched = false;   // total length agrees with the in-game navigation distance
    std::vector<RouteService> services;  // fuel and weigh stops on the way, in route order

    // Index of `segment` in steps at or after `from`, or -1.
    int find(std::uint32_t segment, int from = 0) const;
};

// A lane change needs enough road to establish the manoeuvre, blend across the
// lane boundary and settle before the source segment ends. Routes that discover
// a change later than this should miss the turn and recalculate instead of
// forcing a sideways correction beside the truck.
inline constexpr double kMinLaneChangeRoom = 60.0;

// The in-game GPS route as a polyline (plan coordinates, truck to destination),
// with a grid index for distance queries.
class GpsCorridor {
public:
    explicit GpsCorridor(std::vector<Vec2> points);

    bool empty() const { return points_.size() < 2; }
    const std::vector<Vec2>& points() const { return points_; }
    // Distance from p to the polyline; anything beyond about one grid cell is
    // reported as `kFar`.
    double distanceTo(const Vec2& p) const;

    static constexpr double kFar = 1e9;

private:
    static constexpr double kCell = 64.0;
    static std::uint64_t key(std::int64_t ix, std::int64_t iy) {
        return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(ix)) << 32) | static_cast<std::uint32_t>(iy);
    }
    std::vector<Vec2> points_;
    std::unordered_map<std::uint64_t, std::vector<std::uint32_t>> grid_;  // cell -> polyline segments
};

struct RouteOptions {
    double laneChangeCost = 60.0;      // metres of driving a lane change is "worth"
    std::size_t maxExpansions = 4000000;
    // When set, lanes away from the in-game GPS route cost `offCorridorFactor`
    // times their length, so the search follows the game's own route. Costs only
    // ever grow, so the straight-line heuristic stays admissible.
    std::shared_ptr<const GpsCorridor> corridor;
    double corridorWidth = 18.0;       // m from the GPS polyline still counted as on it
    double offCorridorFactor = 30.0;
};

// A* over the lane graph from (startSegment, startS) to any of `goals`.
//
// Edges are lane successors (cost: lane length) and lane changes to a parallel
// lane of the same road (cost: laneChangeCost). The heuristic is the straight-line
// distance to the goal area, which never exceeds the true remaining driving
// distance, so the search is admissible and only returns routes that exist in
// the graph.
Route planRoute(const RoadNetwork& net, std::uint32_t startSegment, double startS,
                const std::vector<std::uint32_t>& goals, const RouteOptions& options = {});

// Like planRoute, but the result should be `targetLength` long: the in-game GPS
// reports only its remaining distance, so among the shortest route and the
// routes taking a different branch at one of the next `maxBranches` forks, the
// one whose length matches that distance is taken to be the GPS route.
Route planRouteMatching(const RoadNetwork& net, std::uint32_t startSegment, double startS,
                        const std::vector<std::uint32_t>& goals, double targetLength, double tolerance,
                        const RouteOptions& options = {}, int maxBranches = 6);

}  // namespace atspilot

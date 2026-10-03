#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "map/RoadNetwork.h"

namespace atspilot {

struct RouteStep {
    std::uint32_t segment = 0;
    bool laneChange = false;  // reached by moving sideways from the previous step's lane
};

struct Route {
    bool found = false;
    std::vector<RouteStep> steps;
    double length = 0.0;       // metres along lanes, excluding lane-change penalties
    std::size_t expanded = 0;  // search effort, for diagnostics
    std::string failure;

    // Index of `segment` in steps at or after `from`, or -1.
    int find(std::uint32_t segment, int from = 0) const;
};

struct RouteOptions {
    double laneChangeCost = 60.0;      // metres of driving a lane change is "worth"
    std::size_t maxExpansions = 4000000;
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

}  // namespace atspilot

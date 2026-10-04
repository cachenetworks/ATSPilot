#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "map/RoadNetwork.h"
#include "map/RoutePlanner.h"
#include "pilot/PilotTypes.h"

namespace atspilot {

struct LocalizationResult {
    bool valid = false;
    LaneMatch match;
    double headingError = 0.0;
    std::string reason;
};

// Map matching: chooses the lane the truck is driving in. Candidates are scored
// by lateral distance and heading, with strong preference for lanes on the
// currently planned chain so the result does not flicker between overlapping
// junction curves or nearby parallel roads.
class Localizer {
public:
    struct Params {
        double maxDistance = 12.0;            // m
        double maxHeadingError = 0.7;         // rad (~40 degrees)
        double headingWeight = 8.0;           // m per rad
        double plannedBonus = 3.0;            // m
        double switchMargin = 1.0;            // m a new lane must win by
    };

    explicit Localizer(const RoadNetwork& net) : net_(net) {}
    void setParams(const Params& p) { params_ = p; }
    void reset() { previous_.reset(); }

    LocalizationResult update(const Vec2& pos, double yaw, const std::vector<std::uint32_t>& plannedChain);

private:
    const RoadNetwork& net_;
    Params params_;
    std::optional<std::uint32_t> previous_;
};

struct PathBuildParams {
    double ahead = 600.0;    // m
    double behind = 30.0;    // m
    double spacing = 2.0;    // m between output points
    double choiceHorizon = 40.0;  // m into each successor used to judge "straight ahead"
};

struct PlannedPath {
    std::vector<std::uint32_t> chain;  // lane segments in driving order
    Path path;
    double truckS = 0.0;               // arc length of the truck's projection on `path`
    std::string nextManeuver;
    double nextManeuverDistance = 0.0;
    bool onRoute = false;          // path follows the navigation route
    double routeRemaining = 0.0;   // m, when on route
    std::vector<PathStop> stops;   // controlled junction lanes on the path (s on `path`)
};

// Builds the rolling driving path along the lane graph. With a route that
// contains the truck's lane, the path follows the route (including lane
// changes). Otherwise each branch continues on the straightest successor
// ("follow the road") and keeps the previous plan's choice so the path is stable.
PlannedPath buildPlannedPath(const RoadNetwork& net, const LaneMatch& start, const PathBuildParams& params,
                             const std::vector<std::uint32_t>& previousChain, const Route* route = nullptr);

}  // namespace atspilot

#pragma once

#include <cstdint>
#include <vector>

#include "map/RoadNetwork.h"
#include "map/RoutePlanner.h"

namespace atspilot {

struct ServicePlanOptions {
    bool fuel = false;               // route via a fuel pump first
    bool weigh = true;               // pull into weigh stations along the route
    double fuelSearchRadius = 15000.0;  // m (straight line) a pump may be from the truck
    double fuelMaxDistance = 25000.0;   // m of driving to the pump at most
    double weighCatchment = 150.0;   // m from the route a weigh station scale may be
    double weighMaxDetour = 3000.0;  // m a weigh station may add to the route
    double minLead = 250.0;          // m of route needed to still reach a weigh station's ramp
    std::vector<std::uint32_t> skipLanes;  // service lanes already visited (or that failed)
};

// Plans from (startSegment, startS) to `goals` like planRoute, then:
//
//   - with `fuel`, via the fuel pump nearest by driving distance; with no
//     goals the route ends at the pump;
//   - with `weigh`, via the first weigh station scale that lies beside the
//     route ahead in its direction of travel, when the detour is small.
//
// The services on the result are listed in Route::services. `targetLength`
// > 0 matches the in-game GPS distance as planRouteMatching does (main route only).
Route planRouteWithServices(const RoadNetwork& net, std::uint32_t startSegment, double startS,
                            const std::vector<std::uint32_t>& goals, const RouteOptions& options,
                            double targetLength, double tolerance, const ServicePlanOptions& services);

}  // namespace atspilot

#include "map/ServicePlanner.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "math/MathUtil.h"

namespace atspilot {
namespace {

bool skipped(const ServicePlanOptions& o, std::uint32_t lane) {
    return std::find(o.skipLanes.begin(), o.skipLanes.end(), lane) != o.skipLanes.end();
}

// leg1 ends on the service lane, leg2 starts there.
Route splice(const RoadNetwork& net, Route leg1, const Route& leg2, const RouteService& svc) {
    const double laneRest = std::max(0.0, static_cast<double>(net.segment(svc.lane).length) - svc.s);
    leg1.steps.insert(leg1.steps.end(), leg2.steps.begin() + (leg2.steps.empty() ? 0 : 1), leg2.steps.end());
    leg1.length = std::max(0.0, leg1.length - laneRest) + leg2.length;
    leg1.expanded += leg2.expanded;
    leg1.services.push_back(svc);
    return leg1;
}

Vec2 lanePoint(const RoadNetwork& net, std::uint32_t lane, double s) {
    const auto& pts = net.segment(lane).points;
    double along = 0.0;
    for (std::size_t i = 1; i < pts.size(); ++i) {
        const double d = distance(pts[i - 1].plan(), pts[i].plan());
        if (along + d >= s) return lerp(pts[i - 1].plan(), pts[i].plan(), d > 1e-9 ? (s - along) / d : 0.0);
        along += d;
    }
    return pts.empty() ? Vec2{} : pts.back().plan();
}

Route viaFuel(const RoadNetwork& net, std::uint32_t startSegment, double startS, const std::vector<std::uint32_t>& goals,
              const RouteOptions& options, const ServicePlanOptions& o) {
    const Vec2 here = lanePoint(net, startSegment, startS);
    std::vector<std::uint32_t> lanes;
    for (const auto& sp : net.services()) {
        if (sp.kind != ServiceKind::Fuel || skipped(o, sp.lane)) continue;
        const auto& seg = net.segment(sp.lane);
        // A truck must be able to drive in and on past the pump along mapped lanes;
        // pumps in open lots (most stations) have lanes that no road reaches.
        if (seg.next.empty() || seg.prev.empty() || (seg.rules & LaneRule::NoTrucks) || sp.offset > 7.5f) continue;
        if (distance(sp.position, here) > o.fuelSearchRadius) continue;
        if (sp.lane == startSegment && sp.s < startS + 30.0) continue;  // too close to stop for
        lanes.push_back(sp.lane);
    }
    Route r;
    if (lanes.empty()) {
        r.failure = "no fuel station within range";
        return r;
    }
    RouteOptions o1 = options;
    o1.corridor = nullptr;  // the way to the pump is off the GPS route
    o1.maxExpansions = 600000;
    Route leg1 = planRoute(net, startSegment, startS, lanes, o1);
    if (!leg1.found) return leg1;
    if (leg1.length > o.fuelMaxDistance) {
        Route far;
        far.failure = "nearest fuel station ATSPilot can drive into is too far";
        return far;
    }
    const std::uint32_t lane = leg1.steps.back().segment;
    RouteService svc;
    svc.lane = lane;
    svc.kind = ServiceKind::Fuel;
    svc.s = std::numeric_limits<double>::max();
    for (const auto& sp : net.services()) {
        if (sp.kind == ServiceKind::Fuel && sp.lane == lane) svc.s = std::min(svc.s, static_cast<double>(sp.s));
    }
    if (goals.empty()) {
        leg1.services.push_back(svc);
        return leg1;
    }
    const Route leg2 = planRoute(net, lane, svc.s, goals, options);
    if (!leg2.found) {
        leg1.services.push_back(svc);  // to the pump; the rest is planned after refuelling
        return leg1;
    }
    return splice(net, std::move(leg1), leg2, svc);
}

}  // namespace

Route planRouteWithServices(const RoadNetwork& net, std::uint32_t startSegment, double startS,
                            const std::vector<std::uint32_t>& goals, const RouteOptions& options,
                            double targetLength, double tolerance, const ServicePlanOptions& o) {
    if (o.fuel) {
        Route r = viaFuel(net, startSegment, startS, goals, options, o);
        if (r.found || goals.empty()) return r;
    }
    if (goals.empty()) {
        Route r;
        r.failure = "no destination";
        return r;
    }
    Route main = targetLength > 0.0 ? planRouteMatching(net, startSegment, startS, goals, targetLength, tolerance, options)
                                    : planRoute(net, startSegment, startS, goals, options);
    if (!main.found || !o.weigh) return main;

    // Weigh stations the route already passes over.
    for (const auto& sp : net.services()) {
        if (sp.kind != ServiceKind::Weigh || skipped(o, sp.lane)) continue;
        if (main.find(sp.lane) >= 0) main.services.push_back({sp.lane, sp.s, ServiceKind::Weigh});
    }
    if (!main.services.empty()) return main;

    // The first weigh station beside the route ahead, in the direction of travel.
    const ServicePoint* best = nullptr;
    double along = -startS;
    for (std::size_t i = 0; i < main.steps.size() && !best; ++i) {
        const auto& seg = net.segment(main.steps[i].segment);
        if (along >= o.minLead && seg.points.size() >= 2) {
            const Vec2 a = seg.points.front().plan();
            const Vec2 b = seg.points.back().plan();
            const double laneYaw = std::atan2(b.y - a.y, b.x - a.x);
            for (const auto& sp : net.services()) {
                if (sp.kind != ServiceKind::Weigh || skipped(o, sp.lane)) continue;
                if (distance(sp.position, a) > o.weighCatchment + seg.length) continue;
                const LaneMatch scale = net.project(sp.lane, sp.position);
                if (std::cos(headingDifference(scale.yaw, laneYaw)) < 0.7) continue;
                if (net.project(main.steps[i].segment, sp.position).distance > o.weighCatchment) continue;
                best = &sp;
                break;
            }
        }
        if (!main.steps[i].laneChange) along += seg.length;
    }
    if (!best) return main;

    RouteOptions o1 = options;
    o1.corridor = nullptr;
    Route leg1 = planRoute(net, startSegment, startS, {best->lane}, o1);
    if (!leg1.found) return main;
    const RouteService svc{best->lane, best->s, ServiceKind::Weigh};
    const Route leg2 = planRoute(net, best->lane, best->s, goals, options);
    if (!leg2.found) return main;
    Route via = splice(net, std::move(leg1), leg2, svc);
    if (via.length > main.length + o.weighMaxDetour) return main;
    via.gpsMatched = false;
    return via;
}

}  // namespace atspilot

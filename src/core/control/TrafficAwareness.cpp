#include "control/TrafficAwareness.h"

#include <algorithm>
#include <array>
#include <cmath>

#include "math/MathUtil.h"

namespace atspilot {

const char* toString(LightState s) {
    switch (s) {
        case LightState::Unknown: return "unknown";
        case LightState::Off: return "off";
        case LightState::Red: return "red";
        case LightState::AmberToRed: return "amber";
        case LightState::AmberToGreen: return "red-amber";
        case LightState::Green: return "green";
        case LightState::Flashing: return "flashing";
    }
    return "?";
}

namespace {

Vec2 direction(double yaw) { return {std::cos(yaw), std::sin(yaw)}; }

// Corners and edge midpoints of a vehicle footprint centred at `c`.
std::array<Vec2, 8> footprint(const Vec2& c, double yaw, double length, double width) {
    const Vec2 f = direction(yaw) * (0.5 * length);
    const Vec2 l = direction(yaw + 0.5 * kPi) * (0.5 * width);
    return {c + f + l, c + f - l, c - f + l, c - f - l, c + f, c - f, c + l, c - l};
}

struct Occupancy {
    double sMin = 0.0;
    double sMax = 0.0;
    bool inCorridor = false;
};

// Where along the path a footprint lies, and whether it reaches into the corridor.
Occupancy occupancy(const Path& path, const std::array<Vec2, 8>& pts, std::size_t hint, double halfWidth) {
    Occupancy o;
    o.sMin = 1e18;
    o.sMax = -1e18;
    double eMin = 1e18, eMax = -1e18;
    for (const Vec2& q : pts) {
        const auto pr = path.project(q, hint, 20);
        if (!pr) continue;
        o.sMin = std::min(o.sMin, pr->s);
        o.sMax = std::max(o.sMax, pr->s);
        eMin = std::min(eMin, pr->crossTrackError);
        eMax = std::max(eMax, pr->crossTrackError);
    }
    // The footprint overlaps the corridor when its lateral interval does.
    o.inCorridor = o.sMax >= o.sMin && eMax >= -halfWidth && eMin <= halfWidth;
    return o;
}

// Time for the truck to cover `d` metres from `speed`, accelerating from low speed.
double arrivalTime(double d, double speed, double accel) {
    if (d <= 0.0) return 0.0;
    if (speed > 3.0) return d / speed;
    return (-speed + std::sqrt(speed * speed + 2.0 * accel * d)) / accel;
}

}  // namespace

TrafficPicture assessTraffic(const Path& path, double frontS, double speed, const std::vector<WorldVehicle>& vehicles,
                             const TrafficParams& p) {
    TrafficPicture out;
    if (!path.valid()) return out;
    const double sEnd = std::min(path.length(), frontS + p.horizon);
    const Vec2 front = path.positionAt(std::clamp(frontS, 0.0, path.length()));

    auto consider = [&](const PathObstacle& ob) {
        if (!out.nearest || ob.s < out.nearest->s) out.nearest = ob;
        // Follow at a time gap behind a moving vehicle; stop short of a stopped or crossing one.
        const double v = std::max(0.0, ob.speed);
        const double target = ob.s - p.standstillGap - p.timeGap * v;
        out.constraints.push_back({target, v});
    };

    for (const WorldVehicle& v : vehicles) {
        const double reach = 0.5 * v.length + v.speed * p.predictionTime + p.horizon;
        if (distance(v.position, front) > reach) continue;

        const auto centre = path.project(v.position);
        if (!centre) continue;
        // The footprint points already include the vehicle's own width.
        const double halfWidth = p.corridorHalfWidth;
        const double pathYaw = path.yawAt(centre->s);
        const double rel = headingDifference(pathYaw, v.yaw);

        // On the path now.
        const Occupancy now = occupancy(path, footprint(v.position, v.yaw, v.length, v.width), centre->index, halfWidth);
        // Ahead of the truck's front; a vehicle beside it (overlapping its length)
        // cannot be braked for and is left to the lateral clearance of the lane.
        if (now.inCorridor && now.sMin > frontS - 0.5 && now.sMin < sEnd) {
            PathObstacle ob;
            ob.id = v.id;
            ob.s = std::max(now.sMin, frontS);
            ob.speed = v.speed * std::cos(rel);
            consider(ob);
            continue;
        }

        // Crossing traffic: extrapolate along its heading and see whether it meets
        // the path ahead around the time the truck gets there. Vehicles moving with
        // or against the path are left to the "on the path" check above.
        if (v.speed < 1.0) continue;
        const double crossAngle = std::abs(std::sin(rel));
        if (crossAngle < 0.5) continue;  // within 30° of parallel
        for (double t = 0.5; t <= p.predictionTime + 1e-9; t += 0.5) {
            const Vec2 c = v.position + direction(v.yaw) * (v.speed * t);
            const auto pr = path.project(c);
            if (!pr) continue;
            const Occupancy later = occupancy(path, footprint(c, v.yaw, v.length, v.width), pr->index, halfWidth);
            if (!later.inCorridor || later.sMax <= frontS || later.sMin >= sEnd) continue;
            const double meetS = std::max(later.sMin, frontS);
            if (arrivalTime(meetS - frontS, speed, p.startAccel) <= t + p.conflictMargin) {
                PathObstacle ob;
                ob.id = v.id;
                ob.s = meetS;
                ob.speed = 0.0;
                ob.crossing = true;
                consider(ob);
            }
            break;
        }
    }

    if (out.nearest) {
        const double room = out.nearest->s - p.standstillGap - frontS;
        const double closing = speed - std::max(0.0, out.nearest->speed);
        if (closing > 0.0) out.requiredDecel = room > 0.1 ? closing * closing / (2.0 * room) : 99.0;
    }
    return out;
}

namespace {

// Whether a light state lets traffic go (for comparing candidate lights).
bool permits(LightState s) { return s == LightState::Green || s == LightState::Flashing || s == LightState::Off; }

}  // namespace

LightMatch lightForStop(const PathStop& stop, const Path& path, const std::vector<WorldLight>& lights,
                        const TrafficParams& p, int facing) {
    LightMatch out;
    const double lineS = std::clamp(stop.s, 0.0, path.length());
    const Vec2 line = path.positionAt(lineS);

    if (stop.semaphoreId >= 0) {
        double bestD = p.lightMatchRadius;
        for (const WorldLight& l : lights) {
            if (l.semaphoreId != stop.semaphoreId) continue;
            const double d = distance(l.position, line);
            if (d < bestD) {
                bestD = d;
                out.light = &l;
                out.byId = true;
            }
        }
        if (out.light) return out;
    }

    // By position, in the frame of the lane at the stop line.
    const double yaw = path.yawAt(lineS);
    const Vec2 dir = direction(yaw);
    const Vec2 left = direction(yaw + 0.5 * kPi);
    double bestScore = 1e18;
    bool agree = true;
    std::optional<bool> firstPermits;
    for (const WorldLight& l : lights) {
        const Vec2 rel = l.position - line;
        const double along = dot(rel, dir);
        const double lateral = dot(rel, left);
        if (along < -10.0 || along > 50.0 || std::abs(lateral) > 25.0) continue;
        const double c = std::cos(l.yaw - yaw);
        if (std::abs(c) < 0.8) continue;  // serves a crossing road
        if (facing != 0 && (c > 0.0 ? 1 : -1) != facing) continue;  // serves oncoming traffic
        if (!firstPermits) firstPermits = permits(l.state);
        else if (*firstPermits != permits(l.state)) agree = false;
        const double score = std::abs(lateral) + 0.3 * std::abs(along);
        if (score < bestScore) {
            bestScore = score;
            out.light = &l;
        }
    }
    if (facing == 0 && !agree) out.light = nullptr;  // ambiguous without knowing which way lights face
    return out;
}

SignalDecision decideSignal(LightState state, double distance, double speed, const TrafficParams& p) {
    switch (state) {
        case LightState::Green:
            return SignalDecision::Go;
        case LightState::Red:
        case LightState::AmberToGreen:
            return SignalDecision::Stop;
        case LightState::AmberToRed: {
            // Stop if that is comfortably possible, otherwise clear the junction.
            if (distance <= 0.5) return speed > 2.0 ? SignalDecision::Go : SignalDecision::Stop;
            const double decel = speed * speed / (2.0 * distance);
            return decel > p.amberMaxDecel ? SignalDecision::Go : SignalDecision::Stop;
        }
        case LightState::Off:
        case LightState::Flashing:
            return SignalDecision::GiveWay;
        case LightState::Unknown:
        default:
            return SignalDecision::Unknown;
    }
}

}  // namespace atspilot

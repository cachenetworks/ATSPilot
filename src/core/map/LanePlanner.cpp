#include "map/LanePlanner.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "math/MathUtil.h"

namespace atspilot {
namespace {

bool contains(const std::vector<std::uint32_t>& v, std::uint32_t x) { return std::find(v.begin(), v.end(), x) != v.end(); }

double endYaw(const LaneSegment& s) {
    const std::size_t n = s.points.size();
    const Vec2 d = s.points[n - 1].plan() - s.points[n - 2].plan();
    return std::atan2(d.y, d.x);
}

// Direction from a segment's start to the point `horizon` metres along it (or
// along its straightest continuation), which separates curves that start parallel.
double headingInto(const RoadNetwork& net, std::uint32_t id, double horizon) {
    const LaneSegment* seg = &net.segment(id);
    const Vec2 start = seg->points.front().plan();
    double remaining = horizon;
    Vec2 last = start;
    for (int hops = 0; hops < 4; ++hops) {
        for (std::size_t i = 1; i < seg->points.size(); ++i) {
            const Vec2 p = seg->points[i].plan();
            remaining -= distance(last, p);
            last = p;
            if (remaining <= 0.0) break;
        }
        if (remaining <= 0.0 || seg->next.empty()) break;
        seg = &net.segment(seg->next.front());
    }
    const Vec2 d = last - start;
    return d.lengthSq() > 1e-6 ? std::atan2(d.y, d.x) : 0.0;
}

std::uint32_t chooseSuccessor(const RoadNetwork& net, std::uint32_t from, const std::vector<std::uint32_t>& previous,
                              double horizon) {
    const auto& next = net.segment(from).next;
    // Keep the previous plan's decision while it remains valid.
    for (std::size_t i = 0; i + 1 < previous.size(); ++i) {
        if (previous[i] == from && contains(next, previous[i + 1])) return previous[i + 1];
    }
    const double yaw = endYaw(net.segment(from));
    std::uint32_t best = next.front();
    double bestTurn = std::numeric_limits<double>::max();
    for (auto n : next) {
        if (net.segment(n).points.size() < 2) continue;
        const double turn = std::abs(headingDifference(yaw, headingInto(net, n, horizon)));
        if (turn < bestTurn) {
            bestTurn = turn;
            best = n;
        }
    }
    return best;
}

}  // namespace

LocalizationResult Localizer::update(const Vec2& pos, double yaw, const std::vector<std::uint32_t>& plannedChain) {
    LocalizationResult r;
    const auto matches = net_.query(pos, params_.maxDistance);
    if (matches.empty()) {
        r.reason = "Searching For Road";
        previous_.reset();
        return r;
    }

    auto related = [&](std::uint32_t id) {
        if (!previous_) return false;
        if (id == *previous_) return true;
        const auto& prev = net_.segment(*previous_);
        return contains(prev.next, id) || contains(prev.prev, id);
    };

    double bestScore = std::numeric_limits<double>::max();
    const LaneMatch* best = nullptr;
    double previousScore = std::numeric_limits<double>::max();
    const LaneMatch* previousMatch = nullptr;
    for (const auto& m : matches) {
        const double headingErr = std::abs(headingDifference(yaw, m.yaw));
        if (headingErr > params_.maxHeadingError) continue;
        double score = m.distance + params_.headingWeight * headingErr;
        if (contains(plannedChain, m.segment)) score -= params_.plannedBonus;
        if (related(m.segment)) score -= params_.switchMargin;
        if (score < bestScore) {
            bestScore = score;
            best = &m;
        }
        if (previous_ && m.segment == *previous_) {
            previousScore = score;
            previousMatch = &m;
        }
    }
    if (!best) {
        r.reason = "No lane in driving direction";
        previous_.reset();
        return r;
    }
    // Hysteresis: stay on the previous lane unless the new one is clearly better
    // or the truck has driven past its end.
    if (previousMatch) {
        const auto& prevSeg = net_.segment(previousMatch->segment);
        const bool pastEnd = previousMatch->pointIndex + 2 >= prevSeg.points.size() && previousMatch->t >= 0.999;
        if (!pastEnd && previousScore <= bestScore + params_.switchMargin) best = previousMatch;
    }

    r.valid = true;
    r.match = *best;
    r.headingError = headingDifference(yaw, best->yaw);
    previous_ = best->segment;
    return r;
}

namespace {

double smoothstep(double x) {
    x = clamp(x, 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
}

// Describes the route's choice at a branch relative to the straightest option.
std::string maneuverFor(const RoadNetwork& net, std::uint32_t from, std::uint32_t chosen, double horizon) {
    const double turn = headingDifference(endYaw(net.segment(from)), headingInto(net, chosen, horizon));
    const bool left = turn > 0.0;
    if (std::abs(turn) > degToRad(35.0)) return left ? "Turn Left" : "Turn Right";
    return left ? "Keep Left" : "Keep Right";
}

}  // namespace

PlannedPath buildPlannedPath(const RoadNetwork& net, const LaneMatch& start, const PathBuildParams& params,
                             const std::vector<std::uint32_t>& previousChain, const Route* route) {
    PlannedPath out;

    // Walk backwards for the trailing context.
    std::vector<std::uint32_t> behind;
    double back = start.s;
    std::uint32_t cur = start.segment;
    while (back < params.behind && behind.size() < 8) {
        const auto& prev = net.segment(cur).prev;
        if (prev.empty()) break;
        std::uint32_t chosen = prev.front();
        for (auto p : prev) {
            if (contains(previousChain, p)) {
                chosen = p;
                break;
            }
        }
        behind.push_back(chosen);
        back += net.segment(chosen).length;
        cur = chosen;
    }
    std::reverse(behind.begin(), behind.end());
    out.chain = behind;
    std::vector<bool> laneChange(behind.size(), false);
    out.chain.push_back(start.segment);
    laneChange.push_back(false);

    const int routeIndex = route && route->found ? route->find(start.segment) : -1;
    out.onRoute = routeIndex >= 0;

    double ahead = net.segment(start.segment).length - start.s;
    bool reportedManeuver = false;
    if (out.onRoute) {
        // Follow the route; report the first step where it departs from "straight on".
        double dist = ahead;
        for (std::size_t i = static_cast<std::size_t>(routeIndex) + 1; i < route->steps.size(); ++i) {
            const RouteStep& step = route->steps[i];
            const std::uint32_t prevSeg = route->steps[i - 1].segment;
            if (!reportedManeuver) {
                if (step.laneChange) {
                    const auto& a = net.segment(prevSeg);
                    const auto& b = net.segment(step.segment);
                    const Vec2 dir = (a.points.back().plan() - a.points.front().plan()).normalized();
                    const bool left = cross(dir, b.points.front().plan() - a.points.front().plan()) > 0.0;
                    out.nextManeuver = left ? "Change Lane Left" : "Change Lane Right";
                    out.nextManeuverDistance = std::max(0.0, dist - net.segment(prevSeg).length);
                    reportedManeuver = true;
                } else if (net.segment(prevSeg).next.size() > 1 &&
                           chooseSuccessor(net, prevSeg, {}, params.choiceHorizon) != step.segment) {
                    out.nextManeuver = maneuverFor(net, prevSeg, step.segment, params.choiceHorizon);
                    out.nextManeuverDistance = dist;
                    reportedManeuver = true;
                }
            }
            if (dist < params.ahead) {
                out.chain.push_back(step.segment);
                laneChange.push_back(step.laneChange);
            }
            if (!step.laneChange) dist += net.segment(step.segment).length;
            if (reportedManeuver && dist >= params.ahead) break;
            if (dist > 30000.0) break;
        }
        out.routeRemaining = net.segment(start.segment).length - start.s;
        for (std::size_t i = static_cast<std::size_t>(routeIndex) + 1; i < route->steps.size(); ++i) {
            if (!route->steps[i].laneChange) out.routeRemaining += net.segment(route->steps[i].segment).length;
        }
        if (!reportedManeuver) {
            out.nextManeuver = "Arrive";
            out.nextManeuverDistance = dist;
        }
    } else {
        // No route: at every branch continue on the straightest successor.
        cur = start.segment;
        while (ahead < params.ahead && out.chain.size() < 400) {
            const auto& next = net.segment(cur).next;
            if (next.empty()) {
                if (!reportedManeuver) {
                    out.nextManeuver = "Road data ends";
                    out.nextManeuverDistance = ahead;
                    reportedManeuver = true;
                }
                break;
            }
            const std::uint32_t chosen = chooseSuccessor(net, cur, previousChain, params.choiceHorizon);
            if (!reportedManeuver && next.size() > 1) {
                const double turn =
                    headingDifference(endYaw(net.segment(cur)), headingInto(net, chosen, params.choiceHorizon));
                out.nextManeuver = std::abs(turn) < degToRad(10.0) ? "Continue" : (turn > 0 ? "Keep Left" : "Keep Right");
                out.nextManeuverDistance = ahead;
                reportedManeuver = true;
            }
            if (contains(out.chain, chosen)) break;  // loop guard
            out.chain.push_back(chosen);
            laneChange.push_back(false);
            ahead += net.segment(chosen).length;
            cur = chosen;
        }
    }

    // Assemble the polyline. A lane change replaces the tail of the previous lane
    // with a smooth lateral blend into the parallel lane; lanes of one road share
    // their sampling, so point j of one lane sits beside point j of the other.
    std::vector<Vec2> pts;
    std::vector<std::uint32_t> ids;
    std::vector<std::pair<std::size_t, Vec2>> joins;  // index of the first point after a join, gap vector
    double truckS = 0.0;
    auto arcLength = [&]() {
        double s = 0.0;
        for (std::size_t i = 1; i < pts.size(); ++i) s += distance(pts[i - 1], pts[i]);
        return s;
    };
    for (std::size_t c = 0; c < out.chain.size(); ++c) {
        const auto& seg = net.segment(out.chain[c]);
        if (laneChange[c] && c > 0) {
            const auto& from = net.segment(out.chain[c - 1]);
            if (from.points.size() == seg.points.size() && pts.size() >= from.points.size()) {
                const std::size_t base = pts.size() - from.points.size();
                // Start blending at the truck if it is on the source lane, else at its start.
                double startAlong = out.chain[c - 1] == start.segment ? start.s : 0.0;
                const double remaining = std::max(1.0, from.length - startAlong);
                const double blend = clamp(remaining * 0.8, 30.0, 150.0);
                double along = 0.0;
                for (std::size_t j = 0; j < from.points.size(); ++j) {
                    if (j > 0) along += distance(from.points[j - 1].plan(), from.points[j].plan());
                    const double w = smoothstep((along - startAlong) / std::min(blend, remaining));
                    pts[base + j] = lerp(from.points[j].plan(), seg.points[j].plan(), w);
                    ids[base + j] = out.chain[c];
                }
                continue;  // the blended points already end on the target lane
            }
        }
        if (out.chain[c] == start.segment) truckS = arcLength() + start.s;
        if (!pts.empty()) {
            // The gap between this lane's start and the previous lane's end, which is
            // what a merge, lane drop or imperfect join leaves behind.
            const Vec2 gap = seg.points.front().plan() - pts.back();
            if (gap.length() >= 1.0) joins.push_back({pts.size(), gap});
        }
        for (const auto& p : seg.points) {
            if (!pts.empty() && distance(pts.back(), p.plan()) < 1e-3) continue;
            pts.push_back(p.plan());
            ids.push_back(out.chain[c]);
        }
    }

    // Lateral discontinuities (merges, lane drops, small join gaps) become smooth
    // transitions: the offset is blended into the preceding stretch of path.
    constexpr double kBlendBack = 60.0;
    for (const auto& [k, jump] : joins) {
        if (k < 2 || k > pts.size()) continue;
        const Vec2 dir = (pts[k - 1] - pts[k - 2]).normalized();
        // Mostly longitudinal gaps are just spacing, not an offset.
        if (std::abs(cross(dir, jump)) < 1.0) continue;
        double behindJoin = 0.0;
        for (std::size_t j = k; j-- > 0;) {
            if (j + 1 < k) behindJoin += distance(pts[j], pts[j + 1]);
            if (behindJoin > kBlendBack) break;
            pts[j] += jump * smoothstep(1.0 - behindJoin / kBlendBack);
        }
    }

    Path raw;
    for (std::size_t i = 0; i < pts.size(); ++i) raw.append(pts[i], 0.0, ids[i]);
    // Keep only `behind` metres before the truck so the truck sits near the start of
    // the path, where the controller's windowed projection begins searching.
    const double from = std::max(0.0, truckS - params.behind);
    out.path = raw.trimmed(from, raw.length()).resampled(params.spacing);
    out.truckS = truckS - from;
    return out;
}

}  // namespace atspilot

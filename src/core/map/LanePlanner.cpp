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

PlannedPath buildPlannedPath(const RoadNetwork& net, const LaneMatch& start, const PathBuildParams& params,
                             const std::vector<std::uint32_t>& previousChain) {
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
    double sBeforeStart = 0.0;
    for (auto id : behind) sBeforeStart += net.segment(id).length;
    out.chain.push_back(start.segment);

    // Walk forwards, choosing a successor at every branch.
    double ahead = net.segment(start.segment).length - start.s;
    cur = start.segment;
    bool reportedManeuver = false;
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
            const double turn = headingDifference(endYaw(net.segment(cur)), headingInto(net, chosen, params.choiceHorizon));
            out.nextManeuver = std::abs(turn) < degToRad(10.0) ? "Continue" : (turn > 0 ? "Keep Left" : "Keep Right");
            out.nextManeuverDistance = ahead;
            reportedManeuver = true;
        }
        if (contains(out.chain, chosen)) break;  // loop guard
        out.chain.push_back(chosen);
        ahead += net.segment(chosen).length;
        cur = chosen;
    }

    Path raw;
    for (auto id : out.chain) {
        for (const auto& p : net.segment(id).points) raw.append(p.plan(), 0.0, id);
    }
    // Keep only `behind` metres before the truck so the truck sits near the start of
    // the path, where the controller's windowed projection begins searching.
    const double truckS = sBeforeStart + start.s;
    const double from = std::max(0.0, truckS - params.behind);
    out.path = raw.trimmed(from, raw.length()).resampled(params.spacing);
    out.truckS = truckS - from;
    return out;
}

}  // namespace atspilot

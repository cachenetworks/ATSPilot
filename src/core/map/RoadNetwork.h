#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "math/Vec.h"

namespace atspilot {

// A point on a lane centreline in plan coordinates (see math/Coordinates.h),
// stored as float to keep the whole-map graph compact.
struct LanePoint {
    float x = 0.0f;
    float y = 0.0f;
    float height = 0.0f;

    Vec2 plan() const { return {x, y}; }
};

enum class LaneKind : std::uint8_t { Road = 0, Prefab = 1 };

// Traffic control at the start of a lane, from prefab nav-curve data.
namespace LaneRule {
inline constexpr std::uint8_t Signal = 1;       // traffic light
inline constexpr std::uint8_t Stop = 2;         // stop sign
inline constexpr std::uint8_t Yield = 4;        // give way
inline constexpr std::uint8_t RailCrossing = 8;
inline constexpr std::uint8_t NoTrucks = 16;    // cars/buses/trams only: never routed through
}  // namespace LaneRule

// One drivable lane between two connection points, in travel direction.
struct LaneSegment {
    std::uint64_t itemUid = 0;  // road or prefab item it belongs to
    LaneKind kind = LaneKind::Road;
    std::uint8_t laneIndex = 0;  // roads: 0 = innermost; prefabs: nav curve index (mod 256)
    bool leftSide = false;       // roads: lane travels against the road's node order
    std::uint8_t rules = 0;      // LaneRule bits
    float length = 0.0f;
    std::vector<LanePoint> points;
    std::vector<std::uint32_t> next;
    std::vector<std::uint32_t> prev;
};

struct LaneMatch {
    std::uint32_t segment = 0;
    std::size_t pointIndex = 0;  // segment [pointIndex, pointIndex + 1]
    double t = 0.0;
    double distance = 0.0;       // unsigned lateral distance, m
    double crossTrack = 0.0;     // signed, positive when the query point is left of the lane
    double yaw = 0.0;            // lane direction
    double s = 0.0;              // distance along the segment
};

// A company depot that jobs deliver to, with the lanes of its prefab.
struct Destination {
    std::uint64_t city = 0;     // token
    std::uint64_t company = 0;  // token
    Vec2 position;              // plan coordinates of the company node
    std::vector<std::uint32_t> lanes;
};

// Lane-level road graph for the whole map with a uniform-grid spatial index.
class RoadNetwork {
public:
    static constexpr std::uint32_t kFormatVersion = 3;

    std::uint32_t add(LaneSegment seg);
    void finalize();  // builds the spatial index and predecessor lists

    const LaneSegment& segment(std::uint32_t id) const { return segments_[id]; }
    std::size_t size() const { return segments_.size(); }
    const std::vector<LaneSegment>& segments() const { return segments_; }
    std::vector<LaneSegment>& mutableSegments() { return segments_; }

    void addDestination(Destination d) { destinations_.push_back(std::move(d)); }
    const std::vector<Destination>& destinations() const { return destinations_; }
    const Destination* findDestination(std::uint64_t city, std::uint64_t company) const;

    // Parallel lanes of the same road travelling the same way (laneIndex +-1),
    // i.e. the lanes a lane change can move to.
    std::vector<std::uint32_t> laneNeighbors(std::uint32_t id) const;

    // All lane matches within `radius` of p, nearest first.
    std::vector<LaneMatch> query(const Vec2& p, double radius) const;
    // Projection of p onto one segment.
    LaneMatch project(std::uint32_t segment, const Vec2& p) const;

    bool save(const std::filesystem::path& file, const std::string& fingerprint) const;
    static std::optional<RoadNetwork> load(const std::filesystem::path& file, const std::string& expectedFingerprint,
                                           std::string* error = nullptr);

private:
    static constexpr double kCellSize = 64.0;
    static std::uint64_t cellKey(std::int32_t ix, std::int32_t iy) {
        return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(ix)) << 32) | static_cast<std::uint32_t>(iy);
    }

    std::vector<LaneSegment> segments_;
    std::vector<Destination> destinations_;
    std::unordered_map<std::uint64_t, std::vector<std::uint32_t>> roadLanes_;  // road item uid -> lanes
    std::unordered_map<std::uint64_t, std::vector<std::uint32_t>> grid_;
};

}  // namespace atspilot

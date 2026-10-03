#pragma once

#include <atomic>
#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "map/RoadNetwork.h"
#include "util/Logger.h"

namespace atspilot {

struct MapBuildOptions {
    std::filesystem::path gameDir;
    std::vector<std::filesystem::path> extraArchives;  // e.g. map mods, applied after game archives
    double laneWidth = 4.5;
    std::string mapName = "usa";
    std::function<void(double progress, const std::string& phase)> progress;
    const std::atomic<bool>* cancel = nullptr;
};

struct MapBuildStats {
    std::size_t archives = 0;
    std::size_t sectors = 0;
    std::size_t sectorErrors = 0;
    std::size_t roads = 0;
    std::size_t roadsWithoutLook = 0;
    std::size_t roadsMissingNodes = 0;
    std::size_t prefabs = 0;
    std::size_t prefabsWithoutDesc = 0;
    std::size_t roadLooks = 0;
    std::size_t prefabDescs = 0;
    std::size_t lanes = 0;
    std::size_t points = 0;
    // Lane-end connectivity: how many lane ends found a geometric successor.
    std::size_t laneEnds = 0;
    std::size_t laneEndsConnected = 0;
    // Residual gap between road lane ends and the prefab curves they joined,
    // a direct check of the lane-offset model (see docs/map-parsing.md).
    double roadPrefabGapSum = 0.0;
    std::size_t roadPrefabJoins = 0;
    // Histograms (0.25 m buckets up to 8 m) of road<->prefab join gaps and of the
    // nearest same-direction prefab endpoint for road lane ends left unconnected.
    std::vector<std::size_t> joinGapHistogram = std::vector<std::size_t>(32, 0);
    std::vector<std::size_t> missGapHistogram = std::vector<std::size_t>(32, 0);
    std::size_t roadEndsUnconnected = 0;
    std::size_t lookSidesCalibrated = 0;
    struct GapSample {
        std::string look;
        bool left = false;
        int lane = 0;
        int laneCount = 0;
        double lateral = 0.0;       // prefab point relative to road lane end, + = right of travel
        double longitudinal = 0.0;
        bool roadEndsIntoPrefab = true;
    };
    std::vector<GapSample> gapSamples;  // road<->prefab joins with gap > 0.75 m (first 5000)
    std::vector<std::string> errors;  // first few, for logging
};

// Identifies the archives a cache was built from (names, sizes, timestamps).
std::string mapFingerprint(const MapBuildOptions& options);

std::vector<std::filesystem::path> gameArchives(const std::filesystem::path& gameDir);

// Parses road looks, prefab descriptors and every sector of the map into a
// lane graph. Expensive (tens of seconds); run off the game thread.
std::optional<RoadNetwork> buildRoadNetwork(const MapBuildOptions& options, MapBuildStats& stats, Logger* log);

}  // namespace atspilot

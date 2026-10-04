#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "math/Vec.h"

namespace atspilot {

// What ATSPilot knows about the world around the truck beyond the SDK telemetry:
// AI traffic and traffic-light states, read from game memory when that is enabled
// (src/plugin/GameMemory.cpp). Plan coordinates throughout.

enum class LightState : std::uint8_t {
    Unknown,
    Off,
    Red,
    AmberToRed,    // amber after green: stop if there is room
    AmberToGreen,  // red-amber: about to turn green
    Green,
    Flashing,      // flashing amber: treat as give way
};

const char* toString(LightState s);

// One rigid body of traffic: a car, a truck tractor or one of its trailers.
struct WorldVehicle {
    int id = 0;            // stable per vehicle while it exists; trailers share their tractor's id
    Vec2 position;         // centre of the body
    double yaw = 0.0;      // heading, plan frame
    double length = 4.5;
    double width = 1.9;
    double speed = 0.0;    // m/s along its heading
};

struct WorldLight {
    int semaphoreId = -1;  // the prefab's semaphore id (matches PathStop::semaphoreId)
    Vec2 position;
    double yaw = 0.0;  // direction the light's model faces, plan frame
    LightState state = LightState::Unknown;
    double timeRemaining = 0.0;  // s in the current state
};

struct WorldSnapshot {
    double time = 0.0;  // simulation time it was taken at
    bool valid = false;
    std::vector<WorldVehicle> vehicles;
    std::vector<WorldLight> lights;
};

using WorldSnapshotPtr = std::shared_ptr<const WorldSnapshot>;

}  // namespace atspilot

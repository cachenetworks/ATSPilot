#pragma once

#include <string>

#include "math/Vec.h"

namespace atspilot {

// Snapshot of the truck assembled from one telemetry frame. All values use the
// SDK's conventions; plan-space values are derived through coords:: helpers.
struct VehicleState {
    bool valid = false;
    double time = 0.0;             // simulation time, seconds
    bool paused = true;

    Vec3 worldPosition;            // truck origin, world space
    double headingUnit = 0.0;      // SDK heading, [0, 1)
    double pitchUnit = 0.0;
    double rollUnit = 0.0;
    Vec3 localVelocity;            // vehicle space, m/s (-z forward)
    Vec3 localAngularVelocity;     // rotations per second
    double speed = 0.0;            // speedometer speed, m/s (negative when reversing)

    double inputSteering = 0.0;    // [-1, 1], counterclockwise (left) positive
    double inputThrottle = 0.0;
    double inputBrake = 0.0;
    double effectiveSteering = 0.0;
    double effectiveThrottle = 0.0;
    double effectiveBrake = 0.0;
    double steerableWheelAngle = 0.0;  // radians, mean of steerable wheels, left positive

    double cruiseControlSpeed = 0.0;   // ATS's own cruise control, m/s, 0 = off
    int gear = 0;
    int displayedGear = 0;
    bool parkingBrake = false;
    bool engineEnabled = false;
    bool electricEnabled = false;
    bool wipers = false;

    double navigationDistance = 0.0;   // m
    double navigationTime = 0.0;       // s
    double navigationSpeedLimit = 0.0; // m/s, 0 = none
};

// Semi-static truck properties from the "truck"/"trailer" configuration events.
struct VehicleConfig {
    bool valid = false;
    std::string truckId;
    std::string truckName;
    double wheelbase = 6.0;      // m, steer axle to driven-axle centre
    double frontAxleZ = -3.0;    // vehicle-space z of the steer axle
    double rearAxleZ = 3.0;      // vehicle-space z of the rear axle group centre
    int trailerCount = 0;
    double cargoMassKg = 0.0;
    bool hasJob = false;
    std::string destinationCity;
    std::string destinationCityId;     // job "destination.city.id", e.g. "fresno"
    std::string destinationCompanyId;  // job "destination.company.id"; empty for special transport
};

}  // namespace atspilot

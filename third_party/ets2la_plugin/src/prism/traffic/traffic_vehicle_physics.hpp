#pragma once

#include "prism/traffic/traffic_chassis_physics.hpp"

namespace ets2la_plugin::prism
{
#pragma pack( push, 1 )

    // Size: W 0x0198, L|A 0x0178
    class traffic_vehicle_physics_t : public /* [W 0x140, L|A 0x128] @ W|L|A 0x00 */ traffic_chassis_physics_t
    {
    public:
        char pad_W_0x0140__LA_0x0128[ 16 ];                     // W 0x0140, L|A 0x0128 (W|L|A 0x10)
        class ray_cast_sagging_probe_t* ray_cast_sagging_probe; // W 0x0150, L|A 0x0138 (W|L|A 0x08)
        array_dyn_t< float > N0000B994;                         // W 0x0158, L|A 0x0140 (W 0x28, L|A 0x20)
        char pad_W_0x0180__LA_0x0160[ 16 ];                     // W 0x0180, L|A 0x0160 (W|L|A 0x10)
        class traffic_vehicle_u* traffic_vehicle;               // W 0x0190, L|A 0x0170 (W|L|A 0x08)
    };

#if defined( _WIN32 )
    static_assert( sizeof( traffic_vehicle_physics_t ) == 0x198 ); // W
#elif defined( __linux__ ) || defined( __APPLE__ )
    static_assert( sizeof( traffic_vehicle_physics_t ) == 0x178 ); // L|A
#endif

    // Size: W 0x0028, L|A 0x0030 1.60
    class traffic_vehicle_physics_object_t
        : public /* [W 0x20, L|A 0x28] @ W|L|A 0x00 */ traffic_chassis_physics_object_t
    {
    public:
        class traffic_vehicle_physics_t* physics; // W 0x0020, L|A 0x0028 (W|L|A 0x08)
    };
#if defined( _WIN32 )
    static_assert( sizeof( traffic_vehicle_physics_object_t ) == 0x28 ); // W
#elif defined( __linux__ ) || defined( __APPLE__ )
    static_assert( sizeof( traffic_vehicle_physics_object_t ) == 0x30 ); // L|A
#endif

#pragma pack( pop )
}

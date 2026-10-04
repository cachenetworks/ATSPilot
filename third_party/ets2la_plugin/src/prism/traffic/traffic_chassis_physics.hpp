#pragma once

#include "prism/collections/array_dyn.hpp"
#include "prism/common.hpp"
#include "prism/traffic/traffic_physics_shared.hpp"

namespace ets2la_plugin::prism
{
#pragma pack( push, 1 )

    // Size: W 0x0020, L|A 0x0028 1.60
    class traffic_chassis_physics_object_t : public /* [W 0x20, L|A 0x28] @ W|L|A 0x00 */ traffic_physics_object_t
    {
    public:
    };
#if defined( _WIN32 )
    static_assert( sizeof( traffic_chassis_physics_object_t ) == 0x20 ); // W
#elif defined( __linux__ ) || defined( __APPLE__ )
    static_assert( sizeof( traffic_chassis_physics_object_t ) == 0x28 ); // L|A
#endif

    // Size: W 0x0140, L|A 0x0128 1.60
    class traffic_chassis_physics_t : public /* [W|L|A 0x20] @ W|L|A 0x00 */ traffic_physics_t
    {
        class wheel_t;

    public:
        class physics_actor_t* physics_actor;                // W|L|A 0x0020 (W|L|A 0x08)
        array_dyn_t< class physics_shape_t* > physics_shape; // W|L|A 0x0028 (W 0x28, L|A 0x20)
        char pad_W_0x0050__LA_0x0048[ 32 ];                  // W 0x0050, L|A 0x0048 (W|L|A 0x20)
        float speed;                                         // W 0x0070, L|A 0x0068 (W|L|A 0x04)
        float acceleration;                                  // W 0x0074, L|A 0x006C (W|L|A 0x04)
        float3_t bounding_box;                               // W 0x0078, L|A 0x0070 (W|L|A 0x0c)
        float3_t center_point;                               // W 0x0084, L|A 0x007C (W|L|A 0x0c)
        array_dyn_t< class wheel_t > wheels;                 // W 0x0090, L|A 0x0088 (W 0x28, L|A 0x20)
        char pad_W_0x00B8__LA_0x00A8[ 8 ];                   // W 0x00B8, L|A 0x00A8 (W|L|A 0x08)
        array_dyn_t< uint32_t > N0000C0DB;                   // W 0x00C0, L|A 0x00B0 (W 0x28, L|A 0x20)
        float speed2;                                        // W 0x00E8, L|A 0x00D0 (W|L|A 0x04)
        char pad_W_0x00EC__LA_0x00D4[ 84 ];                  // W 0x00EC, L|A 0x00D4 (W|L|A 0x54)
    };

#if defined( _WIN32 )
    static_assert( sizeof( traffic_chassis_physics_t ) == 0x140 ); // W
#elif defined( __linux__ ) || defined( __APPLE__ )
    static_assert( sizeof( traffic_chassis_physics_t ) == 0x128 ); // L|A
#endif

#pragma pack( pop )
}

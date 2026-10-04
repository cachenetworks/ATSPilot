#pragma once

#include "prism/traffic/traffic_chassis_physics.hpp"

namespace ets2la_plugin::prism
{
#pragma pack( push, 1 )

    // Size: W 0x0238, L|A 0x0200 1.60
    class traffic_trailer_physics_t : public /* [W 0x140, L|A 0x128] @ W|L|A 0x00 */ traffic_chassis_physics_t
    {
    public:
        char pad_W_0x0140__LA_0x0128[ 8 ];                                    // W 0x0140, L|A 0x0128 (W|L|A 0x08)
        array_dyn_t< float > N0000BCA6;                                       // W 0x0148, L|A 0x0130 (W 0x28, L|A 0x20)
        char pad_W_0x0170__LA_0x0150[ 72 ];                                   // W 0x0170, L|A 0x0150 (W|L|A 0x48)
        array_dyn_t< class physics_shape_t* > physics_shapes;                 // W 0x01B8, L|A 0x0198 (W 0x28, L|A 0x20)
        array_dyn_t< class physics_joint_t* > physics_joints;                 // W 0x01E0, L|A 0x01B8 (W 0x28, L|A 0x20)
        array_dyn_t< class physics_actor_dynamic_t* > physics_actors_dynamic; // W 0x0208, L|A 0x01D8 (W 0x28, L|A 0x20)
        char pad_W_0x0230__LA_0x01F8[ 8 ];                                    // W 0x0230, L|A 0x01F8 (W|L|A 0x08)
    };

#if defined( _WIN32 )
    static_assert( sizeof( traffic_trailer_physics_t ) == 0x238 ); // W
#elif defined( __linux__ ) || defined( __APPLE__ )
    static_assert( sizeof( traffic_trailer_physics_t ) == 0x200 ); // L|A
#endif

    // Size: W 0x0028, L|A 0x0030
    class traffic_trailer_physics_object_t
        : public /* [W 0x20, L|A 0x28] @ W|L|A 0x00 */ traffic_chassis_physics_object_t
    {
    public:
        class traffic_trailer_physics_t* physics; // W 0x0020, L|A 0x0028 (W|L|A 0x08)
    };
#if defined( _WIN32 )
    static_assert( sizeof( traffic_trailer_physics_object_t ) == 0x28 ); // W
#elif defined( __linux__ ) || defined( __APPLE__ )
    static_assert( sizeof( traffic_trailer_physics_object_t ) == 0x30 ); // L|A
#endif

#pragma pack( pop )
}

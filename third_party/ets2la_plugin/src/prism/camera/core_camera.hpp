#pragma once
#include "prism/collections/array_dyn.hpp"
#include "prism/common.hpp"
#include "prism/unit/unit.hpp"

namespace ets2la_plugin::prism
{
#pragma pack( push, 1 )

    // Size: W 0x03E8, L|A 0x03E0 1.60
    class core_camera_u : public /* [W|L|A 0x10] @ W|L|A 0x00 */ unit_t
    {
    public:
        char pad_WLA_0x0010[ 12 ];           // W|L|A 0x0010 (W|L|A 0x0c)
        float mouse_sensitivity;             // W|L|A 0x001C (W|L|A 0x04)
        float camera_fov;                    // W|L|A 0x0020 (W|L|A 0x04)
        float near_plane;                    // W|L|A 0x0024 (W|L|A 0x04)
        float far_plane;                     // W|L|A 0x0028 (W|L|A 0x04)
        char pad_WLA_0x002C[ 12 ];           // W|L|A 0x002C (W|L|A 0x0c)
        float h_fov;                         // W|L|A 0x0038 (W|L|A 0x04) horizontal FOV
        float v_fov;                         // W|L|A 0x003C (W|L|A 0x04) vertical FOV
        placement_t placement;               // W|L|A 0x0040 (W|L|A 0x20)
        float4x4_t projection_matrix;        // W|L|A 0x0060 (W|L|A 0x40)
        char pad_WLA_0x00A0[ 88 ];           // W|L|A 0x00A0 (W|L|A 0x58)
        array_dyn_t< float3_t > shake_anim;  // W|L|A 0x00F8 (W 0x28, L|A 0x20)
        char pad_W_0x0120__LA_0x0118[ 32 ];  // W 0x0120, L|A 0x0118 (W|L|A 0x20)
        float shake_anim_step;               // W 0x0140, L|A 0x0138 (W|L|A 0x04)
        float shake_anim_scale_min;          // W 0x0144, L|A 0x013C (W|L|A 0x04)
        float shake_anim_scale_max;          // W 0x0148, L|A 0x0140 (W|L|A 0x04)
        char pad_W_0x014C__LA_0x0144[ 668 ]; // W 0x014C, L|A 0x0144 (W|L|A 0x29c)
    };
#if defined( _WIN32 )
    static_assert( sizeof( core_camera_u ) == 0x3E8 ); // W
#elif defined( __linux__ ) || defined( __APPLE__ )
    static_assert( sizeof( core_camera_u ) == 0x3E0 ); // L|A
#endif

#pragma pack( pop )
}

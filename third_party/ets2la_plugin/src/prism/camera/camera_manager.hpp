#pragma once
#include "prism/collections/array_dyn.hpp"
#include "prism/collections/array_local.hpp"
#include "prism/unit/unit.hpp"

namespace ets2la_plugin::prism
{
#pragma pack( push, 1 )

    // Size: W 0x00A0, L|A 0x0098 1.60
    class camera_manager_u : public /* [W|L|A 0x10] @ W|L|A 0x00 */ unit_t
    {
    public:
        uint32_t current_camera;                            // W|L|A 0x0010 (W|L|A 0x04)
        uint32_t total_camera_count;                        // W|L|A 0x0014 (W|L|A 0x04)
        char pad_WLA_0x0018[ 24 ];                          // W|L|A 0x0018 (W|L|A 0x18)
        array_dyn_t< class core_camera_u* > cameras;        // W|L|A 0x0030 (W 0x28, L|A 0x20)
        char pad_W_0x0058__LA_0x0050[ 8 ];                  // W 0x0058, L|A 0x0050 (W|L|A 0x08)
        array_local_t< class core_camera_u*, 4 > N0001DC34; // W 0x0060, L|A 0x0058 (W|L|A 0x40)

        static uint64_t instance_ptr_address;

        static bool scan_patterns();
        static camera_manager_u* get();
    };

#if defined( _WIN32 )
    static_assert( sizeof( camera_manager_u ) == 0xA0 ); // W
#elif defined( __linux__ ) || defined( __APPLE__ )
    static_assert( sizeof( camera_manager_u ) == 0x98 ); // L|A
#endif

#pragma pack( pop )
}

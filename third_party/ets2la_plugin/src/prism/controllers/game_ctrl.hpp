#pragma once

#include "base_ctrl.hpp"
#include <cstdint>

namespace ets2la_plugin::prism
{
#pragma pack( push, 1 )

    // commented out because maintaining this is too annoying now, we use patterns to find the values we want anyway
    // Size: 0x4180 1.58
    class game_ctrl_u : public /* [0x3088] @ 0x00 */ base_ctrl_u
    {
    public:
        // char pad_3088[ 8 ];                  // 0x3088 (0x08)
        // class radio_u* radio_u;              // 0x3090 (0x08)
        // class police_ctrl_u* police_ctrl_u;  // 0x3098 (0x08)
        // char pad_30A0[ 2848 ];               // 0x30A0 (0xb20)
        // gps_manager_t gps_manager__dont_use; // 0x3BC0 (0x590)
        // char pad_4150[ 48 ];                 // 0x4150 (0x30)

        static uint32_t gps_manager_offset;

        static bool scan_patterns();
        static game_ctrl_u* get();
    };
    // static_assert( sizeof( game_ctrl_u ) == 0x4180 );

#pragma pack( pop )
}

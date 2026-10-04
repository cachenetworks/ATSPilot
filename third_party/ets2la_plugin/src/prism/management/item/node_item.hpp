#pragma once

#include "prism/common.hpp"
#include <cstdint>

#pragma pack( push, 1 )

namespace ets2la_plugin::prism
{
    class node_item_t // Size: 0x0038
    {
    public:
        int3_t coords;                        // 0x0000 (0x0c)
        uint32_t flags;                       // 0x000C (0x04)
        quat_t rotation;                      // 0x0010 (0x10)
        class kdop_item_t* backward_item_uid; // 0x0020 (0x08)
        class kdop_item_t* forward_item_uid;  // 0x0028 (0x08)
        uint64_t uid;                         // 0x0030 (0x08)
    };
    static_assert( sizeof( node_item_t ) == 0x38 );
}

#pragma pack( pop )

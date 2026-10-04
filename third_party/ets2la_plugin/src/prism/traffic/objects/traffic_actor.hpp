#pragma once

#include "prism/common.hpp"
#include "prism/traffic/traffic_object.hpp"
#include "prism/traffic/traffic_obstacle.hpp"

#pragma pack( push, 1 )

namespace ets2la_plugin::prism
{
    // Size: 0x0080
    class traffic_actor_t : public /* [0x10] @ 0x00 */ traffic_object_t,
                            public /* [0x08] @ 0x10 */ traffic_obstacle_t
    {
    public:
        char pad_0018[ 16 ];    // 0x0018 (0x10)
        placement_t placement;  // 0x0028 (0x20)
        aabox_t aabox;          // 0x0048 (0x18)
        traffic_actor_t* owner; // 0x0060 (0x08)
        traffic_actor_t* slave; // 0x0068 (0x08)
        char pad_0070[ 16 ];    // 0x0070 (0x10)
    };
    static_assert( sizeof( traffic_actor_t ) == 0x80 );
}

#pragma pack( pop )

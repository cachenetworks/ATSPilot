#pragma once

#include "prism/traffic/objects/traffic_item_actor.hpp"

#pragma pack( push, 1 )

namespace ets2la_plugin::prism
{
    // Size: 0x00F0
    class traffic_item_passive_actor_t : public /* [0xA8] @ 0x00 */ traffic_item_actor_t
    {
    public:
        char pad_00A8[ 72 ]; // 0x00A8 (0x48)
    };
    static_assert( sizeof( traffic_item_passive_actor_t ) == 0xF0 );
}

#pragma pack( pop )

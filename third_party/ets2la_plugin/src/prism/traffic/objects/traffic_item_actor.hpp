#pragma once

#include "prism/traffic/objects/traffic_actor.hpp"
#include "prism/traffic/traffic_object.hpp"

#pragma pack( push, 1 )

namespace ets2la_plugin::prism
{
    // Size: 0x00A8
    class traffic_item_actor_t : public /* [0x80] @ 0x00 */ traffic_actor_t,
                                 public /* [0x28] @ 0x80 */ traffic_item_object_t
    {
    public:
    };
    static_assert( sizeof( traffic_item_actor_t ) == 0xA8 );

}

#pragma pack( pop )

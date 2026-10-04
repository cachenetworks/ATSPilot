#pragma once

#include "prism/collections/array_dyn.hpp"
#include "prism/traffic/objects/traffic_item_passive_actor.hpp"
#include "prism/unit/unit.hpp"

#pragma pack( push, 1 )

namespace ets2la_plugin::prism
{

    // Size: 0x0128
    class traffic_player_trailer_t : public /* [0xF0] @ 0x00 */ traffic_item_passive_actor_t
    {
    public:
        traffic_player_base_object_t_templ< unit_t >*
            traffic_player_object;                // 0x00F0 (0x08) remote_vehicle_base_u|physics_trailer_u
        class dangerous_actor_t* dangerous_actor; // 0x00F8 (0x08)
        char pad_0100[ 8 ];                       // 0x0100 (0x08)
        placement_t placement_2;                  // 0x0108 (0x20)
    };
    static_assert( sizeof( traffic_player_trailer_t ) == 0x128 );

}

#pragma pack( pop )

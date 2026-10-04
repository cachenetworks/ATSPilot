#pragma once

#include "prism/traffic/objects/traffic_actor.hpp"

namespace ets2la_plugin::prism
{
#pragma pack( push, 1 )

    // Size: W|L|A 0x00A8 1.60
    class traffic_parked_actor_t : public /* [W|L|A 0x80] @ W|L|A 0x00 */ traffic_actor_t
    {
    public:
        class parked_model_base_u* parked_model_base;  // W|L|A 0x0080 (W|L|A 0x08)
        class traffic_parked_trailer_t* slave_trailer; // W|L|A 0x0088 (W|L|A 0x08)
        class traffic_model_t** model;                 // W|L|A 0x0090 (W|L|A 0x08)
        class traffic_chassis_physics_t** physics;     // W|L|A 0x0098 (W|L|A 0x08)
        char pad_WLA_0x00A0[ 8 ];                      // W|L|A 0x00A0 (W|L|A 0x08)
    };

    static_assert( sizeof( traffic_parked_actor_t ) == 0xA8 ); // W|L|A

#pragma pack( pop )
}

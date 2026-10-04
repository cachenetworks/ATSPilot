#pragma once

#include "prism/traffic/objects/traffic_item_passive_actor.hpp"
#include "prism/traffic/traffic_trailer_model.hpp"
#include "prism/traffic/traffic_trailer_physics.hpp"
#include "prism/traffic/traffic_vehicle_mp.hpp"

namespace ets2la_plugin::prism
{
#pragma pack( push, 1 )

    // Size: W 0x0320, L|A 0x0328
    class traffic_ai_trailer_t : public /* [W|L|A 0xF0] @ W|L|A 0x00 */ traffic_item_passive_actor_t,
                                 public /* [W|L|A 0x10] @ W|L|A 0xF0 */ traffic_trailer_model_object_t,
                                 public /* [W 0x28, L|A 0x30] @ W|L|A 0x100 */ traffic_trailer_physics_object_t,
                                 public /* [W|L|A 0x1C8] @ W 0x128, L|A 0x130 */ traffic_vehicle_mp_t
    {
    public:
        class traffic_trailer_u* traffic_trailer;  // W 0x02F0, L|A 0x02F8 (W|L|A 0x08)
        class traffic_ai_trailer_t* slave_trailer; // W 0x02F8, L|A 0x0300 (W|L|A 0x08)
        char pad_W_0x0300__LA_0x0308[ 32 ];        // W 0x0300, L|A 0x0308 (W|L|A 0x20)
    };
#if defined( _WIN32 )
    static_assert( sizeof( traffic_ai_trailer_t ) == 0x320 ); // W
#elif defined( __linux__ ) || defined( __APPLE__ )
    static_assert( sizeof( traffic_ai_trailer_t ) == 0x328 ); // L|A
#endif

#pragma pack( pop )
}

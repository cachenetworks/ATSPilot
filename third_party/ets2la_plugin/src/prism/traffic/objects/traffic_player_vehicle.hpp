#pragma once

#include "prism/collections/array_dyn.hpp"
#include "prism/collections/array_local.hpp"
#include "prism/traffic/objects/traffic_item_active_actor.hpp"
#include "prism/unit/unit.hpp"

namespace ets2la_plugin::prism
{
#pragma pack( push, 1 )

    // Size: W 0x0398, L|A 0x0358 1.60
    class traffic_player_vehicle_t : public /* [W 0x208, L|A 0x1D8] @ W|L|A 0x00 */ traffic_item_active_actor_t
    {
        class crashed_actor_t;
        class red_light_offence_check_t;
        class navigation_t;

    public:
        traffic_player_base_object_t_templ< unit_t >* traffic_player_object;     // W 0x0208, L|A 0x01D8 (W|L|A 0x08)
                                                                                 // physics_vehicle_u | remote_vehicle_u
        class traffic_player_trailer_t* trailer;                                 // W 0x0210, L|A 0x01E0 (W|L|A 0x08)
        float speed;                                                             // W 0x0218, L|A 0x01E8 (W|L|A 0x04)
        float acceleration;                                                      // W 0x021C, L|A 0x01EC (W|L|A 0x04)
        char pad_W_0x0220__LA_0x01F0[ 16 ];                                      // W 0x0220, L|A 0x01F0 (W|L|A 0x10)
        class navigation_t* navigation;                                          // W 0x0230, L|A 0x0200 (W|L|A 0x08)
        char pad_W_0x0238__LA_0x0208[ 16 ];                                      // W 0x0238, L|A 0x0208 (W|L|A 0x10)
        class dangerous_actor_t* dangerous_actor;                                // W 0x0248, L|A 0x0218 (W|L|A 0x08)
        char pad_W_0x0250__LA_0x0220[ 80 ];                                      // W 0x0250, L|A 0x0220 (W|L|A 0x50)
        array_local_t< placement_t, 4 > N0000CE2B;                               // W 0x02A0, L|A 0x0270 (W|L|A 0xa0)
        array_dyn_t< class crashed_actor_t > crashed_actors;                     // W 0x0340, L|A 0x0310 (W 0x28,
                                                                                 // L|A 0x20)
        array_dyn_t< class red_light_offence_check_t > red_light_offence_checks; // W 0x0368, L|A 0x0330 (W 0x28, L|A
                                                                                 // 0x20)
        char pad_W_0x0390__LA_0x0350[ 8 ];                                       // W 0x0390, L|A 0x0350 (W|L|A 0x08)
    };

#if defined( _WIN32 )
    static_assert( sizeof( traffic_player_vehicle_t ) == 0x398 ); // W
#elif defined( __linux__ ) || defined( __APPLE__ )
    static_assert( sizeof( traffic_player_vehicle_t ) == 0x358 ); // L|A
#endif

#pragma pack( pop )
}

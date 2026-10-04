#pragma once

#include "prism/collections/list_dyn.hpp"
#include "prism/traffic/objects/traffic_item_actor.hpp"

namespace ets2la_plugin::prism
{
#pragma pack( push, 1 )

    // Size: W 0x0208, L|A 0x01D8 1.60
    class traffic_item_active_actor_t : public /* [W|L|A 0xA8] @ W|L|A 0x00 */ traffic_item_actor_t
    {
    public:
        class path_item_t;
        class path_rule_t;

        char pad_WLA_0x00A8[ 72 ];                   // W|L|A 0x00A8 (W|L|A 0x48)
        list_dyn_t< class path_item_t* > path_items; // W|L|A 0x00F0 (W 0x70, L|A 0x58)
        char pad_W_0x0160__LA_0x0148[ 56 ];          // W 0x0160, L|A 0x0148 (W|L|A 0x38)
        list_dyn_t< class path_rule_t* > path_rules; // W 0x0198, L|A 0x0180 (W 0x70, L|A
                                                     // 0x58)
    };
#if defined( _WIN32 )
    static_assert( sizeof( traffic_item_active_actor_t ) == 0x208 ); // W
#elif defined( __linux__ ) || defined( __APPLE__ )
    static_assert( sizeof( traffic_item_active_actor_t ) == 0x1D8 ); // L|A
#endif

#pragma pack( pop )
}

#pragma once

#include "prism/traffic/objects/traffic_parked_actor.hpp"
#include "prism/traffic/traffic_trailer_model.hpp"
#include "prism/traffic/traffic_trailer_physics.hpp"

namespace ets2la_plugin::prism
{
#pragma pack( push, 1 )

    // Size: W 0x00E0, L|A 0x00E8
    class traffic_parked_trailer_t : public /* [W|L|A 0x10] @ W|L|A 0x00 */ traffic_trailer_model_object_t,
                                     public /* [W 0x28, L|A 0x30] @ W|L|A 0x10 */ traffic_trailer_physics_object_t,
                                     public /* [W|L|A 0xA8] @ W 0x38, L|A 0x40 */ traffic_parked_actor_t
    {
    public:
    };

#if defined( _WIN32 )
    static_assert( sizeof( traffic_parked_trailer_t ) == 0xE0 ); // W
#elif defined( __linux__ ) || defined( __APPLE__ )
    static_assert( sizeof( traffic_parked_trailer_t ) == 0xE8 ); // L|A
#endif

#pragma pack( pop )
}

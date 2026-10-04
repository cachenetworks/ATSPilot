#pragma once

#include "./traffic_semaphore.hpp"

namespace ets2la_plugin::prism
{
#pragma pack( push, 1 )

    // Size: W 0x00C0, L|A 0x00B0
    class traffic_light_t : public /* [W 0x78, L|A 0x68] @ W|L|A 0x00 */ traffic_semaphore_t
    {
    public:
        char pad_W_0x0078__LA_0x0068[ 64 ]; // W 0x0078, L|A 0x0068 (W|L|A 0x40)
        float state_time_remaining;         // W 0x00B8, L|A 0x00A8 (W|L|A 0x04)
        char pad_W_0x00BC__LA_0x00AC[ 4 ];  // W 0x00BC, L|A 0x00AC (W|L|A 0x04)

        struct ELightState
        {
            enum
            {
                OFF             = 0,
                ORANGE_TO_RED   = 1 << 0,
                RED             = 1 << 1,
                ORANGE_TO_GREEN = 1 << 2,
                GREEN           = 1 << 3,
                SLEEP           = 1 << 5,
            };
        };
    };

#if defined( _WIN32 )
    static_assert( sizeof( traffic_light_t ) == 0xC0 ); // W
#elif defined( __linux__ ) || defined( __APPLE__ )
    static_assert( sizeof( traffic_light_t ) == 0xB0 ); // L|A
#endif

#pragma pack( pop )
}

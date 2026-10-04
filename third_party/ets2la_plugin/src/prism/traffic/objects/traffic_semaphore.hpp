#pragma once

#include "prism/traffic/traffic_obstacle.hpp"
#include "prism/traffic/traffic_rule.hpp"

#include <cstdint>

namespace ets2la_plugin::prism
{
#pragma pack( push, 1 )

    // Size: W 0x0078, L|A 0x0068
    class traffic_semaphore_t : public /* [W|L|A 0x10] @ W|L|A 0x00 */ traffic_rule_t,
                                public /* [W|L|A 0x08] @ W|L|A 0x10 */ traffic_obstacle_t,
                                public /* [W 0x58, L|A 0x48] @ W|L|A 0x18 */ traffic_item_rule_source_t
    {
    public:
        uint32_t state;                    // W 0x0070, L|A 0x0060 (W|L|A 0x04)
        char pad_W_0x0074__LA_0x0064[ 4 ]; // W 0x0074, L|A 0x0064 (W|L|A 0x04)
    };

#if defined( _WIN32 )
    static_assert( sizeof( traffic_semaphore_t ) == 0x78 ); // W
#elif defined( __linux__ ) || defined( __APPLE__ )
    static_assert( sizeof( traffic_semaphore_t ) == 0x68 ); // L|A
#endif

#pragma pack( pop )
}

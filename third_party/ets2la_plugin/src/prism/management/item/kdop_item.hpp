#pragma once

#include "prism/management/item/kdop.hpp"
#include <cstdint>

namespace ets2la_plugin::prism
{
#pragma pack( push, 1 )

    // Size: W|L|A 0x0048 1.60
    class kdop_item_t
    {
    public:
        uint8_t view_dist;        // W|L|A 0x0008 (W|L|A 0x01)
        char pad_WLA_0x0009[ 1 ]; // W|L|A 0x0009 (W|L|A 0x01)
        uint8_t item_type;        // W|L|A 0x000A (W|L|A 0x01)
        char pad_WLA_0x000B[ 1 ]; // W|L|A 0x000B (W|L|A 0x01)
        kdop_t kdop;              // W|L|A 0x000C (W|L|A 0x28)
        uint32_t flags;           // W|L|A 0x0034 (W|L|A 0x04)
        char pad_WLA_0x0038[ 8 ]; // W|L|A 0x0038 (W|L|A 0x08)
        uint64_t uid;             // W|L|A 0x0040 (W|L|A 0x08)

        virtual void destructor();
#if defined( __linux__ ) || defined( __APPLE__ )
        virtual void destructor2();
#endif
    };

    static_assert( sizeof( kdop_item_t ) == 0x48 ); // W|L|A

#pragma pack( pop )
}

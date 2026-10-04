#pragma once

#include "prism/management/item/kdop_item.hpp"

namespace ets2la_plugin::prism
{
#pragma pack( push, 1 )

    // Size: W|L|A 0x0050 1.60
    class kdop_segment_item_t : public /* [W|L|A 0x48] @ W|L|A 0x00 */ kdop_item_t
    {
    public:
        class segment_t* segment; // W|L|A 0x0048 (W|L|A 0x08)
    };

    static_assert( sizeof( kdop_segment_item_t ) == 0x50 ); // W|L|A

#pragma pack( pop )
}

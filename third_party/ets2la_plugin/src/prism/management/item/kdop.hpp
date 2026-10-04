#pragma once

namespace ets2la_plugin::prism
{
#pragma pack( push, 1 )

    // Size: W|L|A 0x0028
    class kdop_t
    {
    public:
        float minimums[ 5 ]; // W|L|A 0x0000 (W|L|A 0x14)
        float maximums[ 5 ]; // W|L|A 0x0014 (W|L|A 0x14)
    };

    static_assert( sizeof( kdop_t ) == 0x28 ); // W|L|A

#pragma pack( pop )
}

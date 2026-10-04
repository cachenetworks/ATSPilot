#pragma once

#include <cstdint>

namespace ets2la_plugin::prism
{
#pragma pack( push, 1 )

    // Size: W|L|A 0x0010 1.60
    class driveable_item_t
    {
    public:
        uint8_t item_type;        // W|L|A 0x0008 (W|L|A 0x01)
        char pad_WLA_0x0009[ 7 ]; // W|L|A 0x0009 (W|L|A 0x07)

        virtual void destructor();
#if defined( __linux__ ) || defined( __APPLE__ )
        virtual void destructor2();
#endif
    };
    static_assert( sizeof( driveable_item_t ) == 0x10 ); // W|L|A

#pragma pack( pop )
}

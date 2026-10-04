#pragma once

namespace ets2la_plugin::prism
{
#pragma pack( push, 1 )

    // Size: W|L|A 0x01C8 1.60
    class traffic_vehicle_mp_t
    {
    public:
        char pad_WLA_0x0000[ 448 ]; // W|L|A 0x0000 (W|L|A 0x1c0)

        virtual void destructor();
    };
    static_assert( sizeof( traffic_vehicle_mp_t ) == 0x1C8 ); // W|L|A

#pragma pack( pop )
}

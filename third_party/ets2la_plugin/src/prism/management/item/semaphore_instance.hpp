#pragma once

#include <cstdint>

#pragma pack( push, 1 )

namespace ets2la_plugin::prism
{
    class semaphore_instance_t // Size: 0x0010
    {
    public:
        uint32_t id;                  //0x0000 (0x04) maybe
        uint32_t N000071C0;           //0x0004 (0x04)
        class traffic_actor_t* actor; //0x0008 (0x08)
    };

    static_assert( sizeof( semaphore_instance_t ) == 0x10 );
}

#pragma pack( pop )

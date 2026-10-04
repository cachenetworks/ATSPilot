#pragma once

#include "prism/traffic/traffic_model_shared.hpp"

#pragma pack( push, 1 )

namespace ets2la_plugin::prism
{
    // Size: 0x0010
    class traffic_vehicle_model_object_t : public /* [0x08] @ 0x00 */ traffic_model_object_t
    {
    public:
        class traffic_vehicle_model_t* vehicle_model; // 0x0008 (0x08)
    };
    static_assert( sizeof( traffic_vehicle_model_object_t ) == 0x10 );
}

#pragma pack( pop )

#pragma once

#pragma pack( push, 1 )

namespace ets2la_plugin::prism
{
    // Size: 0x0020
    class traffic_model_t
    {
    public:
        char pad_0008[ 24 ]; // 0x0008 (0x18)

        virtual void destructor();
    };
    static_assert( sizeof( traffic_model_t ) == 0x20 );

    // Size: 0x0008
    class traffic_model_object_t
    {
    public:
        virtual void destructor();
    };
    static_assert( sizeof( traffic_model_object_t ) == 0x8 );
}

#pragma pack( pop )

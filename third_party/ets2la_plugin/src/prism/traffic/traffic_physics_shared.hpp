#pragma once

namespace ets2la_plugin::prism
{
#pragma pack( push, 1 )
    // Size: 0x0020
    class traffic_physics_t
    {
    public:
        char pad_0008[ 24 ]; // 0x0008 (0x18)

        virtual void destructor();
    };
    static_assert( sizeof( traffic_physics_t ) == 0x20 );

    // Size: W 0x0020, L|A 0x0028 1.60
    class traffic_physics_object_t
    {
    public:
#if defined( _WIN32 )
        char pad_WLA_0x0000[ 24 ]; // W 0x0000 (W|L|A 0x18)
#endif
#if defined( __linux__ ) || defined( __APPLE__ )
        char pad_WLA_0x0018[ 32 ]; // L|A 0x0018 (W|L|A 0x20)
#endif

        virtual void destructor();
    };
#if defined( _WIN32 )
    static_assert( sizeof( traffic_physics_object_t ) == 0x20 ); // W
#elif defined( __linux__ ) || defined( __APPLE__ )
    static_assert( sizeof( traffic_physics_object_t ) == 0x28 ); // L|A
#endif

#pragma pack( pop )
}

#pragma once
#include "./route_source.hpp"
#include "prism/collections/array_dyn.hpp"
#include "prism/collections/array_local.hpp"
#include "prism/string.hpp"

namespace ets2la_plugin::prism
{
#pragma pack( push, 1 )

    // Size: W 0x05C8, L|A 0x0590 1.60
    class gps_manager_t
    {
    public:
        simple_route_source_t simple_route_source;              // W|L|A 0x0008 (W 0x188, L|A 0x170)
        char pad_W_0x0190__LA_0x0178[ 64 ];                     // W 0x0190, L|A 0x0178 (W|L|A 0x40)
        array_dyn_t< class route_task_node_t > start_point;     // W 0x01D0, L|A 0x01B8 (W 0x28, L|A 0x20)
        array_dyn_t< class route_task_node_t > waypoints;       // W 0x01F8, L|A 0x01D8 (W 0x28, L|A 0x20)
        array_dyn_t< class route_task_node_t > avoid_waypoints; // W 0x0220, L|A 0x01F8 (W 0x28, L|A 0x20)
        array_dyn_t< class route_task_node_t > N0000F08F;       // W 0x0248, L|A 0x0218 (W 0x28, L|A 0x20)
        char pad_W_0x0270__LA_0x0238[ 4 ];                      // W 0x0270, L|A 0x0238 (W|L|A 0x04)
        float trip_distance;                                    // W 0x0274, L|A 0x023C (W|L|A 0x04)
        float trip_time;                                        // W 0x0278, L|A 0x0240 (W|L|A 0x04)
        float next_checkpoint_distance;                         // W 0x027C, L|A 0x0244 (W|L|A 0x04)
        float next_checkpoint_time;                             // W 0x0280, L|A 0x0248 (W|L|A 0x04)
        char pad_W_0x0284__LA_0x024C[ 12 ];                     // W 0x0284, L|A 0x024C (W|L|A 0x0c)
        string_local_t< 256 > N0000F727;                        // W 0x0290, L|A 0x0258 (W|L|A 0x118)
        char pad_W_0x03A8__LA_0x0370[ 216 ];                    // W 0x03A8, L|A 0x0370 (W|L|A 0xd8)
        array_local_t< char[ 48 ], 5 > gps_voice_events;        // W 0x0480, L|A 0x0448 (W|L|A 0x110)
        char pad_W_0x0590__LA_0x0558[ 56 ];                     // W 0x0590, L|A 0x0558 (W|L|A 0x38)

        virtual void destructor();

        static gps_manager_t* get();
    };

#if defined( _WIN32 )
    static_assert( sizeof( gps_manager_t ) == 0x5C8 ); // W
#elif defined( __linux__ ) || defined( __APPLE__ )
    static_assert( sizeof( gps_manager_t ) == 0x590 ); // L|A
#endif

#pragma pack( pop )
}

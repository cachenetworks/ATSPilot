#pragma once

#include "prism/collections/array_local.hpp"
#include "prism/collections/list_dyn.hpp"
#include "prism/navigation/route_task.hpp"

namespace ets2la_plugin::prism
{
#pragma pack( push, 1 )

    // Size: W 0x0188, L|A 0x0170 1.60
    class simple_route_source_t
    {
    public:
        char pad_WLA_0x0008[ 24 ];                           // W|L|A 0x0008 (W|L|A 0x18)
        class route_task_t* route_task;                      // W|L|A 0x0020 (W|L|A 0x08)
        array_local_t< route_item_t, 2 > N0000F6E6;          // W|L|A 0x0028 (W|L|A 0x80)
        array_local_t< physical_route_item_t, 2 > N0000F6EC; // W|L|A 0x00A8 (W|L|A 0x60)
        class lane_routing_task_t* lane_routing_task;        // W|L|A 0x0108 (W|L|A 0x08)
        char pad_WLA_0x0110[ 8 ];                            // W|L|A 0x0110 (W|L|A 0x08)
        list_dyn_t< class thread::task_t* > N0000F704;       // W|L|A 0x0118 (W 0x70, L|A 0x58)

        virtual void destructor();
    };

#if defined( _WIN32 )
    static_assert( sizeof( simple_route_source_t ) == 0x188 ); // W
#elif defined( __linux__ ) || defined( __APPLE__ )
    static_assert( sizeof( simple_route_source_t ) == 0x170 ); // L|A
#endif

#pragma pack( pop )
}

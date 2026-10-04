#pragma once

#include "prism/collections/array_dyn.hpp"

#include "prism/thread/task.hpp"

namespace ets2la_plugin::prism
{
#pragma pack( push, 1 )

    class route_task_node_t // Size: 0x0018
    {
    public:
        uint32_t unk0;           // 0x0000 (0x04)
        uint32_t unk1;           // 0x0004 (0x04)
        class node_item_t* node; // 0x0008 (0x08)
        uint32_t unk2;           // 0x0010 (0x04)
        uint32_t unk3;           // 0x0014 (0x04)
    };
    static_assert( sizeof( route_task_node_t ) == 0x18 );

    class route_item_t // Size: 0x0030
    {
    public:
        uint32_t type;                 // 0x0000 (0x04) 0 = ptr is node_item_t; 1 = ptr is prefab_item_t
        uint32_t node_id;              // 0x0004 (0x04)
        class node_item_t* item_ptr;   // 0x0008 (0x08) node_item_t or prefab_item_t
        char pad_0010[ 8 ];            // 0x0010 (0x08)
        float distance_from_previous;  // 0x0018 (0x04) meters
        float time_from_previous;      // 0x001C (0x04) seconds
        uint16_t N00007113;            // 0x0020 (0x02)
        uint16_t N000070E6;            // 0x0022 (0x02)
        float total_distance_till_end; // 0x0024 (0x04) meters
        float total_time_till_end;     // 0x0028 (0x04) seconds
        char pad_002C[ 4 ];            // 0x002C (0x04)
    };
    static_assert( sizeof( route_item_t ) == 0x30 );

    class physical_route_item_t // Size: 0x0020
    {
    public:
        class node_item_t* node;       // 0x0000 (0x08)
        char pad_0008[ 4 ];            // 0x0008 (0x04)
        float distance_from_previous;  // 0x000C (0x04) meters
        float time_from_previous;      // 0x0010 (0x04) seconds
        float total_distance_till_end; // 0x0014 (0x04) meters
        float total_time_till_end;     // 0x0018 (0x04) seconds
        char pad_001C[ 4 ];            // 0x001C (0x04)
    };
    static_assert( sizeof( physical_route_item_t ) == 0x20 );

    // Size: W 0x0160, L|A 0x0138
    class route_task_t : public /* [W|L|A 0x38] @ W|L|A 0x00 */ thread::task_t
    {
    public:
        char pad_WLA_0x0038[ 24 ];                                       // W|L|A 0x0038 (W|L|A 0x18)
        array_dyn_t< class physical_route_item_t > physical_route_items; // W|L|A 0x0050 (W 0x28, L|A 0x20)
        array_dyn_t< class route_item_t > route_items;                   // W 0x0078, L|A 0x0070 (W 0x28, L|A 0x20)
        char pad_W_0x00A0__LA_0x0090[ 40 ];                              // W 0x00A0, L|A 0x0090 (W|L|A 0x28)
        array_dyn_t< class private_route_task_target_t > N000071A4;      // W 0x00C8, L|A 0x00B8 (W 0x28, L|A 0x20)
        char pad_W_0x00F0__LA_0x00D8[ 24 ];                              // W 0x00F0, L|A 0x00D8 (W|L|A 0x18)
        array_dyn_t< uint32_t > N00007011;                               // W 0x0108, L|A 0x00F0 (W 0x28, L|A 0x20)
        char pad_W_0x0130__LA_0x0110[ 8 ];                               // W 0x0130, L|A 0x0110 (W|L|A 0x08)
        array_dyn_t< uint32_t > N00007013;                               // W 0x0138, L|A 0x0118 (W 0x28, L|A 0x20)
    };

#if defined( _WIN32 )
    static_assert( sizeof( route_task_t ) == 0x160 ); // W
#elif defined( __linux__ ) || defined( __APPLE__ )
    static_assert( sizeof( route_task_t ) == 0x138 ); // L|A
#endif

#pragma pack( pop )
}

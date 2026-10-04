#pragma once

#include "prism/collections/array_dyn.hpp"
#include "prism/collections/array_local.hpp"

namespace ets2la_plugin::prism
{
#pragma pack( push, 1 )

    // Size: W 0x04B8, L|A 0x0400 1.60
    class segment_t
    {
    public:
        char pad_WLA_0x0008[ 32 ];                                       // W|L|A 0x0008 (W|L|A 0x20)
        class kdop_segment_item_t* item;                                 // W|L|A 0x0028 (W|L|A 0x08)
        array_dyn_t< class seg_hookup_instance_t > seg_hookup_instances; // W|L|A 0x0030 (W 0x28, L|A 0x20)
        array_dyn_t< class sign_item* > sign_items;                      // W 0x0058, L|A 0x0050 (W 0x28, L|A 0x20)
        array_dyn_t< class semaphore_instance_t > semaphore_instances;   // W 0x0080, L|A 0x0070 (W 0x28, L|A 0x20)
        char pad_W_0x00A8__LA_0x0090[ 32 ];                              // W 0x00A8, L|A 0x0090 (W|L|A 0x20)
        array_dyn_t< class segment_data_t > segment_datas;               // W 0x00C8, L|A 0x00B0 (W 0x28, L|A 0x20)
        array_dyn_t< class seg_handle_pos > seg_handles_pos;             // W 0x00F0, L|A 0x00D0 (W 0x28, L|A 0x20)
        array_local_t< array_dyn_t< class terrain_vertex_t >, 6 > terrain_vertices; // W 0x0118, L|A 0x00F0 (W 0x110,
                                                                                    // L|A 0xe0)
        array_local_t< array_dyn_t< class terrain_vertex_t >, 6 > N0000D31E; // W 0x0228, L|A 0x01D0 (W 0x110, L|A 0xe0)
        char pad_W_0x0338__LA_0x02B0[ 8 ];                                   // W 0x0338, L|A 0x02B0 (W|L|A 0x08)
        array_dyn_t< class segment_col_data_t > segment_col_datas;           // W 0x0340, L|A 0x02B8 (W 0x28, L|A 0x20)
        array_dyn_t< class vegetation_col_data_t > vegetation_col_datas;     // W 0x0368, L|A 0x02D8 (W 0x28, L|A 0x20)
        char pad_W_0x0390__LA_0x02F8[ 48 ];                                  // W 0x0390, L|A 0x02F8 (W|L|A 0x30)
        array_dyn_t< class instance_cluster_t* > instance_clusters;          // W 0x03C0, L|A 0x0328 (W 0x28, L|A 0x20)
        char pad_W_0x03E8__LA_0x0348[ 8 ];                                   // W 0x03E8, L|A 0x0348 (W|L|A 0x08)
        array_dyn_t< class instance_detail_veg_info_t > instance_detail_veg_infos; // W 0x03F0, L|A 0x0350 (W 0x28, L|A
                                                                                   // 0x20)
        char pad_W_0x0418__LA_0x0370[ 8 ];                                         // W 0x0418, L|A 0x0370 (W|L|A 0x08)
        array_dyn_t< class physics_shape_t* > physics_shapes;               // W 0x0420, L|A 0x0378 (W 0x28, L|A 0x20)
        array_dyn_t< class physics_actor_static_t* > physics_actors_static; // W 0x0448, L|A 0x0398 (W 0x28, L|A 0x20)
        char pad_W_0x0470__LA_0x03B8[ 8 ];                                  // W 0x0470, L|A 0x03B8 (W|L|A 0x08)
        class pp_model_simple_u* pp_model_simple;                           // W 0x0478, L|A 0x03C0 (W|L|A 0x08)
        class eut2pp_lod_info_u* eut2pp_lod_info;                           // W 0x0480, L|A 0x03C8 (W|L|A 0x08)
        array_local_t< float, 3 > N0000D342;                                // W 0x0488, L|A 0x03D0 (W|L|A 0x2c)
        char pad_W_0x04B4__LA_0x03FC[ 4 ];                                  // W 0x04B4, L|A 0x03FC (W|L|A 0x04)

        virtual void destructor();
#if defined( __linux__ ) || defined( __APPLE__ )
        virtual void destructor2();
#endif
    };

#if defined( _WIN32 )
    static_assert( sizeof( segment_t ) == 0x4B8 ); // W
#elif defined( __linux__ ) || defined( __APPLE__ )
    static_assert( sizeof( segment_t ) == 0x400 ); // L|A
#endif

#pragma pack( pop )
}

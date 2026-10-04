#pragma once
#include "prism/collections/array_dyn.hpp"
#include "prism/string.hpp"
#include "prism/traffic/objects/traffic_item_active_actor.hpp"
#include "prism/traffic/traffic_ai_task.hpp"
#include "prism/traffic/traffic_vehicle_model.hpp"
#include "prism/traffic/traffic_vehicle_mp.hpp"
#include "prism/traffic/traffic_vehicle_physics.hpp"

namespace ets2la_plugin::prism
{
#pragma pack( push, 1 )

    // Size: W 0x0598, L|A 0x0558 1.60
    class traffic_ai_vehicle_t : public /* [W 0x208, L|A 0x1D8] @ W|L|A 0x00 */ traffic_item_active_actor_t,
                                 public /* [W|L|A 0x10] @ W 0x208, L|A 0x1D8 */ traffic_vehicle_model_object_t,
                                 public /* [W 0x28, L|A 0x30] @ W 0x218, L|A 0x1E8 */ traffic_vehicle_physics_object_t,
                                 public /* [W|L|A 0x08] @ W 0x240, L|A 0x218 */ traffic_ai_task_handler_t,
                                 public /* [W|L|A 0x1C8] @ W 0x248, L|A 0x220 */ traffic_vehicle_mp_t
    {
    public:
        class traffic_vehicle_u* traffic_vehicle;           // W 0x0410, L|A 0x03E8 (W|L|A 0x08)
        class traffic_ai_trailer_t* trailer;                // W 0x0418, L|A 0x03F0 (W|L|A 0x08)
        char pad_W_0x0420__LA_0x03F8[ 160 ];                // W 0x0420, L|A 0x03F8 (W|L|A 0xa0)
        array_dyn_t< class traffic_ai_task_t* > ai_tasks;   // W 0x04C0, L|A 0x0498 (W 0x28, L|A 0x20)
        char pad_W_0x04E8__LA_0x04B8[ 8 ];                  // W 0x04E8, L|A 0x04B8 (W|L|A 0x08)
        class traffic_navigation_t* traffic_navigation;     // W 0x04F0, L|A 0x04C0 (W|L|A 0x08)
        class sound_player_traffic_vehicle_t* sound_player; // W 0x04F8, L|A 0x04C8 (W|L|A 0x08)
        char pad_W_0x0500__LA_0x04D0[ 16 ];                 // W 0x0500, L|A 0x04D0 (W|L|A 0x10)
        string_dyn_t vehicle_id_name;                       // W 0x0510, L|A 0x04E0 (W 0x20, L|A 0x18)
        string_dyn_t vehicle_type_name;                     // W 0x0530, L|A 0x04F8 (W 0x20, L|A 0x18)
        char pad_W_0x0550__LA_0x0510[ 72 ];                 // W 0x0550, L|A 0x0510 (W|L|A 0x48)
    };

#if defined( _WIN32 )
    static_assert( sizeof( traffic_ai_vehicle_t ) == 0x598 ); // W
#elif defined( __linux__ ) || defined( __APPLE__ )
    static_assert( sizeof( traffic_ai_vehicle_t ) == 0x558 ); // L|A
#endif

#pragma pack( pop )
}

#pragma once

#include "prism/sound/sound_player_semaphore.hpp"
#include "prism/traffic/objects/traffic_actor.hpp"
#include "prism/traffic/traffic_model_shared.hpp"
#include "prism/traffic/traffic_physics_shared.hpp"

namespace ets2la_plugin::prism
{
#pragma pack( push, 1 )

    // Size: W 0x01F0, L|A 0x01E0 1.60
    class traffic_semaphore_actor_t : public /* [W|L|A 0x80] @ W|L|A 0x00 */ traffic_actor_t,
                                      public /* [W|L|A 0x20] @ W|L|A 0x80 */ traffic_model_t,
                                      public /* [W 0x20, L|A 0x28] @ W|L|A 0xA0 */ traffic_physics_object_t
    {
        class collision_group_t;

    public:
        class semaphore_model_u* semaphore_model;                       // W 0x00C0, L|A 0x00C8 (W|L|A 0x08)
        char pad_W_0x00C8__LA_0x00D0[ 16 ];                             // W 0x00C8, L|A 0x00D0 (W|L|A 0x10)
        class game_traffic_model_effects_t* game_traffic_model_effects; // W 0x00D8, L|A 0x00E0 (W|L|A 0x08)
        class model_object_u* model_object;                             // W 0x00E0, L|A 0x00E8 (W|L|A 0x08)
        char pad_W_0x00E8__LA_0x00F0[ 24 ];                             // W 0x00E8, L|A 0x00F0 (W|L|A 0x18)
        class traffic_light_t* traffic_rule;                            // W 0x0100, L|A 0x0108 (W|L|A 0x08)
        uint32_t state; // W 0x0108, L|A 0x0110 (W|L|A 0x04) 2 = opening; 3 = open; 0 = closing; 1 = closed
        char pad_W_0x010C__LA_0x0114[ 12 ]; // W 0x010C, L|A 0x0114 (W|L|A 0x0c)
        float time_to_open;                 // W 0x0118, L|A 0x0120 (W|L|A 0x04)
        float open_time_remaining;          // W 0x011C, L|A 0x0124 (W|L|A 0x04) used for double animation
        char pad_W_0x0120__LA_0x0128[ 8 ];  // W 0x0120, L|A 0x0128 (W|L|A 0x08)
        float time_to_close;                // W 0x0128, L|A 0x0130 (W|L|A 0x04)
        float close_time_remaining;         // W 0x012C, L|A 0x0134 (W|L|A 0x04) used for double animation
        char pad_W_0x0130__LA_0x0138[ 6 ];  // W 0x0130, L|A 0x0138 (W|L|A 0x06)
        uint8_t animation_mode;             // W 0x0136, L|A 0x013E (W|L|A 0x01)
        uint8_t coll_mode;                  // W 0x0137, L|A 0x013F (W|L|A 0x01)
        char pad_W_0x0138__LA_0x0140[ 4 ];  // W 0x0138, L|A 0x0140 (W|L|A 0x04)
        float animation_time_elapsed;       // W 0x013C, L|A 0x0144 (W|L|A 0x04) used for single animation
        uint8_t N00007222;                  // W 0x0140, L|A 0x0148 (W|L|A 0x01)
        uint8_t N0000723A;                  // W 0x0141, L|A 0x0149 (W|L|A 0x01)
        char pad_W_0x0142__LA_0x014A[ 6 ];  // W 0x0142, L|A 0x014A (W|L|A 0x06)
        array_dyn_t< class collision_group_t > collision_groups; // W 0x0148, L|A 0x0150 (W 0x28, L|A 0x20)
        array_dyn_t< class physics_shape_t* > physics_shapes; // W|L|A 0x0170 (W 0x28, L|A 0x20)
        char pad_W_0x0198__LA_0x0190[ 8 ];                    // W 0x0198, L|A 0x0190 (W|L|A 0x08)
        sound_player_semaphore_t sound_player_semaphore;      // W 0x01A0, L|A 0x0198 (W 0x50, L|A 0x48)
    };
#if defined( _WIN32 )
    static_assert( sizeof( traffic_semaphore_actor_t ) == 0x1F0 ); // W
#elif defined( __linux__ ) || defined( __APPLE__ )
    static_assert( sizeof( traffic_semaphore_actor_t ) == 0x1E0 ); // L|A
#endif

#pragma pack( pop )
}

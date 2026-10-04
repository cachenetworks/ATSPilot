#pragma once
#include "vehicle_shared.hpp"

namespace ets2la_plugin::prism
{
#pragma pack( push, 1 )

    class physics_trailer_u : public vehicle_shared_u // Size: 0x0ED0 1.54
    {
    public:
        // char pad_0D68[ 8 ]; // 0x0D68 (0x08)
        // class physics_joint_physx_t* physics_joint; // 0x0D70 (0x08) nullptr when trailer not attached
        // char pad_0D78[ 128 ]; // 0x0D78 (0x80)
        // class vehicle_shared_u* parent_vehicle; // 0x0DF8 (0x08) truck for 1st trailer
        // class game_trailer_actor_u* slave_trailer__dont_use; // 0x0E00 (0x08)
        // class accessory_trailer_data_u* accessory_trailer_data; // 0x0E08 (0x08)
        // class vehicle_accessory_u* transmission_accessory; // 0x0E10 (0x08)
        // class vehicle_accessory_u* engine_accessory; // 0x0E18 (0x08)
        // class array_dyn_t< class model_object_u* > body_model_object; // 0x0E20 (0x20)
        // char pad_0E40[ 32 ]; // 0x0E40 (0x20)
        // class accessory_trailer_body_data_u* accessory_trailer_body_data; // 0x0E60 (0x08)
        // char pad_0E68[ 64 ]; // 0x0E68 (0x40)
        // class array_dyn_t< class dynamic_cargo_t* > dynamic_cargoes; // 0x0EA8 (0x20)
        // char pad_0EC8[ 8 ]; // 0x0EC8 (0x08)
    };
    // static_assert( sizeof( physics_trailer_u ) == 0xED0 );

    class physics_trailer_actor_u : public physics_trailer_u // Size: 0x0ED0 1.54
    {
    public:
    };

    // static_assert( sizeof( physics_trailer_actor_u ) == 0xED0 );

    class game_trailer_actor_u : public physics_trailer_actor_u // Size: 0x1000 1.54
    {
    public:
        // class model_object_u* loading_model_object; // 0x0ED0 (0x08)
        // class eutpp_paint_u* paint; // 0x0ED8 (0x08)
        // char pad_0EE0[ 96 ]; // 0x0EE0 (0x60)
        // class eutpp_paint_u* paint_2; // 0x0F40 (0x08)
        // class eut2pp_truckpaint_u* truck_paint; // 0x0F48 (0x08)
        // char pad_0F50[ 56 ]; // 0x0F50 (0x38)
        // class pp_lamp_colors_u* lamp_colors; // 0x0F88 (0x08)
        // char pad_0F90[ 20 ]; // 0x0F90 (0x14)
        // float brace_state; // 0x0FA4 (0x04)
        // float brace_anim_state; // 0x0FA8 (0x04)
        // float brace_speed; // 0x0FAC (0x04)
        // char pad_0FB0[ 8 ]; // 0x0FB0 (0x08)
        // class array_dyn_t< class dynamic_load_unload_compartment_t > dynamic_load_unload_compartments; // 0x0FB8
        // (0x20) char pad_0FD8[ 8 ]; // 0x0FD8 (0x08) class history_stream_set_u* history_stream; // 0x0FE0 (0x08) char
        // pad_0FE8[ 16 ]; // 0x0FE8 (0x10) class sound_player_trailer_t* sound_player; // 0x0FF8 (0x08)

        static uint32_t slave_trailer_offset;

        static bool scan_patterns();
        game_trailer_actor_u* get_slave_trailer();
    };
    // static_assert( sizeof( game_trailer_actor_u ) == 0x1000 );

#pragma pack( pop )
}

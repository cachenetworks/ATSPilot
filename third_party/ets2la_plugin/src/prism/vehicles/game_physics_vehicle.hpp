#pragma once
#include "vehicle_shared.hpp"

namespace ets2la_plugin::prism
{
#pragma pack( push, 1 )

    class physics_vehicle_u : public vehicle_shared_u // Size: 0x1218 1.54
    {
    public:
        // class array_dyn_t< model_object_u* > cab_model_objects; // 0x0D68 (0x20)
        // class model_object_u* ext_interior_model_object; // 0x0D88 (0x08)
        // class model_object_u* wipers_model_object; // 0x0D90 (0x08)
        // class model_object_u* driver_swheel_model_object; // 0x0D98 (0x08)
        // class model_object_u* win_open_model_object; // 0x0DA0 (0x08)
        // char pad_0DA8[ 8 ]; // 0x0DA8 (0x08)
        // class N0000592A* N00004468; // 0x0DB0 (0x08)
        // class N0000592A* N00004469; // 0x0DB8 (0x08)
        // class accessory_truck_data_u* accessory_truck_data; // 0x0DC0 (0x08)
        // class vehicle_accessory_u* transmission_accessory; // 0x0DC8 (0x08)
        // class vehicle_accessory_u* engine_accessory; // 0x0DD0 (0x08)
        // class vehicle_accessory_u* chassis_accessory; // 0x0DD8 (0x08)
        // class vehicle_accessory_u* cabin_accessory; // 0x0DE0 (0x08)
        // char pad_0DE8[ 176 ]; // 0x0DE8 (0xb0)
        // float air_pressure; // 0x0E98 (0x04)
        // char pad_0E9C[ 4 ]; // 0x0E9C (0x04)
        // float oil_pressure; // 0x0EA0 (0x04)
        // float oil_temperature; // 0x0EA4 (0x04)
        // char pad_0EA8[ 4 ]; // 0x0EA8 (0x04)
        // float water_temperature; // 0x0EAC (0x04)
        // char pad_0EB0[ 4 ]; // 0x0EB0 (0x04)
        // float nominal_battery_voltage_factor; // 0x0EB4 (0x04)
        // float brake_temps; // 0x0EB8 (0x04)
        // float brake_input; // 0x0EBC (0x04)
        // char pad_0EC0[ 416 ]; // 0x0EC0 (0x1a0)
        // class array_dyn_t< class physics_shape_t* > N00002D81; // 0x1060 (0x20)
        // class array_dyn_t< class physics_shape_t* > N00002D85; // 0x1080 (0x20)
        // char pad_10A0[ 88 ]; // 0x10A0 (0x58)
        // float physics_steering; // 0x10F8 (0x04)
        // char pad_10FC[ 24 ]; // 0x10FC (0x18)
        // float throttle; // 0x1114 (0x04)
        // float throttle_2; // 0x1118 (0x04)
        // float clutch; // 0x111C (0x04)
        // char pad_1120[ 16 ]; // 0x1120 (0x10)
        // uint64_t coasting_idk; // 0x1130 (0x08)
        // int32_t gear; // 0x1138 (0x04)
        // char pad_113C[ 4 ]; // 0x113C (0x04)
        // uint32_t trailer_attach_state; // 0x1140 (0x04) 0 = detached, 1 = detaching,
        // 2 = kingpin locked brace not raised, 3 = attaching 4 = attached
        // char pad_1144[ 212 ]; // 0x1144 (0xd4)
    };
    // static_assert( sizeof( physics_vehicle_u ) == 0x1218 );

    class game_physics_vehicle_u : public physics_vehicle_u // Size: 0x12E8 1.54
    {
    public:
        // char pad_1218[ 64 ]; // 0x1218 (0x40)
        // class eutpp_paint_u* paint; // 0x1258 (0x08)
        // class eut2pp_truckpaint_u* truckpaint; // 0x1260 (0x08)
        // char pad_1268[ 56 ]; // 0x1268 (0x38)
        // class pp_lamp_colors_u* lamp_colors; // 0x12A0 (0x08)
        // class pp_lamp_colors_u* lamp_colors2; // 0x12A8 (0x08)
        // class pp_lamp_colors_u* lamp_colors3; // 0x12B0 (0x08)
        // class eut2pp_luminosity_u* luminosity; // 0x12B8 (0x08)
        // char pad_12C0[ 40 ]; // 0x12C0 (0x28)

        static uint32_t trailer_offset;

        static bool scan_patterns();
        class game_trailer_actor_u* get_trailer();
    };
    // static_assert( sizeof( game_physics_vehicle_u ) == 0x12E8 );

#pragma pack( pop )
}

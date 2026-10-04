#pragma once

#include "prism/common.hpp"

#pragma pack( push, 1 )

namespace ets2la_plugin::prism
{
    enum ETrafficObjectType : uint64_t
    {
        traffic_ai_vehicle      = 0x01,
        traffic_player_vehicle  = 0x02,
        traffic_ai_trailer      = 0x03,
        traffic_player_trailer  = 0x04,
        traffic_parked_vehicle  = 0x05,
        traffic_parked_trailer  = 0x06,
        traffic_semaphore_actor = 0x07,

        speed_limit             = 0x1000,
        traffic_light           = 0x2000,
        automatic_barrier       = 0x3000,
        crossing_object_barrier = 0x4000,
        intersection            = 0x5000,
        vehicle_lights_modifier = 0x6000,
        stopper                 = 0x7000,
        dangerous_actor         = 0x8000,
        traffic_item_property   = 0x9000,
        country_border          = 0xA000,
        traffic_environment     = 0xB000,
        actor_block             = 0xC000,
        priority_modifier       = 0xD000,
        actor_priority_modifier = 0xE000,
        road_block              = 0xF000,

        intersection_slot            = 0x10000,
        traffic_spawn_point          = 0x11000,
        traffic_light_blockable      = 0x12000,
        traffic_curvature            = 0x13000,
        traffic_semaphore_stop_point = 0x14000,
        traffic_access_modifier      = 0x15000,
        traffic_spawn_density        = 0x16000,
        traffic_light_stopper        = 0x17000,
        trajectory_order_rule        = 0x18000,
        traffic_light_roadwork       = 0x19000,
        traffic_lane_modifier        = 0x20000,
        vehicle_horn_control         = 0x21000,

        traffic_road                 = 0x100000,
        traffic_prefab               = 0x200000,
        wrong_way_item               = 0x300000,
        traffic_segment_dynamic_item = 0x400000,
        traffic_road_lane            = 0x500000,
        traffic_prefab_part          = 0x600000,
        traffic_trajectory_item      = 0x900000,
        traffic_trajectory           = 0xA00000,
        traffic_gate                 = 0xB00000,
    };

    class traffic_object_t // Size: 0x0010
    {
    public:
        char pad_0008[ 8 ]; // 0x0008 (0x08)

        virtual void destructor();

#if defined( __linux__ )
        virtual void destructor2();
#endif

        virtual ETrafficObjectType get_type() const;
    };
    static_assert( sizeof( traffic_object_t ) == 0x10 );

    // Size: 0x0028
    class traffic_item_object_t
    {
    public:
        class traffic_item_t* traffic_item; // 0x0008 (0x08)
        char pad_0010[ 24 ];                // 0x0010 (0x18)

        virtual void destructor();
    };
    static_assert( sizeof( traffic_item_object_t ) == 0x28 );

    // Size: 0x0010
    template < typename T >
    class traffic_player_base_object_t_templ
    {
    public:
        T* object;

        virtual void destructor();
    };
    static_assert( sizeof( traffic_player_base_object_t_templ< class physics_vehicle_u > ) == 0x10 );
}

#pragma pack( pop )

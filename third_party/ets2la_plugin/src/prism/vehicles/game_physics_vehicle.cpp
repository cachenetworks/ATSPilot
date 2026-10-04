#include "game_physics_vehicle.hpp"

#include "core.hpp"
#include "memory/memory_utils.hpp"
#include "patterns.hpp"

namespace ets2la_plugin::prism
{
    uint32_t game_physics_vehicle_u::trailer_offset = 0;

    bool game_physics_vehicle_u::scan_patterns()
    {
        const auto addr = memory::get_address_for_pattern(
            patterns::game_physics_vehicle::trailer::pattern, patterns::game_physics_vehicle::trailer::offset
        );

        if ( addr == 0 )
        {
            return false;
        }

        game_physics_vehicle_u::trailer_offset = *reinterpret_cast< uint32_t* >( addr );

        CCore::g_instance->debug(
            "Found game_physics_vehicle::trailer_offset {:x}", game_physics_vehicle_u::trailer_offset
        );

        return true;
    }

    game_trailer_actor_u* game_physics_vehicle_u::get_trailer()
    {
        if ( game_physics_vehicle_u::trailer_offset == 0 )
        {
            return nullptr;
        }

        return *reinterpret_cast< game_trailer_actor_u** >(
            reinterpret_cast< char* >( this ) + game_physics_vehicle_u::trailer_offset
        );
    }
}

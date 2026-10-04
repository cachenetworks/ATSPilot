#include "game_trailer_actor.hpp"

#include "core.hpp"
#include "memory/memory_utils.hpp"
#include "patterns.hpp"

namespace ets2la_plugin::prism
{
    uint32_t game_trailer_actor_u::slave_trailer_offset = 0;

    bool game_trailer_actor_u::scan_patterns()
    {
        const auto addr = memory::get_address_for_pattern(
            patterns::game_trailer_actor::slave_trailer::pattern, patterns::game_trailer_actor::slave_trailer::offset
        );

        if ( addr == 0 )
        {
            return false;
        }

        game_trailer_actor_u::slave_trailer_offset = *reinterpret_cast< uint32_t* >( addr );

        CCore::g_instance->debug(
            "Found game_trailer_actor_u::slave_trailer_offset {:x}", game_trailer_actor_u::slave_trailer_offset
        );

        return true;
    }

    game_trailer_actor_u* game_trailer_actor_u::get_slave_trailer()
    {
        if ( game_trailer_actor_u::slave_trailer_offset == 0 )
        {
            return nullptr;
        }

        return *reinterpret_cast< game_trailer_actor_u** >(
            reinterpret_cast< char* >( this ) + game_trailer_actor_u::slave_trailer_offset
        );
    }
}

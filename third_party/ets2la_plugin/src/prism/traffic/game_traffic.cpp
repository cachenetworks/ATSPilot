#include "game_traffic.hpp"

#include "core.hpp"
#include "memory/memory_utils.hpp"
#include "patterns.hpp"

namespace ets2la_plugin::prism
{
    uint64_t game_traffic_u::instance_ptr_address = 0;

    bool game_traffic_u::scan_patterns()
    {
        const auto addr =
            memory::get_address_for_pattern( patterns::game_traffic::pattern, patterns::game_traffic::offset_instance );

        if ( addr == 0 )
        {
            return false;
        }
        game_traffic_u::instance_ptr_address = memory::get_absolute_address_from_offset( addr );

        CCore::g_instance->debug(
            "Found game_traffic @ +{:x}", memory::as_offset( game_traffic_u::instance_ptr_address )
        );

        return true;
    }

    game_traffic_u* game_traffic_u::get()
    {
        if ( game_traffic_u::instance_ptr_address != 0 )
        {
            return *reinterpret_cast< game_traffic_u** >( game_traffic_u::instance_ptr_address );
        }
        return nullptr;
    }
}

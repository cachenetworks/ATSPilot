#include "game_ctrl.hpp"
#include "memory/memory_utils.hpp"
#include "patterns.hpp"
#include <cstring>
#include <stdexcept>

namespace ets2la_plugin::prism
{
    uint32_t game_ctrl_u::gps_manager_offset = 0;

    bool game_ctrl_u::scan_patterns()
    {
        const auto gps_manager_addr = memory::get_address_for_pattern(
            patterns::game_ctrl::gps_manager::pattern, patterns::game_ctrl::gps_manager::offset
        );
        if ( gps_manager_addr == 0 )
        {
            throw std::runtime_error( "Failed to find gps_manager offset" );
        }
        gps_manager_offset = *reinterpret_cast< uint32_t* >( gps_manager_addr );

        return true;
    }

    game_ctrl_u* game_ctrl_u::get()
    {
        static unit_descriptor_t* stored_game_ctrl_unit_descriptor = nullptr;

        auto* base_ctrl = base_ctrl_u::get();

        if ( base_ctrl == nullptr )
        {
            return nullptr;
        }

        // base_ctrl has multiple derived units that this ptr could be
        // so we need to check that the current one is 'game_ctrl'

        if ( stored_game_ctrl_unit_descriptor == nullptr )
        {
            // check if the base_ctrl ptr is of type 'game_ctrl' and store the address of the unit descriptor if it is
            // that way we can just compare the unit descriptor addresses instead of comparing strings every time
            if ( strcmp( *base_ctrl->get_unit_descriptor()->p_class_name, "game_ctrl" ) == 0 )
            {
                stored_game_ctrl_unit_descriptor = base_ctrl->get_unit_descriptor();
            }
        }

        if ( base_ctrl->get_unit_descriptor() == stored_game_ctrl_unit_descriptor )
        {
            return static_cast< game_ctrl_u* >( base_ctrl );
        }
        else
        {
            return nullptr;
        }
    }
}

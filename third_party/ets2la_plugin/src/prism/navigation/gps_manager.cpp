#include "gps_manager.hpp"
#include "prism/controllers/game_ctrl.hpp"

namespace ets2la_plugin::prism
{
    gps_manager_t* gps_manager_t::get()
    {
        const auto* game_ctrl = game_ctrl_u::get();

        if ( game_ctrl == nullptr || game_ctrl_u::gps_manager_offset == 0 )
        {
            return nullptr;
        }

        return reinterpret_cast< gps_manager_t* >(
            reinterpret_cast< uint64_t >( game_ctrl ) + game_ctrl_u::gps_manager_offset
        );
    }
}

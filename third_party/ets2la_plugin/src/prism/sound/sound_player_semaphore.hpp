#pragma once

#include "prism/collections/array_dyn.hpp"
#include "prism/common.hpp"

namespace ets2la_plugin::prism
{
#pragma pack( push, 1 )

    // Size: W 0x0050, L|A 0x0048
    class sound_player_semaphore_t
    {
    public:
        placement_t placement;                                  // W|L|A 0x0000 (W|L|A 0x20)
        array_dyn_t< class sound_semaphore_item_t* > N0000C0A7; // W|L|A 0x0020 (W 0x28, L|A 0x20)

        virtual void destructor();

#if defined( __linux__ ) || defined( __APPLE__ )
        virtual void destructor2();
#endif
    };

#if defined( _WIN32 )
    static_assert( sizeof( sound_player_semaphore_t ) == 0x50 ); // W
#elif defined( __linux__ ) || defined( __APPLE__ )
    static_assert( sizeof( sound_player_semaphore_t ) == 0x48 ); // L|A
#endif

#pragma pack( pop )
}

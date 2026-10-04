#pragma once
#include "./array_dyn.hpp"
#include <cstdint>

namespace ets2la_plugin::prism
{
#pragma pack( push, 1 )

    // Size: W 0x080, L|A 0x0060 1.60 ; has more template fields in actuality, but not ones we care about
    template < class K, class V >
    class map_hashed_t
    {
    public:
#if defined( _WIN32 )
        void* allocator; // W 0x0008 (W 0x08)
#endif
        array_dyn_t< K > keys;   // W 0x0010, L|A 0x0008 (W 0x28, L|A 0x20)
        array_dyn_t< V > values; // W 0x0038, L|A 0x0028 (W 0x28, L|A 0x20)

#if defined( _WIN32 )
        void* allocator2; // W 0x0060 (W 0x08)
#endif

        char pad_W_0x0068__LA_0x0048[ 24 ]; // W 0x0068, L|A 0x0048 (W|L|A 0x18)

        virtual void destructor();
    };

#if defined( _WIN32 )
    static_assert( sizeof( map_hashed_t< int32_t, void* > ) == 0x80, "map_hashed_t is incorrect in size" ); // W
#elif defined( __linux__ ) || defined( __APPLE__ )
    static_assert( sizeof( map_hashed_t< int32_t, void* > ) == 0x60, "map_hashed_t is incorrect in size" ); // L|A
#endif

#pragma pack( pop )
}

#pragma once
#include "./array.hpp"
#include <cstdint>

namespace ets2la_plugin::prism
{
#pragma pack( push, 1 )

    // Size: W|L|A 0x0048 1.60
    template < class T, uint64_t S >
    class array_local_t : public array_t< T >
    {
    public:
        uint64_t capacity;

    private:
        T _items[ S ];
    };

    static_assert( sizeof( array_local_t< void*, 5 > ) == 0x48 );

#pragma pack( pop )
}

#include "memory_utils.hpp"

#include "memory_scan.hpp"

namespace ets2la_plugin::memory
{
    uint64_t g_game_base = 0;
    uint64_t g_module_size = 0;

    uint64_t as_offset( const uint64_t address )
    {
        ensure_module_info();
        return address - g_game_base;
    }

    uint64_t get_address_for_pattern( const char* pattern, const uint64_t offset )
    {
        ensure_module_info();

        const auto scan_result = pattern::scan( pattern, g_game_base, g_module_size );

        if ( scan_result == 0 )
        {
            return 0;
        }

        return scan_result + offset;
    }

    uint64_t get_absolute_address_from_offset( const uint64_t address, const uint64_t offset )
    {
        return address + offset + *reinterpret_cast< int32_t* >( address + offset ) + 4;
    }
}

#pragma once

#include <cstdint>

namespace ets2la_plugin::memory
{
    extern uint64_t g_game_base;
    extern uint64_t g_module_size;

    void ensure_module_info();

    uint64_t as_offset( const uint64_t address );

    uint64_t get_address_for_pattern( const char* pattern, const uint64_t offset = 0 );

    uint64_t get_absolute_address_from_offset( const uint64_t address, const uint64_t offset = 0 );

    template < class T >
    T* get_function_from_pattern( const char* pattern, const uint64_t offset = 0 )
    {
        const auto address = get_address_for_pattern( pattern, offset );

        if ( address == 0 )
        {
            return nullptr;
        }

        return reinterpret_cast< T* >( address );
    }

    template < class T >
    T* get_vtable_function( const uint64_t vtable_address, const uint64_t offset )
    {
        return reinterpret_cast< T* >( *reinterpret_cast< uint64_t* >( vtable_address + offset * 0x08 ) );
    }
}

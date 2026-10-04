#include "memory_utils.hpp"

#include <Windows.h>
#include "core.hpp"

namespace ets2la_plugin::memory
{
    void ensure_module_info()
    {
        if ( g_game_base != 0 && g_module_size != 0 )
        {
            return;
        }
        g_game_base = reinterpret_cast< uint64_t >( GetModuleHandle( nullptr ) );
        if ( g_game_base == NULL )
        {
            return;
        }
        const auto header    = reinterpret_cast< const IMAGE_DOS_HEADER* >( g_game_base );
        const auto nt_header = reinterpret_cast< const IMAGE_NT_HEADERS64* >(
            reinterpret_cast< const uint8_t* >( header ) + header->e_lfanew
        );
        g_module_size = nt_header->OptionalHeader.SizeOfImage;

        CCore::g_instance->debug( "Found game_base: {:x} size: {:x}", g_game_base, g_module_size );
    }
}

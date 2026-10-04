// ATSPilot shim replacing ets2la_plugin's core.hpp. The vendored prism sources only
// use CCore for logging; ATSPilot routes those messages into its own log.
#pragma once

#include <format>
#include <string>

namespace ets2la_plugin
{
    class CCore
    {
    public:
        using sink_t = void ( * )( int level, const std::string& message );

        static CCore* g_instance;
        sink_t sink = nullptr;

        template < class... T >
        void debug( const char* fmt_s, T&&... args ) const
        {
            write( 0, fmt_s, args... );
        }

        template < class... T >
        void info( const char* fmt_s, T&&... args ) const
        {
            write( 1, fmt_s, args... );
        }

        template < class... T >
        void warning( const char* fmt_s, T&&... args ) const
        {
            write( 2, fmt_s, args... );
        }

        template < class... T >
        void error( const char* fmt_s, T&&... args ) const
        {
            write( 3, fmt_s, args... );
        }

    private:
        template < class... T >
        void write( int level, const char* fmt_s, T&... args ) const
        {
            if ( sink == nullptr ) return;
            try
            {
                sink( level, std::vformat( fmt_s, std::make_format_args( args... ) ) );
            }
            catch ( ... )
            {
            }
        }
    };
}

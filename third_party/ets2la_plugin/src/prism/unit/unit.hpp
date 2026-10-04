#pragma once
#include "unit_descriptor.hpp"
#include <cstdint>

#pragma pack( push, 1 )
namespace ets2la_plugin::prism
{
    class unit_t
    {
    public:
        uint32_t unk;
        uint32_t unk2;

    public:
        virtual void destructor();

#if defined( __linux__ )
        virtual void destructor2();
#endif

        virtual void destroy();
        virtual void clone();
        virtual void vfunc_3();
        virtual void vfunc_4();
        virtual unit_descriptor_t* get_unit_descriptor();
        virtual void vfunc_6();
        virtual void vfunc_7();
        virtual void pre_save();
        virtual void post_save();
        virtual void pre_load();
        virtual void post_load();
    };

    static_assert( sizeof( unit_t ) == 0x10, "unit_t is incorrect in size" );
}
#pragma pack( pop )

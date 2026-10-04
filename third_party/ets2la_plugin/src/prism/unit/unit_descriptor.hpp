#pragma once
#include <cstdint>

#pragma pack( push, 1 )
namespace ets2la_plugin::prism
{
    class unit_descriptor_t // Size: 0x0038
    {
    public:
        const char** p_class_name;             //0x0000 (0x08)
        void* constructor;                     //0x0008 (0x08)
        void* destructor;                      //0x0010 (0x08)
        class unit_descriptor_t* parent_class; //0x0018 (0x08)
        char pad_0020[ 24 ];                   //0x0020 (0x18)
    };
    static_assert( sizeof( unit_descriptor_t ) == 0x38 );
}
#pragma pack( pop )

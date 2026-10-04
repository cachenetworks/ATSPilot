#pragma once

#pragma pack( push, 1 )

namespace ets2la_plugin::prism::thread
{

    class task_t // Size: 0x0038
    {
    public:
        char pad_0008[ 40 ]; // 0x0008 (0x28)
        char* name;          // 0x0030 (0x08)

        virtual void destructor();
    };
    static_assert( sizeof( task_t ) == 0x38 );

}

#pragma pack( pop )

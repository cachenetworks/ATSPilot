#pragma once

#pragma pack( push, 1 )

namespace ets2la_plugin::prism
{

    // Size: 0x0008
    class traffic_ai_task_handler_t
    {
    public:
        virtual void destructor();
    };
    static_assert( sizeof( traffic_ai_task_handler_t ) == 0x8 );

}

#pragma pack( pop )

#pragma once

#include "prism/collections/array_dyn.hpp"
#include "prism/traffic/traffic_object.hpp"

namespace ets2la_plugin::prism
{
#pragma pack( push, 1 )

    // Size: 0x0010
    class traffic_rule_t : public /* [0x10] @ 0x00 */ traffic_object_t
    {
    public:
    };
    static_assert( sizeof( traffic_rule_t ) == 0x10 );

    // Size: W 0x0058, L|A 0x0048 1.60
    class traffic_item_rule_source_t
    {
    public:
        array_dyn_t< class traffic_item_rule_link_t* > N0000C14C; // W|L|A 0x0000 (W 0x28, L|A 0x20)
        array_dyn_t< class traffic_item_rule_link_t* > N0000C150; // W 0x0028, L|A 0x0020 (W 0x28, L|A 0x20)

        virtual void destructor();
#if defined( __linux__ ) || defined( __APPLE__ )
        virtual void destructor2();
#endif
    };

#if defined( _WIN32 )
    static_assert( sizeof( traffic_item_rule_source_t ) == 0x58 ); // W
#elif defined( __linux__ ) || defined( __APPLE__ )
    static_assert( sizeof( traffic_item_rule_source_t ) == 0x48 ); // L|A
#endif

    // Size: 0x0038
    class traffic_item_rule_t : public /* [0x10] @ 0x00 */ traffic_rule_t,
                                public /* [0x28] @ 0x10 */ traffic_item_object_t
    {
    public:
    };
    static_assert( sizeof( traffic_item_rule_t ) == 0x38 );

    // Size: 0x0038
    class traffic_item_rule_link_t : public /* [0x38] @ 0x00 */ traffic_item_rule_t
    {
    public:
    };
    static_assert( sizeof( traffic_item_rule_link_t ) == 0x38 );

#pragma pack( pop )
}

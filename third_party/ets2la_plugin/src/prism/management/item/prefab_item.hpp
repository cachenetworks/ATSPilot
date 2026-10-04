#pragma once

#include "./kdop_item.hpp"
#include "prism/collections/array_dyn.hpp"
#include "prism/management/item/driveable_item.hpp"
#include "prism/management/item/kdop_segment_item.hpp"
#include "prism/token.hpp"

namespace ets2la_plugin::prism
{
#pragma pack( push, 1 )

    // Size: W 0x01C8, L|A 0x01B0 1.60
    class prefab_item_t : public /* [W|L|A 0x50] @ W|L|A 0x00 */ kdop_segment_item_t,
                          public /* [W|L|A 0x10] @ W|L|A 0x50 */ driveable_item_t
    {
        class prefab_trigger_t;

    public:
        char pad_WLA_0x0060[ 48 ];                       // W|L|A 0x0060 (W|L|A 0x30)
        class prefab_model_u* prefab_model;              // W|L|A 0x0090 (W|L|A 0x08)
        char pad_WLA_0x0098[ 16 ];                       // W|L|A 0x0098 (W|L|A 0x10)
        array_dyn_t< token_t > extra_parts;              // W|L|A 0x00A8 (W 0x28, L|A 0x20)
        char pad_W_0x00D0__LA_0x00C8[ 88 ];              // W 0x00D0, L|A 0x00C8 (W|L|A 0x58)
        class prefab_payload_t* prefab_payload;          // W 0x0128, L|A 0x0120 (W|L|A 0x08)
        char pad_W_0x0130__LA_0x0128[ 56 ];              // W 0x0130, L|A 0x0128 (W|L|A 0x38)
        array_dyn_t< class kdop_item_t* > N0000D39F;     // W 0x0168, L|A 0x0160 (W 0x28, L|A 0x20)
        char pad_W_0x0190__LA_0x0180[ 8 ];               // W 0x0190, L|A 0x0180 (W|L|A 0x08)
        array_dyn_t< class prefab_trigger_t* > triggers; // W 0x0198, L|A 0x0188 (W 0x28, L|A 0x20)
        char pad_W_0x01C0__LA_0x01A8[ 8 ];               // W 0x01C0, L|A 0x01A8 (W|L|A 0x08)
    };

#if defined( _WIN32 )
    static_assert( sizeof( prefab_item_t ) == 0x1C8 ); // W
#elif defined( __linux__ ) || defined( __APPLE__ )
    static_assert( sizeof( prefab_item_t ) == 0x1B0 ); // L|A
#endif

#pragma pack( pop )
}

#include "camera_manager.hpp"

#include "core.hpp"
#include "memory/memory_utils.hpp"
#include "patterns.hpp"

namespace ets2la_plugin::prism
{

    uint64_t camera_manager_u::instance_ptr_address = 0;

    bool camera_manager_u::scan_patterns()
    {
        const auto addr = memory::get_address_for_pattern(
            patterns::camera_manager::pattern, patterns::camera_manager::offset_instance
        );

        if ( addr == 0 )
        {
            return false;
        }
        camera_manager_u::instance_ptr_address = memory::get_absolute_address_from_offset( addr );

        CCore::g_instance->debug(
            "Found camera_manager @ +{:x}", memory::as_offset( camera_manager_u::instance_ptr_address )
        );

        return true;
    }

    camera_manager_u* camera_manager_u::get()
    {
        if ( camera_manager_u::instance_ptr_address != 0 )
        {
            return *reinterpret_cast< camera_manager_u** >( camera_manager_u::instance_ptr_address );
        }
        return nullptr;
    }

}

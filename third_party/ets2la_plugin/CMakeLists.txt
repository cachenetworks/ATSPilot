# Game structure definitions and memory pattern scanning (MIT, see LICENSE.md).
add_library(ets2la_prism STATIC
  src/memory/memory_utils.shared.cpp
  src/memory/memory_utils.win32.cpp
  src/prism/common.cpp
  src/prism/controllers/base_ctrl.cpp
  src/prism/controllers/game_ctrl.cpp
  src/prism/game_actor.cpp
  src/prism/navigation/gps_manager.cpp
  src/prism/traffic/game_traffic.cpp
  src/prism/vehicles/vehicle_shared.cpp
)

target_include_directories(ets2la_prism PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/shim)
target_compile_features(ets2la_prism PUBLIC cxx_std_20)
target_compile_definitions(ets2la_prism PRIVATE UNICODE _UNICODE)
if(MSVC)
  # Third-party code: build it as published, without ATSPilot's warning level.
  target_compile_options(ets2la_prism PRIVATE /W0 /permissive)
endif()

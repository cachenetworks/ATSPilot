set(SCS_SDK_INCLUDE ${PROJECT_SOURCE_DIR}/external/scs_sdk_1_15/include)

add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/ets2la_plugin ${CMAKE_CURRENT_BINARY_DIR}/ets2la_plugin)

add_library(atspilot_plugin SHARED
  DevApi.cpp
  GameMemory.cpp
  Hud.cpp
  Input.cpp
  Runtime.cpp
  Telemetry.cpp
  atspilot.def
)

set_target_properties(atspilot_plugin PROPERTIES OUTPUT_NAME atspilot)
target_include_directories(atspilot_plugin PRIVATE ${SCS_SDK_INCLUDE})
target_link_libraries(atspilot_plugin PRIVATE atspilot_core ets2la_prism shell32 ole32 winmm user32 gdi32)

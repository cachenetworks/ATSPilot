add_executable(atspilot_sim sim_main.cpp)
target_link_libraries(atspilot_sim PRIVATE atspilot_core)

add_executable(atspilot_mapdump mapdump_main.cpp mapdump_map.cpp)
target_link_libraries(atspilot_mapdump PRIVATE atspilot_core)

if(WIN32)
  add_executable(atspilot_plugin_host plugin_host.cpp)
  target_include_directories(atspilot_plugin_host PRIVATE ${PROJECT_SOURCE_DIR}/external/scs_sdk_1_15/include)
  target_link_libraries(atspilot_plugin_host PRIVATE atspilot_core)
endif()

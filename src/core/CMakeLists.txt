add_library(atspilot_core STATIC
  config/Config.cpp
  config/Toml.cpp
  control/Lateral.cpp
  control/Longitudinal.cpp
  control/Pid.cpp
  control/SpeedPlanner.cpp
  control/SteeringShaper.cpp
  path/Path.cpp
  pilot/Autopilot.cpp
  pilot/Safety.cpp
  sim/Scenario.cpp
  sim/VehicleSim.cpp
  util/Logger.cpp
)

target_include_directories(atspilot_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(atspilot_core PUBLIC zlibstatic)

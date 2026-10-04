add_library(atspilot_core STATIC
  config/Config.cpp
  config/Toml.cpp
  control/GameCruise.cpp
  control/Lateral.cpp
  map/CityHash.cpp
  map/HashFs.cpp
  map/LanePlanner.cpp
  map/MapBuilder.cpp
  map/MapService.cpp
  map/PrefabDescription.cpp
  map/RoadNetwork.cpp
  map/RoutePlanner.cpp
  map/SectorParser.cpp
  map/Sii.cpp
  map/Token.cpp
  control/Longitudinal.cpp
  control/Pid.cpp
  control/SpeedPlanner.cpp
  control/SteeringShaper.cpp
  path/Path.cpp
  pilot/Autopilot.cpp
  pilot/Safety.cpp
  sim/Scenario.cpp
  sim/VehicleSim.cpp
  util/KeyBinding.cpp
  util/Logger.cpp
  util/TelemetryRecorder.cpp
)

target_include_directories(atspilot_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(atspilot_core PUBLIC zlibstatic)

add_executable(atspilot_tests
  main.cpp
  test_map.cpp
  test_math.cpp
  test_path.cpp
  test_route.cpp
  test_control.cpp
  test_config.cpp
  test_safety.cpp
  test_autopilot.cpp
  test_scenarios.cpp
)

target_link_libraries(atspilot_tests PRIVATE atspilot_core doctest::doctest)
add_test(NAME atspilot_tests COMMAND atspilot_tests)

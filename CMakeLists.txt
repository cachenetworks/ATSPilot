cmake_minimum_required(VERSION 3.24)

project(ATSPilot VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(ATSPILOT_BUILD_TESTS "Build unit and controller tests" ON)
option(ATSPILOT_BUILD_PLUGIN "Build the ATS plugin DLL (Windows x64 only)" ON)
option(ATSPILOT_BUILD_TOOLS "Build the simulator and map tools" ON)

if(MSVC)
  # Static runtime: the plugin DLL must not depend on a VC++ redistributable being installed.
  set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
  add_compile_options(/W4 /permissive- /utf-8 /Zc:__cplusplus /EHsc)
  add_compile_definitions(_CRT_SECURE_NO_WARNINGS NOMINMAX WIN32_LEAN_AND_MEAN)
else()
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(FetchContent)
set(FETCHCONTENT_QUIET ON)
# Third-party CMakeLists (doctest 2.4.11) still declare compatibility with CMake < 3.5.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

# zlib: inflating HashFS archive entries.
FetchContent_Declare(zlib
  URL https://github.com/madler/zlib/releases/download/v1.3.1/zlib-1.3.1.tar.gz
  URL_HASH SHA256=9a93b2b7dfdac77ceba5a558a580e74667dd6fede4585b91eefb60f03b72df23
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  EXCLUDE_FROM_ALL)
set(ZLIB_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(SKIP_INSTALL_ALL ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(zlib)
# zlib's CMake builds both shared and static targets; ATSPilot links the static one.
target_include_directories(zlibstatic PUBLIC ${zlib_SOURCE_DIR} ${zlib_BINARY_DIR})
if(MSVC)
  target_compile_options(zlibstatic PRIVATE /W0)
endif()

add_subdirectory(src/core)

if(ATSPILOT_BUILD_PLUGIN AND WIN32 AND CMAKE_SIZEOF_VOID_P EQUAL 8)
  add_subdirectory(src/plugin)
endif()

if(ATSPILOT_BUILD_TOOLS)
  add_subdirectory(src/tools)
endif()

if(ATSPILOT_BUILD_TESTS)
  FetchContent_Declare(doctest
    URL https://github.com/doctest/doctest/archive/refs/tags/v2.4.11.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  FetchContent_MakeAvailable(doctest)
  enable_testing()
  add_subdirectory(tests)
endif()

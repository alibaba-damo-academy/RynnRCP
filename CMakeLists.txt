cmake_minimum_required(VERSION 3.16)
project(rynnrcp LANGUAGES CXX)

option(RYNNRCP_BUILD_PYTHON_BINDINGS "Build Python native extension" OFF)
option(RYNNRCP_BUILD_TESTS "Build C++ tests" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile_commands.json for IDE integration
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Platform detection
if(WIN32)
    add_compile_definitions(RYNNRCP_PLATFORM_WINDOWS)
elseif(UNIX AND NOT APPLE)
    add_compile_definitions(RYNNRCP_PLATFORM_LINUX)
elseif(APPLE)
    add_compile_definitions(RYNNRCP_PLATFORM_MACOS)
endif()

add_subdirectory(rynnrcp/native/cpp)
add_subdirectory(rynnrcp/native/cpp/core)

# Testing
if(RYNNRCP_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests/cpp)
endif()

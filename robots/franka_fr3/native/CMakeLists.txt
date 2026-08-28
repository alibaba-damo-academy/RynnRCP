cmake_minimum_required(VERSION 3.16)
project(rynnrcp_franka_native LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Franka 0.13.3 REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

if(NOT RUCKIG_SOURCE_DIR OR NOT EXISTS "${RUCKIG_SOURCE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "RUCKIG_SOURCE_DIR must point to the pinned Ruckig source tree")
endif()
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_PYTHON_MODULE OFF CACHE BOOL "" FORCE)
set(BUILD_CLOUD_CLIENT OFF CACHE BOOL "" FORCE)
set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
add_subdirectory("${RUCKIG_SOURCE_DIR}" "${CMAKE_BINARY_DIR}/ruckig" EXCLUDE_FROM_ALL)

pybind11_add_module(_franka_native MODULE franka_native.cpp)
target_link_libraries(_franka_native PRIVATE Eigen3::Eigen Franka::Franka ruckig::ruckig)
target_compile_options(_franka_native PRIVATE -Wall -Wextra -Wpedantic)

if(FRANKA_RCP_OUTPUT_DIR)
  set_target_properties(
    _franka_native
    PROPERTIES
      LIBRARY_OUTPUT_DIRECTORY "${FRANKA_RCP_OUTPUT_DIR}"
  )
endif()
if(FRANKA_RCP_LIB_DIR)
  set_target_properties(
    _franka_native
    PROPERTIES
      BUILD_RPATH "${FRANKA_RCP_LIB_DIR}"
      INSTALL_RPATH "${FRANKA_RCP_LIB_DIR}"
  )
endif()

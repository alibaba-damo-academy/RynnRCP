cmake_minimum_required(VERSION 3.16)
project(aero_hand_mtnn LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

find_package(pybind11 CONFIG REQUIRED)

set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(PKG_ACCEL_DIR ${SRC_DIR}/../rynnrcp_robot_aero_hand/accelerators)

# System MTNN runtime only; preprocessing is pure C++ (no OpenCV).
find_library(MTNNRT_LIB mtnnrt PATHS /usr/lib REQUIRED)

pybind11_add_module(aero_hand_mtnn
  aero_hand_mtnn.cpp
  bindings_mtnn.cpp
)
target_include_directories(aero_hand_mtnn PRIVATE
  /usr/include/npu/include
)
target_link_libraries(aero_hand_mtnn PRIVATE ${MTNNRT_LIB})

set_target_properties(aero_hand_mtnn PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${PKG_ACCEL_DIR}
  BUILD_RPATH "\$ORIGIN"
  INSTALL_RPATH "\$ORIGIN"
  SKIP_BUILD_RPATH FALSE
  BUILD_WITH_INSTALL_RPATH TRUE
)

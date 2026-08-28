cmake_minimum_required(VERSION 3.16)
project(aero_hand_rga LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(pybind11 CONFIG REQUIRED)

set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(PKG_ACCEL_DIR ${SRC_DIR}/../rynnrcp_robot_aero_hand/accelerators)

# librknnrt 2.3.x is no longer shipped in git. Place a copy at
# <repo root>/models/rknn_runtime/librknnrt.so (or rely on the system
# /usr/lib/librknnrt.so >= 2.3.x).
set(RKNNRT_CANDIDATES
  ${SRC_DIR}/../../../models/rknn_runtime/librknnrt.so
  ${SRC_DIR}/lib/librknnrt.so
  /usr/lib/librknnrt.so
)
set(RKNNRT_LIB "")
foreach(_rknnrt_candidate ${RKNNRT_CANDIDATES})
  if(EXISTS "${_rknnrt_candidate}")
    set(RKNNRT_LIB "${_rknnrt_candidate}")
    break()
  endif()
endforeach()
if(NOT RKNNRT_LIB)
  message(FATAL_ERROR
    "librknnrt.so not found (searched: ${RKNNRT_CANDIDATES}). "
    "Place a RKNN runtime >= 2.3.x at models/rknn_runtime/librknnrt.so "
    "or install it system-wide (/usr/lib/librknnrt.so).")
endif()
# System librga (headers under /usr/include/rga).
find_library(RGA_LIB rga PATHS /usr/lib/aarch64-linux-gnu REQUIRED)

pybind11_add_module(aero_hand_rga
  aero_hand_pipeline.cpp
  bindings.cpp
)
target_include_directories(aero_hand_rga PRIVATE
  ${SRC_DIR}/include
  /usr/include/rga
)
target_link_libraries(aero_hand_rga PRIVATE ${RKNNRT_LIB} ${RGA_LIB})

# Ship the module next to the Python glue; resolve librknnrt via $ORIGIN.
set_target_properties(aero_hand_rga PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${PKG_ACCEL_DIR}
  BUILD_RPATH "\$ORIGIN"
  INSTALL_RPATH "\$ORIGIN"
  SKIP_BUILD_RPATH FALSE
  BUILD_WITH_INSTALL_RPATH TRUE
)

# Copy the bundled runtime next to the module so $ORIGIN resolves it.
add_custom_command(TARGET aero_hand_rga POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${RKNNRT_LIB} ${PKG_ACCEL_DIR}/librknnrt.so
)

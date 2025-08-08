cmake_minimum_required(VERSION 3.10)
project(RynnRCP LANGUAGES CXX)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-fPIC)
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/common
    ${CMAKE_CURRENT_SOURCE_DIR}/lcm
    ${PB_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/common/third_party/libpaho.mqtt.c-1.3.14/src
    ${CMAKE_CURRENT_SOURCE_DIR}/common/third_party/lcm-1.5.0
    ${CMAKE_CURRENT_SOURCE_DIR}/build/third_party/lcm-1.5.0/build/lcm
    ${CMAKE_CURRENT_SOURCE_DIR}/common/third_party/libwebsockets-4.0.20/include
    ${CMAKE_CURRENT_SOURCE_DIR}/build/third_party/libwebsockets-4.0.20/build
    ${CMAKE_CURRENT_SOURCE_DIR}/common/third_party/libyaml-cpp-0.7.0/include
    ${CMAKE_CURRENT_SOURCE_DIR}/build/third_party/openssl-3.2.0/include
)
link_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/build/third_party/libpaho.mqtt.c-1.3.14/build/src
    ${CMAKE_CURRENT_SOURCE_DIR}/build/third_party/lcm-1.5.0/build/lcm
    ${CMAKE_CURRENT_SOURCE_DIR}/build/third_party/libwebsockets-4.0.20/build/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/build/third_party/libyaml-cpp-0.7.0/build/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/build/third_party/libyaml-cpp-0.7.0/build
    ${CMAKE_CURRENT_SOURCE_DIR}/build/third_party/openssl-3.2.0
)
message(STATUS "Current build path: ${CMAKE_BINARY_DIR}")

set(CMAKE_INSTALL_RPATH
    "${CMAKE_CURRENT_SOURCE_DIR}/build/third_party/libpaho.mqtt.c-1.3.14/build/src;"
    "${CMAKE_CURRENT_SOURCE_DIR}/build/third_party/lcm-1.5.0/build/lcm;"
    "${CMAKE_CURRENT_SOURCE_DIR}/build/third_party/libwebsockets-4.0.20/build/lib;"
    "${CMAKE_CURRENT_SOURCE_DIR}/build/third_party/libyaml-cpp-0.7.0/build/lib;"
    "${CMAKE_CURRENT_SOURCE_DIR}/build/third_party/libyaml-cpp-0.7.0/build;"
    "${CMAKE_CURRENT_SOURCE_DIR}/build/third_party/openssl-3.2.0;"
    "${CMAKE_CURRENT_SOURCE_DIR}/build/rcp_framework/cpp/robot_server/;"
)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(Protobuf REQUIRED)
if (PROTOBUF_FOUND)
    message(STATUS "Protobuf library found")
else()
    message(FATAL_ERROR "Protobuf library is needed but can't be found")
endif()

set(PROTO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/common/proto)
set(PBCCFILE_DIR ${CMAKE_BINARY_DIR}/pbccfile)

file(MAKE_DIRECTORY ${PBCCFILE_DIR})

file(GLOB PROTO_FILES "${PROTO_DIR}/*.proto")

set(GENERATED_FILES "")
foreach(PROTO_FILE ${PROTO_FILES})
    get_filename_component(PROTO_NAME ${PROTO_FILE} NAME_WE)
    set(GENERATED_FILE "${PBCCFILE_DIR}/${PROTO_NAME}.pb.cc")
    list(APPEND GENERATED_FILES ${GENERATED_FILE})

    add_custom_command(
        OUTPUT ${GENERATED_FILE}
        COMMAND ${Protobuf_PROTOC_EXECUTABLE} --cpp_out=${PBCCFILE_DIR} -I=${PROTO_DIR} ${PROTO_FILE}
        DEPENDS ${PROTO_FILE}
        COMMENT "Generating ${GENERATED_FILE}"
    )
endforeach()

add_custom_target(protos ALL DEPENDS ${GENERATED_FILES})

add_compile_definitions(CPPHTTPLIB_OPENSSL_SUPPORT)

add_subdirectory(rcp_framework/cpp/robot_server)

add_subdirectory(rcp_framework/robots/so100/so100_servers)

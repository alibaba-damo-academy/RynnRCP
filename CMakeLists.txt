cmake_minimum_required(VERSION 3.10)
project(RynnRCP LANGUAGES CXX)

include(ExternalProject)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

if(NOT DEFINED FORCE_CXX11)
    set(CMAKE_CXX_STANDARD 17)
else()
    set(CMAKE_CXX_STANDARD 11)
    message(WARNING "Using C++11 - some features may not work with upgraded dependencies")
endif()
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-fPIC)

if(APPLE)
    add_compile_options(-stdlib=libc++)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -framework CoreFoundation -framework IOKit -framework Security")
    find_library(CORE_FOUNDATION CoreFoundation)
    find_library(IOKIT IOKit)
    find_library(SECURITY Security)
elseif(UNIX AND NOT APPLE)
    add_compile_options(-pthread)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread")
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LIBUDEV libudev)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O3 -DNDEBUG)
    if(NOT APPLE)
        add_compile_options(-march=native)
    endif()
endif()

if(APPLE)
    if(EXISTS "/opt/homebrew")
        set(HOMEBREW_PREFIX "/opt/homebrew")
    elseif(EXISTS "/usr/local")
        set(HOMEBREW_PREFIX "/usr/local")
    endif()
    
    if(HOMEBREW_PREFIX)
        include_directories(${HOMEBREW_PREFIX}/include)
        link_directories(${HOMEBREW_PREFIX}/lib)
        
        if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/build/protobuf-3.19.1/install")
            include_directories(${CMAKE_CURRENT_SOURCE_DIR}/build/protobuf-3.19.1/install/include)
            link_directories(${CMAKE_CURRENT_SOURCE_DIR}/build/protobuf-3.19.1/install/lib)
            message(STATUS "Using custom protobuf 3.19.1 at: ${CMAKE_CURRENT_SOURCE_DIR}/build/protobuf-3.19.1/install")
        endif()
        
        message(STATUS "Using Homebrew at: ${HOMEBREW_PREFIX}")
    endif()
elseif(UNIX AND NOT APPLE)
    include_directories(/usr/include /usr/local/include)
    link_directories(/usr/lib /usr/local/lib)
    
    if(EXISTS "/usr/lib/x86_64-linux-gnu")
        link_directories(/usr/lib/x86_64-linux-gnu)
    endif()
    if(EXISTS "/usr/lib64")
        link_directories(/usr/lib64)
    endif()
endif()

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

if(APPLE)
    link_directories(${HOMEBREW_PREFIX}/lib)
    
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/build/third_party/lcm-1.5.0/build/lcm")
        link_directories(${CMAKE_CURRENT_SOURCE_DIR}/build/third_party/lcm-1.5.0/build/lcm)
    endif()
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/build/third_party/libpaho.mqtt.c-1.3.14/build/src")
        link_directories(${CMAKE_CURRENT_SOURCE_DIR}/build/third_party/libpaho.mqtt.c-1.3.14/build/src)
    endif()
elseif(UNIX AND NOT APPLE)
    link_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/build/third_party/libpaho.mqtt.c-1.3.14/build/src
        ${CMAKE_CURRENT_SOURCE_DIR}/build/third_party/lcm-1.5.0/build/lcm
        ${CMAKE_CURRENT_SOURCE_DIR}/build/third_party/libwebsockets-4.0.20/build/lib
        ${CMAKE_CURRENT_SOURCE_DIR}/build/third_party/libyaml-cpp-0.7.0/build/lib
        ${CMAKE_CURRENT_SOURCE_DIR}/build/third_party/libyaml-cpp-0.7.0/build
        ${CMAKE_CURRENT_SOURCE_DIR}/build/third_party/openssl-3.2.0
    )
endif()
message(STATUS "Current build path: ${CMAKE_BINARY_DIR}")

if(APPLE)
    set(CMAKE_INSTALL_RPATH "${HOMEBREW_PREFIX}/lib")
    
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/build/third_party/lcm-1.5.0/build/lcm")
        set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${CMAKE_CURRENT_SOURCE_DIR}/build/third_party/lcm-1.5.0/build/lcm")
    endif()
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/build/third_party/libpaho.mqtt.c-1.3.14/build/src")
        set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${CMAKE_CURRENT_SOURCE_DIR}/build/third_party/libpaho.mqtt.c-1.3.14/build/src")
    endif()
    
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${CMAKE_CURRENT_SOURCE_DIR}/build/rcp_framework/cpp/robot_server")
elseif(UNIX AND NOT APPLE)
    set(CMAKE_INSTALL_RPATH
        "${CMAKE_CURRENT_SOURCE_DIR}/build/third_party/libpaho.mqtt.c-1.3.14/build/src;"
        "${CMAKE_CURRENT_SOURCE_DIR}/build/third_party/lcm-1.5.0/build/lcm;"
        "${CMAKE_CURRENT_SOURCE_DIR}/build/third_party/libwebsockets-4.0.20/build/lib;"
        "${CMAKE_CURRENT_SOURCE_DIR}/build/third_party/libyaml-cpp-0.7.0/build/lib;"
        "${CMAKE_CURRENT_SOURCE_DIR}/build/third_party/libyaml-cpp-0.7.0/build;"
        "${CMAKE_CURRENT_SOURCE_DIR}/build/third_party/openssl-3.2.0;"
        "${CMAKE_CURRENT_SOURCE_DIR}/build/rcp_framework/cpp/robot_server/;"
    )
endif()
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

set(CMAKE_PREFIX_PATH "/usr" ${CMAKE_PREFIX_PATH})
set(Protobuf_ROOT "/usr")
find_package(Protobuf REQUIRED)
if (PROTOBUF_FOUND)
    message(STATUS "Protobuf library found at: ${Protobuf_INCLUDE_DIRS}")
    message(STATUS "Protobuf version: ${Protobuf_VERSION}")
    message(STATUS "Protobuf libraries: ${Protobuf_LIBRARIES}")
    set(PROTOBUF_INCLUDE_DIRS ${Protobuf_INCLUDE_DIRS})
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

include_directories(${Protobuf_INCLUDE_DIRS})

add_compile_definitions(CPPHTTPLIB_OPENSSL_SUPPORT)

if(APPLE)
    add_compile_definitions(MACOS_BUILD)
elseif(UNIX AND NOT APPLE)
    add_compile_definitions(LINUX_BUILD)
endif()

if(CMAKE_CXX_STANDARD EQUAL 17)
    add_compile_definitions(CXX17_BUILD)
elseif(CMAKE_CXX_STANDARD EQUAL 11)
    add_compile_definitions(CXX11_BUILD)
endif()

set(PAHO_BUILD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build/third_party/libpaho.mqtt.c-1.3.14/build)

if(APPLE)
    set(SHARED_LIB_EXT "dylib")
elseif(UNIX AND NOT APPLE)
    set(SHARED_LIB_EXT "so")
else()
    set(SHARED_LIB_EXT "dll")
endif()

ExternalProject_Add(paho-mqtt
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/common/third_party/libpaho.mqtt.c-1.3.14
    BINARY_DIR ${PAHO_BUILD_DIR}
    CMAKE_ARGS
        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
        -DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}
        -DPAHO_BUILD_STATIC=TRUE
        -DPAHO_BUILD_SHARED=TRUE
        -DPAHO_HIGH_PERFORMANCE=TRUE
        -DCMAKE_C_FLAGS=-fPIC
    BUILD_COMMAND ${CMAKE_COMMAND} --build . --config ${CMAKE_BUILD_TYPE}
    INSTALL_COMMAND ""
    BUILD_BYPRODUCTS 
        ${PAHO_BUILD_DIR}/src/libpaho-mqtt3a.${SHARED_LIB_EXT}
        ${PAHO_BUILD_DIR}/src/libpaho-mqtt3c.${SHARED_LIB_EXT}
        ${PAHO_BUILD_DIR}/src/libpaho-mqtt3a.a
        ${PAHO_BUILD_DIR}/src/libpaho-mqtt3c.a
)

add_library(paho-mqtt3a SHARED IMPORTED)
add_library(paho-mqtt3c SHARED IMPORTED)
set_target_properties(paho-mqtt3a PROPERTIES 
    IMPORTED_LOCATION ${PAHO_BUILD_DIR}/src/libpaho-mqtt3a.${SHARED_LIB_EXT}
)
set_target_properties(paho-mqtt3c PROPERTIES 
    IMPORTED_LOCATION ${PAHO_BUILD_DIR}/src/libpaho-mqtt3c.${SHARED_LIB_EXT}
)
add_dependencies(paho-mqtt3a paho-mqtt)
add_dependencies(paho-mqtt3c paho-mqtt)

add_subdirectory(rcp_framework/cpp/robot_server)

add_subdirectory(rcp_framework/robots/so100/so100_servers)

add_subdirectory(rcp_framework/robots/realman/realman_servers)

add_subdirectory(rcp_framework/robots/xtrainer/xtrainer_servers)
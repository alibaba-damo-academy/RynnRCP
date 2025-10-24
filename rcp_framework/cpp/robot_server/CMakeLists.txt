cmake_minimum_required(VERSION 3.10)
project(ROBOT_SERVER)

set(LIBRARY_NAME robot_server)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Protobuf REQUIRED)

set(COMMON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../common)
set(SERVER_COMMON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/common)
set(ACTION_SERVER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/action_server)
set(DEVICE_MONITOR_DIR ${CMAKE_CURRENT_SOURCE_DIR}/device_monitor_server)
set(SENSOR_SERVER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/sensor_server)

file(GLOB SRC_FILES
    ${COMMON_DIR}/auth_manager/*.cpp
    ${COMMON_DIR}/communication/*.cpp
    ${SERVER_COMMON_DIR}
    ${SERVER_COMMON_DIR}/server/*.cpp
    ${SERVER_COMMON_DIR}/terminal_device_server/*.cpp
    ${ACTION_SERVER_DIR}/action_server.cpp
    ${DEVICE_MONITOR_DIR}/device_monitor_server.cpp
    ${SENSOR_SERVER_DIR}/sensor_server.cpp
)

set(OBJECT_FILES)

if(APPLE)
    message(STATUS "Detected macOS, collecting Homebrew and macOS SDK include paths...")

    execute_process(COMMAND brew --prefix
                    OUTPUT_VARIABLE HOMEBREW_PREFIX
                    OUTPUT_STRIP_TRAILING_WHITESPACE)
    execute_process(COMMAND xcrun --show-sdk-path
                    OUTPUT_VARIABLE MACOSX_SDK_PATH
                    OUTPUT_STRIP_TRAILING_WHITESPACE)

    set(SYS_INCLUDE_DIR
        ${HOMEBREW_PREFIX}/include
        ${MACOSX_SDK_PATH}/usr/include
        ${MACOSX_SDK_PATH}/usr/include/c++/v1
    )
endif()

foreach(SRC ${GENERATED_FILES})
    get_filename_component(OBJ ${SRC} NAME_WE)
    set(OBJ_FILE ${SRC}.o)
    add_custom_command(
        OUTPUT ${OBJ_FILE}
        COMMAND ${CMAKE_CXX_COMPILER} ${CMAKE_CXX_FLAGS} -fPIC -std=c++17 -I${PROTOBUF_INCLUDE_DIRS} -I${HOMEBREW_PREFIX}/include -I${MACOSX_SDK_PATH}/usr/include/c++/v1 -I${MACOSX_SDK_PATH}/usr/include -c ${SRC} -o ${OBJ_FILE}
        DEPENDS ${SRC}
    )
    list(APPEND OBJECT_FILES ${OBJ_FILE})
endforeach()

add_library(${LIBRARY_NAME}_static STATIC ${SRC_FILES} ${OBJECT_FILES})
add_library(${LIBRARY_NAME} SHARED ${SRC_FILES} ${OBJECT_FILES})

add_dependencies(${LIBRARY_NAME}_static protos paho-mqtt)
add_dependencies(${LIBRARY_NAME} protos paho-mqtt)

set_target_properties(${LIBRARY_NAME} PROPERTIES
    VERSION 1.0.1
    SOVERSION 1
)

target_include_directories(${LIBRARY_NAME}_static PUBLIC
    ${PBCCFILE_DIR}
    ${PROTOBUF_INCLUDE_DIRS}
    ${COMMON_DIR}
    ${ACTION_SERVER_DIR}
    ${DEVICE_MONITOR_DIR}
    ${SENSOR_SERVER_DIR}
    ${COMMON_DIR}/auth_manager
    ${COMMON_DIR}/communication
    ${COMMON_DIR}/third_party
    ${SERVER_COMMON_DIR}
    ${SERVER_COMMON_DIR}/server
    ${SERVER_COMMON_DIR}/terminal_device_server
    ${CMAKE_SOURCE_DIR}/common/lcm
)

target_include_directories(${LIBRARY_NAME} PUBLIC
    ${PBCCFILE_DIR}
    ${PROTOBUF_INCLUDE_DIRS}
    ${COMMON_DIR}
    ${ACTION_SERVER_DIR}
    ${DEVICE_MONITOR_DIR}
    ${SENSOR_SERVER_DIR}
    ${COMMON_DIR}/auth_manager
    ${COMMON_DIR}/communication
    ${COMMON_DIR}/third_party
    ${SERVER_COMMON_DIR}
    ${SERVER_COMMON_DIR}/server
    ${SERVER_COMMON_DIR}/terminal_device_server
    ${CMAKE_SOURCE_DIR}/common/lcm
)

set(DEPENDENCIES
    lcm
    ssl
    crypto
    glog
    protobuf
    yaml-cpp
    paho-mqtt3a
    paho-mqtt3c
    websockets
)

target_link_libraries(${LIBRARY_NAME}_static PUBLIC ${DEPENDENCIES})
target_link_libraries(${LIBRARY_NAME} PUBLIC ${DEPENDENCIES})

# Link against Abseil logging libraries required by protobuf (mainly on macOS)
if(APPLE)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(ABSEIL_LOG_PKG REQUIRED absl_log absl_log_internal_message absl_log_internal_check_op)
    
    target_link_libraries(${LIBRARY_NAME}_static PUBLIC ${ABSEIL_LOG_PKG_LIBRARIES})
    target_link_libraries(${LIBRARY_NAME} PUBLIC ${ABSEIL_LOG_PKG_LIBRARIES})
    target_link_directories(${LIBRARY_NAME}_static PUBLIC ${ABSEIL_LOG_PKG_LIBRARY_DIRS})
    target_link_directories(${LIBRARY_NAME} PUBLIC ${ABSEIL_LOG_PKG_LIBRARY_DIRS})
elseif(UNIX AND NOT APPLE)
    # On Linux, Abseil libraries may be required depending on protobuf version
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(ABSEIL_LOG_PKG QUIET absl_log absl_log_internal_message absl_log_internal_check_op)
        if(ABSEIL_LOG_PKG_FOUND)
            target_link_libraries(${LIBRARY_NAME}_static PUBLIC ${ABSEIL_LOG_PKG_LIBRARIES})
            target_link_libraries(${LIBRARY_NAME} PUBLIC ${ABSEIL_LOG_PKG_LIBRARIES})
            target_link_directories(${LIBRARY_NAME}_static PUBLIC ${ABSEIL_LOG_PKG_LIBRARY_DIRS})
            target_link_directories(${LIBRARY_NAME} PUBLIC ${ABSEIL_LOG_PKG_LIBRARY_DIRS})
        endif()
    endif()
endif()

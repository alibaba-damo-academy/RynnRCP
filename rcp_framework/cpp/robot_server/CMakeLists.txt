cmake_minimum_required(VERSION 3.10)
project(ROBOT_SERVER)

set(LIBRARY_NAME robot_server)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find additional required packages
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

foreach(SRC ${GENERATED_FILES})
    get_filename_component(OBJ ${SRC} NAME_WE)
    set(OBJ_FILE ${SRC}.o)
    add_custom_command(
        OUTPUT ${OBJ_FILE}
        COMMAND ${CMAKE_CXX_COMPILER} ${CMAKE_CXX_FLAGS} -fPIC -c ${SRC} -o ${OBJ_FILE}
        DEPENDS ${SRC}
    )
    list(APPEND OBJECT_FILES ${OBJ_FILE})
endforeach()

add_library(${LIBRARY_NAME}_static STATIC ${SRC_FILES} ${OBJECT_FILES})
add_library(${LIBRARY_NAME} SHARED ${SRC_FILES} ${OBJECT_FILES})

add_dependencies(${LIBRARY_NAME}_static protos)
add_dependencies(${LIBRARY_NAME} protos)

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

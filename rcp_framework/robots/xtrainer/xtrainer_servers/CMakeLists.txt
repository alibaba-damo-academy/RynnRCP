cmake_minimum_required(VERSION 3.10)

set(TARGET xtrainer_servers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Protobuf REQUIRED)

add_executable(${TARGET} xtrainer_servers.cpp)

target_link_libraries(${TARGET} robot_server ${Protobuf_LIBRARIES})

target_include_directories(${TARGET} PRIVATE ${Protobuf_INCLUDE_DIRS})

add_dependencies(${TARGET} protos)

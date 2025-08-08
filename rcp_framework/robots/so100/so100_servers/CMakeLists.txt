cmake_minimum_required(VERSION 3.10)

set(TARGET so100_servers)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(${TARGET} so100_servers.cpp)

target_link_libraries(${TARGET} robot_server)

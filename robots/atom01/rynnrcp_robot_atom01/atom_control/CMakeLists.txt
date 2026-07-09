cmake_minimum_required(VERSION 3.16)
project(atom_control VERSION 1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ===== 依赖 =====
set(CMAKE_PREFIX_PATH "/usr;/usr/local")
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED)
find_package(Threads REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 REQUIRED)

# ===== 源文件 =====
set(CORE_SRCS
    core/src/robot_interface.cpp
    core/src/motor/motor_driver.cpp
    core/src/motor/dm_motor_driver.cpp
    core/src/motor/socket_can.cpp
    core/src/imu/imu_driver.cpp
    core/src/imu/hipnuc_imu_driver.cpp
    core/src/imu/serial_port.cpp
    core/src/imu/imu_socket_can.cpp
    core/src/utils/close_chain_mapping.cpp
    core/src/utils/decouple_rpo.cpp
)

set(C_SRCS
    core/src/imu/hipnuc_dec.c
    core/src/imu/hipnuc_can_common.c
    core/src/imu/hipnuc_j1939_parser.c
)
set_source_files_properties(${C_SRCS} PROPERTIES LANGUAGE C)

# ===== 核心静态库 =====
add_library(atom_core STATIC ${CORE_SRCS} ${C_SRCS})

target_include_directories(atom_core PUBLIC
    core/include
    core/include/imu          # HiPNUC C 文件用裸文件名 include
    core/include/motor
    core/include/utils
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

target_link_libraries(atom_core PUBLIC
    ${EIGEN3_LIBRARIES}
    spdlog::spdlog
    fmt::fmt
    yaml-cpp
    ${Boost_LIBRARIES}
    Threads::Threads
)

# ===== Python 绑定 (atom01_py) =====
pybind11_add_module(atom01_py bindings/pybind_module.cpp)
target_link_libraries(atom01_py PRIVATE atom_core)
target_include_directories(atom01_py PRIVATE core/include)

execute_process(
    COMMAND ${Python3_EXECUTABLE} -c "import site; print(site.getsitepackages()[0])"
    OUTPUT_VARIABLE PYTHON_SITE
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
install(TARGETS atom01_py LIBRARY DESTINATION ${PYTHON_SITE})

# 打印信息
message(STATUS "=========================================")
message(STATUS "  atom_control build configuration:")
message(STATUS "  C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Python: ${Python3_VERSION}")
message(STATUS "  atom01_py → ${PYTHON_SITE}")
message(STATUS "=========================================")

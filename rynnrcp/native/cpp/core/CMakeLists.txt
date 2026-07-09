# Core layer C++ implementation
add_library(rynnrcp_core STATIC
    ring_buffer.cc
    transport.cc
    channel_manager.cc
    tf.cc
)
set_target_properties(rynnrcp_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(rynnrcp_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(rynnrcp_core PUBLIC rynnrcp_native)

# Optional: pybind11 Python extension module
if(RYNNRCP_BUILD_PYTHON_BINDINGS)
    set(PYBIND11_FINDPYTHON ON)
    find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
    find_package(pybind11 CONFIG REQUIRED)
    pybind11_add_module(rynnrcp_core_native bindings.cc)
    target_link_libraries(rynnrcp_core_native PRIVATE rynnrcp_core)
    install(TARGETS rynnrcp_core_native LIBRARY DESTINATION . RUNTIME DESTINATION .)
endif()

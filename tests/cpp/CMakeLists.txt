# Tests for native layer
# Uses a minimal test harness (no external dependencies)

macro(rynnrcp_add_test test_name)
    add_executable(${test_name} ${test_name}.cc)
    target_link_libraries(${test_name} PRIVATE rynnrcp_native)
    add_test(NAME ${test_name} COMMAND ${test_name})
endmacro()

rynnrcp_add_test(test_atomic_ops)
rynnrcp_add_test(test_config)
rynnrcp_add_test(test_shm)
rynnrcp_add_test(test_event)
rynnrcp_add_test(test_logging_sys)

# Tests for core layer
macro(rynnrcp_add_core_test test_name)
    add_executable(${test_name} ${test_name}.cc)
    target_link_libraries(${test_name} PRIVATE rynnrcp_core)
    add_test(NAME ${test_name} COMMAND ${test_name})
endmacro()

rynnrcp_add_core_test(test_ring_buffer)
rynnrcp_add_core_test(test_transport)
rynnrcp_add_core_test(test_channel_manager)
rynnrcp_add_core_test(test_tf)

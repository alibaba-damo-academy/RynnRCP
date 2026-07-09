# Native primitives - header-only + compiled sources
add_library(rynnrcp_native STATIC)
set_target_properties(rynnrcp_native PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_sources(rynnrcp_native PRIVATE
    shm.cc
    event.cc
    log.cc
)

target_include_directories(rynnrcp_native PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link OS-specific libraries
if(WIN32)
    # No extra libs needed on Windows (kernel32 is implicit)
elseif(APPLE)
    target_link_libraries(rynnrcp_native PUBLIC pthread)
elseif(UNIX)
    target_link_libraries(rynnrcp_native PUBLIC pthread rt)
endif()

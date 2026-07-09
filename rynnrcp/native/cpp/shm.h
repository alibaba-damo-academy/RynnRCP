// Copyright 2026 RynnRCP Authors. All rights reserved.
// Native primitives: shared memory operations (cross-platform).
// Linux: POSIX shm_open / mmap
// Windows: CreateFileMapping / MapViewOfFile

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace rynnrcp {
namespace native {

/// Result of a shared memory operation
struct ShmRegion {
    void*  ptr  = nullptr;   ///< Mapped address
    size_t size = 0;         ///< Mapped size in bytes
#ifdef _WIN32
    void*  handle = nullptr; ///< Windows HANDLE (cast from HANDLE)
#else
    int    fd   = -1;        ///< POSIX file descriptor
#endif

    /// Check if the region is valid
    bool IsValid() const noexcept { return ptr != nullptr; }
};

/// Create a NEW shared memory region. Fails if it already exists.
/// @param name  Unique name (e.g. "/rynnrcp_channel_imu"). On Windows, leading '/' is stripped.
/// @param size  Size in bytes (will be page-aligned internally).
/// @return ShmRegion with mapped pointer, or invalid region on failure.
ShmRegion ShmCreate(const std::string& name, size_t size);

/// Open an EXISTING shared memory region.
/// @param name  Same name used in ShmCreate.
/// @param size  Expected size (must match or be <= actual size).
/// @return ShmRegion with mapped pointer, or invalid region on failure.
ShmRegion ShmOpen(const std::string& name, size_t size);

/// Unmap a shared memory region from this process.
/// Does NOT remove the shared memory object from the system.
void ShmClose(ShmRegion& region);

/// Remove the shared memory object from the system.
/// After this call, no new processes can open it.
/// Existing mappings remain valid until ShmClose.
void ShmUnlink(const std::string& name);

}  // namespace native
}  // namespace rynnrcp

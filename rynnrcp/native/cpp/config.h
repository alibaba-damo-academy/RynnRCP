// Copyright 2026 RynnRCP Authors. All rights reserved.
// Native primitives: compile-time and runtime platform constants.

#pragma once

#include <cstddef>
#include <cstdint>

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

namespace rynnrcp {
namespace native {

// ---------------------------------------------------------------------------
// Compile-time constants
// ---------------------------------------------------------------------------

/// Cache line size (bytes). Conservative default for x86-64 / ARM Cortex-A.
/// ARM Cortex-A75+ may use 128-byte lines; override via compile flag if needed.
#if defined(__aarch64__) && defined(RYNNRCP_CACHE_LINE_128)
constexpr size_t kCacheLineSize = 128;
#else
constexpr size_t kCacheLineSize = 64;
#endif

/// Maximum length for a channel name (including null terminator)
constexpr size_t kMaxChannelNameLen = 64;

/// Default SHM ring buffer slot count (must be power of 2)
constexpr size_t kDefaultRingSlots = 16;

// ---------------------------------------------------------------------------
// Alignment helpers (compile-time, constexpr)
// ---------------------------------------------------------------------------

/// Align `size` up to the nearest multiple of `alignment`.
/// `alignment` must be a power of 2.
constexpr size_t AlignUp(size_t size, size_t alignment) noexcept {
    return (size + alignment - 1) & ~(alignment - 1);
}

/// Align `size` up to cache line boundary
constexpr size_t AlignToCacheLine(size_t size) noexcept {
    return AlignUp(size, kCacheLineSize);
}

// ---------------------------------------------------------------------------
// Runtime platform queries
// ---------------------------------------------------------------------------

/// Get the system page size in bytes
inline size_t GetPageSize() noexcept {
#ifdef _WIN32
    SYSTEM_INFO si;
    GetSystemInfo(&si);
    return static_cast<size_t>(si.dwPageSize);
#else
    return static_cast<size_t>(sysconf(_SC_PAGESIZE));
#endif
}

/// Align `size` up to the system page size
inline size_t AlignToPage(size_t size) noexcept {
    return AlignUp(size, GetPageSize());
}

}  // namespace native
}  // namespace rynnrcp

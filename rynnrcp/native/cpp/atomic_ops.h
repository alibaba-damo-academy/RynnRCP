// Copyright 2026 RynnRCP Authors. All rights reserved.
// Native primitives: atomic operations with explicit memory ordering.
// Pure C++ standard atomics - works on all platforms (Windows/Linux/macOS).

#pragma once

#include <atomic>
#include <cstdint>

namespace rynnrcp {
namespace native {

// ---------------------------------------------------------------------------
// 64-bit atomic operations
// ---------------------------------------------------------------------------

/// Store with release semantics (ensures prior writes are visible to acquire loads)
inline void store_release(std::atomic<uint64_t>& target, uint64_t value) noexcept {
    target.store(value, std::memory_order_release);
}

/// Load with acquire semantics (ensures subsequent reads see prior release stores)
inline uint64_t load_acquire(const std::atomic<uint64_t>& target) noexcept {
    return target.load(std::memory_order_acquire);
}

/// Atomic fetch-add with release semantics
inline uint64_t fetch_add_release(std::atomic<uint64_t>& target, uint64_t delta) noexcept {
    return target.fetch_add(delta, std::memory_order_release);
}

/// Atomic fetch-sub with release semantics
inline uint64_t fetch_sub_release(std::atomic<uint64_t>& target, uint64_t delta) noexcept {
    return target.fetch_sub(delta, std::memory_order_release);
}

/// Atomic compare-exchange (strong) with acq_rel on success, acquire on failure
inline bool compare_exchange_strong(std::atomic<uint64_t>& target,
                                     uint64_t& expected,
                                     uint64_t desired) noexcept {
    return target.compare_exchange_strong(expected, desired,
                                          std::memory_order_acq_rel,
                                          std::memory_order_acquire);
}

// ---------------------------------------------------------------------------
// 32-bit atomic operations
// ---------------------------------------------------------------------------

inline void store_release_u32(std::atomic<uint32_t>& target, uint32_t value) noexcept {
    target.store(value, std::memory_order_release);
}

inline uint32_t load_acquire_u32(const std::atomic<uint32_t>& target) noexcept {
    return target.load(std::memory_order_acquire);
}

inline uint32_t fetch_add_release_u32(std::atomic<uint32_t>& target, uint32_t delta) noexcept {
    return target.fetch_add(delta, std::memory_order_release);
}

inline uint32_t fetch_sub_release_u32(std::atomic<uint32_t>& target, uint32_t delta) noexcept {
    return target.fetch_sub(delta, std::memory_order_release);
}

// ---------------------------------------------------------------------------
// Fence operations
// ---------------------------------------------------------------------------

/// Full memory fence (sequentially consistent)
inline void memory_fence() noexcept {
    std::atomic_thread_fence(std::memory_order_seq_cst);
}

/// Acquire fence
inline void acquire_fence() noexcept {
    std::atomic_thread_fence(std::memory_order_acquire);
}

/// Release fence
inline void release_fence() noexcept {
    std::atomic_thread_fence(std::memory_order_release);
}

}  // namespace native
}  // namespace rynnrcp

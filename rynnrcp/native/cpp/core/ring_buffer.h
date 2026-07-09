// Copyright 2026 RynnRCP Authors. All rights reserved.
// Core layer: SHM-backed Ring Buffer (SPMC - single producer, multiple consumer).
// Memory layout is byte-compatible with the Python implementation.

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "shm.h"
#include "config.h"
#include "atomic_ops.h"

namespace rynnrcp {
namespace core {

/// SHM-backed ring buffer for zero-copy inter-process communication.
///
/// SPMC model: one writer, multiple readers.
/// The writer advances write_index monotonically. Readers track their own
/// read position and use `index & (slot_count - 1)` to find the physical slot.
///
/// Memory layout (compatible with Python RingBuffer):
///   [0..63]   Header: write_index(u64) + slot_size(u64) + slot_count(u64) + flags(u64) + padding
///   [64..]    Slots: slot_count * slot_size bytes
class RingBuffer {
public:
    /// Create a new ring buffer backed by fresh SHM.
    /// @param name       Unique SHM identifier.
    /// @param slot_size  Size of each slot in bytes.
    /// @param slot_count Number of slots (must be power of 2). Default 16.
    /// @return Valid RingBuffer or nullptr on failure.
    static std::unique_ptr<RingBuffer> Create(
        const std::string& name, size_t slot_size, size_t slot_count = 16);

    /// Open an existing ring buffer by name.
    /// @param name       SHM name to open.
    /// @param slot_size  Expected slot size.
    /// @param slot_count Expected slot count.
    /// @return Valid RingBuffer or nullptr on failure.
    static std::unique_ptr<RingBuffer> Open(
        const std::string& name, size_t slot_size, size_t slot_count = 16);

    ~RingBuffer();

    // Non-copyable, movable
    RingBuffer(const RingBuffer&) = delete;
    RingBuffer& operator=(const RingBuffer&) = delete;
    RingBuffer(RingBuffer&& other) noexcept;
    RingBuffer& operator=(RingBuffer&& other) noexcept;

    /// Write data to the next slot in the ring buffer.
    /// @param data Pointer to data to write.
    /// @param len  Length of data. Must be <= slot_size.
    /// @return Logical write index, or UINT64_MAX on error.
    uint64_t Write(const void* data, size_t len);

    /// Write a transport envelope directly from payload chunks.
    /// The 16-byte Python SHM transport header is built after the write index
    /// is reserved, avoiding a Python-side callback while keeping the protocol
    /// byte-compatible with existing readers.
    uint64_t WriteEnvelopeParts(size_t payload_size, const std::vector<std::string>& parts);
    uint64_t WriteEnvelopeRawParts(
        size_t payload_size,
        const std::vector<std::pair<const uint8_t*, size_t>>& parts);

    /// Read data from a specific logical index.
    /// @param index   Logical index to read.
    /// @param out     Output buffer (must be at least slot_size bytes).
    /// @param out_len [out] Actual slot size read.
    /// @return true on success, false if index is invalid (overwritten or not yet written).
    bool Read(uint64_t index, void* out, size_t* out_len) const;

    /// Try to read, returning false silently on any error.
    bool TryRead(uint64_t index, void* out, size_t* out_len) const;

    /// Read a bounded byte range from a specific logical slot.
    bool ReadAt(uint64_t index, size_t offset, size_t length, void* out, size_t* out_len) const;

    /// Get the latest write index (last written = LatestIndex(), -1 equivalent = UINT64_MAX if empty).
    uint64_t LatestIndex() const;

    /// Get the oldest logical index still available, or UINT64_MAX if empty.
    uint64_t OldestIndex() const;

    /// Get total write count.
    uint64_t WriteCount() const;

    /// Close the mapping (unmap SHM). Does not remove SHM.
    void Close();

    /// Remove the SHM from the system.
    void Unlink();

    /// Check if the buffer is valid and open.
    bool IsValid() const noexcept { return !closed_ && region_.IsValid(); }

    const std::string& name() const noexcept { return name_; }
    size_t slot_size() const noexcept { return slot_size_; }
    size_t slot_count() const noexcept { return slot_count_; }

private:
    RingBuffer() = default;

    // Header layout
    static constexpr size_t kHeaderSize = native::kCacheLineSize;  // 64 bytes
    static constexpr uint64_t kMagicFlags = 0x524F424F52494E47ULL;   // "ROBORING"

    struct Header {
        std::atomic<uint64_t> write_index;
        uint64_t slot_size;
        uint64_t slot_count;
        uint64_t flags;
        // padding to kHeaderSize
    };

    Header* header() const {
        return reinterpret_cast<Header*>(region_.ptr);
    }

    uint8_t* data_ptr() const {
        return static_cast<uint8_t*>(region_.ptr) + kHeaderSize;
    }

    uint8_t* slot_ptr(uint64_t index) const {
        size_t physical = static_cast<size_t>(index & slot_mask_);
        return data_ptr() + physical * slot_size_;
    }

    bool ValidateReadIndex(uint64_t index) const;

    std::string name_;
    size_t slot_size_ = 0;
    size_t slot_count_ = 0;
    size_t slot_mask_ = 0;
    native::ShmRegion region_{};
    mutable std::mutex write_mutex_;
    bool closed_ = true;
};

}  // namespace core
}  // namespace rynnrcp

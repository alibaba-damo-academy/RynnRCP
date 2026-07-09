// Copyright 2026 RynnRCP Authors. All rights reserved.
// Core layer: SHM-backed Ring Buffer implementation.

#include "ring_buffer.h"
#include "core_log_defs.h"

#include <cstring>

namespace rynnrcp {
namespace core {

// --- Static factory methods ---

std::unique_ptr<RingBuffer> RingBuffer::Create(
    const std::string& name, size_t slot_size, size_t slot_count) {
    // Validate
    if (slot_count == 0 || (slot_count & (slot_count - 1)) != 0) {
        ROBO_LOG_ERROR("slot_count must be power of 2, got %zu", slot_count);
        return nullptr;
    }
    if (slot_size == 0) {
        ROBO_LOG_ERROR("slot_size must be > 0");
        return nullptr;
    }

    size_t total_size = kHeaderSize + slot_size * slot_count;

    // Clean up stale SHM
    native::ShmUnlink(name);

    native::ShmRegion region = native::ShmCreate(name, total_size);
    if (!region.IsValid()) {
        ROBO_LOG_ERROR("Failed to create SHM '%s'", name.c_str());
        return nullptr;
    }

    // Initialize header
    auto rb = std::unique_ptr<RingBuffer>(new RingBuffer());
    rb->name_ = name;
    rb->slot_size_ = slot_size;
    rb->slot_count_ = slot_count;
    rb->slot_mask_ = slot_count - 1;
    rb->region_ = region;
    rb->closed_ = false;

    // Write header fields
    Header* hdr = rb->header();
    hdr->write_index.store(0, std::memory_order_relaxed);
    hdr->slot_size = static_cast<uint64_t>(slot_size);
    hdr->slot_count = static_cast<uint64_t>(slot_count);
    hdr->flags = kMagicFlags;

    // Zero-fill remaining header padding
    size_t header_used = sizeof(Header);
    if (header_used < kHeaderSize) {
        memset(reinterpret_cast<uint8_t*>(hdr) + header_used, 0, kHeaderSize - header_used);
    }

    return rb;
}

std::unique_ptr<RingBuffer> RingBuffer::Open(
    const std::string& name, size_t slot_size, size_t slot_count) {
    if (slot_count == 0 || (slot_count & (slot_count - 1)) != 0) {
        ROBO_LOG_ERROR("slot_count must be power of 2, got %zu", slot_count);
        return nullptr;
    }
    if (slot_size == 0) {
        ROBO_LOG_ERROR("slot_size must be > 0");
        return nullptr;
    }

    size_t total_size = kHeaderSize + slot_size * slot_count;

    native::ShmRegion region = native::ShmOpen(name, total_size);
    if (!region.IsValid()) {
        ROBO_LOG_ERROR("Failed to open SHM '%s'", name.c_str());
        return nullptr;
    }

    // Verify header
    auto* hdr = reinterpret_cast<Header*>(region.ptr);
    if (hdr->flags != kMagicFlags) {
        ROBO_LOG_ERROR("SHM '%s' bad magic", name.c_str());
        native::ShmClose(region);
        return nullptr;
    }
    if (hdr->slot_size != static_cast<uint64_t>(slot_size) ||
        hdr->slot_count != static_cast<uint64_t>(slot_count)) {
        ROBO_LOG_ERROR("SHM '%s' config mismatch: expected %zu/%zu, got %llu/%llu",
                name.c_str(), slot_size, slot_count,
                (unsigned long long)hdr->slot_size,
                (unsigned long long)hdr->slot_count);
        native::ShmClose(region);
        return nullptr;
    }

    auto rb = std::unique_ptr<RingBuffer>(new RingBuffer());
    rb->name_ = name;
    rb->slot_size_ = slot_size;
    rb->slot_count_ = slot_count;
    rb->slot_mask_ = slot_count - 1;
    rb->region_ = region;
    rb->closed_ = false;

    return rb;
}

// --- Destructor and move ---

RingBuffer::~RingBuffer() {
    if (!closed_) {
        Close();
    }
}

RingBuffer::RingBuffer(RingBuffer&& other) noexcept
    : name_(std::move(other.name_)),
      slot_size_(other.slot_size_),
      slot_count_(other.slot_count_),
      slot_mask_(other.slot_mask_),
      region_(other.region_),
      closed_(other.closed_) {
    other.region_ = {};
    other.closed_ = true;
}

RingBuffer& RingBuffer::operator=(RingBuffer&& other) noexcept {
    if (this != &other) {
        if (!closed_) Close();
        name_ = std::move(other.name_);
        slot_size_ = other.slot_size_;
        slot_count_ = other.slot_count_;
        slot_mask_ = other.slot_mask_;
        region_ = other.region_;
        closed_ = other.closed_;
        other.region_ = {};
        other.closed_ = true;
    }
    return *this;
}

// --- Write ---

uint64_t RingBuffer::Write(const void* data, size_t len) {
    if (closed_ || !region_.IsValid()) return UINT64_MAX;
    if (len > slot_size_) return UINT64_MAX;

    std::lock_guard<std::mutex> guard(write_mutex_);
    Header* hdr = header();

    // Read current write index
    uint64_t idx = native::load_acquire(hdr->write_index);

    // Write data to slot
    uint8_t* slot = slot_ptr(idx);
    memcpy(slot, data, len);

    // Zero-pad remainder
    if (len < slot_size_) {
        memset(slot + len, 0, slot_size_ - len);
    }

    // Advance write index with release semantics
    native::store_release(hdr->write_index, idx + 1);

    return idx;
}

uint64_t RingBuffer::WriteEnvelopeParts(size_t payload_size, const std::vector<std::string>& parts) {
    std::vector<std::pair<const uint8_t*, size_t>> raw_parts;
    raw_parts.reserve(parts.size());
    for (const auto& part : parts) {
        raw_parts.emplace_back(reinterpret_cast<const uint8_t*>(part.data()), part.size());
    }
    return WriteEnvelopeRawParts(payload_size, raw_parts);
}

uint64_t RingBuffer::WriteEnvelopeRawParts(
    size_t payload_size,
    const std::vector<std::pair<const uint8_t*, size_t>>& parts) {
    static constexpr uint8_t kMagic[4] = {'R', 'C', 'T', '1'};
    static constexpr size_t kEnvelopeHeaderSize = 16;

    if (closed_ || !region_.IsValid()) return UINT64_MAX;
    if (payload_size > UINT32_MAX) return UINT64_MAX;
    if (payload_size + kEnvelopeHeaderSize > slot_size_) return UINT64_MAX;

    size_t checked_size = 0;
    for (const auto& part : parts) {
        checked_size += part.second;
        if (checked_size > payload_size) return UINT64_MAX;
    }
    if (checked_size != payload_size) return UINT64_MAX;

    std::lock_guard<std::mutex> guard(write_mutex_);
    Header* hdr = header();
    uint64_t idx = native::load_acquire(hdr->write_index);

    uint8_t* slot = slot_ptr(idx);
    memcpy(slot, kMagic, sizeof(kMagic));

    const uint32_t size32 = static_cast<uint32_t>(payload_size);
    memcpy(slot + 4, &size32, sizeof(size32));
    memcpy(slot + 8, &idx, sizeof(idx));

    uint8_t* cursor = slot + kEnvelopeHeaderSize;
    for (const auto& part : parts) {
        if (part.second > 0) {
            memcpy(cursor, part.first, part.second);
            cursor += part.second;
        }
    }

    native::store_release(hdr->write_index, idx + 1);
    return idx;
}

// --- Read ---

bool RingBuffer::Read(uint64_t index, void* out, size_t* out_len) const {
    if (closed_ || !region_.IsValid()) return false;

    if (!ValidateReadIndex(index)) return false;

    const uint8_t* slot = slot_ptr(index);
    memcpy(out, slot, slot_size_);
    if (out_len) *out_len = slot_size_;

    if (!ValidateReadIndex(index)) return false;
    return true;
}

bool RingBuffer::TryRead(uint64_t index, void* out, size_t* out_len) const {
    return Read(index, out, out_len);
}

bool RingBuffer::ReadAt(uint64_t index, size_t offset, size_t length, void* out, size_t* out_len) const {
    if (closed_ || !region_.IsValid()) return false;
    if (offset > slot_size_ || length > slot_size_ || offset + length > slot_size_) return false;
    if (!ValidateReadIndex(index)) return false;

    const uint8_t* slot = slot_ptr(index);
    memcpy(out, slot + offset, length);
    if (out_len) *out_len = length;

    if (!ValidateReadIndex(index)) return false;
    return true;
}

// --- Queries ---

uint64_t RingBuffer::LatestIndex() const {
    if (closed_ || !region_.IsValid()) return UINT64_MAX;
    uint64_t wi = native::load_acquire(header()->write_index);
    return wi > 0 ? wi - 1 : UINT64_MAX;
}

uint64_t RingBuffer::OldestIndex() const {
    if (closed_ || !region_.IsValid()) return UINT64_MAX;
    uint64_t wi = native::load_acquire(header()->write_index);
    if (wi == 0) return UINT64_MAX;
    return wi > slot_count_ ? wi - slot_count_ : 0;
}

uint64_t RingBuffer::WriteCount() const {
    if (closed_ || !region_.IsValid()) return 0;
    return native::load_acquire(header()->write_index);
}

bool RingBuffer::ValidateReadIndex(uint64_t index) const {
    if (closed_ || !region_.IsValid()) return false;

    Header* hdr = header();
    uint64_t current_write = native::load_acquire(hdr->write_index);
    if (index >= current_write) return false;
    if (current_write - index > slot_count_) return false;
    return true;
}

// --- Lifecycle ---

void RingBuffer::Close() {
    if (!closed_) {
        closed_ = true;
        native::ShmClose(region_);
    }
}

void RingBuffer::Unlink() {
    native::ShmUnlink(name_);
}

}  // namespace core
}  // namespace rynnrcp

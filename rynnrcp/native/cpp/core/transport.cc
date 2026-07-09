// Copyright 2026 RynnRCP Authors. All rights reserved.
// Core layer: Transport implementations.

#include "transport.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <thread>

namespace rynnrcp {
namespace core {

// ===========================================================================
// IntraProcessTransport
// ===========================================================================

IntraProcessTransport::IntraProcessTransport(const std::string& name,
                                             size_t msg_size,
                                             size_t buffer_size)
    : name_(name), msg_size_(msg_size) {
    // Round up to power of 2
    size_t actual = 1;
    while (actual < buffer_size) actual <<= 1;
    buffer_.resize(actual);
    for (auto& slot : buffer_) {
        slot.data.resize(msg_size, 0);
    }
    buffer_mask_ = actual - 1;
    event_ = native::EventCreate();
}

IntraProcessTransport::~IntraProcessTransport() {
    Close();
}

void IntraProcessTransport::Publish(const void* data, size_t len) {
    if (closed_.load()) return;
    if (len > msg_size_) len = msg_size_;

    // Write to circular buffer
    uint64_t idx = write_idx_.load(std::memory_order_relaxed);
    Slot& slot = buffer_[idx & buffer_mask_];
    memcpy(slot.data.data(), data, len);
    if (len < msg_size_) memset(slot.data.data() + len, 0, msg_size_ - len);
    slot.len = len;
    write_idx_.store(idx + 1, std::memory_order_release);

    // Signal event for pollers
    native::EventSignal(event_);

    // Invoke callbacks
    std::lock_guard<std::mutex> lock(cb_mutex_);
    for (auto& cb : callbacks_) {
        cb(data, len);
    }
}

void IntraProcessTransport::Subscribe(SubscriberCallback cb) {
    std::lock_guard<std::mutex> lock(cb_mutex_);
    callbacks_.push_back(std::move(cb));
}

bool IntraProcessTransport::Poll(void* out, size_t* out_len, int timeout_ms) {
    if (closed_.load()) return false;

    // Check if data available
    uint64_t current_write = write_idx_.load(std::memory_order_acquire);
    if (read_idx_ < current_write) {
        Slot& slot = buffer_[read_idx_ & buffer_mask_];
        size_t copy_len = std::min(slot.len, *out_len);
        memcpy(out, slot.data.data(), copy_len);
        *out_len = copy_len;
        read_idx_++;
        return true;
    }

    // Wait for event
    auto result = native::EventWait(event_, timeout_ms);
    if (result == native::WaitResult::kSignaled) {
        current_write = write_idx_.load(std::memory_order_acquire);
        if (read_idx_ < current_write) {
            Slot& slot = buffer_[read_idx_ & buffer_mask_];
            size_t copy_len = std::min(slot.len, *out_len);
            memcpy(out, slot.data.data(), copy_len);
            *out_len = copy_len;
            read_idx_++;
            return true;
        }
    }
    return false;
}

bool IntraProcessTransport::ReadLatest(void* out, size_t* out_len) {
    if (closed_.load()) return false;

    uint64_t current_write = write_idx_.load(std::memory_order_acquire);
    if (current_write == 0) return false;

    uint64_t latest = current_write - 1;
    Slot& slot = buffer_[latest & buffer_mask_];
    size_t copy_len = std::min(slot.len, *out_len);
    memcpy(out, slot.data.data(), copy_len);
    *out_len = copy_len;
    read_idx_ = current_write;  // Skip to latest
    return true;
}

void IntraProcessTransport::Close() {
    if (!closed_.exchange(true)) {
        native::EventClose(event_);
        std::lock_guard<std::mutex> lock(cb_mutex_);
        callbacks_.clear();
    }
}

// ===========================================================================
// ShmTransport
// ===========================================================================

ShmTransport::ShmTransport(const std::string& name, size_t msg_size,
                           size_t slot_count, bool create)
    : name_(name), msg_size_(msg_size) {
    std::string shm_name = "rc_transport_" + name;
    if (create) {
        ring_ = RingBuffer::Create(shm_name, msg_size, slot_count);
    } else {
        ring_ = RingBuffer::Open(shm_name, msg_size, slot_count);
    }
    event_ = native::EventCreate();
}

ShmTransport::~ShmTransport() {
    Close();
}

void ShmTransport::Publish(const void* data, size_t len) {
    if (closed_.load() || !ring_ || !ring_->IsValid()) return;
    if (len > msg_size_) len = msg_size_;

    ring_->Write(data, len);
    native::EventSignal(event_);

    // Local callbacks
    std::lock_guard<std::mutex> lock(cb_mutex_);
    for (auto& cb : callbacks_) {
        cb(data, len);
    }
}

void ShmTransport::Subscribe(SubscriberCallback cb) {
    std::lock_guard<std::mutex> lock(cb_mutex_);
    callbacks_.push_back(std::move(cb));
}

bool ShmTransport::Poll(void* out, size_t* out_len, int timeout_ms) {
    if (closed_.load() || !ring_) return false;

    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(timeout_ms);

    while (true) {
        // Check ring buffer
        int64_t next_idx = last_read_index_ + 1;
        uint64_t latest = ring_->LatestIndex();
        if (latest != UINT64_MAX && (int64_t)latest >= next_idx) {
            size_t len = msg_size_;
            if (ring_->Read(static_cast<uint64_t>(next_idx), out, &len)) {
                last_read_index_ = next_idx;
                if (out_len) *out_len = len;
                return true;
            }
        }

        // Non-blocking event check
        if (native::EventWait(event_, 0) == native::WaitResult::kSignaled) {
            continue;
        }

        // Timeout check
        if (std::chrono::steady_clock::now() >= deadline) {
            return false;
        }

        // Short sleep for cross-process polling
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}

bool ShmTransport::ReadLatest(void* out, size_t* out_len) {
    if (closed_.load() || !ring_) return false;

    uint64_t latest = ring_->LatestIndex();
    if (latest == UINT64_MAX) return false;

    size_t len = msg_size_;
    if (ring_->Read(latest, out, &len)) {
        last_read_index_ = static_cast<int64_t>(latest);
        if (out_len) *out_len = len;
        return true;
    }
    return false;
}

void ShmTransport::Close() {
    if (!closed_.exchange(true)) {
        native::EventClose(event_);
        if (ring_) ring_->Close();
        std::lock_guard<std::mutex> lock(cb_mutex_);
        callbacks_.clear();
    }
}

void ShmTransport::Unlink() {
    if (ring_) ring_->Unlink();
}

}  // namespace core
}  // namespace rynnrcp

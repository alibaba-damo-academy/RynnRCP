// Copyright 2026 RynnRCP Authors. All rights reserved.
// Core layer: Transport abstraction with multiple levels.
// L0+L1 (IntraProcess): lock-free in-process message passing.
// L2 (SHM): cross-process via SHM Ring Buffer.

#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "ring_buffer.h"
#include "event.h"

namespace rynnrcp {
namespace core {

/// Transport routing levels (compatible with Python TransportLevel enum values).
enum class TransportLevel : int {
    kIntraThread = 0,
    kIntraProcess = 1,
    kShm = 2
};

/// Callback type for subscriber notifications.
using SubscriberCallback = std::function<void(const void* data, size_t len)>;

/// Abstract base class for all transport implementations.
class TransportBase {
public:
    virtual ~TransportBase() = default;

    /// Publish data to all subscribers.
    virtual void Publish(const void* data, size_t len) = 0;

    /// Register a subscriber callback.
    virtual void Subscribe(SubscriberCallback cb) = 0;

    /// Poll for the next message (blocking with timeout).
    /// @param out        Output buffer.
    /// @param out_len    [in/out] On input: buffer capacity. On output: bytes written.
    /// @param timeout_ms Timeout in ms. 0 = non-blocking, -1 = infinite.
    /// @return true if a message was received.
    virtual bool Poll(void* out, size_t* out_len, int timeout_ms = 100) = 0;

    /// Read the latest message without blocking.
    /// @return true if a message was available.
    virtual bool ReadLatest(void* out, size_t* out_len) = 0;

    /// Get the transport level.
    virtual TransportLevel Level() const = 0;

    /// Close and release resources.
    virtual void Close() = 0;

    /// Check if transport is open.
    virtual bool IsOpen() const = 0;
};

/// L0+L1 transport: in-process message passing.
/// Uses a lock-free circular buffer and Event notification.
class IntraProcessTransport : public TransportBase {
public:
    /// @param name        Channel name (for logging).
    /// @param buffer_size Max messages buffered (must be power of 2).
    /// @param msg_size    Max message size in bytes.
    explicit IntraProcessTransport(const std::string& name,
                                    size_t msg_size,
                                    size_t buffer_size = 64);
    ~IntraProcessTransport() override;

    void Publish(const void* data, size_t len) override;
    void Subscribe(SubscriberCallback cb) override;
    bool Poll(void* out, size_t* out_len, int timeout_ms = 100) override;
    bool ReadLatest(void* out, size_t* out_len) override;
    TransportLevel Level() const override { return TransportLevel::kIntraProcess; }
    void Close() override;
    bool IsOpen() const override { return !closed_.load(); }

private:
    struct Slot {
        std::vector<uint8_t> data;
        size_t len = 0;
    };

    std::string name_;
    size_t msg_size_;
    std::vector<Slot> buffer_;
    size_t buffer_mask_;

    std::atomic<uint64_t> write_idx_{0};
    uint64_t read_idx_ = 0;

    std::mutex cb_mutex_;
    std::vector<SubscriberCallback> callbacks_;

    native::EventHandle event_;
    std::atomic<bool> closed_{false};
};

/// L2 transport: cross-process communication via SHM Ring Buffer.
class ShmTransport : public TransportBase {
public:
    /// @param name       Channel name.
    /// @param msg_size   Max message size.
    /// @param slot_count Ring buffer slots (power of 2).
    /// @param create     true = create SHM, false = open existing.
    ShmTransport(const std::string& name, size_t msg_size,
                 size_t slot_count = 16, bool create = true);
    ~ShmTransport() override;

    void Publish(const void* data, size_t len) override;
    void Subscribe(SubscriberCallback cb) override;
    bool Poll(void* out, size_t* out_len, int timeout_ms = 100) override;
    bool ReadLatest(void* out, size_t* out_len) override;
    TransportLevel Level() const override { return TransportLevel::kShm; }
    void Close() override;
    void Unlink();
    bool IsOpen() const override { return !closed_.load(); }

private:
    std::string name_;
    size_t msg_size_;
    std::unique_ptr<RingBuffer> ring_;
    native::EventHandle event_;

    std::mutex cb_mutex_;
    std::vector<SubscriberCallback> callbacks_;

    int64_t last_read_index_ = -1;
    std::atomic<bool> closed_{false};
};

}  // namespace core
}  // namespace rynnrcp

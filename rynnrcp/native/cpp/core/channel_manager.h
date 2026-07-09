// Copyright 2026 RynnRCP Authors. All rights reserved.
// Core layer: ChannelManager - singleton pub/sub channel registry.

#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "transport.h"

namespace rynnrcp {
namespace core {

/// Channel entry in the registry.
struct ChannelEntry {
    std::string name;
    size_t msg_size;
    TransportLevel level;
    std::shared_ptr<TransportBase> transport;
};

/// Publisher handle.
class Publisher {
public:
    Publisher(const std::string& name, std::shared_ptr<TransportBase> transport)
        : name_(name), transport_(std::move(transport)) {}

    void Publish(const void* data, size_t len) {
        if (transport_) transport_->Publish(data, len);
    }

    const std::string& channel_name() const { return name_; }

private:
    std::string name_;
    std::shared_ptr<TransportBase> transport_;
};

/// Subscriber handle.
class Subscriber {
public:
    Subscriber(const std::string& name, std::shared_ptr<TransportBase> transport)
        : name_(name), transport_(std::move(transport)) {}

    /// Poll for one message.
    bool Poll(void* out, size_t* out_len, int timeout_ms = 100) {
        if (transport_) return transport_->Poll(out, out_len, timeout_ms);
        return false;
    }

    /// Read latest message (non-blocking).
    bool ReadLatest(void* out, size_t* out_len) {
        if (transport_) return transport_->ReadLatest(out, out_len);
        return false;
    }

    const std::string& channel_name() const { return name_; }

private:
    std::string name_;
    std::shared_ptr<TransportBase> transport_;
};

/// Singleton channel registry managing transport lifecycle and auto-routing.
class ChannelManager {
public:
    /// Get the singleton instance.
    static ChannelManager& Instance();

    /// Reset singleton (for testing).
    static void Reset();

    /// Create a publisher for a named channel.
    std::unique_ptr<Publisher> CreatePublisher(
        const std::string& name, size_t msg_size,
        TransportLevel level = TransportLevel::kIntraProcess);

    /// Create a subscriber for a named channel.
    std::unique_ptr<Subscriber> CreateSubscriber(
        const std::string& name, size_t msg_size,
        SubscriberCallback callback = nullptr,
        TransportLevel level = TransportLevel::kIntraProcess);

    /// Get channel info. Returns nullptr if not found.
    const ChannelEntry* GetChannelInfo(const std::string& name) const;

    /// List all channel names.
    std::vector<std::string> ListChannels() const;

    /// Close all channels.
    void CloseAll();

private:
    ChannelManager() = default;

    std::shared_ptr<TransportBase> GetOrCreateChannel(
        const std::string& name, size_t msg_size, TransportLevel level);

    mutable std::mutex mutex_;
    std::unordered_map<std::string, ChannelEntry> channels_;

    static std::mutex instance_mutex_;
    static ChannelManager* instance_;
};

}  // namespace core
}  // namespace rynnrcp

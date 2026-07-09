// Copyright 2026 RynnRCP Authors. All rights reserved.
// Core layer: ChannelManager implementation.

#include "channel_manager.h"

#include <stdexcept>

namespace rynnrcp {
namespace core {

std::mutex ChannelManager::instance_mutex_;
ChannelManager* ChannelManager::instance_ = nullptr;

ChannelManager& ChannelManager::Instance() {
    if (instance_ == nullptr) {
        std::lock_guard<std::mutex> lock(instance_mutex_);
        if (instance_ == nullptr) {
            instance_ = new ChannelManager();
        }
    }
    return *instance_;
}

void ChannelManager::Reset() {
    std::lock_guard<std::mutex> lock(instance_mutex_);
    if (instance_ != nullptr) {
        instance_->CloseAll();
        delete instance_;
        instance_ = nullptr;
    }
}

std::shared_ptr<TransportBase> ChannelManager::GetOrCreateChannel(
    const std::string& name, size_t msg_size, TransportLevel level) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = channels_.find(name);
    if (it != channels_.end()) {
        if (it->second.msg_size != msg_size) {
            throw std::invalid_argument(
                "Channel '" + name + "' already exists with msg_size=" +
                std::to_string(it->second.msg_size) +
                ", requested " + std::to_string(msg_size));
        }
        return it->second.transport;
    }

    // Create new transport
    std::shared_ptr<TransportBase> transport;
    switch (level) {
        case TransportLevel::kIntraThread:
        case TransportLevel::kIntraProcess:
            transport = std::make_shared<IntraProcessTransport>(name, msg_size);
            break;
        case TransportLevel::kShm:
            transport = std::make_shared<ShmTransport>(name, msg_size, 16, true);
            break;
        default:
            throw std::invalid_argument("Unknown transport level");
    }

    ChannelEntry entry{name, msg_size, level, transport};
    channels_.emplace(name, std::move(entry));
    return transport;
}

std::unique_ptr<Publisher> ChannelManager::CreatePublisher(
    const std::string& name, size_t msg_size, TransportLevel level) {
    auto transport = GetOrCreateChannel(name, msg_size, level);
    return std::make_unique<Publisher>(name, std::move(transport));
}

std::unique_ptr<Subscriber> ChannelManager::CreateSubscriber(
    const std::string& name, size_t msg_size,
    SubscriberCallback callback, TransportLevel level) {
    auto transport = GetOrCreateChannel(name, msg_size, level);
    if (callback) {
        transport->Subscribe(callback);
    }
    return std::make_unique<Subscriber>(name, std::move(transport));
}

const ChannelEntry* ChannelManager::GetChannelInfo(const std::string& name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = channels_.find(name);
    if (it != channels_.end()) {
        return &it->second;
    }
    return nullptr;
}

std::vector<std::string> ChannelManager::ListChannels() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> names;
    names.reserve(channels_.size());
    for (const auto& kv : channels_) {
        names.push_back(kv.first);
    }
    return names;
}

void ChannelManager::CloseAll() {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& kv : channels_) {
        kv.second.transport->Close();
        if (kv.second.level == TransportLevel::kShm) {
            auto* shm = dynamic_cast<ShmTransport*>(kv.second.transport.get());
            if (shm) shm->Unlink();
        }
    }
    channels_.clear();
}

}  // namespace core
}  // namespace rynnrcp

// Copyright 2026 RynnRCP Authors. All rights reserved.
// Tests for ChannelManager.

#include "channel_manager.h"

#include <atomic>
#include <cassert>
#include <cstring>
#include <iostream>
#include <thread>
#include <vector>

using namespace rynnrcp::core;

static int tests_passed = 0;

#define TEST(name) \
    static void test_##name(); \
    struct Register_##name { Register_##name() { test_##name(); tests_passed++; std::cout << "  PASS: " #name << std::endl; } } reg_##name; \
    static void test_##name()

TEST(create_publisher_subscriber) {
    ChannelManager::Reset();
    auto& mgr = ChannelManager::Instance();

    auto pub = mgr.CreatePublisher("test_ch", 64);
    assert(pub != nullptr);
    assert(pub->channel_name() == "test_ch");

    auto sub = mgr.CreateSubscriber("test_ch", 64);
    assert(sub != nullptr);
    assert(sub->channel_name() == "test_ch");

    ChannelManager::Reset();
}

TEST(pub_sub_communication) {
    ChannelManager::Reset();
    auto& mgr = ChannelManager::Instance();

    std::atomic<int> count{0};
    auto sub = mgr.CreateSubscriber("comm_ch", 32,
        [&count](const void*, size_t) { count++; });
    auto pub = mgr.CreatePublisher("comm_ch", 32);

    uint8_t msg[4] = {1, 2, 3, 4};
    pub->Publish(msg, 4);
    assert(count.load() == 1);

    pub->Publish(msg, 4);
    assert(count.load() == 2);

    ChannelManager::Reset();
}

TEST(poll_communication) {
    ChannelManager::Reset();
    auto& mgr = ChannelManager::Instance();

    auto pub = mgr.CreatePublisher("poll_ch", 64);
    auto sub = mgr.CreateSubscriber("poll_ch", 64);

    uint8_t msg[] = {10, 20, 30};
    pub->Publish(msg, 3);

    uint8_t buf[64];
    size_t len = sizeof(buf);
    bool ok = sub->Poll(buf, &len, 100);
    assert(ok);
    assert(len == 3);
    assert(buf[0] == 10 && buf[2] == 30);

    ChannelManager::Reset();
}

TEST(list_channels) {
    ChannelManager::Reset();
    auto& mgr = ChannelManager::Instance();

    mgr.CreatePublisher("ch_a", 16);
    mgr.CreatePublisher("ch_b", 32);
    mgr.CreatePublisher("ch_c", 64);

    auto channels = mgr.ListChannels();
    assert(channels.size() == 3);

    ChannelManager::Reset();
}

TEST(channel_info) {
    ChannelManager::Reset();
    auto& mgr = ChannelManager::Instance();

    mgr.CreatePublisher("info_ch", 128, TransportLevel::kIntraProcess);
    auto* info = mgr.GetChannelInfo("info_ch");
    assert(info != nullptr);
    assert(info->msg_size == 128);
    assert(info->level == TransportLevel::kIntraProcess);

    auto* missing = mgr.GetChannelInfo("nonexistent");
    assert(missing == nullptr);

    ChannelManager::Reset();
}

TEST(msg_size_mismatch_throws) {
    ChannelManager::Reset();
    auto& mgr = ChannelManager::Instance();

    mgr.CreatePublisher("mismatch_ch", 64);
    bool threw = false;
    try {
        mgr.CreatePublisher("mismatch_ch", 128);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);

    ChannelManager::Reset();
}

TEST(multithread_pub_sub) {
    ChannelManager::Reset();
    auto& mgr = ChannelManager::Instance();

    constexpr int N = 50;
    auto pub = mgr.CreatePublisher("mt_ch", 64);
    auto sub = mgr.CreateSubscriber("mt_ch", 64);

    std::atomic<int> received{0};

    std::thread reader([&]() {
        uint8_t buf[64];
        for (int i = 0; i < N; i++) {
            size_t len = sizeof(buf);
            if (sub->Poll(buf, &len, 500)) {
                received++;
            }
        }
    });

    std::thread writer([&]() {
        for (int i = 0; i < N; i++) {
            uint8_t msg[4] = {(uint8_t)i, 0, 0, 0};
            pub->Publish(msg, 4);
            std::this_thread::sleep_for(std::chrono::microseconds(200));
        }
    });

    writer.join();
    reader.join();
    assert(received.load() == N);

    ChannelManager::Reset();
}

int main() {
    std::cout << "=== ChannelManager Tests ===" << std::endl;
    std::cout << tests_passed << " tests passed." << std::endl;
    return 0;
}

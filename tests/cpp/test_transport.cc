// Copyright 2026 RynnRCP Authors. All rights reserved.
// Tests for Transport (IntraProcess + SHM).

#include "transport.h"

#include <atomic>
#include <cassert>
#include <chrono>
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

// ===========================================================================
// IntraProcessTransport tests
// ===========================================================================

TEST(intra_publish_poll) {
    IntraProcessTransport t("test_ch", 64);
    uint8_t msg[] = {1, 2, 3, 4};
    t.Publish(msg, sizeof(msg));

    uint8_t buf[64];
    size_t len = sizeof(buf);
    bool ok = t.Poll(buf, &len, 100);
    assert(ok);
    assert(len == 4);
    assert(buf[0] == 1 && buf[3] == 4);
}

TEST(intra_callback) {
    IntraProcessTransport t("cb_ch", 32);
    std::atomic<int> count{0};
    t.Subscribe([&count](const void*, size_t) { count++; });

    uint8_t msg[4] = {0};
    t.Publish(msg, 4);
    t.Publish(msg, 4);
    assert(count.load() == 2);
}

TEST(intra_read_latest) {
    IntraProcessTransport t("latest_ch", 16);
    for (int i = 0; i < 5; i++) {
        uint8_t msg[4] = {(uint8_t)i, 0, 0, 0};
        t.Publish(msg, 4);
    }

    uint8_t buf[16];
    size_t len = sizeof(buf);
    bool ok = t.ReadLatest(buf, &len);
    assert(ok);
    assert(buf[0] == 4);  // Latest value
}

TEST(intra_timeout_no_data) {
    IntraProcessTransport t("empty_ch", 16);
    uint8_t buf[16];
    size_t len = sizeof(buf);
    auto start = std::chrono::steady_clock::now();
    bool ok = t.Poll(buf, &len, 50);
    auto elapsed = std::chrono::steady_clock::now() - start;
    assert(!ok);
    assert(elapsed >= std::chrono::milliseconds(40));
}

TEST(intra_close) {
    IntraProcessTransport t("close_ch", 16);
    t.Close();
    assert(!t.IsOpen());

    uint8_t msg[4] = {1, 2, 3, 4};
    t.Publish(msg, 4);  // Should not crash

    uint8_t buf[16];
    size_t len = sizeof(buf);
    bool ok = t.Poll(buf, &len, 10);
    assert(!ok);
}

TEST(intra_multithread) {
    IntraProcessTransport t("mt_ch", 64, 256);
    constexpr int N = 100;
    std::atomic<int> received{0};

    std::thread writer([&]() {
        for (int i = 0; i < N; i++) {
            uint8_t msg[4] = {(uint8_t)(i & 0xFF), 0, 0, 0};
            t.Publish(msg, 4);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    std::thread reader([&]() {
        uint8_t buf[64];
        int attempts = 0;
        while (received.load() < N && attempts < N * 20) {
            size_t len = sizeof(buf);
            if (t.Poll(buf, &len, 50)) {
                received++;
            }
            attempts++;
        }
    });

    writer.join();
    reader.join();
    assert(received.load() == N);
}

// ===========================================================================
// ShmTransport tests
// ===========================================================================

TEST(shm_publish_poll) {
    ShmTransport t("test_shm_transport", 128, 16, true);
    uint8_t msg[] = {10, 20, 30, 40, 50};
    t.Publish(msg, sizeof(msg));

    uint8_t buf[128];
    size_t len = sizeof(buf);
    bool ok = t.Poll(buf, &len, 200);
    assert(ok);
    // RingBuffer returns full slot; check data content
    assert(buf[0] == 10 && buf[4] == 50);

    t.Close();
    t.Unlink();
}

TEST(shm_read_latest) {
    ShmTransport t("test_shm_latest", 64, 16, true);
    for (int i = 0; i < 10; i++) {
        uint8_t msg[4] = {(uint8_t)i, 0, 0, 0};
        t.Publish(msg, 4);
    }

    uint8_t buf[64];
    size_t len = sizeof(buf);
    bool ok = t.ReadLatest(buf, &len);
    assert(ok);
    assert(buf[0] == 9);

    t.Close();
    t.Unlink();
}

TEST(shm_multiple_messages) {
    ShmTransport t("test_shm_multi", 32, 16, true);
    constexpr int N = 8;
    for (int i = 0; i < N; i++) {
        uint8_t msg[4] = {(uint8_t)i, 0, 0, 0};
        t.Publish(msg, 4);
    }

    // Read all in sequence
    for (int i = 0; i < N; i++) {
        uint8_t buf[32];
        size_t len = sizeof(buf);
        bool ok = t.Poll(buf, &len, 100);
        assert(ok);
        assert(buf[0] == (uint8_t)i);
    }

    t.Close();
    t.Unlink();
}

int main() {
    std::cout << "=== Transport Tests ===" << std::endl;
    std::cout << tests_passed << " tests passed." << std::endl;
    return 0;
}

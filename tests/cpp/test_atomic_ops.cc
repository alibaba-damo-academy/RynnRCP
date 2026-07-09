// Copyright 2026 RynnRCP Authors. All rights reserved.
// Tests for rynnrcp/native/cpp/atomic_ops.h

#include "atomic_ops.h"

#include <cassert>
#include <cstdio>
#include <atomic>
#include <thread>
#include <vector>

using namespace rynnrcp::native;

void test_store_load_64() {
    std::atomic<uint64_t> val{0};
    store_release(val, 42);
    assert(load_acquire(val) == 42);

    store_release(val, 0xDEADBEEFCAFE0000ULL);
    assert(load_acquire(val) == 0xDEADBEEFCAFE0000ULL);

    store_release(val, 0);
    assert(load_acquire(val) == 0);
    printf("[PASS] test_store_load_64\n");
}

void test_store_load_32() {
    std::atomic<uint32_t> val{0};
    store_release_u32(val, 123);
    assert(load_acquire_u32(val) == 123);

    store_release_u32(val, 0xFFFFFFFF);
    assert(load_acquire_u32(val) == 0xFFFFFFFF);

    store_release_u32(val, 0);
    assert(load_acquire_u32(val) == 0);
    printf("[PASS] test_store_load_32\n");
}

void test_fetch_add_sub() {
    std::atomic<uint64_t> val{100};
    uint64_t old = fetch_add_release(val, 50);
    assert(old == 100);
    assert(load_acquire(val) == 150);

    old = fetch_sub_release(val, 30);
    assert(old == 150);
    assert(load_acquire(val) == 120);
    printf("[PASS] test_fetch_add_sub\n");
}

void test_fetch_add_sub_u32() {
    std::atomic<uint32_t> val{1000};
    uint32_t old = fetch_add_release_u32(val, 234);
    assert(old == 1000);
    assert(load_acquire_u32(val) == 1234);

    old = fetch_sub_release_u32(val, 234);
    assert(old == 1234);
    assert(load_acquire_u32(val) == 1000);
    printf("[PASS] test_fetch_add_sub_u32\n");
}

void test_compare_exchange() {
    std::atomic<uint64_t> val{10};

    uint64_t expected = 10;
    bool ok = compare_exchange_strong(val, expected, 20);
    assert(ok);
    assert(load_acquire(val) == 20);

    // Should fail: expected is stale
    expected = 10;
    ok = compare_exchange_strong(val, expected, 30);
    assert(!ok);
    assert(expected == 20);  // updated to current value
    assert(load_acquire(val) == 20);
    printf("[PASS] test_compare_exchange\n");
}

void test_multithread_visibility() {
    constexpr int kIterations = 100000;
    std::atomic<uint64_t> shared_val{0};
    std::atomic<bool> done{false};

    std::thread writer([&]() {
        for (int i = 1; i <= kIterations; ++i) {
            store_release(shared_val, static_cast<uint64_t>(i));
        }
        done.store(true, std::memory_order_release);
    });

    std::thread reader([&]() {
        uint64_t last_seen = 0;
        while (!done.load(std::memory_order_acquire) || load_acquire(shared_val) != kIterations) {
            uint64_t cur = load_acquire(shared_val);
            assert(cur >= last_seen);
            last_seen = cur;
        }
        assert(last_seen == kIterations);
    });

    writer.join();
    reader.join();
    printf("[PASS] test_multithread_visibility\n");
}

void test_multithread_fetch_add() {
    constexpr int kThreads = 4;
    constexpr int kPerThread = 50000;
    std::atomic<uint64_t> counter{0};

    std::vector<std::thread> threads;
    for (int t = 0; t < kThreads; ++t) {
        threads.emplace_back([&]() {
            for (int i = 0; i < kPerThread; ++i) {
                fetch_add_release(counter, 1);
            }
        });
    }
    for (auto& th : threads) th.join();

    assert(load_acquire(counter) == static_cast<uint64_t>(kThreads * kPerThread));
    printf("[PASS] test_multithread_fetch_add\n");
}

void test_fences() {
    std::atomic<uint64_t> a{0}, b{0};
    store_release(a, 1);
    release_fence();
    store_release(b, 2);
    acquire_fence();
    assert(load_acquire(b) == 2);
    assert(load_acquire(a) == 1);
    memory_fence();
    printf("[PASS] test_fences\n");
}

int main() {
    printf("=== atomic_ops tests ===\n");
    test_store_load_64();
    test_store_load_32();
    test_fetch_add_sub();
    test_fetch_add_sub_u32();
    test_compare_exchange();
    test_multithread_visibility();
    test_multithread_fetch_add();
    test_fences();
    printf("=== All atomic_ops tests passed ===\n");
    return 0;
}

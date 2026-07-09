// Copyright 2026 RynnRCP Authors. All rights reserved.
// Tests for rynnrcp/native/cpp/event.h

#include "event.h"

#include <cassert>
#include <cstdio>
#include <thread>
#include <chrono>

using namespace rynnrcp::native;

void test_create_close() {
    EventHandle ev = EventCreate();
    assert(ev != kInvalidEvent);
    EventClose(ev);
    assert(ev == kInvalidEvent);
    printf("[PASS] test_create_close\n");
}

void test_signal_wait() {
    EventHandle ev = EventCreate();
    assert(ev != kInvalidEvent);

    // Signal then wait should succeed immediately
    bool ok = EventSignal(ev);
    assert(ok);

    WaitResult result = EventWait(ev, 1000);  // 1 second timeout
    assert(result == WaitResult::kSignaled);

    EventClose(ev);
    printf("[PASS] test_signal_wait\n");
}

void test_timeout() {
    EventHandle ev = EventCreate();
    assert(ev != kInvalidEvent);

    // Wait without signal should timeout
    auto start = std::chrono::steady_clock::now();
    WaitResult result = EventWait(ev, 50);  // 50ms timeout
    auto elapsed = std::chrono::steady_clock::now() - start;

    assert(result == WaitResult::kTimeout);
    // Should have waited at least ~40ms (allow some tolerance)
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
    assert(ms >= 30);

    EventClose(ev);
    printf("[PASS] test_timeout (waited %lld ms)\n", static_cast<long long>(ms));
}

void test_nonblocking_poll() {
    EventHandle ev = EventCreate();
    assert(ev != kInvalidEvent);

    // Non-blocking poll (timeout=0) with no signal -> timeout
    WaitResult result = EventWait(ev, 0);
    assert(result == WaitResult::kTimeout);

    // Signal then non-blocking poll -> signaled
    EventSignal(ev);
    result = EventWait(ev, 0);
    assert(result == WaitResult::kSignaled);

    EventClose(ev);
    printf("[PASS] test_nonblocking_poll\n");
}

void test_cross_thread_signal() {
    EventHandle ev = EventCreate();
    assert(ev != kInvalidEvent);

    bool received = false;

    std::thread waiter([&]() {
        WaitResult result = EventWait(ev, 5000);  // 5 sec max
        if (result == WaitResult::kSignaled) {
            received = true;
        }
    });

    // Give waiter time to enter wait
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    bool ok = EventSignal(ev);
    assert(ok);

    waiter.join();
    assert(received);

    EventClose(ev);
    printf("[PASS] test_cross_thread_signal\n");
}

void test_multiple_signals() {
    EventHandle ev = EventCreate();
    assert(ev != kInvalidEvent);

    // Multiple signals before wait
    for (int i = 0; i < 5; ++i) {
        EventSignal(ev);
    }

    // Should be able to consume at least one signal
    WaitResult result = EventWait(ev, 100);
    assert(result == WaitResult::kSignaled);

    EventClose(ev);
    printf("[PASS] test_multiple_signals\n");
}

int main() {
    printf("=== event tests ===\n");
    test_create_close();
    test_signal_wait();
    test_timeout();
    test_nonblocking_poll();
    test_cross_thread_signal();
    test_multiple_signals();
    printf("=== All event tests passed ===\n");
    return 0;
}

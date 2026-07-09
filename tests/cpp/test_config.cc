// Copyright 2026 RynnRCP Authors. All rights reserved.
// Tests for rynnrcp/native/cpp/config.h

#include "config.h"

#include <cassert>
#include <cstdio>

using namespace rynnrcp::native;

void test_cache_line_size() {
    assert(kCacheLineSize >= 64);
    assert(kCacheLineSize <= 256);
    // Must be power of 2
    assert((kCacheLineSize & (kCacheLineSize - 1)) == 0);
    printf("[PASS] test_cache_line_size: %zu bytes\n", kCacheLineSize);
}

void test_page_size() {
    size_t ps = GetPageSize();
    assert(ps >= 4096);
    assert(ps <= 65536);
    assert((ps & (ps - 1)) == 0);
    printf("[PASS] test_page_size: %zu bytes\n", ps);
}

void test_align_up() {
    assert(AlignUp(64, 64) == 64);
    assert(AlignUp(128, 64) == 128);
    assert(AlignUp(1, 64) == 64);
    assert(AlignUp(63, 64) == 64);
    assert(AlignUp(65, 64) == 128);
    assert(AlignUp(0, 64) == 0);
    assert(AlignUp(4097, 4096) == 8192);
    assert(AlignUp(4096, 4096) == 4096);
    printf("[PASS] test_align_up\n");
}

void test_align_to_cache_line() {
    assert(AlignToCacheLine(1) == kCacheLineSize);
    assert(AlignToCacheLine(kCacheLineSize) == kCacheLineSize);
    assert(AlignToCacheLine(kCacheLineSize + 1) == kCacheLineSize * 2);
    assert(AlignToCacheLine(0) == 0);
    printf("[PASS] test_align_to_cache_line\n");
}

void test_align_to_page() {
    size_t ps = GetPageSize();
    assert(AlignToPage(1) == ps);
    assert(AlignToPage(ps) == ps);
    assert(AlignToPage(ps + 1) == ps * 2);
    assert(AlignToPage(0) == 0);
    printf("[PASS] test_align_to_page\n");
}

void test_constants() {
    assert(kMaxChannelNameLen == 64);
    assert(kDefaultRingSlots >= 2);
    assert((kDefaultRingSlots & (kDefaultRingSlots - 1)) == 0);
    printf("[PASS] test_constants\n");
}

void test_constexpr() {
    static_assert(AlignUp(100, 64) == 128, "AlignUp not constexpr");
    static_assert(AlignToCacheLine(1) == kCacheLineSize, "AlignToCacheLine not constexpr");
    static_assert(kMaxChannelNameLen > 0, "Bad channel name len");
    printf("[PASS] test_constexpr\n");
}

int main() {
    printf("=== config tests ===\n");
    test_cache_line_size();
    test_page_size();
    test_align_up();
    test_align_to_cache_line();
    test_align_to_page();
    test_constants();
    test_constexpr();
    printf("=== All config tests passed ===\n");
    return 0;
}

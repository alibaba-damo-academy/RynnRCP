// Copyright 2026 RynnRCP Authors. All rights reserved.
// Tests for TFTree.

#define _USE_MATH_DEFINES
#include "tf.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

using namespace rynnrcp::core;

static int tests_passed = 0;

#define TEST(name) \
    static void test_##name(); \
    struct Register_##name { Register_##name() { test_##name(); tests_passed++; std::cout << "  PASS: " #name << std::endl; } } reg_##name; \
    static void test_##name()

static constexpr double kEps = 1e-6;

static bool near(double a, double b) { return std::abs(a - b) < kEps; }

// Identity quaternion
static const Quat kIdentity = {0.0, 0.0, 0.0, 1.0};

// ======================================================================
// Quaternion math tests
// ======================================================================

TEST(quat_normalize) {
    Quat q = {0.0, 0.0, 0.0, 2.0};
    auto n = QuatNormalize(q);
    assert(near(n[3], 1.0));
}

TEST(quat_multiply_identity) {
    Quat a = {0.1, 0.2, 0.3, 0.9};
    a = QuatNormalize(a);
    auto r = QuatMultiply(a, kIdentity);
    for (int i = 0; i < 4; i++) assert(near(r[i], a[i]));
}

TEST(quat_inverse) {
    Quat a = QuatNormalize({0.1, 0.2, 0.3, 0.9});
    auto inv = QuatInverse(a);
    auto prod = QuatMultiply(a, inv);
    // Should be identity
    assert(near(prod[0], 0.0));
    assert(near(prod[1], 0.0));
    assert(near(prod[2], 0.0));
    assert(near(prod[3], 1.0));
}

TEST(quat_rotate_vec) {
    // 90 degrees around Z axis: (0,0,sin(45),cos(45))
    double angle = M_PI / 2.0;
    Quat q = {0.0, 0.0, std::sin(angle/2), std::cos(angle/2)};
    Vec3 v = {1.0, 0.0, 0.0};
    auto r = QuatRotateVec(q, v);
    assert(near(r[0], 0.0));
    assert(near(r[1], 1.0));
    assert(near(r[2], 0.0));
}

TEST(quat_slerp_endpoints) {
    Quat a = QuatNormalize({0.1, 0.0, 0.0, 0.99});
    Quat b = QuatNormalize({0.0, 0.5, 0.0, 0.87});
    auto s0 = QuatSlerp(a, b, 0.0);
    auto s1 = QuatSlerp(a, b, 1.0);
    for (int i = 0; i < 4; i++) {
        assert(near(s0[i], a[i]));
        assert(near(s1[i], b[i]));
    }
}

// ======================================================================
// Transform composition tests
// ======================================================================

TEST(compose_identity) {
    Transform parent{"world", "a", {1.0, 2.0, 3.0}, kIdentity, 1.0};
    Transform child{"a", "b", {4.0, 5.0, 6.0}, kIdentity, 2.0};
    auto composed = ComposeTransforms(parent, child);
    assert(composed.parent == "world");
    assert(composed.child == "b");
    assert(near(composed.translation[0], 5.0));
    assert(near(composed.translation[1], 7.0));
    assert(near(composed.translation[2], 9.0));
}

TEST(invert_transform) {
    Transform tf{"world", "sensor", {1.0, 0.0, 0.0}, kIdentity, 1.0};
    auto inv = InvertTransform(tf);
    assert(inv.parent == "sensor");
    assert(inv.child == "world");
    assert(near(inv.translation[0], -1.0));
    assert(near(inv.translation[1], 0.0));
    assert(near(inv.translation[2], 0.0));
}

// ======================================================================
// TFTree tests
// ======================================================================

TEST(tree_set_and_lookup) {
    TFTree tree;
    Transform tf{"world", "base", {1.0, 2.0, 3.0}, kIdentity, 1.0};
    tree.SetTransform(tf);

    auto result = tree.Lookup("base", "world");
    assert(near(result.translation[0], 1.0));
    assert(near(result.translation[1], 2.0));
    assert(near(result.translation[2], 3.0));
}

TEST(tree_chain_lookup) {
    TFTree tree;
    tree.SetTransform({"world", "base", {1.0, 0.0, 0.0}, kIdentity, 1.0});
    tree.SetTransform({"base", "sensor", {0.0, 1.0, 0.0}, kIdentity, 1.0});

    auto result = tree.Lookup("sensor", "world");
    assert(near(result.translation[0], 1.0));
    assert(near(result.translation[1], 1.0));
    assert(near(result.translation[2], 0.0));
}

TEST(tree_reverse_lookup) {
    TFTree tree;
    tree.SetTransform({"world", "base", {3.0, 0.0, 0.0}, kIdentity, 1.0});

    // Lookup base -> world (inverse direction)
    auto result = tree.Lookup("world", "base");
    assert(near(result.translation[0], -3.0));
}

TEST(tree_can_transform) {
    TFTree tree;
    tree.SetTransform({"world", "a", {1.0, 0.0, 0.0}, kIdentity, 1.0});
    tree.SetTransform({"a", "b", {0.0, 1.0, 0.0}, kIdentity, 1.0});

    assert(tree.CanTransform("b", "world"));
    assert(tree.CanTransform("world", "b"));
    assert(!tree.CanTransform("world", "nonexistent"));
}

TEST(tree_all_frames) {
    TFTree tree;
    tree.SetTransform({"world", "base", {0,0,0}, kIdentity, 1.0});
    tree.SetTransform({"base", "cam", {0,0,0}, kIdentity, 1.0});

    auto frames = tree.AllFrames();
    assert(frames.size() == 3);
}

TEST(tree_interpolation) {
    TFTree tree;
    tree.SetTransform({"world", "obj", {0.0, 0.0, 0.0}, kIdentity, 1.0});
    tree.SetTransform({"world", "obj", {10.0, 0.0, 0.0}, kIdentity, 2.0});

    // Query at t=1.5 -> interpolated to x=5.0
    auto result = tree.Lookup("obj", "world", 1.5);
    assert(near(result.translation[0], 5.0));
}

TEST(tree_history_pruning) {
    TFTree tree(2.0);  // 2 second history
    tree.SetTransform({"world", "a", {1.0, 0.0, 0.0}, kIdentity, 1.0});
    tree.SetTransform({"world", "a", {2.0, 0.0, 0.0}, kIdentity, 2.0});
    tree.SetTransform({"world", "a", {3.0, 0.0, 0.0}, kIdentity, 3.0});
    tree.SetTransform({"world", "a", {4.0, 0.0, 0.0}, kIdentity, 4.0});

    // Latest should be 4.0
    auto result = tree.Lookup("a", "world");
    assert(near(result.translation[0], 4.0));
}

TEST(tree_no_path_throws) {
    TFTree tree;
    tree.SetTransform({"world", "a", {0,0,0}, kIdentity, 1.0});

    bool threw = false;
    try {
        tree.Lookup("nonexistent", "world");
    } catch (const std::runtime_error&) {
        threw = true;
    }
    assert(threw);
}

TEST(tree_thread_safety) {
    TFTree tree;
    constexpr int N = 100;

    std::thread writer([&]() {
        for (int i = 0; i < N; i++) {
            tree.SetTransform({"world", "moving",
                {(double)i, 0.0, 0.0}, kIdentity, (double)i});
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
    });

    std::thread reader([&]() {
        for (int i = 0; i < N; i++) {
            if (tree.CanTransform("moving", "world")) {
                auto tf = tree.Lookup("moving", "world");
                (void)tf;  // Just ensure no crash
            }
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
    });

    writer.join();
    reader.join();
}

int main() {
    std::cout << "=== TF Tests ===" << std::endl;
    std::cout << tests_passed << " tests passed." << std::endl;
    return 0;
}

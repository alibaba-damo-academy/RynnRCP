// Copyright 2026 RynnRCP Authors. All rights reserved.
// Core layer: TFTree – ROS-style coordinate-frame transform tree.
// Features:
//   - Broadcast parent→child transforms with timestamps
//   - Maintain a time-indexed history per edge (configurable window)
//   - Lookup composed transforms along arbitrary chains
//   - SLERP interpolation between the two nearest timestamps
//   - Pure C++ quaternion math

#pragma once

#include <array>
#include <cstddef>
#include <deque>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace rynnrcp {
namespace core {

// ======================================================================
// Data types
// ======================================================================

using Vec3 = std::array<double, 3>;       // (x, y, z)
using Quat = std::array<double, 4>;       // (x, y, z, w)

struct Transform {
    std::string parent;
    std::string child;
    Vec3 translation;
    Quat rotation;
    double timestamp;
};

// ======================================================================
// Quaternion helpers (x, y, z, w convention)
// ======================================================================

Quat QuatNormalize(const Quat& q);
Quat QuatMultiply(const Quat& a, const Quat& b);
Quat QuatConjugate(const Quat& q);
Quat QuatInverse(const Quat& q);
Vec3 QuatRotateVec(const Quat& q, const Vec3& v);
Quat QuatSlerp(const Quat& a, const Quat& b, double t);

// ======================================================================
// Transform composition
// ======================================================================

Transform ComposeTransforms(const Transform& parent_tf, const Transform& child_tf);
Transform InvertTransform(const Transform& tf);

// ======================================================================
// TFTree
// ======================================================================

/// Thread-safe transform tree with time-indexed history.
class TFTree {
public:
    /// @param history_duration  How many seconds of history to keep per edge.
    explicit TFTree(double history_duration = 10.0);

    /// Insert or update a transform in the tree.
    void SetTransform(const Transform& tf);

    /// Lookup composed transform from source to target.
    /// @param time  Query timestamp. 0 = use latest.
    /// @throws std::runtime_error if no path exists.
    Transform Lookup(const std::string& target, const std::string& source,
                     double time = 0.0) const;

    /// Check if a path exists between source and target.
    bool CanTransform(const std::string& target, const std::string& source) const;

    /// Get the frame chain from source to target.
    std::vector<std::string> GetChain(const std::string& target,
                                       const std::string& source) const;

    /// List all known frames.
    std::vector<std::string> AllFrames() const;

private:
    using EdgeKey = std::pair<std::string, std::string>;

    struct EdgeKeyHash {
        size_t operator()(const EdgeKey& k) const {
            auto h1 = std::hash<std::string>{}(k.first);
            auto h2 = std::hash<std::string>{}(k.second);
            return h1 ^ (h2 << 32) ^ (h2 >> 32);
        }
    };

    std::vector<std::string> FindChain(const std::string& source,
                                        const std::string& target) const;

    const Transform* InterpolateEdge(const std::string& a, const std::string& b,
                                      double time, Transform* out) const;

    const Transform* InterpolateBuf(const std::deque<Transform>& buf,
                                     double time, Transform* out) const;

    mutable std::recursive_mutex mutex_;
    double history_duration_;
    std::unordered_map<EdgeKey, std::deque<Transform>, EdgeKeyHash> edges_;
    std::unordered_map<std::string, std::unordered_set<std::string>> adjacency_;
};

}  // namespace core
}  // namespace rynnrcp

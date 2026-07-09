// Copyright 2026 RynnRCP Authors. All rights reserved.
// Core layer: TFTree implementation.

#include "tf.h"

#include <algorithm>
#include <cmath>
#include <queue>
#include <stdexcept>

namespace rynnrcp {
namespace core {

// ======================================================================
// Quaternion helpers
// ======================================================================

Quat QuatNormalize(const Quat& q) {
    double n = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n < 1e-12) return {0.0, 0.0, 0.0, 1.0};
    double inv = 1.0 / n;
    return {q[0]*inv, q[1]*inv, q[2]*inv, q[3]*inv};
}

Quat QuatMultiply(const Quat& a, const Quat& b) {
    // Hamilton product (x, y, z, w)
    return {
        a[3]*b[0] + a[0]*b[3] + a[1]*b[2] - a[2]*b[1],
        a[3]*b[1] - a[0]*b[2] + a[1]*b[3] + a[2]*b[0],
        a[3]*b[2] + a[0]*b[1] - a[1]*b[0] + a[2]*b[3],
        a[3]*b[3] - a[0]*b[0] - a[1]*b[1] - a[2]*b[2]
    };
}

Quat QuatConjugate(const Quat& q) {
    return {-q[0], -q[1], -q[2], q[3]};
}

Quat QuatInverse(const Quat& q) {
    return QuatNormalize(QuatConjugate(q));
}

Vec3 QuatRotateVec(const Quat& q, const Vec3& v) {
    Quat vq = {v[0], v[1], v[2], 0.0};
    Quat r = QuatMultiply(QuatMultiply(q, vq), QuatConjugate(q));
    return {r[0], r[1], r[2]};
}

Quat QuatSlerp(const Quat& a, const Quat& b, double t) {
    double dot = a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3];
    Quat b2 = b;
    if (dot < 0.0) {
        b2 = {-b[0], -b[1], -b[2], -b[3]};
        dot = -dot;
    }
    if (dot > 0.9995) {
        // Linear interpolation for very close quaternions
        Quat result = {
            a[0] + t*(b2[0] - a[0]),
            a[1] + t*(b2[1] - a[1]),
            a[2] + t*(b2[2] - a[2]),
            a[3] + t*(b2[3] - a[3])
        };
        return QuatNormalize(result);
    }
    double theta = std::acos(std::max(-1.0, std::min(1.0, dot)));
    double sin_theta = std::sin(theta);
    double wa = std::sin((1.0 - t) * theta) / sin_theta;
    double wb = std::sin(t * theta) / sin_theta;
    return QuatNormalize({
        wa*a[0] + wb*b2[0],
        wa*a[1] + wb*b2[1],
        wa*a[2] + wb*b2[2],
        wa*a[3] + wb*b2[3]
    });
}

// ======================================================================
// Transform composition
// ======================================================================

static Vec3 LerpVec(const Vec3& a, const Vec3& b, double t) {
    return {
        a[0] + t*(b[0] - a[0]),
        a[1] + t*(b[1] - a[1]),
        a[2] + t*(b[2] - a[2])
    };
}

Transform ComposeTransforms(const Transform& parent_tf, const Transform& child_tf) {
    Vec3 rotated = QuatRotateVec(parent_tf.rotation, child_tf.translation);
    Transform result;
    result.parent = parent_tf.parent;
    result.child = child_tf.child;
    result.translation = {
        parent_tf.translation[0] + rotated[0],
        parent_tf.translation[1] + rotated[1],
        parent_tf.translation[2] + rotated[2]
    };
    result.rotation = QuatMultiply(parent_tf.rotation, child_tf.rotation);
    result.timestamp = std::max(parent_tf.timestamp, child_tf.timestamp);
    return result;
}

Transform InvertTransform(const Transform& tf) {
    Quat inv_rot = QuatInverse(tf.rotation);
    Vec3 inv_trans = QuatRotateVec(inv_rot, tf.translation);
    Transform result;
    result.parent = tf.child;
    result.child = tf.parent;
    result.translation = {-inv_trans[0], -inv_trans[1], -inv_trans[2]};
    result.rotation = inv_rot;
    result.timestamp = tf.timestamp;
    return result;
}

// ======================================================================
// TFTree
// ======================================================================

TFTree::TFTree(double history_duration)
    : history_duration_(history_duration) {}

void TFTree::SetTransform(const Transform& tf) {
    EdgeKey key{tf.parent, tf.child};
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    auto& buf = edges_[key];
    buf.push_back(tf);

    // Update adjacency
    adjacency_[tf.parent].insert(tf.child);
    adjacency_[tf.child].insert(tf.parent);

    // Prune old
    double cutoff = tf.timestamp - history_duration_;
    while (!buf.empty() && buf.front().timestamp < cutoff) {
        buf.pop_front();
    }
}

Transform TFTree::Lookup(const std::string& target, const std::string& source,
                          double time) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    auto chain = FindChain(source, target);
    if (chain.empty()) {
        throw std::runtime_error("no transform chain from '" + source + "' to '" + target + "'");
    }

    Transform result{};
    bool has_result = false;
    for (size_t i = 0; i + 1 < chain.size(); ++i) {
        Transform edge_tf;
        if (!InterpolateEdge(chain[i], chain[i+1], time, &edge_tf)) {
            throw std::runtime_error("no data for edge (" + chain[i] + " -> " + chain[i+1] + ")");
        }
        if (!has_result) {
            result = edge_tf;
            has_result = true;
        } else {
            result = ComposeTransforms(result, edge_tf);
        }
    }
    if (!has_result) {
        throw std::runtime_error("empty chain");
    }
    return result;
}

bool TFTree::CanTransform(const std::string& target, const std::string& source) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return !FindChain(source, target).empty();
}

std::vector<std::string> TFTree::GetChain(const std::string& target,
                                            const std::string& source) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    auto chain = FindChain(source, target);
    if (chain.empty()) {
        throw std::runtime_error("no chain from '" + source + "' to '" + target + "'");
    }
    return chain;
}

std::vector<std::string> TFTree::AllFrames() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::vector<std::string> frames;
    frames.reserve(adjacency_.size());
    for (const auto& kv : adjacency_) {
        frames.push_back(kv.first);
    }
    std::sort(frames.begin(), frames.end());
    return frames;
}

// BFS from source to target
std::vector<std::string> TFTree::FindChain(const std::string& source,
                                            const std::string& target) const {
    if (source == target) return {source};
    if (adjacency_.find(source) == adjacency_.end() ||
        adjacency_.find(target) == adjacency_.end()) {
        return {};
    }

    std::unordered_map<std::string, std::string> parent_map;
    std::unordered_set<std::string> visited;
    visited.insert(source);
    std::queue<std::string> q;
    q.push(source);

    while (!q.empty()) {
        std::string node = q.front();
        q.pop();
        auto adj_it = adjacency_.find(node);
        if (adj_it == adjacency_.end()) continue;
        for (const auto& neighbour : adj_it->second) {
            if (visited.count(neighbour)) continue;
            visited.insert(neighbour);
            parent_map[neighbour] = node;
            if (neighbour == target) {
                // Reconstruct path
                std::vector<std::string> path;
                std::string cur = target;
                while (cur != source) {
                    path.push_back(cur);
                    cur = parent_map[cur];
                }
                path.push_back(source);
                std::reverse(path.begin(), path.end());
                return path;
            }
            q.push(neighbour);
        }
    }
    return {};
}

const Transform* TFTree::InterpolateEdge(const std::string& a, const std::string& b,
                                          double time, Transform* out) const {
    EdgeKey key_fwd{a, b};
    EdgeKey key_rev{b, a};

    auto it = edges_.find(key_fwd);
    if (it != edges_.end()) {
        return InterpolateBuf(it->second, time, out);
    }
    it = edges_.find(key_rev);
    if (it != edges_.end()) {
        if (InterpolateBuf(it->second, time, out)) {
            *out = InvertTransform(*out);
            return out;
        }
    }
    return nullptr;
}

const Transform* TFTree::InterpolateBuf(const std::deque<Transform>& buf,
                                          double time, Transform* out) const {
    if (buf.empty()) return nullptr;
    if (time <= 0.0) {
        *out = buf.back();
        return out;
    }

    const Transform* prev = nullptr;
    for (const auto& tf : buf) {
        if (tf.timestamp >= time) {
            if (prev == nullptr) {
                *out = tf;
                return out;
            }
            double span = tf.timestamp - prev->timestamp;
            if (span < 1e-12) {
                *out = tf;
                return out;
            }
            double t = (time - prev->timestamp) / span;
            out->parent = tf.parent;
            out->child = tf.child;
            out->translation = LerpVec(prev->translation, tf.translation, t);
            out->rotation = QuatSlerp(prev->rotation, tf.rotation, t);
            out->timestamp = time;
            return out;
        }
        prev = &tf;
    }
    // time is after last entry
    *out = buf.back();
    return out;
}

}  // namespace core
}  // namespace rynnrcp

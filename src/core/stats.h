#ifndef STATS_H
#define STATS_H

#include <atomic>
#include <cstdint>
#include <iostream>

struct BvhStats {
    std::atomic<std::uint64_t> rays_traced{0};
    std::atomic<std::uint64_t> aabb_tests{0};
    std::atomic<std::uint64_t> aabb_hits{0};
    std::atomic<std::uint64_t> bvh_node_visits{0};
    std::atomic<std::uint64_t> triangle_tests{0};
    std::atomic<std::uint64_t> triangle_hits{0};
    std::atomic<std::uint64_t> bvh_nodes_built{0};
};

inline BvhStats& bvh_stats() {
    static BvhStats s;
    return s;
}

inline void reset_bvh_stats() {
    auto& s = bvh_stats();
    s.rays_traced.store(0, std::memory_order_relaxed);
    s.aabb_tests.store(0, std::memory_order_relaxed);
    s.aabb_hits.store(0, std::memory_order_relaxed);
    s.bvh_node_visits.store(0, std::memory_order_relaxed);
    s.triangle_tests.store(0, std::memory_order_relaxed);
    s.triangle_hits.store(0, std::memory_order_relaxed);
    s.bvh_nodes_built.store(0, std::memory_order_relaxed);
}

inline void print_bvh_stats(std::ostream& os) {
    auto& s = bvh_stats();
    const auto rays = s.rays_traced.load(std::memory_order_relaxed);
    const auto aabbT = s.aabb_tests.load(std::memory_order_relaxed);
    const auto aabbH = s.aabb_hits.load(std::memory_order_relaxed);
    const auto nodeV = s.bvh_node_visits.load(std::memory_order_relaxed);
    const auto triT  = s.triangle_tests.load(std::memory_order_relaxed);
    const auto triH  = s.triangle_hits.load(std::memory_order_relaxed);
    const auto nodes = s.bvh_nodes_built.load(std::memory_order_relaxed);

    os << "\nBVH Stats:\n";
    os << "  Rays traced:         " << rays << "\n";
    os << "  BVH nodes built:     " << nodes << "\n";
    os << "  AABB tests:          " << aabbT << " (hits: " << aabbH << ")\n";
    os << "  BVH node visits:     " << nodeV << "\n";
    os << "  Triangle tests:      " << triT  << " (hits: " << triH  << ")\n";
}

#endif



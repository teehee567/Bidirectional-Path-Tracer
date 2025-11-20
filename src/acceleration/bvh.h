#ifndef BVH_H
#define BVH_H

#include "aabb.h"
#include "hittable.h"
#include "triangle.h"
#include "stats.h"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <vector>

// Flat BVH node storing a bounding box and either a triangle range (leaf)
// or a left child index (internal node). If amt == 0, the node is internal
// and idx is the index of the left child; the right child is idx + 1.
// If amt != 0, the node is a leaf and [idx, idx + amt) is the triangle range.
struct flat_bvh_node {
    aabb bbox;
    std::uint32_t idx = 0;
    std::uint32_t amt = 0;

    flat_bvh_node() = default;

    bool is_leaf() const { return amt != 0; }

    double area() const {
        // Returns the surface area of the bounding box (0 if empty).
        double dx = bbox.x.size();
        double dy = bbox.y.size();
        double dz = bbox.z.size();
        if (dx <= 0.0 || dy <= 0.0 || dz <= 0.0)
            return 0.0;
        return 2.0 * (dx * dy + dx * dz + dy * dz);
    }

    void expand(const triangle& tri) {
        bbox = aabb(bbox, tri.bounding_box());
    }

    void initialize(const std::vector<triangle>& triangles,
                    const std::vector<std::uint32_t>& indices,
                    std::size_t begin,
                    std::size_t end,
                    std::uint32_t offset) {
        bbox = aabb::empty;
        for (std::size_t i = begin; i < end; ++i) {
            const auto& tri = triangles[indices[i]];
            expand(tri);
        }
        idx = offset;
        amt = static_cast<std::uint32_t>(end - begin);
    }
};

class bvh_node : public hittable {
  public:
    bvh_node() = default;

    // Build a flat BVH from a triangle_collection. Triangles are copied into
    // an internal vector and reordered for cache-friendly traversal.
    explicit bvh_node(const triangle_collection& collection) {
        const auto& src = collection.triangles();
        triangles.assign(src.begin(), src.end());
        build();
    }

    bool hit(const ray& r, interval ray_t, hit_record& rec) const override {
        if (nodes.empty() || triangles.empty())
            return false;

        bool hit_anything = false;
        double closest = ray_t.max;
        hit_record temp_rec;

        // Simple explicit stack for depth-first traversal.
        std::vector<std::uint32_t> stack;
        stack.reserve(64);
        stack.push_back(0);

        while (!stack.empty()) {
            std::uint32_t node_index = stack.back();
            stack.pop_back();

            const auto& node = nodes[node_index];
            bvh_stats().bvh_node_visits.fetch_add(1, std::memory_order_relaxed);

            if (!node.bbox.hit(r, interval(ray_t.min, closest)))
                continue;

            if (node.is_leaf()) {
                std::uint32_t start = node.idx;
                std::uint32_t count = node.amt;
                for (std::uint32_t i = 0; i < count; ++i) {
                    const auto& tri = triangles[start + i];
                    if (tri.hit(r, interval(ray_t.min, closest), temp_rec)) {
                        hit_anything = true;
                        closest = temp_rec.t;
                        rec = temp_rec;
                    }
                }
            } else {
                std::uint32_t left  = node.idx;
                std::uint32_t right = left + 1;
                if (right < nodes.size())
                    stack.push_back(right);
                if (left < nodes.size())
                    stack.push_back(left);
            }
        }

        return hit_anything;
    }

    aabb bounding_box() const override {
        if (nodes.empty())
            return aabb::empty;
        return nodes[0].bbox;
    }

  private:
    static constexpr std::uint32_t BVH_MAX_DEPTH = 64;
    static constexpr int SPLIT_ATTEMPTS = 8;

    std::vector<triangle> triangles;
    std::vector<flat_bvh_node> nodes;

    void build() {
        nodes.clear();

        if (triangles.empty())
            return;

        std::vector<std::uint32_t> indices(triangles.size());
        for (std::uint32_t i = 0; i < indices.size(); ++i) {
            indices[i] = i;
        }

        nodes.emplace_back();
        bvh_stats().bvh_nodes_built.fetch_add(1, std::memory_order_relaxed);

        build_recursively(0, indices, 0, indices.size(), 0, 0, std::numeric_limits<double>::infinity());

        apply_ordering(indices);
    }

    void build_recursively(std::uint32_t node_index,
                           std::vector<std::uint32_t>& indices,
                           std::size_t begin,
                           std::size_t end,
                           std::uint32_t depth,
                           std::uint32_t offset,
                           double parent_cost) {
        nodes[node_index].initialize(triangles, indices, begin, end, offset);

        const std::size_t count = end - begin;
        if (depth >= BVH_MAX_DEPTH || count <= 1)
            return;

        auto [split_axis, split_pos] = find_best_split(node_index, indices, begin, end);
        double best_cost = split_cost(indices, begin, end, split_axis, split_pos);

        if (best_cost >= parent_cost)
            return;

        std::size_t left = begin;
        std::size_t right = end - 1;
        std::size_t left_count = 0;

        while (left <= right) {
            const triangle& tri = triangles[indices[left]];
            double center = triangle_center_axis(tri, split_axis);
            if (center < split_pos) {
                ++left;
                ++left_count;
            } else {
                std::swap(indices[left], indices[right]);
                if (right == 0)
                    break;
                --right;
            }
        }

        if (left_count == 0 || left_count == count)
            return;

        std::uint32_t left_node_index = static_cast<std::uint32_t>(nodes.size());
        nodes.emplace_back();
        nodes.emplace_back();
        bvh_stats().bvh_nodes_built.fetch_add(2, std::memory_order_relaxed);

        // Refresh reference after potential reallocation above.
        nodes[node_index].idx = left_node_index;
        nodes[node_index].amt = 0;

        std::size_t mid = begin + left_count;
        build_recursively(left_node_index, indices, begin, mid, depth + 1, offset, best_cost);
        build_recursively(left_node_index + 1, indices, mid, end, depth + 1,
                          offset + static_cast<std::uint32_t>(left_count), best_cost);
    }

    std::pair<int, double> find_best_split(std::uint32_t node_index,
                                           const std::vector<std::uint32_t>& indices,
                                           std::size_t begin,
                                           std::size_t end) const {
        int split_axis = 0;
        double split_pos = 0.0;
        double best_cost = std::numeric_limits<double>::infinity();

        const auto& node = nodes[node_index];
        point3 node_min(node.bbox.x.min, node.bbox.y.min, node.bbox.z.min);
        point3 node_max(node.bbox.x.max, node.bbox.y.max, node.bbox.z.max);
        vec3 dims = node_max - node_min;

        for (int axis = 0; axis < 3; ++axis) {
            double step = dims[axis] / static_cast<double>(SPLIT_ATTEMPTS + 1);
            for (int attempt = 1; attempt <= SPLIT_ATTEMPTS; ++attempt) {
                double pos = node_min[axis] + step * static_cast<double>(attempt);
                double cost = split_cost(indices, begin, end, axis, pos);
                if (cost < best_cost) {
                    best_cost = cost;
                    split_axis = axis;
                    split_pos = pos;
                }
            }
        }

        return {split_axis, split_pos};
    }

    double split_cost(const std::vector<std::uint32_t>& indices,
                      std::size_t begin,
                      std::size_t end,
                      int axis,
                      double location) const {
        std::size_t left_amount = 0;
        flat_bvh_node node_left;
        flat_bvh_node node_right;

        for (std::size_t i = begin; i < end; ++i) {
            const triangle& tri = triangles[indices[i]];
            double center = triangle_center_axis(tri, axis);
            if (center < location) {
                node_left.expand(tri);
                ++left_amount;
            } else {
                node_right.expand(tri);
            }
        }

        std::size_t right_amount = (end - begin) - left_amount;
        return static_cast<double>(left_amount) * node_left.area()
             + static_cast<double>(right_amount) * node_right.area();
    }

    static double triangle_center_axis(const triangle& tri, int axis) {
        aabb box = tri.bounding_box();
        const interval& ax = box.axis_interval(axis);
        return 0.5 * (ax.min + ax.max);
    }

    void apply_ordering(const std::vector<std::uint32_t>& ordering) {
        std::vector<triangle> sorted;
        sorted.reserve(triangles.size());
        for (auto idx : ordering) {
            sorted.push_back(triangles[static_cast<std::size_t>(idx)]);
        }
        triangles.swap(sorted);
    }
};

#endif
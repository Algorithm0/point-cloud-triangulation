#pragma once

#include "geometry.h"
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>

class KDTree {
public:
    void build(const PointCloud& points);

    void searchKNN(
        size_t point_index,
        int k,
        std::vector<size_t>& out_indices) const;

    void radiusSearch(
        const Point& target,
        double radius,
        size_t ignore_index,
        std::vector<size_t>& out_indices) const;

    // Удобный доступ к точке по индексу
    const Point& point(size_t idx) const {
        return (*points_)[idx];
    }

private:
    static constexpr size_t INVALID_NODE = std::numeric_limits<size_t>::max();

    struct Node {
        size_t point_idx;
        int split_axis;
        size_t left_idx = INVALID_NODE;
        size_t right_idx = INVALID_NODE;
        bool is_leaf = false;
    };

    const PointCloud* points_ = nullptr;
    std::vector<Node> nodes_;

    size_t buildRecursive(std::vector<size_t>& indices, int depth);

    void searchRecursive(
        size_t node_idx,
        const Point& target,
        size_t ignore_index,
        int k,
        std::priority_queue<std::pair<double, size_t>>& best_neighbors) const;

    void radiusSearchRecursive(
        size_t node_idx,
        const Point& target,
        double radius_squared,
        size_t ignore_index,
        std::vector<size_t>& result) const;
};
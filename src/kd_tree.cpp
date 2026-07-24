#include "kd_tree.h"
#include <algorithm>
#include <queue>
#include <cmath>
#include <utility>

void KDTree::build(const PointCloud& points) {
    points_ = &points;
    nodes_.clear();
    nodes_.reserve(points.size());

    if (points.empty()) return;

    std::vector<size_t> indices(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        indices[i] = i;
    }

    buildRecursive(indices, 0);
}

size_t KDTree::buildRecursive(std::vector<size_t>& indices, int depth) {
    if (indices.empty()) return 0;
    int axis = depth % 3;
    size_t median_idx = indices.size() / 2;
    std::nth_element(indices.begin(), indices.begin() + median_idx, indices.end(),
        [this, axis](size_t a, size_t b) {
            const auto& pa = (*points_)[a];
            const auto& pb = (*points_)[b];
            if (axis == 0) return pa.x < pb.x;
            if (axis == 1) return pa.y < pb.y;
            return pa.z < pb.z;
        });

    size_t node_idx = nodes_.size();
    nodes_.push_back({indices[median_idx], axis, 0, 0, false});

    std::vector<size_t> left_indices(indices.begin(), indices.begin() + median_idx);
    std::vector<size_t> right_indices(indices.begin() + median_idx + 1, indices.end());

    if (left_indices.empty() && right_indices.empty()) {
        nodes_[node_idx].is_leaf = true;
        return node_idx;
    }

    if (!left_indices.empty()) {
        nodes_[node_idx].left_idx = buildRecursive(left_indices, depth + 1);
    }
    if (!right_indices.empty()) {
        nodes_[node_idx].right_idx = buildRecursive(right_indices, depth + 1);
    }

    return node_idx;
}

void KDTree::searchKNN(const Point& target, int k, std::vector<size_t>& out_indices) const {
    out_indices.clear();
    if (nodes_.empty()) return;

    std::priority_queue<std::pair<double, size_t>> best_neighbors;

    searchRecursive(0, target, k, best_neighbors);

    out_indices.reserve(best_neighbors.size());
    while (!best_neighbors.empty()) {
        out_indices.push_back(best_neighbors.top().second);
        best_neighbors.pop();
    }
}

void KDTree::searchRecursive(size_t node_idx, const Point& target, int k, 
                             std::priority_queue<std::pair<double, size_t>>& best_neighbors) const {
    const Node& node = nodes_[node_idx];
    const Point& p = (*points_)[node.point_idx];

    double dx = target.x - p.x;
    double dy = target.y - p.y;
    double dz = target.z - p.z;
    double dist_sq = dx * dx + dy * dy + dz * dz;

    if (best_neighbors.size() < (size_t)k) {
        best_neighbors.push({dist_sq, node.point_idx});
    } else if (dist_sq < best_neighbors.top().first) {
        best_neighbors.pop();
        best_neighbors.push({dist_sq, node.point_idx});
    }

    if (node.is_leaf) return;

    double diff;
    if (node.split_axis == 0) diff = target.x - p.x;
    else if (node.split_axis == 1) diff = target.y - p.y;
    else diff = target.z - p.z;

    size_t first_idx = (diff <= 0) ? node.left_idx : node.right_idx;
    size_t second_idx = (diff <= 0) ? node.right_idx : node.left_idx;

    if (first_idx != 0) {
        searchRecursive(first_idx, target, k, best_neighbors);
    }

    if (second_idx != 0) {
        double dist_to_plane_sq = diff * diff;
        if (best_neighbors.size() < (size_t)k || dist_to_plane_sq < best_neighbors.top().first) {
            searchRecursive(second_idx, target, k, best_neighbors);
        }
    }
}
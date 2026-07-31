#include "kd_tree.h"

void KDTree::build(const PointCloud& points) {
    points_ = &points;
    nodes_.clear();
    if (points.empty()) return;

    std::vector<size_t> indices(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        indices[i] = i;
    }
    buildRecursive(indices, 0);
}

size_t KDTree::buildRecursive(std::vector<size_t>& indices, int depth) {
    if (indices.empty()) {
        return INVALID_NODE;
    }

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
    Node node;
    node.point_idx = indices[median_idx];
    node.split_axis = axis;
    nodes_.push_back(node);

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

void KDTree::searchKNN(size_t point_index, int k, std::vector<size_t>& out_indices) const {
    out_indices.clear();
    if (!points_ || points_->empty() || nodes_.empty()) return;

    const Point& target = (*points_)[point_index];
    std::priority_queue<std::pair<double, size_t>> best_neighbors;

    searchRecursive(0, target, point_index, k, best_neighbors);

    out_indices.reserve(best_neighbors.size());
    while (!best_neighbors.empty()) {
        out_indices.push_back(best_neighbors.top().second);
        best_neighbors.pop();
    }
    
    std::reverse(out_indices.begin(), out_indices.end());
}

void KDTree::searchRecursive(
    size_t node_idx,
    const Point& target,
    size_t ignore_index,
    int k,
    std::priority_queue<std::pair<double, size_t>>& best_neighbors) const {

    const Node& node = nodes_[node_idx];
    const Point& p = (*points_)[node.point_idx];

    if (node.point_idx != ignore_index) {
        double dist_sq = distanceSquared(target, p);

        if (best_neighbors.size() < (size_t)k) {
            best_neighbors.push({dist_sq, node.point_idx});
        } else if (dist_sq < best_neighbors.top().first) {
            best_neighbors.pop();
            best_neighbors.push({dist_sq, node.point_idx});
        }
    }

    if (node.is_leaf) return;

    double diff;
    if (node.split_axis == 0) diff = target.x - p.x;
    else if (node.split_axis == 1) diff = target.y - p.y;
    else diff = target.z - p.z;

    size_t first_idx = (diff <= 0) ? node.left_idx : node.right_idx;
    size_t second_idx = (diff <= 0) ? node.right_idx : node.left_idx;

    if (first_idx != INVALID_NODE) {
        searchRecursive(first_idx, target, ignore_index, k, best_neighbors);
    }

    if (second_idx != INVALID_NODE) {
        double dist_to_plane_sq = diff * diff;
        if (best_neighbors.size() < (size_t)k || dist_to_plane_sq < best_neighbors.top().first) {
            searchRecursive(second_idx, target, ignore_index, k, best_neighbors);
        }
    }
}

void KDTree::radiusSearch(
    const Point& target, 
    double radius, 
    size_t ignore_index, 
    std::vector<size_t>& out_indices) const {

    out_indices.clear();
    if (!points_ || points_->empty() || nodes_.empty()) return;

    double radius_squared = radius * radius;
    radiusSearchRecursive(0, target, radius_squared, ignore_index, out_indices);
}

void KDTree::radiusSearchRecursive(
    size_t node_idx,
    const Point& target,
    double radius_squared,
    size_t ignore_index,
    std::vector<size_t>& result) const 
{
    const Node& node = nodes_[node_idx];
    const Point& p = (*points_)[node.point_idx];

    if (node.point_idx != ignore_index) {
        double dist_sq = distanceSquared(target, p);
        if (dist_sq <= radius_squared) {
            result.push_back(node.point_idx);
        }
    }

    if (node.is_leaf) return;

    double diff;
    if (node.split_axis == 0) diff = target.x - p.x;
    else if (node.split_axis == 1) diff = target.y - p.y;
    else diff = target.z - p.z;

    size_t first_idx = (diff <= 0) ? node.left_idx : node.right_idx;
    size_t second_idx = (diff <= 0) ? node.right_idx : node.left_idx;

    if (first_idx != INVALID_NODE) {
        radiusSearchRecursive(first_idx, target, radius_squared, ignore_index, result);
    }

    if (second_idx != INVALID_NODE) {
        double dist_to_plane_sq = diff * diff;
        if (dist_to_plane_sq <= radius_squared) {
            radiusSearchRecursive(second_idx, target, radius_squared, ignore_index, result);
        }
    }
}

void KDTree::radiusSearch(
    const Vector3& target, 
    double radius, 
    size_t ignore_index, 
    std::vector<size_t>& out_indices) const {
        
    Point center_pt(target.x, target.y, target.z);
    radiusSearch(center_pt, radius, ignore_index, out_indices);
}
#pragma once

#include "geometry.h"
#include <vector>
#include <queue> 

class KDTree {
public:
    void build(const PointCloud& points);
    void searchKNN(const Point& target_point, int k, std::vector<size_t>& out_indices) const;

private:
    struct Node {
        size_t point_idx; 
        int split_axis;
        size_t left_idx;
        size_t right_idx;
        bool is_leaf;
    };

    std::vector<Node> nodes_;
    const PointCloud* points_ = nullptr;

    size_t buildRecursive(std::vector<size_t>& indices, int depth);
    
    void searchRecursive(size_t node_idx, const Point& target, int k, 
                         std::priority_queue<std::pair<double, size_t>>& best_neighbors) const;
};
#pragma once

#include "geometry.h"
#include "kd_tree.h"

class NormalEstimator {
public:
    static void estimate(PointCloud& points, const KDTree& kd_tree, int k_neighbors, int num_treads);
};
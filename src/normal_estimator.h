#pragma once

#include "geometry.h"

class NormalEstimator {
public:
    static void estimate(PointCloud& points, int k_neighbors, int num_treads);
};
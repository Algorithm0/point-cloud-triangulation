#include "normal_estimator.h"
#include "kd_tree.h"
#include "eigen33.h"
#include <cmath>
#include <iostream>
#include <omp.h>

void NormalEstimator::estimate(PointCloud& points, int k_neighbors, int num_threads) {
    if (points.empty() || k_neighbors < 3) {
        std::cerr << "Warning: Not enough points or neighbors for PCA." << std::endl;
        return;
    }

    if (num_threads > 0) {
        omp_set_num_threads(num_threads);
    }

    int k = std::min(k_neighbors, (int)points.size() - 1);

    KDTree tree;
    tree.build(points);

    #pragma omp parallel for schedule(dynamic)
    for (size_t i = 0; i < points.size(); ++i) {
        std::vector<size_t> neighbors;
        tree.searchKNN(points[i], k, neighbors);

        double cx = 0, cy = 0, cz = 0;
        int count = k + 1;
        
        cx += points[i].x; cy += points[i].y; cz += points[i].z;
        for (size_t idx : neighbors) {
            cx += points[idx].x;
            cy += points[idx].y;
            cz += points[idx].z;
        }
        cx /= count; cy /= count; cz /= count;

        double C[3][3] = {{0,0,0}, {0,0,0}, {0,0,0}};

        auto addToCovariance = [&](const Point& p) {
            double dx = p.x - cx;
            double dy = p.y - cy;
            double dz = p.z - cz;
            C[0][0] += dx * dx; C[0][1] += dx * dy; C[0][2] += dx * dz;
            C[1][0] += dy * dx; C[1][1] += dy * dy; C[1][2] += dy * dz;
            C[2][0] += dz * dx; C[2][1] += dz * dy; C[2][2] += dz * dz;
        };

        addToCovariance(points[i]);
        for (size_t idx : neighbors) {
            addToCovariance(points[idx]);
        }

        for(int r=0; r<3; ++r)
            for(int c=0; c<3; ++c)
                C[r][c] /= count;

        double eigenvalues[3];
        double eigenvectors[3][3];
        computeEigenVectors3x3(C, eigenvalues, eigenvectors);

        double nx = eigenvectors[0][0];
        double ny = eigenvectors[1][0];
        double nz = eigenvectors[2][0];

        double length = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (length > 1e-8) {
            nx /= length; ny /= length; nz /= length;
        }

        if (nz < 0) {
            nx = -nx; ny = -ny; nz = -nz;
        }

        points[i].nx = nx;
        points[i].ny = ny;
        points[i].nz = nz;
    }

    std::cout << "Normal estimation completed for " << points.size() << " points." << std::endl;
}
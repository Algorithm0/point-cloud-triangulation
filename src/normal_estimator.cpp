#include "normal_estimator.h"
#include "kd_tree.h"
#include "eigen33.h"
#include <cmath>
#include <iostream>
#include <omp.h>
#include <ranges>
#include <iterator>

namespace NormalEstimator {
    void estimate(PointCloud& points, const KDTree& kd_tree,
            int k_neighbors, int num_threads) {
        if (points.empty() || k_neighbors < 3) {
            std::cerr << "Warning: Not enough points or neighbors for PCA." << std::endl;
            return;
        }

        if (num_threads > 0) {
            omp_set_num_threads(num_threads);
        }

        int k = std::min(k_neighbors, static_cast<int>(points.size()) - 1);

        double cx = 0, cy = 0, cz = 0;
        for (const auto& p : points) {
            cx += p.x; cy += p.y; cz += p.z;
        }
        cx /= points.size(); 
        cy /= points.size(); 
        cz /= points.size();

        #pragma omp parallel
        {
            std::vector<size_t> neighbors;
            neighbors.reserve(k);

            #pragma omp for schedule(dynamic)
            for (int i = 0; i < static_cast<int>(points.size()); ++i) {
                neighbors.clear();
                
                kd_tree.searchKNN(i, k, neighbors);

                int count = static_cast<int>(neighbors.size()) + 1;

                double local_cx = points[i].x;
                double local_cy = points[i].y;
                double local_cz = points[i].z;
                
                for (size_t idx : neighbors) {
                    local_cx += points[idx].x;
                    local_cy += points[idx].y;
                    local_cz += points[idx].z;
                }
                local_cx /= count;
                local_cy /= count;
                local_cz /= count;

                double C[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

                auto addToCovariance = [&](const Point& p) {
                    double dx = p.x - local_cx;
                    double dy = p.y - local_cy;
                    double dz = p.z - local_cz;
                    C[0][0] += dx * dx; C[0][1] += dx * dy; C[0][2] += dx * dz;
                    C[1][0] += dy * dx; C[1][1] += dy * dy; C[1][2] += dy * dz;
                    C[2][0] += dz * dx; C[2][1] += dz * dy; C[2][2] += dz * dz;
                };

                addToCovariance(points[i]);
                for (size_t idx : neighbors) {
                    addToCovariance(points[idx]);
                }

                for (int r = 0; r < 3; ++r) {
                    for (int c = 0; c < 3; ++c) {
                        C[r][c] /= count;
                    }
                }

                double eigenvalues[3];
                double eigenvectors[3][3];

                computeEigenVectors3x3(C, eigenvalues, eigenvectors); 

                const auto minIt = std::ranges::min_element(eigenvalues);
                const int minIdx = static_cast<int>(minIt - std::begin(eigenvalues));

                Vector3 normal(
                    eigenvectors[0][minIdx],
                    eigenvectors[1][minIdx],
                    eigenvectors[2][minIdx]
                );

                double len = norm(normal);

                if (len < 1e-10) {
                    points[i].nx = 0.0;
                    points[i].ny = 0.0;
                    points[i].nz = 1.0;
                    continue;
                }

                normal = normal / len;

                Vector3 to_centroid(points[i].x - cx, points[i].y - cy, points[i].z - cz);
                
                if (dot(normal, to_centroid) < 0.0) {
                    normal = -normal;
                }

                points[i].nx = normal.x;
                points[i].ny = normal.y;
                points[i].nz = normal.z;
            }
        }

        std::cout << "Normal estimation completed for " << points.size() << " points." << std::endl;
    }
}
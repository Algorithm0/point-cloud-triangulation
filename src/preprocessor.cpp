#include "preprocessor.h"
#include <iostream>
#include <algorithm>
#include <vector>
#include <numeric>
#include <ranges>

namespace Preprocessor {
    PointCloud getUniquePoints(const PointCloud& points, double tolerance) {
        if (points.empty()) {
            return {};
        }

        std::vector<size_t> indices(points.size());
        std::iota(indices.begin(), indices.end(), size_t{0});

        std::ranges::sort(indices, [&](size_t a, size_t b) {
            if (points[a].x != points[b].x) return points[a].x < points[b].x;
            if (points[a].y != points[b].y) return points[a].y < points[b].y;
            return points[a].z < points[b].z;
        });
        std::vector<bool> is_duplicate(points.size(), false);
        const double tolerance_sq = tolerance * tolerance;

        for (size_t i = 1; i < indices.size(); ++i) {
            size_t curr_idx = indices[i];
            const auto& p_curr = points[curr_idx];

            for (size_t j = i; j > 0; --j) {
                size_t prev_idx = indices[j - 1];
                const auto& p_prev = points[prev_idx];

                if (p_curr.x - p_prev.x > tolerance) {
                    break; 
                }

                double dx = p_curr.x - p_prev.x;
                double dy = p_curr.y - p_prev.y;
                double dz = p_curr.z - p_prev.z;
                double dist_sq = dx * dx + dy * dy + dz * dz;

                if (dist_sq <= tolerance_sq) {
                    is_duplicate[curr_idx] = true;
                    break;
                }
            }
        }

        auto unique_indices = indices
            | std::views::filter([&](size_t i){
                return !is_duplicate[i];
            });

        PointCloud unique_points;
        unique_points.reserve(points.size());
        for (size_t i : unique_indices) {
            unique_points.push_back(points[i]);
        }

        size_t removed_count = points.size() - unique_points.size();
        if (removed_count > 0) {
            std::cout << "Preprocessing: Found " << removed_count 
                << " duplicate points. Algorithm will use " 
                << unique_points.size() << " unique points." << std::endl;
        }

        return unique_points;
    }
}
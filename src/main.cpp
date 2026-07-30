#include <iostream>
#include <chrono>
#include "file_parser.h"
#include "config.h"
#include "preprocessor.h"
#include "normal_estimator.h"
#include "kd_tree.h"
#include "ball_pivoting.h"
#include <unordered_map>
#include <utility>


int main() {
    auto start_time = std::chrono::high_resolution_clock::now();
    Config config = Config::loadFromFile("config.json");

    FileParser parser;
    PointCloud all_points = parser.readXYZ("sphere.xyz");
    //PointCloud all_points = parser.readXYZ("sphere-nodes.xyz");
    //PointCloud all_points = parser.readXYZ("saddle-nodes.xyz");
    //PointCloud all_points = parser.readXYZ("barrel-nodes.xyz");
    //PointCloud all_points = parser.readXYZ("input.xyz");

    PointCloud unique_points = Preprocessor::getUniquePoints(all_points, config.algorithm.duplicate_tolerance);

    KDTree kd_tree;
    kd_tree.build(unique_points);

    NormalEstimator::estimate(unique_points, kd_tree, config.algorithm.pca_neighbors, config.performance.threads);

    TriangleMesh mesh = BallPivoting::reconstruct(
        unique_points,
        kd_tree,
        config.algorithm.radius_multiplier,
        config.algorithm.max_edge_multiplier,
        config.algorithm.max_retries
    );

    parser.writeMesh("output.txt", all_points, unique_points, mesh);

    if (config.output.export_obj) {
        parser.writeOBJ("output.obj", unique_points, mesh);
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;
    std::cout << "Total execution time: " << std::fixed << std::setprecision(3) 
              << elapsed.count() << " seconds." << std::endl;
    
    return 0;
}
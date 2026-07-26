#include <iostream>
#include <chrono>
#include "file_parser.h"
#include "config.h"
#include "preprocessor.h"
#include "normal_estimator.h"
#include "kd_tree.h"

int main() {
    auto start_time = std::chrono::high_resolution_clock::now();
    Config config = Config::loadFromFile("config.json");

    FileParser parser;
    PointCloud all_points = parser.readXYZ("input.xyz");

    PointCloud unique_points = Preprocessor::getUniquePoints(all_points, config.algorithm.duplicate_tolerance);

    KDTree kd_tree;
    kd_tree.build(unique_points);

    NormalEstimator::estimate(unique_points, kd_tree, config.algorithm.pca_neighbors, config.performance.threads);
    if (!unique_points.empty()) {
        std::cout << "Normal for point 1: [" 
                    << unique_points[0].nx << ", " 
                    << unique_points[0].ny << ", " 
                    << unique_points[0].nz << "]" << std::endl;
    }

    TriangleMesh empty_mesh;

    parser.writeMesh("output.txt", all_points, empty_mesh);

    if (config.output.export_obj) {
        parser.writeOBJ("output.obj", unique_points, empty_mesh);
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;
    std::cout << "Total execution time: " << std::fixed << std::setprecision(3) 
              << elapsed.count() << " seconds." << std::endl;
    
    return 0;
}
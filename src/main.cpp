#include <iostream>
#include "file_parser.h"
#include "config.h"
#include "preprocessor.h"
#include "normal_estimator.h"

int main() {
    Config config = Config::loadFromFile("config.json");

    FileParser parser;
    PointCloud all_points = parser.readXYZ("input.xyz");
 //   PointCloud all_points = parser.readXYZ("barrel-nodes.xyz");

    PointCloud unique_points = Preprocessor::getUniquePoints(all_points, config.algorithm.duplicate_tolerance);

    NormalEstimator::estimate(unique_points, config.algorithm.pca_neighbors, config.performance.threads);
    if (!unique_points.empty()) {
        std::cout << "Normal for point 1: [" 
                    << unique_points[0].nx << ", " 
                    << unique_points[0].ny << ", " 
                    << unique_points[0].nz << "]" << std::endl;
    }

    TriangleMesh empty_mesh;
    parser.writeMesh("output.txt", all_points, empty_mesh);
    
    return 0;
}
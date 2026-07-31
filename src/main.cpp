#include <iostream>
#include <chrono>
#include <unordered_map>
#include <utility>
#include "file_parser.h"
#include "config.h"
#include "preprocessor.h"
#include "normal_estimator.h"
#include "kd_tree.h"
#include "ball_pivoting.h"
#include "cli_args.h"

// commands to run:
// .\build\triangulator.exe -i sphere.xyz -o sphere-output.txt
// .\build\triangulator.exe -i sphere-nodes.xyz -o sphere-nodes-output.txt
// .\build\triangulator.exe -i saddle-nodes.xyz -o saddle-nodes-output.txt
// .\build\triangulator.exe -i barrel-nodes.xyz -o barrel-nodes-output.txt

int main(int argc, char* argv[]) {
    try {
        auto args = CliArgs::parse(argc, argv);
        auto start_time = std::chrono::high_resolution_clock::now();
        Config config = Config::loadFromFile("args.config.json");

        FileParser parser;
        PointCloud all_points = parser.readXYZ(args.input_file);

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

        parser.writeMesh(args.output_file, all_points, unique_points, mesh);

        if (config.output.export_obj) {
            std::string obj_file = args.output_file;
            auto dot_pos = obj_file.rfind('.');
            if (dot_pos != std::string::npos) {
                obj_file = obj_file.substr(0, dot_pos) + ".obj";
            } else {
                obj_file += ".obj";
            }
            parser.writeOBJ(obj_file, unique_points, mesh);
        }if (config.output.export_obj) {
            parser.writeOBJ("output.obj", unique_points, mesh);
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end_time - start_time;
        std::cout << "Total execution time: " << std::fixed << std::setprecision(3) 
                << elapsed.count() << " seconds." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
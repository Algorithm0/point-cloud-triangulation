#define CATCH_CONFIG_MAIN
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/catch_approx.hpp> 
#include <fstream>        

#include "../src/geometry.h"     
#include "../src/preprocessor.h"
#include "../src/kd_tree.h"
#include "../src/eigen33.h"
#include "../src/file_parser.h"

#include <vector>
#include <cmath>

using Catch::Approx;

TEST_CASE("Preprocessor removes duplicate points within tolerance", "[preprocessor]") {
    PointCloud points;
    points.push_back({0.0, 0.0, 0.0, 1});
    points.push_back({0.0001, 0.0001, 0.0001, 2});
    points.push_back({1.0, 1.0, 1.0, 3});

    double tolerance = 0.01;
    PointCloud unique_points = Preprocessor::getUniquePoints(points, tolerance);

    REQUIRE(unique_points.size() == 2);
    
    bool found_origin = false;
    bool found_far = false;
    for (const auto& p : unique_points) {
        if (distanceSquared(p, Point{0.0, 0.0, 0.0, 0}) < 0.001) found_origin = true;
        if (distanceSquared(p, Point{1.0, 1.0, 1.0, 0}) < 0.001) found_far = true;
    }
    REQUIRE(found_origin);
    REQUIRE(found_far);
}


TEST_CASE("KDTree finds correct nearest neighbors", "[kd_tree]") {
    PointCloud points = {
        {0.0, 0.0, 0.0, 1},
        {1.0, 0.0, 0.0, 2},
        {0.0, 1.0, 0.0, 3},
        {10.0, 10.0, 10.0, 4}
    };

    KDTree tree;
    tree.build(points);

    SECTION("KNN search ignores the query point itself") {
        std::vector<size_t> neighbors;
        tree.searchKNN(0, 2, neighbors);

        REQUIRE(neighbors.size() == 2);
        REQUIRE((neighbors[0] == 1 || neighbors[0] == 2));
        REQUIRE((neighbors[1] == 1 || neighbors[1] == 2));
        REQUIRE(neighbors[0] != neighbors[1]);
    }

    SECTION("Radius search finds all points within radius") {
        std::vector<size_t> neighbors;
        Point target{0.0, 0.0, 0.0, 0};
        tree.radiusSearch(target, 1.5, std::numeric_limits<size_t>::max(), neighbors);

        REQUIRE(neighbors.size() == 3);
        for (size_t idx : neighbors) {
            REQUIRE(idx != 3);
        }
    }
}

TEST_CASE("computeEigenVectors3x3 solves diagonal matrix correctly", "[eigen33]") {
    double A[3][3] = {
        {2.0, 0.0, 0.0},
        {0.0, 3.0, 0.0},
        {0.0, 0.0, 4.0}
    };

    double eigenvalues[3];
    double eigenvectors[3][3];

    computeEigenVectors3x3(A, eigenvalues, eigenvectors);

    REQUIRE(eigenvalues[0] == Approx(2.0).margin(1e-6));
    REQUIRE(eigenvalues[1] == Approx(3.0).margin(1e-6));
    REQUIRE(eigenvalues[2] == Approx(4.0).margin(1e-6));
}

TEST_CASE("FileParser correctly parses XYZ line", "[file_parser]") {
    
    std::string test_file = "temp_test.xyz";
    std::ofstream out(test_file);
    out << "* This is a comment\n";
    out << "1, 0.5, 1.5, 2.5\n";
    out << "invalid line should be skipped\n";
    out << "2, 3.0, 4.0, 5.0\n";
    out.close();

    PointCloud points = FileParser::readXYZ(test_file);

    REQUIRE(points.size() == 2);
    REQUIRE(points[0].node_id == 1);
    REQUIRE(points[0].x == Approx(0.5));
    REQUIRE(points[0].y == Approx(1.5));
    REQUIRE(points[0].z == Approx(2.5));
    
    REQUIRE(points[1].node_id == 2);
    REQUIRE(points[1].x == Approx(3.0));

    std::remove(test_file.c_str());
}
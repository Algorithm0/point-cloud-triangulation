#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "../src/geometry.h"
#include "../src/preprocessor.h"
#include "../src/kd_tree.h"
#include "../src/normal_estimator.h"
#include "../src/file_parser.h"
#include "../src/ball_pivoting.h"

#include <vector>
#include <cmath>
#include <algorithm>

using Catch::Approx;

TEST_CASE("NormalEstimator calculates correct normals for a flat plane", "[integration][normals]") {
    PointCloud points = {
        {0.0, 0.0, 0.0, 1},
        {1.0, 0.0, 0.0, 2},
        {1.0, 1.0, 0.0, 3},
        {0.0, 1.0, 0.0, 4}
    };

    KDTree kd_tree;
    kd_tree.build(points);

    NormalEstimator::estimate(points, kd_tree, 3, 1);

    for (const auto& p : points) {
        REQUIRE(std::abs(p.nz) == Approx(1.0).margin(1e-5));
        REQUIRE(std::abs(p.nx) == Approx(0.0).margin(1e-5));
        REQUIRE(std::abs(p.ny) == Approx(0.0).margin(1e-5));
    }
}

TEST_CASE("BPA reconstructs a simple square into 2 triangles", "[integration][bpa]") {
    PointCloud points = {
        {0.0, 0.0, 0.0, 1},
        {1.0, 0.0, 0.0, 2},
        {1.0, 1.0, 0.0, 3},
        {0.0, 1.0, 0.0, 4}
    };

    PointCloud unique_points = Preprocessor::getUniquePoints(points, 1e-6);
    REQUIRE(unique_points.size() == 4);

    KDTree kd_tree;
    kd_tree.build(unique_points);

    NormalEstimator::estimate(unique_points, kd_tree, 3, 1);

    double radius_multiplier = 1.0; 
    double max_edge_multiplier = 2.0;
    
    TriangleMesh mesh = BallPivoting::reconstruct(
        unique_points, 
        kd_tree, 
        radius_multiplier, 
        max_edge_multiplier, 
        1
    );

    SECTION("Mesh is not empty") {
        REQUIRE(mesh.size() > 0);
    }

    SECTION("Mesh forms exactly 2 triangles (a quad split in two)") {
        REQUIRE(mesh.size() == 2);
    }

    SECTION("All 4 points are used in the mesh") {
        std::vector<bool> used(4, false);
        for (const auto& tri : mesh) {
            used[tri.v1] = true;
            used[tri.v2] = true;
            used[tri.v3] = true;
        }
        for (int i = 0; i < 4; ++i) {
            REQUIRE(used[i]);
        }
    }
    
    SECTION("Mesh has consistent unique triangles") {
        std::vector<std::array<size_t, 3>> sorted_tris;
        for (const auto& t : mesh) {
            auto key = makeCanonicalKey(t.v1, t.v2, t.v3);
            sorted_tris.push_back(key);
        }
        std::sort(sorted_tris.begin(), sorted_tris.end());
        REQUIRE(std::unique(sorted_tris.begin(), sorted_tris.end()) == sorted_tris.end());
    }
}

TEST_CASE("BPA reconstructs cube (cube.xyz) with exactly 12 triangles", "[integration][regression]") {
    PointCloud points;
    
    std::string cube_path = std::string(TEST_DATA_DIR) + "/cube.xyz";
    REQUIRE_NOTHROW(points = FileParser::readXYZ(cube_path));
    REQUIRE(points.size() > 0);

    PointCloud unique_points = Preprocessor::getUniquePoints(points, 1e-6);
    REQUIRE(unique_points.size() == 8);

    KDTree kd_tree;
    kd_tree.build(unique_points);

    NormalEstimator::estimate(unique_points, kd_tree, 15, 2);

    double radius_multiplier = 0.8;   
    double max_edge_multiplier = 1.5;
    int max_retries = 3;
    
    TriangleMesh mesh = BallPivoting::reconstruct(
        unique_points, 
        kd_tree, 
        radius_multiplier, 
        max_edge_multiplier, 
        max_retries
    );

    SECTION("Cube produces exactly 12 triangles (2 per face)") {
        REQUIRE(mesh.size() == 12);
    }
    
    SECTION("All 8 vertices of the cube are used") {
        std::vector<bool> used(8, false);
        for (const auto& tri : mesh) {
            used[tri.v1] = true;
            used[tri.v2] = true;
            used[tri.v3] = true;
        }
        for (int i = 0; i < 8; ++i) {
            REQUIRE(used[i]);
        }
    }
}


TEST_CASE("BPA reconstructs sphere.xyz with exact expected triangle count", "[integration][regression]") {
    PointCloud points;
    
    std::string sphere_path = std::string(TEST_DATA_DIR) + "/sphere.xyz";
    REQUIRE_NOTHROW(points = FileParser::readXYZ(sphere_path));
    REQUIRE(points.size() > 0);

    PointCloud unique_points = Preprocessor::getUniquePoints(points, 1e-6);

    KDTree kd_tree;
    kd_tree.build(unique_points);

    NormalEstimator::estimate(unique_points, kd_tree, 15, 8);

    double radius_multiplier = 1.5;
    double max_edge_multiplier = 2.5;
    int max_retries = 3;
    
    TriangleMesh mesh = BallPivoting::reconstruct(
        unique_points, 
        kd_tree, 
        radius_multiplier, 
        max_edge_multiplier, 
        max_retries
    );

    SECTION("Triangle count matches the golden master") {
        REQUIRE(mesh.size() == 19996);
    }
}
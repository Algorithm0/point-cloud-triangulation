#pragma once

#include "geometry.h"
#include "kd_tree.h"
#include <vector>
#include <deque>
#include <unordered_set>
#include <unordered_map>
#include <limits>
#include <cmath>
#include <array>

class BallPivoting {
public:
    static constexpr size_t INVALID_INDEX = std::numeric_limits<size_t>::max();

    struct Edge {
        size_t v1 = 0;
        size_t v2 = 0;
        size_t opposite_vertex = INVALID_INDEX;
        Vector3 ball_center;
        int retry_count = 0; 

        Edge() = default;
        Edge(size_t a, size_t b, size_t opp = INVALID_INDEX, const Vector3& center = {})
            : v1(a), v2(b), opposite_vertex(opp), ball_center(center), retry_count(0) {}

        bool operator==(const Edge& other) const noexcept {
            return v1 == other.v1 && v2 == other.v2;
        }
    };

    struct EdgeHash {
        size_t operator()(const Edge& e) const noexcept {
            size_t h1 = std::hash<size_t>{}(e.v1);
            size_t h2 = std::hash<size_t>{}(e.v2);
            return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
        }
    };

    struct TriangleKeyHash {
        size_t operator()(const std::array<size_t, 3>& arr) const noexcept {
            return std::hash<size_t>()(arr[0]) ^ 
            (std::hash<size_t>()(arr[1]) << 1) ^ (std::hash<size_t>()(arr[2]) << 2);
        }
    };

    struct PairHash {
        size_t operator()(const std::pair<size_t, size_t>& p) const noexcept {
            return std::hash<size_t>()(p.first) ^ (std::hash<size_t>()(p.second) << 1);
        }
    };

    using FrontSet = std::unordered_set<Edge, EdgeHash>;
    using EdgeUsageMap = std::unordered_map<std::pair<size_t, size_t>, int, PairHash>;

    static TriangleMesh reconstruct(
        const PointCloud& points,
        const KDTree& kd_tree,
        double radius_multiplier,
        double max_edge_multiplier,
        int max_retries
    );

private:
    static double computeAverageEdgeLength(const PointCloud& points, const KDTree& kd_tree);
    static bool isCompatible(const Point& p1, const Point& p2, 
        const Point& p3, double max_edge_length);
    static bool computeBallCenters(const Point& p1, const Point& p2, 
        const Point& p3, double radius, Vector3& center1, Vector3& center2);
    static bool isBallEmpty(const KDTree& kd_tree, const PointCloud& points, 
        const Vector3& center, double radius, size_t a, size_t b, size_t c);

    static bool findSeedTriangle(
        const PointCloud& points, const KDTree& kd_tree,
        double radius, double max_edge_length,
        Triangle& seed, std::deque<Edge>& front, FrontSet& front_set
    );

    static double computePivotAngle(
        const Vector3& e, const Vector3& mid,
        const Vector3& old_center, const Vector3& new_center
    );
    
    static size_t findThirdPoint(
        const PointCloud& points, const KDTree& kd_tree,
        const Edge& edge, double radius, double max_edge_length,
        const std::unordered_set<std::array<size_t, 3>, TriangleKeyHash>& created_triangles,
        const EdgeUsageMap& edge_usage,
        Vector3& out_ball_center,
        std::vector<size_t>& candidates_buffer
    );

    static void addTriangle(
        TriangleMesh& mesh, std::deque<Edge>& front, FrontSet& front_set,
        const Edge& current_edge, const Triangle& triangle, const Vector3& ball_center
    );
};
#include "ball_pivoting.h"
#include <iostream>
#include <algorithm>
#include <random>
#include <unordered_set>
#include <numeric>

static constexpr double PI = 3.14159265358979323846;
static constexpr double EPS_COLLINEAR = 1e-12;
static constexpr double EPS_EMPTY = 1e-6;
static constexpr double EPS_ANGLE = 1e-6;
static constexpr double EPS_VEC_LEN = 1e-8;

inline std::array<size_t, 3> makeCanonicalKey(size_t a, size_t b, size_t c) {
    if (a > b) std::swap(a, b);
    if (b > c) std::swap(b, c);
    if (a > b) std::swap(a, b);
    return {a, b, c};
}

double BallPivoting::computeAverageEdgeLength(const PointCloud& points, const KDTree& kd_tree) {
    double total_length = 0.0;
    int count = 0;
    int target_samples = std::min(100, static_cast<int>(points.size()));
    constexpr int k = 3;

    if (k < 1 || target_samples < 1) return 1.0;
    
    std::vector<int> sample_indices;
    sample_indices.reserve(target_samples);
    
    if (target_samples == static_cast<int>(points.size())) {
        sample_indices.resize(points.size());
        std::iota(sample_indices.begin(), sample_indices.end(), 0);
    } else {
        std::unordered_set<int> seen;
        static thread_local std::mt19937 g(42);
        std::uniform_int_distribution<int> dist(0, static_cast<int>(points.size()) - 1);
        
        while (sample_indices.size() < target_samples) {
            int idx = dist(g);
            if (seen.insert(idx).second) {
                sample_indices.push_back(idx);
            }
        }
    }

    std::vector<size_t> neighbors;
    neighbors.reserve(k);
    
    for (int idx : sample_indices) {
        neighbors.clear();
        kd_tree.searchKNN(idx, k, neighbors);
        double local = 0;

        for (auto id : neighbors) {
            local += distance(points[idx], points[id]);
        }

        total_length += local / neighbors.size();
        count++;
    }
    return count > 0 ? total_length / count : 1.0;
}

bool BallPivoting::isCompatible(const Point& p1, const Point& p2, const Point& p3, double max_edge_length) {
    double max_sq = max_edge_length * max_edge_length;
    if (distanceSquared(p1, p2) > max_sq || 
        distanceSquared(p2, p3) > max_sq || 
        distanceSquared(p3, p1) > max_sq) {
        return false;
    }
    Vector3 v1 = p2 - p1;
    Vector3 v2 = p3 - p1;
    if (normSquared(cross(v1, v2)) < EPS_COLLINEAR) return false;
    return true;
}

bool BallPivoting::computeBallCenters(const Point& p1, const Point& p2, 
    const Point& p3, double radius, Vector3& center1, Vector3& center2) {
        
    Vector3 v1 = p2 - p1;
    Vector3 v2 = p3 - p1;
    Vector3 n = cross(v1, v2);
    double n_len_sq = normSquared(n);

    if (n_len_sq < EPS_COLLINEAR) return false;

    Vector3 circum_offset = (cross(n, v1) * dot(v2, v2) - cross(n, v2) * dot(v1, v1)) / (2.0 * n_len_sq);
    double r_circum_sq = normSquared(circum_offset);
    
    if (r_circum_sq > radius * radius + EPS_EMPTY) return false;

    double h = std::sqrt(std::max(0.0, radius * radius - r_circum_sq));
    double inv_len = 1.0 / std::sqrt(n_len_sq);
    Vector3 n_norm = n * inv_len;
    
    Vector3 base = Vector3(p1.x, p1.y, p1.z) + circum_offset;
    center1 = base + n_norm * h;
    center2 = base - n_norm * h;
    
    return true;
}

bool BallPivoting::isBallEmpty(const KDTree& kd_tree, const PointCloud& points, 
    const Vector3& center, double radius, size_t a, size_t b, size_t c) {

    std::vector<size_t> neighbors;
    neighbors.reserve(64);
    kd_tree.radiusSearch(center, radius, INVALID_INDEX, neighbors);

    double r_sq = radius * radius;
    for (size_t idx : neighbors) {
        if (idx == a || idx == b || idx == c) continue;
        if (distanceSquared(points[idx], center) < r_sq - EPS_EMPTY) {
            return false;
        }
    }
    return true;
}

bool BallPivoting::findSeedTriangle(
    const PointCloud& points, const KDTree& kd_tree,
    double radius, double max_edge_length,
    Triangle& seed, std::deque<Edge>& front, FrontSet& front_set) {

    static constexpr int MAX_SEED_SEARCH = 200;
    int target_samples = std::min(MAX_SEED_SEARCH, static_cast<int>(points.size()));
    
    std::vector<int> sample_indices;
    sample_indices.reserve(target_samples);
    
    if (target_samples == static_cast<int>(points.size())) {
        sample_indices.resize(points.size());
        std::iota(sample_indices.begin(), sample_indices.end(), 0);
    } else {
        std::unordered_set<int> seen;
        static thread_local std::mt19937 g(42);
        std::uniform_int_distribution<int> dist(0, static_cast<int>(points.size()) - 1);
        
        while (sample_indices.size() < target_samples) {
            int idx = dist(g);
            if (seen.insert(idx).second) {
                sample_indices.push_back(idx);
            }
        }
    }
    
    int k = std::min(15, static_cast<int>(points.size()) - 1);
    if (k < 2) return false;
    
    std::vector<size_t> neighbors;
    neighbors.reserve(k);
    
    for (int current_idx : sample_indices) {
        neighbors.clear();
        kd_tree.searchKNN(current_idx, k, neighbors);

        for (size_t j = 0; j < neighbors.size(); ++j) {
            for (size_t kk = j + 1; kk < neighbors.size(); ++kk) {
                size_t idx_j = neighbors[j];
                size_t idx_k = neighbors[kk];

                const Point& p1 = points[current_idx];
                const Point& p2 = points[idx_j];
                const Point& p3 = points[idx_k];

                if (!isCompatible(p1, p2, p3, max_edge_length)) continue;

                Vector3 center1, center2;
                if (!computeBallCenters(p1, p2, p3, radius, center1, center2)) {
                    continue;
                }

                Vector3 avg_normal = 
                    Vector3(p1.nx + p2.nx + p3.nx, p1.ny + p2.ny + p3.ny, p1.nz + p2.nz + p3.nz);
                Vector3 chosen_center;
                
                double avg_normal_sq = normSquared(avg_normal);
                if (avg_normal_sq > EPS_COLLINEAR) {
                    Vector3 n = cross(p2 - p1, p3 - p1);
                    double inv_len = 1.0 / std::sqrt(normSquared(n));
                    Vector3 n_norm = n * inv_len;
                    double inv_avg = 1.0 / std::sqrt(avg_normal_sq);
                    avg_normal = avg_normal * inv_avg;
                    chosen_center = (dot(n_norm, avg_normal) < 0.0) ? center2 : center1;
                } else {
                    chosen_center = center1;
                }

                if (!isBallEmpty(kd_tree, points, chosen_center, radius, current_idx, idx_j, idx_k)) continue;

                seed = Triangle(current_idx, idx_j, idx_k);
                
                Edge e1(current_idx, idx_j, idx_k, chosen_center);
                Edge e2(idx_j, idx_k, current_idx, chosen_center);
                Edge e3(idx_k, current_idx, idx_j, chosen_center);

                front.push_back(e1); front_set.insert(e1);
                front.push_back(e2); front_set.insert(e2);
                front.push_back(e3); front_set.insert(e3);
                
                return true;
            }
        }
    }
    return false;
}

double BallPivoting::computePivotAngle(
    const Vector3& e, const Vector3& mid,
    const Vector3& old_center, const Vector3& new_center) {

    Vector3 r_old = old_center - mid;
    Vector3 r_new = new_center - mid;

    Vector3 v_old = r_old - e * dot(r_old, e);
    Vector3 v_new = r_new - e * dot(r_new, e);

    double len_old = norm(v_old);
    double len_new = norm(v_new);
    
    if (len_old < EPS_VEC_LEN || len_new < EPS_VEC_LEN) return std::numeric_limits<double>::max();

    v_old = v_old / len_old;
    v_new = v_new / len_new;

    double cos_theta = std::clamp(dot(v_old, v_new), -1.0, 1.0);
    double sin_theta = dot(cross(v_old, v_new), e);
    double angle = std::atan2(sin_theta, cos_theta);

    if (angle < 0.0) angle += 2.0 * PI;

    return angle;
}

size_t BallPivoting::findThirdPoint(
    const PointCloud& points, const KDTree& kd_tree,
    const Edge& edge, double radius, double max_edge_length,
    const std::unordered_set<std::array<size_t, 3>, TriangleKeyHash>& created_triangles,
    const EdgeUsageMap& edge_usage,
    Vector3& out_ball_center,
    std::vector<size_t>& candidates_buffer) {

    const Point& p1 = points[edge.v1];
    const Point& p2 = points[edge.v2];

    Vector3 mid_pt((p1.x + p2.x) * 0.5, (p1.y + p2.y) * 0.5, (p1.z + p2.z) * 0.5);
    
    candidates_buffer.clear();
    kd_tree.radiusSearch(mid_pt, 2.0 * radius + EPS_EMPTY, INVALID_INDEX, candidates_buffer);

    size_t best_v3 = INVALID_INDEX;
    double best_angle = std::numeric_limits<double>::max();

    auto get_usage = [&](size_t a, size_t b) {
        if (a > b) std::swap(a, b);
        auto it = edge_usage.find({a, b});
        return it == edge_usage.end() ? 0 : it->second;
    };

    Vector3 edge_vec = Vector3(p2.x, p2.y, p2.z) - Vector3(p1.x, p1.y, p1.z);
    Vector3 e = normalize(edge_vec);

    for (size_t v3 : candidates_buffer) {
        if (v3 == edge.v1 || v3 == edge.v2 || v3 == edge.opposite_vertex) {
            continue;
        }

        const Point& p3 = points[v3];
        if (!isCompatible(p1, p2, p3, max_edge_length)) {
            continue;
        }

        auto key = makeCanonicalKey(edge.v1, edge.v2, v3);
        if (created_triangles.count(key)) {
            continue;
        }

        if (get_usage(edge.v1, edge.v2) >= 2 || 
            get_usage(edge.v2, v3) >= 2 || 
            get_usage(v3, edge.v1) >= 2) 
        {
            continue; 
        }

        Vector3 center1, center2;
        if (!computeBallCenters(p1, p2, p3, radius, center1, center2)){
            continue;
        }

        if (isBallEmpty(kd_tree, points, center1, radius, edge.v1, edge.v2, v3)) {
            double angle1 = computePivotAngle(e, mid_pt, edge.ball_center, center1);
            if (angle1 > EPS_ANGLE && angle1 < best_angle) {
                best_angle = angle1;
                best_v3 = v3;
                out_ball_center = center1;
            }
        }
        
        if (isBallEmpty(kd_tree, points, center2, radius, edge.v1, edge.v2, v3)) {
            double angle2 = computePivotAngle(e, mid_pt, edge.ball_center, center2);
            if (angle2 > EPS_ANGLE && angle2 < best_angle) {
                best_angle = angle2;
                best_v3 = v3;
                out_ball_center = center2;
            }
        }
    }
    return best_v3;
}

void BallPivoting::addTriangle(
    TriangleMesh& mesh, std::deque<Edge>& front, FrontSet& front_set,
    const Edge& current_edge, const Triangle& triangle, const Vector3& ball_center) {
    mesh.push_back(triangle);
    front_set.erase(current_edge);

    auto processNewEdge = [&](const Edge& e) {
        Edge rev(e.v2, e.v1);
        auto it = front_set.find(rev);
        if (it != front_set.end()) {
            front_set.erase(it);
        } else {
            if (front_set.insert(e).second) {
                front.push_back(e);
            }
        }
    };

    processNewEdge(Edge(triangle.v2, triangle.v3, triangle.v1, ball_center));
    processNewEdge(Edge(triangle.v3, triangle.v1, triangle.v2, ball_center));
}

TriangleMesh BallPivoting::reconstruct(
    const PointCloud& points, const KDTree& kd_tree,
    double radius_multiplier, double max_edge_multiplier, int max_retries) {
    if (points.size() < 3) return {};

    double avg_edge = computeAverageEdgeLength(points, kd_tree);
    if (avg_edge < EPS_EMPTY) {
        std::cerr << "BPA: Average edge length is too small. Cannot reconstruct." << std::endl;
        return {};
    }
    
    double radius = radius_multiplier * avg_edge;
    double max_edge_length = max_edge_multiplier * avg_edge;

    std::cout << "BPA: Avg edge = " << avg_edge << ", Radius = " << radius << std::endl;

    TriangleMesh mesh;
    std::deque<Edge> front;
    FrontSet front_set;
    std::unordered_set<std::array<size_t, 3>, TriangleKeyHash> created_triangles;
    EdgeUsageMap edge_usage;
    Triangle seed;
    
    std::vector<size_t> candidates_buffer;
    candidates_buffer.reserve(256);

    if (!findSeedTriangle(points, kd_tree, radius, max_edge_length, seed, front, front_set)) {
        std::cerr << "BPA: Failed to find a valid seed triangle." << std::endl;
        return mesh;
    }

    mesh.push_back(seed);
    created_triangles.insert(makeCanonicalKey(seed.v1, seed.v2, seed.v3));

    auto increment_usage = [&](size_t a, size_t b) {
        if (a > b) std::swap(a, b);
        edge_usage[{a, b}]++;
    };

    increment_usage(seed.v1, seed.v2);
    increment_usage(seed.v2, seed.v3);
    increment_usage(seed.v3, seed.v1);

    std::cout << "BPA: Starting main loop with front size = " << front.size() << std::endl;

    while (!front.empty()) {
        Edge current_edge = front.front();
        front.pop_front();

        if (front_set.find(current_edge) == front_set.end()) continue;

        Vector3 best_center;
        size_t v3 = findThirdPoint(points, kd_tree, current_edge, radius, max_edge_length, 
            created_triangles, edge_usage, best_center, candidates_buffer);

        if (v3 != INVALID_INDEX) {
            created_triangles.insert(makeCanonicalKey(current_edge.v1, current_edge.v2, v3));
            
            increment_usage(current_edge.v1, current_edge.v2);
            increment_usage(current_edge.v2, v3);
            increment_usage(v3, current_edge.v1);

            Triangle new_tri(current_edge.v1, current_edge.v2, v3);
            addTriangle(mesh, front, front_set, current_edge, new_tri, best_center);
        } else {
            if (current_edge.retry_count < max_retries) {
                current_edge.retry_count++;
                front.push_back(current_edge);
            } else {
                front_set.erase(current_edge);
            }
        }
    }

    std::cout << "BPA: Generated " << mesh.size() << " triangles." << std::endl;
    return mesh;
}
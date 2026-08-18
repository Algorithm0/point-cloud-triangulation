#include "ball_pivoting.h"
#include "kd_tree.h"
#include <iostream>
#include <algorithm>
#include <random>
#include <unordered_set>
#include <unordered_map>
#include <numeric>
#include <iomanip>
#include <limits>
#include <numbers>
#include <deque>
#include <ranges>

namespace {
    constexpr double EPS_COLLINEAR = 1e-12;
    constexpr double EPS_EMPTY = 1e-6;
    constexpr double EPS_ANGLE = 1e-6;
    constexpr double EPS_VEC_LEN = 1e-8;
    constexpr double SEED_NORMAL_DOT = 0.8; 
    constexpr size_t INVALID_INDEX = std::numeric_limits<size_t>::max();

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

    /// @brief Вычисляет среднюю длину ребра по выборке точек для масштабирования радиуса.
    double computeAverageEdgeLength(const PointCloud& points, const KDTree& kd_tree) {
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

            std::ranges::for_each(neighbors, [&](size_t id){
                local += distance(points[idx], points[id]);
            });

            total_length += local / neighbors.size();
            count++;
        }
        return count > 0 ? total_length / count : 1.0;
    }

    /// @brief Проверяет геометрическую совместимость трех точек (длины ребер и коллинеарность).
    bool isCompatible(const Point& p1, const Point& p2, const Point& p3, double max_edge_length) {
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

    /// @brief Вычисляет возможные центры сферы, проходящей через три точки.
    ///
    /// Для заданного треугольника вычисляются два возможных положения центра
    /// сферы фиксированного радиуса, расположенные по разные стороны плоскости
    /// треугольника.
    ///
    /// @param p1 Первая вершина.
    /// @param p2 Вторая вершина.
    /// @param p3 Третья вершина.
    /// @param radius Радиус сферы.
    /// @param center1 Первый возможный центр.
    /// @param center2 Второй возможный центр.
    /// @return true, если центры существуют, иначе false.
    bool computeBallCenters(const Point& p1, const Point& p2, 
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

    /// @brief Проверяет, что внутри сферы заданного радиуса нет других точек облака (критерий пустой сферы).
    bool isBallEmpty(const KDTree& kd_tree, const PointCloud& points, 
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

    /// @brief Находит начальный треугольник для запуска алгоритма.
    ///
    /// Выбирает три совместимые точки, проверяет согласованность их нормалей,
    /// существование пустой сферы заданного радиуса и формирует начальный фронт.
    ///
    /// @param points Облако точек.
    /// @param kd_tree KD-дерево.
    /// @param radius Радиус виртуального шара.
    /// @param max_edge_length Максимальная допустимая длина ребра.
    /// @param seed Найденный начальный треугольник.
    /// @param front Очередь ребер фронта.
    /// @param front_set Множество ребер фронта.
    /// @return true, если начальный треугольник найден, иначе false.
    bool findSeedTriangle(
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

                    size_t v_a = current_idx;
                    size_t v_b = idx_j;
                    size_t v_c = idx_k;
                    Point pa = points[v_a];
                    Point pb = points[v_b];
                    Point pc = points[v_c];

                    // Проверяется согласованность нормалей, чтобы не начинать построение с острых граней
                    if (std::abs(dot(Vector3(pa.nx, pa.ny, pa.nz), Vector3(pb.nx, pb.ny, pb.nz))) < SEED_NORMAL_DOT) { 
                        continue;
                    }
                    if (std::abs(dot(Vector3(pa.nx, pa.ny, pa.nz), Vector3(pc.nx, pc.ny, pc.nz))) < SEED_NORMAL_DOT) {
                        continue;
                    }
                    if (std::abs(dot(Vector3(pb.nx, pb.ny, pb.nz), Vector3(pc.nx, pc.ny, pc.nz))) < SEED_NORMAL_DOT) {
                        continue;
                    }

                    Vector3 n = cross(Vector3(pb.x, pb.y, pb.z) - Vector3(pa.x, pa.y, pa.z),
                        Vector3(pc.x, pc.y, pc.z) - Vector3(pa.x, pa.y, pa.z));
                    double n_len_sq = normSquared(n);

                    if (n_len_sq < EPS_COLLINEAR) continue;
                    Vector3 n_norm = n * (1.0 / std::sqrt(n_len_sq));
                    Vector3 avg_normal(pa.nx + pb.nx + pc.nx, pa.ny + pb.ny + pc.ny, pa.nz + pb.nz + pc.nz);
                    double avg_normal_sq = normSquared(avg_normal);

                    if (avg_normal_sq > EPS_COLLINEAR) {
                        Vector3 avg_norm = avg_normal * (1.0 / std::sqrt(avg_normal_sq));
                        
                        if (dot(n_norm, avg_norm) < 0.0) {
                            std::swap(v_b, v_c); 
                            pb = points[v_b];
                            pc = points[v_c];
                            n = cross(Vector3(pb.x, pb.y, pb.z) - Vector3(pa.x, pa.y, pa.z),
                                Vector3(pc.x, pc.y, pc.z) - Vector3(pa.x, pa.y, pa.z));
                        }
                    }

                    // Вычисляются два возможных центра сферы, проходящей через текущий треугольник
                    Vector3 center1, center2;
                    if (!computeBallCenters(pa, pb, pc, radius, center1, center2)) {
                        continue;
                    }

                    Vector3 chosen_center = center1;
                    if (!isBallEmpty(kd_tree, points, chosen_center, radius, v_a, v_b, v_c)) {
                        continue;
                    }

                    seed = Triangle(v_a, v_b, v_c);

                    Edge e1(v_a, v_b, v_c, chosen_center);
                    Edge e2(v_b, v_c, v_a, chosen_center);
                    Edge e3(v_c, v_a, v_b, chosen_center);

                    front.push_back(e1); front_set.insert(e1);
                    front.push_back(e2); front_set.insert(e2);
                    front.push_back(e3); front_set.insert(e3);
                    return true;
                }
            }
        }
        return false;
    }

    /// @brief Вычисляет угол поворота сферы вокруг ребра фронта к новому центру.
    double computePivotAngle(
        const Vector3& e, const Vector3& mid,
        const Vector3& old_center, const Vector3& new_center) {

        Vector3 r_old = old_center - mid;
        Vector3 r_new = new_center - mid;

        Vector3 v_old = r_old - e * dot(r_old, e);
        Vector3 v_new = r_new - e * dot(r_new, e);

        double len_old = norm(v_old);
        double len_new = norm(v_new);
        
        if (len_old < EPS_VEC_LEN || len_new < EPS_VEC_LEN) 
            return std::numeric_limits<double>::max();

        v_old = v_old / len_old;
        v_new = v_new / len_new;

        double cos_theta = std::clamp(dot(v_old, v_new), -1.0, 1.0);
        double sin_theta = dot(cross(v_old, v_new), e);
        double angle = std::atan2(sin_theta, cos_theta);

        if (angle < 0.0) angle += 2.0 * std::numbers::pi;

        if (angle > 2.0 * std::numbers::pi - 1e-5) {
            return std::numeric_limits<double>::max(); 
        }

        if (angle > std::numbers::pi) {
            angle = 2.0 * std::numbers::pi - angle;
        }

        if (angle < EPS_ANGLE) {
            return std::numeric_limits<double>::max();
        }

        return angle;
    }

    /// @brief Ищет третью вершину для построения нового треугольника.
    ///
    /// Для текущего ребра фронта рассматриваются кандидаты из локальной окрестности.
    /// Проверяются геометрическая совместимость, критерий пустой сферы,
    /// отсутствие дубликатов и корректность ориентации. Из допустимых кандидатов
    /// выбирается тот, который обеспечивает минимальный угол поворота шара.
    ///
    /// @param points Облако точек.
    /// @param kd_tree KD-дерево.
    /// @param edge Текущее ребро фронта.
    /// @param radius Радиус шара.
    /// @param max_edge_length Максимальная длина ребра.
    /// @param created_triangles Уже построенные треугольники.
    /// @param edge_usage Счетчик использования ребер.
    /// @param out_ball_center Центр шара для выбранного треугольника.
    /// @param candidates_buffer Буфер кандидатов.
    /// @return Индекс найденной вершины или INVALID_INDEX.
    size_t findThirdPoint(
        const PointCloud& points, const KDTree& kd_tree,
        const Edge& edge, double radius, double max_edge_length,
        const std::unordered_set<std::array<size_t, 3>, TriangleKeyHash>& created_triangles,
        const EdgeUsageMap& edge_usage,
        Vector3& out_ball_center,
        std::vector<size_t>& candidates_buffer) {

        const Point& p1 = points[edge.v1];
        const Point& p2 = points[edge.v2];

        Vector3 mid_pt(std::midpoint(p1.x, p2.x), std::midpoint(p1.y, p2.y), std::midpoint(p1.z, p2.z));
        
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

        auto valid_candidates = candidates_buffer
            | std::views::filter([&](size_t v3) {
                return v3 != edge.v1 && v3 != edge.v2 && v3 != edge.opposite_vertex;
            });

        for (size_t v3 : valid_candidates) {

            const Point& p3_pt = points[v3];
            if (!isCompatible(p1, p2, p3_pt, max_edge_length)) continue;

            const Point& p_opp = points[edge.opposite_vertex];
            Vector3 v1_vec(p1.x, p1.y, p1.z);
            Vector3 v2_vec(p2.x, p2.y, p2.z);
            Vector3 v3_vec(p3_pt.x, p3_pt.y, p3_pt.z);
            Vector3 v_opp_vec(p_opp.x, p_opp.y, p_opp.z);

            Vector3 current_normal = cross(v2_vec - v1_vec, v_opp_vec - v1_vec);
            double len_norm = norm(current_normal);
            
            if (len_norm > 1e-8) {
                current_normal = current_normal / len_norm;
                Vector3 cross_v3 = cross(v2_vec - v1_vec, v3_vec - v1_vec);

                // Отбрасываются кандидаты, которые создадут треугольник с той же стороны от текущего ребра фронта
                if (dot(cross_v3, current_normal) > 1e-6) {
                    continue; 
                }
            }

            auto key = makeCanonicalKey(edge.v1, edge.v2, v3);
            if (created_triangles.contains(key)) continue;

            if (get_usage(edge.v1, edge.v2) >= 2 || 
                get_usage(edge.v2, v3) >= 2 || 
                get_usage(v3, edge.v1) >= 2) {
                continue;
            }

            Vector3 center1, center2;
            if (!computeBallCenters(p1, p2, p3_pt, radius, center1, center2)) continue;

            bool c1_valid = false, c2_valid = false;
            double a1 = std::numeric_limits<double>::max();
            double a2 = std::numeric_limits<double>::max();
            
            if (isBallEmpty(kd_tree, points, center1, radius, edge.v1, edge.v2, v3)) {
                c1_valid = true;
                a1 = computePivotAngle(e, mid_pt, edge.ball_center, center1);
            }

            if (isBallEmpty(kd_tree, points, center2, radius, edge.v1, edge.v2, v3)) {
                c2_valid = true;
                a2 = computePivotAngle(e, mid_pt, edge.ball_center, center2);
            }

            double current_best_a = std::numeric_limits<double>::max();
            int current_best_choice = 0;
            Vector3 current_best_center;

            if (c1_valid && a1 > EPS_ANGLE && a1 < current_best_a) {
                current_best_a = a1;
                current_best_choice = 1;
                current_best_center = center1;
            }
            if (c2_valid && a2 > EPS_ANGLE && a2 < current_best_a) {
                current_best_a = a2;
                current_best_choice = 2;
                current_best_center = center2;
            }

            // Выбирается валидный центр сферы, требующий минимального угла поворота
            if (current_best_choice != 0 && current_best_a < best_angle) {
                best_angle = current_best_a;
                best_v3 = v3;
                out_ball_center = current_best_center;
            }
        }
        return best_v3;
    }

    /// @brief Добавляет новый треугольник в сетку и обновляет множество ребер фронта.
    void addTriangle(
        TriangleMesh& mesh, std::deque<Edge>& front, FrontSet& front_set,
        const Edge& current_edge, const Triangle& triangle, const Vector3& ball_center) {
        mesh.push_back(triangle);
        auto processNewEdge = [&](const Edge& e) {
            if ((e.v1 == current_edge.v1 && e.v2 == current_edge.v2) ||
                (e.v1 == current_edge.v2 && e.v2 == current_edge.v1)) {
                return;
            }
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

        processNewEdge(Edge(triangle.v1, triangle.v2, triangle.v3, ball_center));
        processNewEdge(Edge(triangle.v2, triangle.v3, triangle.v1, ball_center));
        processNewEdge(Edge(triangle.v3, triangle.v1, triangle.v2, ball_center));
        
        front_set.erase(current_edge);
    }
}

namespace BallPivoting {
    TriangleMesh reconstruct(
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

            if (!front_set.contains(current_edge)) continue;

            Vector3 best_center;
            size_t v3 = findThirdPoint(points, kd_tree, current_edge, radius, max_edge_length, 
                created_triangles, edge_usage, best_center, candidates_buffer);

            if (v3 != INVALID_INDEX) {            
                Triangle new_tri(current_edge.v2, current_edge.v1, v3);
                
                created_triangles.insert(makeCanonicalKey(new_tri.v1, new_tri.v2, new_tri.v3));
                
                increment_usage(new_tri.v1, new_tri.v2);
                increment_usage(new_tri.v2, new_tri.v3);
                increment_usage(new_tri.v3, new_tri.v1);

                addTriangle(mesh, front, front_set, current_edge, new_tri, best_center);
            }
        }

        std::cout << "BPA: Generated " << mesh.size() << " triangles." << std::endl;
        return mesh;
    }
}
#pragma once

#include "geometry.h"
#include "kd_tree.h"
#include <vector>
#include <array>

namespace BallPivoting {
    /// @brief Выполняет реконструкцию поверхности методом Ball Pivoting Algorithm.
    ///
    /// Алгоритм строит треугольную сетку, начиная с начального треугольника (seed)
    /// и последовательно расширяя фронт путем перекатывания виртуальной сферы
    /// заданного радиуса по граничным ребрам.
    ///
    /// @param points Облако точек с вычисленными нормалями.
    /// @param kd_tree KD-дерево для поиска соседей.
    /// @param radius_multiplier Множитель среднего расстояния между соседями для вычисления радиуса шара.
    /// @param max_edge_multiplier Максимально допустимая длина ребра.
    /// @param max_retries Максимальное число попыток построения треугольника для одного ребра.
    /// @return Построенная треугольная сетка.
    ///
    TriangleMesh reconstruct(
        const PointCloud& points,
        const class KDTree& kd_tree,
        double radius_multiplier,
        double max_edge_multiplier,
        int max_retries
    );
}

inline std::array<size_t, 3> makeCanonicalKey(size_t a, size_t b, size_t c) {
    if (a > b) std::swap(a, b);
    if (b > c) std::swap(b, c);
    if (a > b) std::swap(a, b);
    return {a, b, c};
}
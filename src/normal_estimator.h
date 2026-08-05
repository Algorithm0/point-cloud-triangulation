#pragma once

#include "geometry.h"
#include "kd_tree.h"

namespace NormalEstimator {
    /// @brief Вычисляет нормали точек методом анализа главных компонент (PCA).
    ///
    /// Для каждой точки строится локальная ковариационная матрица по ближайшим
    /// соседям. Нормалью выбирается собственный вектор, соответствующий
    /// минимальному собственному значению. После вычисления выполняется
    /// глобальная ориентация нормалей относительно центра масс облака.
    ///
    /// @param points Облако точек (нормали записываются в поля nx, ny, nz).
    /// @param kd_tree KD-дерево.
    /// @param k_neighbors Количество соседей для PCA.
    /// @param num_threads Количество потоков OpenMP.
    void estimate(PointCloud& points, const KDTree& kd_tree, int k_neighbors, int num_threads);
};
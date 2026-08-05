#pragma once
#include "geometry.h"

namespace Preprocessor {
    PointCloud getUniquePoints(const PointCloud& points, double tolerance);
};
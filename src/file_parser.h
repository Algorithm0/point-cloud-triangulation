#pragma once

#include "geometry.h"
#include <string>

namespace FileParser {
    PointCloud readXYZ(const std::string& filename);
    void writeMesh(const std::string& filename, 
        const PointCloud& all_points,
        const PointCloud& unique_points, 
        const TriangleMesh& triangles);
    void writeOBJ(const std::string& filename, 
        const PointCloud& unique_points, 
        const TriangleMesh& triangles);

};

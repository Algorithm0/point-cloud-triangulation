#pragma once

#include "geometry.h"
#include <string>

class FileParser {
public:
    PointCloud readXYZ(const std::string& filename) const;
    void writeMesh(const std::string& filename, 
        const PointCloud& all_points,
        const PointCloud& unique_points, 
        const TriangleMesh& triangles) const;
    void writeOBJ(const std::string& filename, 
        const PointCloud& unique_points, 
        const TriangleMesh& triangles) const;
    
private:
    bool isComment(const std::string& line) const;
    bool parsePointLine(const std::string& line, 
        int& id, double& x, double& y, double& z) const;
};

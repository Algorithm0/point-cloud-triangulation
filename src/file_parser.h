#pragma once

#include "geometry.h"
#include <string>

class FileParser {
public:
    PointCloud readXYZ(const std::string& filename);
    void writeMesh(const std::string& filename, 
                   const PointCloud& points, 
                   const TriangleMesh& triangles);
    void writeOBJ(const std::string& filename, const PointCloud& unique_points, const TriangleMesh& triangles) const;
    
private:
    bool isComment(const std::string& line);
    bool parsePointLine(const std::string& line, 
                       int& id, double& x, double& y, double& z);
};

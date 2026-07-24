#pragma once

#include <vector>

struct Point {
    double x;
    double y;
    double z;
    int node_id;

    double nx = 0.0;
    double ny = 0.0;
    double nz = 0.0;
    
    Point() : x(0), y(0), z(0), node_id(0) {}
    Point(double x_, double y_, double z_, int id) 
        : x(x_), y(y_), z(z_), node_id(id) {}
};

struct Triangle {
    size_t v1;
    size_t v2;
    size_t v3;
    
    Triangle() : v1(0), v2(0), v3(0) {}
    Triangle(size_t a, size_t b, size_t c) : v1(a), v2(b), v3(c) {}
};

struct Vector3 {
    double x;
    double y; 
    double z;
};

using PointCloud = std::vector<Point>;

using TriangleMesh = std::vector<Triangle>;
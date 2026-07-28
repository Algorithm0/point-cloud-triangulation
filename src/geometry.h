#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <cstddef>

struct Point {
    double x, y, z;
    double nx, ny, nz;
    size_t node_id;

    Point(double x = 0.0, double y = 0.0, double z = 0.0, size_t id = 0)
        : x(x), y(y), z(z), nx(0.0), ny(0.0), nz(0.0), node_id(id) {}
};

using PointCloud = std::vector<Point>;

struct Triangle {
    size_t v1 = 0;
    size_t v2 = 0;
    size_t v3 = 0;

    Triangle() = default;

    Triangle(size_t a, size_t b, size_t c)
        : v1(a), v2(b), v3(c) {}
};

using TriangleMesh = std::vector<Triangle>;

struct Vector3 {
    double x, y, z;
    Vector3(double x = 0.0, double y = 0.0, double z = 0.0) : x(x), y(y), z(z) {}
};

inline Vector3 operator-(const Point& a, const Point& b) {
    return Vector3(a.x - b.x, a.y - b.y, a.z - b.z);
}

inline Vector3 operator+(const Vector3& a, const Vector3& b) {
    return Vector3(a.x + b.x, a.y + b.y, a.z + b.z);
}

inline Vector3 operator-(const Vector3& a, const Vector3& b) {
    return Vector3(a.x - b.x, a.y - b.y, a.z - b.z);
}

inline Vector3 operator*(const Vector3& v, double scalar) {
    return Vector3(v.x * scalar, v.y * scalar, v.z * scalar);
}

inline Vector3 operator*(double scalar, const Vector3& v) {
    return Vector3(v.x * scalar, v.y * scalar, v.z * scalar);
}

inline Vector3 operator/(const Vector3& v, double scalar) {
    return Vector3(v.x / scalar, v.y / scalar, v.z / scalar);
}

inline Point operator+(const Point& p, const Vector3& v) {
    return Point(p.x + v.x, p.y + v.y, p.z + v.z, p.node_id);
}

inline double dot(const Vector3& a, const Vector3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vector3 cross(const Vector3& a, const Vector3& b) {
    return Vector3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

inline Vector3 operator-(const Vector3& v) {
    return Vector3(-v.x, -v.y, -v.z);
}

inline double norm(const Vector3& v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

inline double normSquared(const Vector3& v) {
    return v.x * v.x + v.y * v.y + v.z * v.z;
}

inline Vector3 normalize(const Vector3& v) {
    double len = norm(v);
    if (len < 1e-12) {
        return Vector3(0.0, 0.0, 0.0);
    }
    return v / len;
}

inline double distanceSquared(const Point& a, const Point& b) {
    Vector3 d = a - b;
    return normSquared(d);
}

inline double distance(const Point& a, const Point& b) {
    return std::sqrt(distanceSquared(a, b));
}

inline double distanceSquared(const Point& a, const Vector3& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;
    return dx * dx + dy * dy + dz * dz;
}

inline double distanceSquared(const Vector3& a, const Point& b) {
    return distanceSquared(b, a);
}
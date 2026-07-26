#include "file_parser.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <iomanip>
#include <unordered_map>

PointCloud FileParser::readXYZ(const std::string& filename) {
    PointCloud points;
    std::ifstream file(filename);
    
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open input file: " + filename);
    }
    
    std::string line;
    int line_number = 0;
    
    while (std::getline(file, line)) {
        line_number++;

        if (line.empty() || isComment(line)) {
            continue;
        }
        
        int id;
        double x, y, z;
        
        if (!parsePointLine(line, id, x, y, z)) {
            std::cerr << "Warning: Skipping invalid line " << line_number 
                      << ": " << line << std::endl;
            continue;
        }
        
        points.emplace_back(x, y, z, id);
    }
    
    if (points.empty()) {
        throw std::runtime_error("No valid points found in file: " + filename);
    }
    
    std::cout << "Loaded " << points.size() << " points from " << filename << std::endl;
    
    return points;
}

void FileParser::writeMesh(const std::string& filename, 
                          const PointCloud& points, 
                          const TriangleMesh& triangles) {
    std::ofstream file(filename);
    
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open output file: " + filename);
    }
    
    file << "* N,\tX\tY\tZ\n";
    file << "* Nodes\n";
    
    for (size_t i = 0; i < points.size(); i++) {
        file << points[i].node_id << ", "
             << std::defaultfloat << std::setprecision(10)
             << points[i].x << ", "
             << points[i].y << ", "
             << points[i].z << "\n";
    }
    
    file << "* Elements\n";
    
    for (size_t i = 0; i < triangles.size(); i++) {
        file << (i + 1) << ", "
             << triangles[i].v1 << ", "
             << triangles[i].v2 << ", "
             << triangles[i].v3 << "\n";
    }
    
    std::cout << "Saved mesh with " << points.size() << " points and " 
              << triangles.size() << " triangles to " << filename << std::endl;
}

bool FileParser::isComment(const std::string& line) {
    size_t start = line.find_first_not_of(" \t");
    if (start == std::string::npos) {
        return true; 
    }
    
    return line[start] == '*';
}

bool FileParser::parsePointLine(const std::string& line, 
                               int& id, double& x, double& y, double& z) {
    std::stringstream ss(line);
    std::string token;
    std::vector<double> values;
    
    while (std::getline(ss, token, ',')) {
        token.erase(0, token.find_first_not_of(" \t"));
        token.erase(token.find_last_not_of(" \t") + 1);
        
        try {
            values.push_back(std::stod(token));
        } catch (...) {
            return false;
        }
    }
    
    if (values.size() != 4) {
        return false;
    }
    
    id = static_cast<int>(values[0]);
    x = values[1];
    y = values[2];
    z = values[3];
    
    return true;
}

void FileParser::writeOBJ(const std::string& filename, const PointCloud& unique_points, const TriangleMesh& triangles) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open output OBJ file: " + filename);
    }

    std::unordered_map<int, size_t> id_to_obj_idx;
    for (size_t i = 0; i < unique_points.size(); ++i) {
        id_to_obj_idx[unique_points[i].node_id] = i + 1;
    }

    for (const auto& p : unique_points) {
        file << "v " << p.x << " " << p.y << " " << p.z << "\n";
    }

    for (const auto& p : unique_points) {
        file << "vn " << p.nx << " " << p.ny << " " << p.nz << "\n";
    }

    for (const auto& t : triangles) {
        size_t idx1 = id_to_obj_idx[t.v1];
        size_t idx2 = id_to_obj_idx[t.v2];
        size_t idx3 = id_to_obj_idx[t.v3];
        
        file << "f " << idx1 << "//" << idx1 << " " 
                   << idx2 << "//" << idx2 << " " 
                   << idx3 << "//" << idx3 << "\n";
    }

    std::cout << "Saved OBJ mesh with " << unique_points.size() << " vertices and " 
              << triangles.size() << " faces to " << filename << std::endl;
}
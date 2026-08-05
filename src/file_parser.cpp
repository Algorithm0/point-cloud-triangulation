#include "file_parser.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <iomanip>
#include <string>
#include <vector>
#include <stdexcept>

namespace {
    bool isComment(const std::string& line) {
        size_t start = line.find_first_not_of(" \t");
        if (start == std::string::npos) {
            return true; 
        }
        
        return line[start] == '*';
    }

    bool parsePointLine(const std::string& line, 
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
}

namespace FileParser {
    PointCloud readXYZ(const std::string& filename) {
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

    void writeMesh(const std::string& filename, 
        const PointCloud& all_points,
        const PointCloud& unique_points, 
        const TriangleMesh& triangles) {

        std::ofstream file(filename);
        
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open output file: " + filename);
        }
        
        file << "* N,\tX\tY\tZ\n";
        file << "* Nodes\n";
        
        for (size_t i = 0; i < all_points.size(); i++) {
            file << all_points[i].node_id << ", "
                << std::defaultfloat << std::setprecision(10)
                << all_points[i].x << ", "
                << all_points[i].y << ", "
                << all_points[i].z << "\n";
        }
        
        file << "* Elements\n";
        
        for (size_t i = 0; i < triangles.size(); i++) {
            int n1 = unique_points[triangles[i].v1].node_id;
            int n2 = unique_points[triangles[i].v2].node_id;
            int n3 = unique_points[triangles[i].v3].node_id;
            
            file << (i + 1) << ", "
                << n1 << ", "
                << n2 << ", "
                << n3 << "\n";
        }
        
        std::cout << "Saved mesh with " << all_points.size() << " total points (" 
            << unique_points.size() << " unique) and " 
            << triangles.size() << " triangles to " << filename << std::endl;
    }


    void writeOBJ(const std::string& filename, const PointCloud& unique_points, 
        const TriangleMesh& triangles) {

        std::ofstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open output OBJ file: " + filename);
        }

        for (const auto& p : unique_points) {
            file << "v " << p.x << " " << p.y << " " << p.z << "\n";
        }

        for (const auto& p : unique_points) {
            file << "vn " << p.nx << " " << p.ny << " " << p.nz << "\n";
        }

        for (const auto& t : triangles) {
            file << "f " << (t.v1 + 1) << "//" << (t.v1 + 1) << " " 
                << (t.v2 + 1) << "//" << (t.v2 + 1) << " " 
                << (t.v3 + 1) << "//" << (t.v3 + 1) << "\n";
        }

        std::cout << "Saved OBJ mesh with " << unique_points.size() << " vertices and " 
            << triangles.size() << " faces to " << filename << std::endl;
    }
}
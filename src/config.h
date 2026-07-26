#pragma once

#include <string>
#include <nlohmann/json.hpp> 

struct AlgorithmConfig {
    double radius_multiplier = 2.5;
    int pca_neighbors = 25;
    double max_edge_multiplier = 2.5;
    double duplicate_tolerance = 1e-6;
    int max_retries = 3;
    
    void loadFromJson(const nlohmann::json& j);
};

struct PerformanceConfig {
    int threads = 4;
    
    void loadFromJson(const nlohmann::json& j);
};

struct OutputConfig {
    bool export_obj = false;
    
    void loadFromJson(const nlohmann::json& j);
};

struct Config {
    AlgorithmConfig algorithm;
    PerformanceConfig performance;
    OutputConfig output;

    static Config loadFromFile(const std::string& filepath);
};
#include "config.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <concepts>

using json = nlohmann::json;

template <typename T>
concept ConfigValue = std::integral<T> || std::floating_point<T>;

template <ConfigValue T>
void load_or_keep(T& value, const json& obj, const std::string& key) {
    if (obj.contains(key)) {
        try {
            value = obj[key].get<T>();
        } catch (const json::exception&) {
            std::cerr << "Warning: Invalid value for '" << key 
                << "' in config. Keeping default: " << value << std::endl;
        }
    }
}

void AlgorithmConfig::loadFromJson(const json& j) {
    load_or_keep(radius_multiplier, j, "radius_multiplier");
    load_or_keep(pca_neighbors, j, "pca_neighbors");
    load_or_keep(max_edge_multiplier, j, "max_edge_multiplier");
    load_or_keep(duplicate_tolerance, j, "duplicate_tolerance");
    load_or_keep(max_retries, j, "max_retries");
}

void PerformanceConfig::loadFromJson(const json& j) {
    load_or_keep(threads, j, "threads");
}

void OutputConfig::loadFromJson(const json& j) {
    load_or_keep(export_obj, j, "export_obj");
}

Config Config::loadFromFile(const std::string& filepath) {
    Config config; 

    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Warning: Config file '" << filepath 
            << "' not found. Using default values." << std::endl;
        return config;
    }

    json j;
    try {
        file >> j;
    } catch (const json::exception& e) {
        std::cerr << "Warning: Failed to parse config file '" << filepath 
            << "': " << e.what() << ". Using default values." << std::endl;
        return config;
    }

    if (j.contains("algorithm")) {
        config.algorithm.loadFromJson(j["algorithm"]);
    }
    
    if (j.contains("performance")) {
        config.performance.loadFromJson(j["performance"]);
    }
    
    if (j.contains("output")) {
        config.output.loadFromJson(j["output"]);
    }

    return config;
}
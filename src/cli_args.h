#pragma once

#include <cxxopts.hpp>
#include <string>
#include <stdexcept>
#include <iostream>

struct CliArgs {
    std::string input_file;
    std::string output_file;
    std::string config_file = "config.json";

    static CliArgs parse(int argc, char* argv[]) {
        cxxopts::Options options("triangulator", "Point cloud triangulation using Ball Pivoting Algorithm");

        options.add_options()
            ("i,input", "Input point cloud file (required)", cxxopts::value<std::string>())
            ("o,output", "Output mesh file (required)", cxxopts::value<std::string>())
            ("c,config", "Configuration file", cxxopts::value<std::string>()->default_value("config.json"))
            ("h,help", "Print usage information");

        auto result = options.parse(argc, argv);

        if (result.count("help")) {
            std::cout << options.help() << std::endl;
            exit(0);
        }

        if (!result.count("input") || !result.count("output")) {
            std::cerr << "Error: Both --input and --output are required.\n" << std::endl;
            std::cerr << options.help() << std::endl;
            throw std::runtime_error("Missing required arguments");
        }

        CliArgs args;
        args.input_file = result["input"].as<std::string>();
        args.output_file = result["output"].as<std::string>();
        args.config_file = result["config"].as<std::string>();

        return args;
    }
};
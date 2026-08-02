cmake_minimum_required(VERSION 3.14)
project(PointCloudTriangulation VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)
FetchContent_Declare(
    json
    URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)

FetchContent_Declare(
    cxxopts
    GIT_REPOSITORY https://github.com/jarro2783/cxxopts.git
    GIT_TAG v3.1.1
)

FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.4.0
)

FetchContent_MakeAvailable(json cxxopts Catch2)

add_library(triangulator_core STATIC
    src/file_parser.cpp
    src/config.cpp
    src/preprocessor.cpp
    src/eigen33.cpp
    src/kd_tree.cpp 
    src/normal_estimator.cpp
    src/ball_pivoting.cpp
)

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(triangulator_core PRIVATE OpenMP::OpenMP_CXX)
endif()

target_link_libraries(triangulator_core PUBLIC nlohmann_json::nlohmann_json)

add_executable(triangulator src/main.cpp)

target_link_libraries(triangulator PRIVATE 
    triangulator_core 
    cxxopts::cxxopts
)

enable_testing()
add_executable(test_runner 
    tests/unit_tests.cpp 
    tests/integration_tests.cpp
)

target_link_libraries(test_runner PRIVATE 
    Catch2::Catch2WithMain 
    triangulator_core
)

add_test(NAME AllTests COMMAND test_runner)
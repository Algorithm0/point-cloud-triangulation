add_library(triangulator_core STATIC
    file_parser.cpp
    config.cpp
    preprocessor.cpp
    eigen33.cpp
    kd_tree.cpp
    normal_estimator.cpp
    ball_pivoting.cpp
)

target_include_directories(triangulator_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(triangulator_core PUBLIC 
    nlohmann_json::nlohmann_json 
    OpenMP::OpenMP_CXX
)
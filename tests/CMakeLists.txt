enable_testing()

add_executable(test_runner 
    unit_tests.cpp 
    integration_tests.cpp
)

target_link_libraries(test_runner PRIVATE 
    Catch2::Catch2WithMain 
    triangulator_core
)

target_compile_definitions(test_runner PRIVATE 
    TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../data"
)

add_test(NAME AllTests COMMAND test_runner)
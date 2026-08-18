add_executable(pct_core_tests
    unit/test_point_cloud.cpp
    unit/test_ply_io.cpp
    unit/test_statistics.cpp
    unit/test_transform.cpp
    unit/test_voxel_grid.cpp
    unit/test_brute_force.cpp
    unit/test_radius_outlier.cpp
    unit/test_kd_tree.cpp
    unit/test_normal_estimation.cpp
    unit/test_icp.cpp
)

target_link_libraries(pct_core_tests
    PRIVATE
        pct::core
        GTest::gtest_main
)
target_compile_features(pct_core_tests PRIVATE cxx_std_17)
target_compile_definitions(
    pct_core_tests
    PRIVATE PCT_TEST_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures"
)

if(MSVC)
    target_compile_options(pct_core_tests PRIVATE /W4 /permissive- /Zc:__cplusplus)
else()
    target_compile_options(pct_core_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

include(GoogleTest)
gtest_discover_tests(pct_core_tests)
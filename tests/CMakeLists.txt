add_executable(pct_core_tests
    unit/test_point_cloud.cpp
)

target_link_libraries(pct_core_tests
    PRIVATE
        pct::core
        GTest::gtest_main
)
target_compile_features(pct_core_tests PRIVATE cxx_std_17)

if(MSVC)
    target_compile_options(pct_core_tests PRIVATE /W4 /permissive- /Zc:__cplusplus)
else()
    target_compile_options(pct_core_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

include(GoogleTest)
gtest_discover_tests(pct_core_tests)
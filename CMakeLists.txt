cmake_minimum_required(VERSION 3.25)

project(
    PointCloudProcessingToolkit
    VERSION 0.1.0
    DESCRIPTION "A lightweight C++ point cloud processing toolkit"
    LANGUAGES CXX
)

include(CTest)
include(FetchContent)

set(CMAKE_CXX_EXTENSIONS OFF)

# Eigen 3.4.1 is a stable maintenance release in the Eigen 3.4 series.
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    Eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.1
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(Eigen)

add_library(pct_core
    src/core/point_cloud.cpp
)
add_library(pct::core ALIAS pct_core)

target_include_directories(pct_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(pct_core PUBLIC Eigen3::Eigen)
target_compile_features(pct_core PUBLIC cxx_std_17)

if(MSVC)
    target_compile_options(pct_core PRIVATE /W4 /permissive- /Zc:__cplusplus)
else()
    target_compile_options(pct_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(pointcloud_tool app/main.cpp)
target_link_libraries(pointcloud_tool PRIVATE pct::core)
target_compile_features(pointcloud_tool PRIVATE cxx_std_17)
target_compile_definitions(
    pointcloud_tool PRIVATE PCT_VERSION="${PROJECT_VERSION}"
)

if(MSVC)
    target_compile_options(pointcloud_tool PRIVATE /W4 /permissive- /Zc:__cplusplus)
else()
    target_compile_options(pointcloud_tool PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(BUILD_TESTING)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_Declare(
        GoogleTest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.17.0
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(GoogleTest)
    add_subdirectory(tests)

    add_test(NAME pct_cli_no_args COMMAND pointcloud_tool)
    add_test(NAME pct_cli_help COMMAND pointcloud_tool --help)
    add_test(NAME pct_cli_version COMMAND pointcloud_tool --version)
endif()

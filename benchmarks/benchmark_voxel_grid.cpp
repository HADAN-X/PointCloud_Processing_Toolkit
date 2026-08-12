#include "pct/filters/voxel_grid.hpp"

#include <charconv>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string_view>

#if defined(_WIN32)
#define NOMINMAX
#include <windows.h>
#include <psapi.h>
#elif defined(__unix__) || defined(__APPLE__)
#include <sys/resource.h>
#endif

namespace
{
    std::size_t parsePointCount(std::string_view text)
    {
        std::size_t value = 0;
        const char *const begin = text.data();
        const char *const end = begin + text.size();
        const auto result = std::from_chars(begin, end, value);
        if (result.ec != std::errc{} || result.ptr != end || value == 0U)
        {
            throw std::invalid_argument(
                "point count must be a positive integer");
        }
        return value;
    }

    float parseLeafSize(std::string_view text)
    {
        float value = 0.0F;
        const char *const begin = text.data();
        const char *const end = begin + text.size();
        const auto result = std::from_chars(begin, end, value);
        if (result.ec != std::errc{} || result.ptr != end ||
            !std::isfinite(value) || !(value > 0.0F))
        {
            throw std::invalid_argument(
                "leaf size must be finite and greater than zero");
        }
        return value;
    }

    std::size_t peakResidentBytes()
    {
#if defined(_WIN32)
        PROCESS_MEMORY_COUNTERS counters{};
        if (GetProcessMemoryInfo(GetCurrentProcess(), &counters, sizeof(counters)) == 0)
        {
            return 0U;
        }
        return counters.PeakWorkingSetSize;
#elif defined(__unix__) || defined(__APPLE__)
        rusage usage{};
        if (getrusage(RUSAGE_SELF, &usage) != 0)
        {
            return 0U;
        }
#if defined(__APPLE__)
        return static_cast<std::size_t>(usage.ru_maxrss);
#else
        return static_cast<std::size_t>(usage.ru_maxrss) * 1024U;
#endif
#else
        return 0U;
#endif
    }

    pct::PointCloud makeSyntheticCloud(std::size_t point_count)
    {
        pct::PointCloud cloud;
        cloud.points().reserve(point_count);
        std::mt19937 generator{20260811U};
        std::uniform_real_distribution<float> coordinate{-100.0F, 100.0F};
        for (std::size_t index = 0; index < point_count; ++index)
        {
            cloud.pushBack(
                pct::Point{{coordinate(generator),
                            coordinate(generator),
                            coordinate(generator)}});
        }
        return cloud;
    }
} // namespace

int main(int argc, char *argv[])
{
    if (argc > 3)
    {
        std::cerr << "Usage: pointcloud_voxel_benchmark "
                     "[point_count] [leaf_size]\n";
        return 2;
    }
    try
    {
        const std::size_t point_count =
            argc >= 2 ? parsePointCount(argv[1]) : 100000U;
        const float leaf_size =
            argc == 3 ? parseLeafSize(argv[2]) : 0.5F;
        const auto cloud = makeSyntheticCloud(point_count);

        const auto start = std::chrono::steady_clock::now();
        const auto result =
            pct::filters::voxelGridDownsample(cloud, leaf_size);
        const auto stop = std::chrono::steady_clock::now();
        const double elapsed_ms =
            std::chrono::duration<double, std::milli>(stop - start).count();
        const double peak_rss_mib =
            static_cast<double>(peakResidentBytes()) / (1024.0 * 1024.0);

        std::cout << std::fixed << std::setprecision(3)
                  << "input_points: " << point_count << '\n'
                  << "output_points: " << result.size() << '\n'
                  << "leaf_size: " << leaf_size << '\n'
                  << "processing_time_ms: " << elapsed_ms << '\n'
                  << "peak_rss_mib: " << peak_rss_mib << '\n';
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "Benchmark error: " << error.what() << '\n';
        return 1;
    }
}

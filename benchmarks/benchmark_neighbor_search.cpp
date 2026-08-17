#include "pct/search/brute_force.hpp"
#include "pct/search/kd_tree.hpp"

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace
{
    std::size_t parsePositiveSize(std::string_view text, const char *name)
    {
        std::size_t value = 0;
        const char *const begin = text.data();
        const char *const end = begin + text.size();
        const auto result = std::from_chars(begin, end, value);
        if (result.ec != std::errc{} || result.ptr != end || value == 0U)
        {
            throw std::invalid_argument(std::string{name} +
                                        " must be a positive integer");
        }
        return value;
    }

    float parsePositiveFloat(std::string_view text, const char *name)
    {
        float value = 0.0F;
        const char *const begin = text.data();
        const char *const end = begin + text.size();
        const auto result = std::from_chars(begin, end, value);
        if (result.ec != std::errc{} || result.ptr != end ||
            !std::isfinite(value) || !(value > 0.0F))
        {
            throw std::invalid_argument(std::string{name} +
                                        " must be finite and positive");
        }
        return value;
    }

    pct::PointCloud makeCloud(std::size_t count)
    {
        pct::PointCloud cloud;
        cloud.points().reserve(count);
        std::mt19937 generator{20260813U};
        std::uniform_real_distribution<float> coordinate{-100.0F, 100.0F};
        for (std::size_t index = 0; index < count; ++index)
        {
            cloud.pushBack(pct::Point{{coordinate(generator),
                                       coordinate(generator),
                                       coordinate(generator)}});
        }
        return cloud;
    }

    std::vector<Eigen::Vector3f> makeQueries(std::size_t count)
    {
        std::vector<Eigen::Vector3f> queries;
        queries.reserve(count);
        std::mt19937 generator{20260814U};
        std::uniform_real_distribution<float> coordinate{-100.0F, 100.0F};
        for (std::size_t index = 0; index < count; ++index)
        {
            queries.emplace_back(coordinate(generator), coordinate(generator),
                                 coordinate(generator));
        }
        return queries;
    }

    template <typename Function>
    double measureMilliseconds(Function &&function)
    {
        const auto start = std::chrono::steady_clock::now();
        function();
        const auto stop = std::chrono::steady_clock::now();
        return std::chrono::duration<double, std::milli>(stop - start).count();
    }

    double median(std::vector<double> values)
    {
        std::sort(values.begin(), values.end());
        return values[values.size() / 2U];
    }

    bool sameNeighbors(const std::vector<pct::search::Neighbor> &lhs,
                       const std::vector<pct::search::Neighbor> &rhs)
    {
        if (lhs.size() != rhs.size())
        {
            return false;
        }
        for (std::size_t index = 0; index < lhs.size(); ++index)
        {
            if (lhs[index].index != rhs[index].index ||
                lhs[index].squared_distance != rhs[index].squared_distance)
            {
                return false;
            }
        }
        return true;
    }
} // namespace

int main(int argc, char *argv[])
{
    if (argc > 6)
    {
        std::cerr << "Usage: pointcloud_neighbor_benchmark "
                     "[point_count] [query_count] [k] [radius] "
                     "[repetitions]\n";
        return 2;
    }
    try
    {
        const std::size_t point_count = argc >= 2
                                            ? parsePositiveSize(argv[1], "point count")
                                            : 100000U;
        const std::size_t query_count = argc >= 3
                                            ? parsePositiveSize(argv[2], "query count")
                                            : 1000U;
        const std::size_t k = argc >= 4 ? parsePositiveSize(argv[3], "k") : 16U;
        const float radius =
            argc >= 5 ? parsePositiveFloat(argv[4], "radius") : 5.0F;
        const std::size_t repetitions = argc == 6
                                            ? parsePositiveSize(argv[5], "repetitions")
                                            : 5U;
        const auto cloud = makeCloud(point_count);
        const auto queries = makeQueries(query_count);

        const pct::search::KdTree warmup_tree{cloud};
        // Query traversal
        for (std::size_t index = 0; index < std::min<std::size_t>(10U, query_count);
             ++index)
        {
            const auto kd_knn = warmup_tree.knnSearch(queries[index], k);
            const auto brute_knn =
                pct::search::knnSearch(cloud, queries[index], k);
            const auto kd_radius =
                warmup_tree.radiusSearch(queries[index], radius);
            const auto brute_radius =
                pct::search::radiusSearch(cloud, queries[index], radius);
            if (!sameNeighbors(kd_knn, brute_knn) ||
                !sameNeighbors(kd_radius, brute_radius))
            {
                throw std::runtime_error(
                    "KD-tree correctness check failed before timing");
            }
        }

        std::vector<double> build_times;
        std::vector<double> brute_knn_times;
        std::vector<double> kd_knn_times;
        std::vector<double> brute_radius_times;
        std::vector<double> kd_radius_times;
        build_times.reserve(repetitions);
        brute_knn_times.reserve(repetitions);
        kd_knn_times.reserve(repetitions);
        brute_radius_times.reserve(repetitions);
        kd_radius_times.reserve(repetitions);
        // Prevent the code from being completely optimized out due to the return value not being utilized.
        std::size_t checksum = 0U;

        for (std::size_t repetition = 0; repetition < repetitions; ++repetition)
        {
            double measured_build_ms = 0.0;
            std::optional<pct::search::KdTree> tree;
            measured_build_ms = measureMilliseconds([&]
                                                    { tree.emplace(cloud); });
            build_times.push_back(measured_build_ms);
            brute_knn_times.push_back(measureMilliseconds([&]
                                                          {
                                                           for (const auto &query : queries)
                                                           {
                                                               const auto result = pct::search::knnSearch(cloud, query, k);
                                                               checksum += result.front().index;
                                                           } }));
            kd_knn_times.push_back(measureMilliseconds([&]
                                                       {
                                                        for (const auto &query : queries)
                                                        {
                                                            const auto result = tree->knnSearch(query, k);
                                                            checksum += result.front().index;
                                                        } }));
            brute_radius_times.push_back(measureMilliseconds([&]
                                                             {
                                                              for (const auto &query : queries)
                                                              {
                                                                  const auto result = pct::search::radiusSearch(cloud, query, radius);
                                                                  checksum += result.size();
                                                              } }));
            kd_radius_times.push_back(measureMilliseconds([&]
                                                          {
                                                           for (const auto &query : queries)
                                                           {
                                                               const auto result = tree->radiusSearch(query, radius);
                                                               checksum += result.size();
                                                           } }));
        }

        const double build_median = median(build_times);
        const double brute_knn_median = median(brute_knn_times);
        const double kd_knn_median = median(kd_knn_times);
        const double brute_radius_median = median(brute_radius_times);
        const double kd_radius_median = median(kd_radius_times);
        std::cout << std::fixed << std::setprecision(3)
                  << "distribution: uniform_cube_seed_20260813\n"
                  << "point_count: " << point_count << '\n'
                  << "query_count: " << query_count << '\n'
                  << "k: " << k << '\n'
                  << "radius: " << radius << '\n'
                  << "repetitions: " << repetitions << '\n'
                  << "kd_build_median_ms: " << build_median << '\n'
                  << "brute_knn_median_ms: " << brute_knn_median << '\n'
                  << "kd_knn_median_ms: " << kd_knn_median << '\n'
                  << "knn_speedup: " << brute_knn_median / kd_knn_median << '\n'
                  << "brute_radius_median_ms: " << brute_radius_median << '\n'
                  << "kd_radius_median_ms: " << kd_radius_median << '\n'
                  << "radius_speedup: "
                  << brute_radius_median / kd_radius_median << '\n'
                  << "node_count: " << warmup_tree.stats().node_count << '\n'
                  << "max_depth: " << warmup_tree.stats().max_depth << '\n'
                  << "checksum: " << checksum << '\n';
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "Benchmark error: " << error.what() << '\n';
        return 1;
    }
}

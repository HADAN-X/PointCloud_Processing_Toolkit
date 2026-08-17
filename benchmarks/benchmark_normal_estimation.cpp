#include "pct/features/normal_estimation.hpp"

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace
{
    constexpr double kPi = 3.14159265358979323846;

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

    pct::PointCloud makeNoisyPlane(std::size_t grid_side,
                                   float noise_sigma,
                                   unsigned int seed)
    {
        pct::PointCloud cloud;
        cloud.points().reserve(grid_side * grid_side);
        std::mt19937 generator{seed};
        std::normal_distribution<float> noise{0.0F, noise_sigma};
        const float denominator =
            static_cast<float>(std::max<std::size_t>(grid_side - 1U, 1U));
        for (std::size_t y = 0; y < grid_side; ++y)
        {
            for (std::size_t x = 0; x < grid_side; ++x)
            {
                const float coordinate_x =
                    -1.0F + 2.0F * static_cast<float>(x) / denominator;
                const float coordinate_y =
                    -1.0F + 2.0F * static_cast<float>(y) / denominator;
                cloud.pushBack(pct::Point{{
                    coordinate_x, coordinate_y, noise(generator)}});
            }
        }
        return cloud;
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

    double percentile95(std::vector<double> values)
    {
        std::sort(values.begin(), values.end());
        const std::size_t index =
            static_cast<std::size_t>(
                0.95 * static_cast<double>(values.size() - 1U));
        return values[index];
    }

    struct Accuracy
    {
        std::size_t valid_count{0};
        double mean_angle_degrees{0.0};
        double median_angle_degrees{0.0};
        double p95_angle_degrees{0.0};
        double mean_curvature{0.0};
    };

    Accuracy evaluate(
        const pct::features::NormalEstimates &estimates)
    {
        Accuracy accuracy;
        std::vector<double> angles;
        angles.reserve(estimates.size());
        double angle_sum = 0.0;
        double curvature_sum = 0.0;
        for (const auto &estimate : estimates)
        {
            if (!estimate.valid())
            {
                continue;
            }
            const double cosine = std::clamp(
                std::abs(static_cast<double>(estimate.normal.z())),
                0.0,
                1.0);
            const double angle = std::acos(cosine) * 180.0 / kPi;
            angles.push_back(angle);
            angle_sum += angle;
            curvature_sum += static_cast<double>(estimate.curvature);
        }
        accuracy.valid_count = angles.size();
        if (angles.empty())
        {
            return accuracy;
        }
        accuracy.mean_angle_degrees =
            angle_sum / static_cast<double>(angles.size());
        accuracy.median_angle_degrees = median(angles);
        accuracy.p95_angle_degrees = percentile95(angles);
        accuracy.mean_curvature =
            curvature_sum / static_cast<double>(angles.size());
        return accuracy;
    }
} // namespace

int main(int argc, char *argv[])
{
    if (argc > 4)
    {
        std::cerr << "Usage: pointcloud_normal_benchmark "
                     "[grid_side] [k] [repetitions]\n";
        return 2;
    }

    try
    {
        const std::size_t grid_side =
            argc >= 2 ? parsePositiveSize(argv[1], "grid side") : 100U;
        const std::size_t k =
            argc >= 3 ? parsePositiveSize(argv[2], "k") : 20U;
        const std::size_t repetitions =
            argc == 4 ? parsePositiveSize(argv[3], "repetitions") : 5U;
        if (grid_side * grid_side < 3U || k < 3U)
        {
            throw std::invalid_argument(
                "the cloud and k must provide at least three neighbors");
        }

        const std::vector<float> noise_levels{
            0.0F, 0.001F, 0.005F, 0.01F};
        std::size_t checksum = 0U;
        std::cout << std::fixed << std::setprecision(6)
                  << "distribution: regular_xy_grid_with_gaussian_z_noise\n"
                  << "seed: 20260817\n"
                  << "grid_side: " << grid_side << '\n'
                  << "point_count: " << grid_side * grid_side << '\n'
                  << "k: " << k << '\n'
                  << "repetitions: " << repetitions << '\n'
                  << "noise_sigma,total_median_ms,valid_count,mean_angle_deg,"
                     "median_angle_deg,p95_angle_deg,mean_curvature\n";

        for (std::size_t level = 0; level < noise_levels.size(); ++level)
        {
            const float sigma = noise_levels[level];
            const auto cloud = makeNoisyPlane(
                grid_side,
                sigma,
                20260817U + static_cast<unsigned int>(level));
            std::vector<double> times;
            times.reserve(repetitions);
            pct::features::NormalEstimates estimates;

            static_cast<void>(
                pct::features::estimateNormalsKnn(cloud, k));
            for (std::size_t repetition = 0;
                 repetition < repetitions;
                 ++repetition)
            {
                times.push_back(measureMilliseconds([&]
                                                    {
                    estimates = pct::features::estimateNormalsKnn(cloud, k);
                }));
                for (const auto &estimate : estimates)
                {
                    checksum += estimate.valid()
                                    ? static_cast<std::size_t>(
                                          estimate.neighbor_count)
                                    : 0U;
                }
            }

            const Accuracy accuracy = evaluate(estimates);
            std::cout << sigma << ','
                      << median(times) << ','
                      << accuracy.valid_count << ','
                      << accuracy.mean_angle_degrees << ','
                      << accuracy.median_angle_degrees << ','
                      << accuracy.p95_angle_degrees << ','
                      << accuracy.mean_curvature << '\n';
        }
        std::cout << "checksum: " << checksum << '\n';
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "Benchmark error: " << error.what() << '\n';
        return 1;
    }
}

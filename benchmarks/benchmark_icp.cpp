#include "pct/geometry/transform.hpp"
#include "pct/registration/icp.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
    constexpr float kPi = 3.14159265358979323846F;

    struct Scenario
    {
        const char *name;
        double noise_sigma;
        double outlier_ratio;
        double overlap_ratio;
        double initial_rotation_error_degrees;
        double initial_translation_error;
    };

    struct Aggregate
    {
        double milliseconds{0.0};
        double iterations{0.0};
        double correspondences{0.0};
        double rmse{0.0};
        double rotation_error_degrees{0.0};
        double translation_error{0.0};
        std::size_t converged_runs{0};
    };

    pct::geometry::Matrix4f makeTransform(
        float angle_degrees,
        const Eigen::Vector3f &axis,
        const Eigen::Vector3f &translation)
    {
        const float radians = angle_degrees * kPi / 180.0F;
        return pct::geometry::makeRigidTransform(
            Eigen::AngleAxisf(radians, axis.normalized()).toRotationMatrix(),
            translation);
    }

    pct::PointCloud makeSource(std::size_t count, std::uint32_t seed)
    {
        std::mt19937 generator(seed);
        std::uniform_real_distribution<float> distribution(-2.0F, 2.0F);
        pct::PointCloud cloud;
        for (std::size_t i = 0; i < count; ++i)
        {
            cloud.pushBack(pct::Point(Eigen::Vector3f{
                distribution(generator),
                distribution(generator),
                distribution(generator)}));
        }
        return cloud;
    }

    pct::PointCloud makeTarget(
        const pct::PointCloud &source,
        const pct::geometry::Matrix4f &ground_truth,
        double overlap_ratio,
        double noise_sigma,
        std::uint32_t seed)
    {
        const std::size_t overlap_count = std::max<std::size_t>(
            3, static_cast<std::size_t>(
                   std::floor(static_cast<double>(source.size()) * overlap_ratio)));
        std::mt19937 generator(seed);
        std::normal_distribution<float> noise(
            0.0F, static_cast<float>(noise_sigma));
        const Eigen::Matrix3f rotation = ground_truth.block<3, 3>(0, 0);
        const Eigen::Vector3f translation = ground_truth.block<3, 1>(0, 3);

        pct::PointCloud target;
        for (std::size_t i = 0; i < overlap_count; ++i)
        {
            Eigen::Vector3f position = rotation * source[i].position + translation;
            position += Eigen::Vector3f{
                noise(generator), noise(generator), noise(generator)};
            target.pushBack(pct::Point(position));
        }
        return target;
    }

    void appendOutliers(
        pct::PointCloud &source,
        std::size_t inlier_count,
        double outlier_ratio,
        std::uint32_t seed)
    {
        const std::size_t outlier_count = static_cast<std::size_t>(
            std::floor(static_cast<double>(inlier_count) * outlier_ratio));
        std::mt19937 generator(seed);
        std::uniform_real_distribution<float> distribution(8.0F, 12.0F);
        for (std::size_t i = 0; i < outlier_count; ++i)
        {
            source.pushBack(pct::Point(Eigen::Vector3f{
                distribution(generator),
                distribution(generator),
                distribution(generator)}));
        }
    }

    double rotationErrorDegrees(
        const pct::geometry::Matrix4f &estimated,
        const pct::geometry::Matrix4f &expected)
    {
        const Eigen::Matrix3f rotation_error =
            estimated.block<3, 3>(0, 0) *
            expected.block<3, 3>(0, 0).transpose();
        const double cosine = std::clamp(
            (static_cast<double>(rotation_error.trace()) - 1.0) * 0.5,
            -1.0,
            1.0);
        return std::acos(cosine) * 180.0 / static_cast<double>(kPi);
    }

    double translationError(
        const pct::geometry::Matrix4f &estimated,
        const pct::geometry::Matrix4f &expected)
    {
        const Eigen::Matrix4f error = estimated * expected.inverse();
        return static_cast<double>(error.block<3, 1>(0, 3).norm());
    }

    pct::geometry::Matrix4f makeInitialTransform(
        const pct::geometry::Matrix4f &ground_truth,
        const Scenario &scenario)
    {
        if (scenario.initial_rotation_error_degrees == 0.0 &&
            scenario.initial_translation_error == 0.0)
        {
            return ground_truth;
        }
        const auto perturbation = makeTransform(
            static_cast<float>(scenario.initial_rotation_error_degrees),
            Eigen::Vector3f{0.2F, -0.6F, 0.7F},
            Eigen::Vector3f{
                static_cast<float>(scenario.initial_translation_error),
                0.0F,
                0.0F});
        return perturbation * ground_truth;
    }

    std::size_t parsePositive(const char *text, const char *name)
    {
        const unsigned long long value = std::stoull(text);
        if (value == 0)
        {
            throw std::invalid_argument(std::string(name) + " must be positive");
        }
        return static_cast<std::size_t>(value);
    }
} // namespace

int main(int argc, char **argv)
{
    try
    {
        if (argc > 3)
        {
            std::cerr << "usage: pointcloud_icp_benchmark [point_count] [repetitions]\n";
            return 2;
        }
        const std::size_t point_count =
            argc >= 2 ? parsePositive(argv[1], "point_count") : 2000U;
        const std::size_t repetitions =
            argc >= 3 ? parsePositive(argv[2], "repetitions") : 5U;

        const std::vector<Scenario> scenarios{
            {"exact_identity_init", 0.0, 0.0, 1.0, -5.0, -0.08},
            {"noise_2mm", 0.002, 0.0, 1.0, -5.0, -0.08},
            {"outliers_20pct", 0.0, 0.2, 1.0, -5.0, -0.08},
            {"overlap_70pct", 0.0, 0.0, 0.7, -5.0, -0.08},
            {"initial_error_5deg", 0.0, 0.0, 1.0, 5.0, 0.05},
            {"initial_error_15deg", 0.0, 0.0, 1.0, 15.0, 0.15},
            {"perfect_initial", 0.0, 0.0, 1.0, 0.0, 0.0}};

        const auto ground_truth = makeTransform(
            5.0F,
            Eigen::Vector3f{0.3F, -0.2F, 0.9F},
            Eigen::Vector3f{0.08F, -0.04F, 0.03F});

        std::cout << "point_count=" << point_count
                  << ", repetitions=" << repetitions << '\n';
        std::cout << "scenario,success,avg_ms,avg_iterations,avg_correspondences,"
                     "avg_rmse,rotation_error_deg,translation_error\n";
        std::cout << std::fixed << std::setprecision(6);

        for (std::size_t scenario_index = 0;
             scenario_index < scenarios.size();
             ++scenario_index)
        {
            const Scenario &scenario = scenarios[scenario_index];
            Aggregate aggregate;

            for (std::size_t repetition = 0; repetition < repetitions; ++repetition)
            {
                const std::uint32_t seed = static_cast<std::uint32_t>(
                    20260817U + scenario_index * 100U + repetition);
                pct::PointCloud source = makeSource(point_count, seed);
                const pct::PointCloud target = makeTarget(
                    source,
                    ground_truth,
                    scenario.overlap_ratio,
                    scenario.noise_sigma,
                    seed + 1U);
                appendOutliers(
                    source, point_count, scenario.outlier_ratio, seed + 2U);

                pct::registration::IcpOptions options;
                options.max_iterations = 80;
                options.max_correspondence_distance = 0.35F;
                options.translation_epsilon = 1.0e-7;
                options.rotation_epsilon_radians = 1.0e-7;
                options.rmse_epsilon = 1.0e-8;
                options.initial_transform =
                    makeInitialTransform(ground_truth, scenario);

                const auto start = std::chrono::steady_clock::now();
                const auto result = pct::registration::alignPointToPoint(
                    source, target, options);
                const auto stop = std::chrono::steady_clock::now();

                aggregate.milliseconds +=
                    std::chrono::duration<double, std::milli>(stop - start).count();
                aggregate.iterations += static_cast<double>(result.history.size());
                if (!result.history.empty())
                {
                    aggregate.correspondences += static_cast<double>(
                        result.history.back().correspondence_count);
                }
                aggregate.rmse += result.final_rmse;
                aggregate.rotation_error_degrees +=
                    rotationErrorDegrees(result.transform, ground_truth);
                aggregate.translation_error +=
                    translationError(result.transform, ground_truth);
                if (result.converged())
                {
                    ++aggregate.converged_runs;
                }
            }

            const double inverse_repetitions =
                1.0 / static_cast<double>(repetitions);
            std::cout << scenario.name << ','
                      << aggregate.converged_runs << '/' << repetitions << ','
                      << aggregate.milliseconds * inverse_repetitions << ','
                      << aggregate.iterations * inverse_repetitions << ','
                      << aggregate.correspondences * inverse_repetitions << ','
                      << aggregate.rmse * inverse_repetitions << ','
                      << aggregate.rotation_error_degrees * inverse_repetitions << ','
                      << aggregate.translation_error * inverse_repetitions << '\n';
        }
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
}
#include "pct/registration/icp.hpp"

#include "pct/geometry/transform.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>

namespace
{
    constexpr float kPi = 3.14159265358979323846F;

    pct::PointCloud makeRandomCloud(std::size_t count)
    {
        std::mt19937 generator(20260817U);
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

    pct::geometry::Matrix4f makeTransform(
        float angle_radians,
        const Eigen::Vector3f &axis,
        const Eigen::Vector3f &translation)
    {
        const Eigen::Matrix3f rotation =
            Eigen::AngleAxisf(angle_radians, axis.normalized()).toRotationMatrix();
        return pct::geometry::makeRigidTransform(rotation, translation);
    }

    double translationError(
        const pct::geometry::Matrix4f &estimated,
        const pct::geometry::Matrix4f &expected)
    {
        const Eigen::Matrix4f error = estimated * expected.inverse();
        return static_cast<double>(error.block<3, 1>(0, 3).norm());
    }

    double rotationError(
        const pct::geometry::Matrix4f &estimated,
        const pct::geometry::Matrix4f &expected)
    {
        const Eigen::Matrix3f error_rotation =
            estimated.block<3, 3>(0, 0) *
            expected.block<3, 3>(0, 0).transpose();
        const double cosine = std::clamp(
            (static_cast<double>(error_rotation.trace()) - 1.0) * 0.5,
            -1.0,
            1.0);
        return std::acos(cosine);
    }

    pct::registration::IcpOptions accurateOptions()
    {
        pct::registration::IcpOptions options;
        options.max_iterations = 80;
        options.max_correspondence_distance = 0.6F;
        options.translation_epsilon = 1.0e-7;
        options.rotation_epsilon_radians = 1.0e-7;
        options.rmse_epsilon = 1.0e-9;
        return options;
    }
} // namespace

TEST(Icp, RecoversPureTranslation)
{
    const pct::PointCloud source = makeRandomCloud(250);
    const auto expected =
        pct::geometry::makeTranslation(Eigen::Vector3f{0.05F, -0.03F, 0.02F});
    const pct::PointCloud target =
        pct::geometry::transformPointCloud(source, expected);

    const auto result =
        pct::registration::alignPointToPoint(source, target, accurateOptions());

    EXPECT_TRUE(result.converged());
    EXPECT_LT(translationError(result.transform, expected), 1.0e-4);
    EXPECT_LT(rotationError(result.transform, expected), 1.0e-3);
    EXPECT_LT(result.final_rmse, 1.0e-5);
}

TEST(Icp, RecoversPureRotation)
{
    const pct::PointCloud source = makeRandomCloud(250);
    const auto expected = makeTransform(
        2.0F * kPi / 180.0F,
        Eigen::Vector3f{0.3F, 0.8F, 0.5F},
        Eigen::Vector3f::Zero());
    const pct::PointCloud target =
        pct::geometry::transformPointCloud(source, expected);

    const auto result =
        pct::registration::alignPointToPoint(source, target, accurateOptions());

    EXPECT_TRUE(result.converged());
    EXPECT_LT(translationError(result.transform, expected), 1.0e-4);
    EXPECT_LT(rotationError(result.transform, expected), 1.0e-3);
}

TEST(Icp, RecoversMixedRigidTransform)
{
    const pct::PointCloud source = makeRandomCloud(300);
    const auto expected = makeTransform(
        3.0F * kPi / 180.0F,
        Eigen::Vector3f{0.2F, -0.4F, 0.9F},
        Eigen::Vector3f{0.04F, -0.02F, 0.03F});
    const pct::PointCloud target =
        pct::geometry::transformPointCloud(source, expected);

    const auto result =
        pct::registration::alignPointToPoint(source, target, accurateOptions());

    EXPECT_TRUE(result.converged());
    EXPECT_LT(translationError(result.transform, expected), 1.0e-4);
    EXPECT_LT(rotationError(result.transform, expected), 1.0e-3);
    EXPECT_TRUE(pct::geometry::isRigidTransform(result.transform));
    const float determinant = result.transform.block<3, 3>(0, 0).determinant();
    EXPECT_GT(determinant, 0.0F);
}

TEST(Icp, InitialPoseEnablesRegistrationOutsideDistanceGate)
{
    const pct::PointCloud source = makeRandomCloud(200);
    const auto expected = makeTransform(
        8.0F * kPi / 180.0F,
        Eigen::Vector3f::UnitZ(),
        Eigen::Vector3f{10.0F, 0.0F, 0.0F});
    const pct::PointCloud target =
        pct::geometry::transformPointCloud(source, expected);
    auto options = accurateOptions();
    options.max_correspondence_distance = 0.25F;

    const auto without_initial =
        pct::registration::alignPointToPoint(source, target, options);
    EXPECT_EQ(without_initial.termination,
              pct::registration::IcpTermination::InsufficientCorrespondences);

    options.initial_transform = makeTransform(
        7.0F * kPi / 180.0F,
        Eigen::Vector3f::UnitZ(),
        Eigen::Vector3f{9.95F, 0.02F, 0.0F});
    const auto with_initial =
        pct::registration::alignPointToPoint(source, target, options);

    EXPECT_TRUE(with_initial.converged());
    EXPECT_LT(translationError(with_initial.transform, expected), 1.0e-4);
    EXPECT_LT(rotationError(with_initial.transform, expected), 1.0e-3);
}

TEST(Icp, ReportsInsufficientCorrespondences)
{
    pct::PointCloud source;
    source.pushBack(pct::Point(Eigen::Vector3f::Zero()));
    source.pushBack(pct::Point(Eigen::Vector3f::UnitX()));
    const pct::PointCloud target = source;

    const auto result = pct::registration::alignPointToPoint(source, target);

    EXPECT_EQ(result.termination,
              pct::registration::IcpTermination::InsufficientCorrespondences);
    EXPECT_FALSE(result.converged());
    EXPECT_TRUE(result.history.empty());
}

TEST(Icp, ReportsDegenerateCollinearGeometry)
{
    pct::PointCloud source;
    for (int i = 0; i < 10; ++i)
    {
        source.pushBack(pct::Point(Eigen::Vector3f{
            static_cast<float>(i), 0.0F, 0.0F}));
    }
    const pct::PointCloud target = source;

    const auto result = pct::registration::alignPointToPoint(source, target);

    EXPECT_EQ(result.termination,
              pct::registration::IcpTermination::DegenerateGeometry);
    EXPECT_FALSE(result.converged());
}

TEST(Icp, ReportsMaximumIterationsAndKeepsHistory)
{
    const pct::PointCloud source = makeRandomCloud(200);
    const auto expected =
        pct::geometry::makeTranslation(Eigen::Vector3f{0.05F, 0.0F, 0.0F});
    const pct::PointCloud target =
        pct::geometry::transformPointCloud(source, expected);
    auto options = accurateOptions();
    options.max_iterations = 1;
    options.translation_epsilon = 0.0;
    options.rotation_epsilon_radians = 0.0;
    options.rmse_epsilon = 0.0;

    const auto result =
        pct::registration::alignPointToPoint(source, target, options);

    EXPECT_EQ(result.termination,
              pct::registration::IcpTermination::MaximumIterations);
    ASSERT_EQ(result.history.size(), 1U);
    EXPECT_EQ(result.history.front().iteration, 1U);
    EXPECT_EQ(result.history.front().correspondence_count, source.size());
    EXPECT_TRUE(std::isfinite(result.history.front().rmse));
}

TEST(Icp, RejectsInvalidOptions)
{
    const pct::PointCloud cloud = makeRandomCloud(10);
    pct::registration::IcpOptions options;
    options.max_iterations = 0;
    EXPECT_THROW(
        (void)pct::registration::alignPointToPoint(cloud, cloud, options),
        std::invalid_argument);

    options = {};
    options.max_correspondence_distance = 0.0F;
    EXPECT_THROW(
        (void)pct::registration::alignPointToPoint(cloud, cloud, options),
        std::invalid_argument);

    options = {};
    options.minimum_correspondences = 2;
    EXPECT_THROW(
        (void)pct::registration::alignPointToPoint(cloud, cloud, options),
        std::invalid_argument);
}

TEST(Icp, RejectsNonFiniteInput)
{
    pct::PointCloud source = makeRandomCloud(10);
    const pct::PointCloud target = source;
    source[3].position.x() = std::numeric_limits<float>::quiet_NaN();

    EXPECT_THROW(
        (void)pct::registration::alignPointToPoint(source, target),
        std::invalid_argument);
}

TEST(Icp, TerminationNamesAreStable)
{
    using pct::registration::IcpTermination;
    EXPECT_STREQ(pct::registration::toString(IcpTermination::ConvergedTransform),
                 "converged_transform");
    EXPECT_STREQ(pct::registration::toString(IcpTermination::ConvergedRmse),
                 "converged_rmse");
    EXPECT_STREQ(pct::registration::toString(IcpTermination::MaximumIterations),
                 "maximum_iterations");
    EXPECT_STREQ(
        pct::registration::toString(IcpTermination::InsufficientCorrespondences),
        "insufficient_correspondences");
    EXPECT_STREQ(pct::registration::toString(IcpTermination::DegenerateGeometry),
                 "degenerate_geometry");
    EXPECT_STREQ(pct::registration::toString(IcpTermination::NumericalFailure),
                 "numerical_failure");
}
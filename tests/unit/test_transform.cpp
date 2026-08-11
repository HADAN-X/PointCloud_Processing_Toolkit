#include "pct/geometry/transform.hpp"
#include <Eigen/LU>
#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>
namespace
{
    constexpr float kPi = 3.14159265358979323846F;

    pct::PointCloud onePoint(const Eigen::Vector3f &position)
    {
        pct::PointCloud cloud;
        cloud.pushBack(pct::Point{position});
        return cloud;
    }

    TEST(RigidTransformTest, IdentityLeavesPointUnchanged)
    {
        const auto cloud = onePoint({1.0F, 2.0F, 3.0F});
        const auto result = pct::geometry::transformPointCloud(
            cloud, pct::geometry::Matrix4f::Identity());
        EXPECT_TRUE(result[0].position.isApprox(cloud[0].position));
    }

    TEST(RigidTransformTest, TranslationMovesPoint)
    {
        const auto result = pct::geometry::transformPointCloud(
            onePoint({1.0F, 2.0F, 3.0F}),
            pct::geometry::makeTranslation({4.0F, -2.0F, 0.5F}));
        EXPECT_TRUE(result[0].position.isApprox(
            Eigen::Vector3f{5.0F, 0.0F, 3.5F}));
    }

    TEST(RigidTransformTest, RotatesNinetyDegreesAroundX)
    {
        const auto result = pct::geometry::transformPointCloud(
            onePoint({0.0F, 1.0F, 0.0F}),
            pct::geometry::makeRotationX(kPi / 2.0F));
        EXPECT_TRUE(result[0].position.isApprox(
            Eigen::Vector3f{0.0F, 0.0F, 1.0F}, 1.0e-5F));
    }

    TEST(RigidTransformTest, RotatesNinetyDegreesAroundY)
    {
        const auto result = pct::geometry::transformPointCloud(
            onePoint({0.0F, 0.0F, 1.0F}),
            pct::geometry::makeRotationY(kPi / 2.0F));
        EXPECT_TRUE(result[0].position.isApprox(
            Eigen::Vector3f{1.0F, 0.0F, 0.0F}, 1.0e-5F));
    }

    TEST(RigidTransformTest, RotatesNinetyDegreesAroundZ)
    {
        const auto result = pct::geometry::transformPointCloud(
            onePoint({1.0F, 0.0F, 0.0F}),
            pct::geometry::makeRotationZ(kPi / 2.0F));
        EXPECT_TRUE(result[0].position.isApprox(
            Eigen::Vector3f{0.0F, 1.0F, 0.0F}, 1.0e-5F));
    }

    TEST(RigidTransformTest, MatrixMultiplicationUsesRightToLeftOrder)
    {
        const auto rotation = pct::geometry::makeRotationZ(kPi / 2.0F);
        const auto translation =
            pct::geometry::makeTranslation({10.0F, 0.0F, 0.0F});
        const auto result = pct::geometry::transformPointCloud(
            onePoint({1.0F, 0.0F, 0.0F}), translation * rotation);
        EXPECT_TRUE(result[0].position.isApprox(
            Eigen::Vector3f{10.0F, 1.0F, 0.0F}, 1.0e-5F));
    }

    TEST(RigidTransformTest, InverseRecoversOriginalPoint)
    {
        const pct::geometry::Matrix4f transform =
            pct::geometry::makeTranslation({1.0F, 2.0F, 3.0F}) *
            pct::geometry::makeRotationY(0.4F);
        const auto cloud = onePoint({-2.0F, 5.0F, 7.0F});
        const auto moved =
            pct::geometry::transformPointCloud(cloud, transform);
        const auto recovered = pct::geometry::transformPointCloud(
            moved, transform.inverse().eval());
        EXPECT_TRUE(recovered[0].position.isApprox(cloud[0].position,
                                                   1.0e-5F));
    }

    TEST(RigidTransformTest, PreservesColor)
    {
        pct::PointCloud cloud;
        cloud.pushBack(pct::Point{{1.0F, 2.0F, 3.0F}, {10, 20, 30}});
        const auto result = pct::geometry::transformPointCloud(
            cloud, pct::geometry::makeTranslation({1.0F, 0.0F, 0.0F}));
        ASSERT_TRUE(result[0].color.has_value());
        EXPECT_EQ(*result[0].color, (pct::ColorRgb8{10, 20, 30}));
    }

    TEST(RigidTransformTest, EmptyCloudRemainsEmpty)
    {
        const pct::PointCloud cloud;
        EXPECT_TRUE(pct::geometry::transformPointCloud(
                        cloud, pct::geometry::Matrix4f::Identity())
                        .empty());
    }
    TEST(RigidTransformTest, RejectsNonFiniteMatrix)
    {
        pct::geometry::Matrix4f transform =
            pct::geometry::Matrix4f::Identity();
        transform(0, 0) = std::numeric_limits<float>::quiet_NaN();
        EXPECT_THROW(static_cast<void>(pct::geometry::transformPointCloud(
                         onePoint({0.0F, 0.0F, 0.0F}), transform)),
                     std::invalid_argument);
    }

    TEST(RigidTransformTest, RejectsScaling)
    {
        pct::geometry::Matrix4f transform =
            pct::geometry::Matrix4f::Identity();
        transform(0, 0) = 2.0F;
        EXPECT_FALSE(pct::geometry::isRigidTransform(transform));
        EXPECT_THROW(static_cast<void>(pct::geometry::transformPointCloud(
                         onePoint({0.0F, 0.0F, 0.0F}), transform)),
                     std::invalid_argument);
    }

    TEST(RigidTransformTest, RejectsInvalidHomogeneousBottomRow)
    {
        pct::geometry::Matrix4f transform =
            pct::geometry::Matrix4f::Identity();
        transform(3, 0) = 1.0F;
        EXPECT_FALSE(pct::geometry::isRigidTransform(transform));
    }

    TEST(RigidTransformTest, RejectsNonFiniteInputPoint)
    {
        EXPECT_THROW(
            static_cast<void>(pct::geometry::transformPointCloud(
                onePoint({std::numeric_limits<float>::infinity(), 0.0F, 0.0F}),
                pct::geometry::Matrix4f::Identity())),
            std::invalid_argument);
    }

} // namespace
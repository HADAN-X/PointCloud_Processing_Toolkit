#include "pct/features/normal_estimation.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>

namespace
{
    constexpr float kPi = 3.14159265358979323846F;

    pct::PointCloud makePlane(int half_extent, float spacing = 1.0F)
    {
        pct::PointCloud cloud;
        const int width = 2 * half_extent + 1;
        cloud.points().reserve(
            static_cast<std::size_t>(width * width));
        for (int y = -half_extent; y <= half_extent; ++y)
        {
            for (int x = -half_extent; x <= half_extent; ++x)
            {
                cloud.pushBack(pct::Point{{spacing * static_cast<float>(x),
                                           spacing * static_cast<float>(y),
                                           2.0F}});
            }
        }
        return cloud;
    }

    pct::PointCloud makeSphere(float radius,
                               int latitude_count,
                               int longitude_count)
    {
        pct::PointCloud cloud;
        cloud.points().reserve(static_cast<std::size_t>(
            latitude_count * longitude_count));
        for (int latitude = 0; latitude < latitude_count; ++latitude)
        {
            const float polar = kPi *
                                (static_cast<float>(latitude) + 0.5F) /
                                static_cast<float>(latitude_count);
            for (int longitude = 0; longitude < longitude_count;
                 ++longitude)
            {
                const float azimuth = 2.0F * kPi *
                                      static_cast<float>(longitude) /
                                      static_cast<float>(longitude_count);
                cloud.pushBack(pct::Point{{radius * std::sin(polar) * std::cos(azimuth),
                                           radius * std::sin(polar) * std::sin(azimuth),
                                           radius * std::cos(polar)}});
            }
        }
        return cloud;
    }

    void expectFiniteInvalid(
        const pct::features::NormalEstimate &estimate,
        pct::features::NormalEstimationStatus expected_status)
    {
        EXPECT_EQ(estimate.status, expected_status);
        EXPECT_FALSE(estimate.valid());
        EXPECT_TRUE(estimate.normal.allFinite());
        EXPECT_FLOAT_EQ(estimate.normal.squaredNorm(), 0.0F);
        EXPECT_TRUE(std::isfinite(estimate.curvature));
        EXPECT_FLOAT_EQ(estimate.curvature, 0.0F);
    }

    TEST(NormalEstimationTest, EmptyCloudProducesEmptyResults)
    {
        const pct::PointCloud cloud;
        EXPECT_TRUE(pct::features::estimateNormalsKnn(cloud, 3U).empty());
        EXPECT_TRUE(
            pct::features::estimateNormalsRadius(cloud, 1.0F).empty());
    }

    TEST(NormalEstimationTest, RejectsInvalidParametersAndPositions)
    {
        const auto cloud = makePlane(1);
        EXPECT_THROW(
            static_cast<void>(
                pct::features::estimateNormalsKnn(cloud, 2U)),
            std::invalid_argument);
        EXPECT_THROW(
            static_cast<void>(
                pct::features::estimateNormalsKnn(cloud, 3U, 2U)),
            std::invalid_argument);
        EXPECT_THROW(
            static_cast<void>(
                pct::features::estimateNormalsRadius(cloud, 0.0F)),
            std::invalid_argument);
        EXPECT_THROW(
            static_cast<void>(pct::features::estimateNormalsRadius(
                cloud, std::numeric_limits<float>::infinity())),
            std::invalid_argument);
        EXPECT_THROW(
            static_cast<void>(pct::features::estimateNormalsKnn(
                cloud,
                3U,
                3U,
                Eigen::Vector3f{
                    std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F})),
            std::invalid_argument);

        auto invalid_cloud = cloud;
        invalid_cloud[0].position.z() =
            std::numeric_limits<float>::quiet_NaN();
        EXPECT_THROW(
            static_cast<void>(
                pct::features::estimateNormalsKnn(invalid_cloud, 3U)),
            std::invalid_argument);
    }

    TEST(NormalEstimationTest, KnnRecoversExactPlaneNormalsAndCurvature)
    {
        const auto cloud = makePlane(3);
        const auto estimates =
            pct::features::estimateNormalsKnn(cloud, 9U);
        ASSERT_EQ(estimates.size(), cloud.size());
        for (const auto &estimate : estimates)
        {
            ASSERT_TRUE(estimate.valid());
            EXPECT_EQ(estimate.neighbor_count, 9U);
            EXPECT_NEAR(estimate.normal.norm(), 1.0F, 1.0e-6F);
            EXPECT_GT(estimate.normal.z(), 0.99999F);
            EXPECT_NEAR(estimate.curvature, 0.0F, 1.0e-7F);
        }
    }

    TEST(NormalEstimationTest, RadiusSearchRecoversPlaneAtBoundaries)
    {
        const auto cloud = makePlane(2);
        const auto estimates =
            pct::features::estimateNormalsRadius(cloud, 1.5F, 4U);
        ASSERT_EQ(estimates.size(), cloud.size());
        for (const auto &estimate : estimates)
        {
            ASSERT_TRUE(estimate.valid());
            EXPECT_GE(estimate.neighbor_count, 4U);
            EXPECT_GT(estimate.normal.z(), 0.99999F);
            EXPECT_NEAR(estimate.curvature, 0.0F, 1.0e-7F);
        }
    }

    TEST(NormalEstimationTest, ViewpointControlsNormalOrientation)
    {
        const auto cloud = makePlane(2);
        const auto above = pct::features::estimateNormalsKnn(
            cloud, 9U, 3U, Eigen::Vector3f{0.0F, 0.0F, 10.0F});
        const auto below = pct::features::estimateNormalsKnn(
            cloud, 9U, 3U, Eigen::Vector3f{0.0F, 0.0F, -10.0F});
        ASSERT_EQ(above.size(), below.size());
        for (std::size_t index = 0; index < above.size(); ++index)
        {
            ASSERT_TRUE(above[index].valid());
            ASSERT_TRUE(below[index].valid());
            EXPECT_GT(above[index].normal.z(), 0.99999F);
            EXPECT_LT(below[index].normal.z(), -0.99999F);
        }
    }

    TEST(NormalEstimationTest, ReportsInsufficientNeighborsWithoutNan)
    {
        pct::PointCloud cloud;
        cloud.pushBack(pct::Point{{0.0F, 0.0F, 0.0F}});
        cloud.pushBack(pct::Point{{1.0F, 0.0F, 0.0F}});
        const auto estimates =
            pct::features::estimateNormalsKnn(cloud, 3U);
        ASSERT_EQ(estimates.size(), 2U);
        for (const auto &estimate : estimates)
        {
            EXPECT_EQ(estimate.neighbor_count, 2U);
            expectFiniteInvalid(
                estimate,
                pct::features::NormalEstimationStatus::InsufficientNeighbors);
        }
    }

    TEST(NormalEstimationTest, ReportsCollinearNeighborhoodsAsDegenerate)
    {
        pct::PointCloud cloud;
        for (int x = -4; x <= 4; ++x)
        {
            cloud.pushBack(
                pct::Point{{static_cast<float>(x), 0.0F, 0.0F}});
        }
        const auto estimates =
            pct::features::estimateNormalsKnn(cloud, 5U);
        for (const auto &estimate : estimates)
        {
            EXPECT_EQ(estimate.neighbor_count, 5U);
            expectFiniteInvalid(
                estimate,
                pct::features::NormalEstimationStatus::DegenerateNeighborhood);
        }
    }

    TEST(NormalEstimationTest, ReportsDuplicateNeighborhoodsAsDegenerate)
    {
        pct::PointCloud cloud;
        for (std::size_t index = 0; index < 6U; ++index)
        {
            cloud.pushBack(pct::Point{{1.0F, 1.0F, 1.0F}});
        }
        const auto estimates =
            pct::features::estimateNormalsKnn(cloud, 6U);
        for (const auto &estimate : estimates)
        {
            EXPECT_EQ(estimate.neighbor_count, 6U);
            expectFiniteInvalid(
                estimate,
                pct::features::NormalEstimationStatus::DegenerateNeighborhood);
        }
    }

    TEST(NormalEstimationTest, SphereNormalsFollowRadialDirectionUpToSign)
    {
        const auto cloud = makeSphere(10.0F, 12, 24);
        const auto estimates =
            pct::features::estimateNormalsKnn(cloud, 16U);
        ASSERT_EQ(estimates.size(), cloud.size());
        for (std::size_t index = 0; index < estimates.size(); ++index)
        {
            ASSERT_TRUE(estimates[index].valid());
            const Eigen::Vector3f radial = cloud[index].position.normalized();
            EXPECT_GT(std::abs(estimates[index].normal.dot(radial)), 0.97F);
            EXPECT_GE(estimates[index].curvature, 0.0F);
            EXPECT_LT(estimates[index].curvature, 0.1F);
        }
    }

    TEST(NormalEstimationTest, NoisyPlaneProducesFiniteUnitNormals)
    {
        auto cloud = makePlane(5, 0.25F);
        std::mt19937 generator{20260817U};
        std::normal_distribution<float> noise{0.0F, 0.005F};
        for (auto &point : cloud.points())
        {
            point.position.z() += noise(generator);
        }
        const auto estimates =
            pct::features::estimateNormalsKnn(cloud, 12U);
        double angle_sum_degrees = 0.0;
        for (const auto &estimate : estimates)
        {
            ASSERT_TRUE(estimate.valid());
            EXPECT_TRUE(estimate.normal.allFinite());
            EXPECT_TRUE(std::isfinite(estimate.curvature));
            EXPECT_NEAR(estimate.normal.norm(), 1.0F, 1.0e-5F);
            const float cosine =
                std::clamp(std::abs(estimate.normal.z()), 0.0F, 1.0F);
            angle_sum_degrees +=
                std::acos(cosine) * 180.0 / static_cast<double>(kPi);
        }
        const double mean_angle =
            angle_sum_degrees / static_cast<double>(estimates.size());
        EXPECT_LT(mean_angle, 2.0);
    }
} // namespace

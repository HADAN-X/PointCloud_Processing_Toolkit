#include "pct/search/brute_force.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

namespace
{
    pct::PointCloud lineCloud()
    {
        pct::PointCloud cloud;
        cloud.pushBack(pct::Point{{-2.0F, 0.0F, 0.0F}});
        cloud.pushBack(pct::Point{{-1.0F, 0.0F, 0.0F}});
        cloud.pushBack(pct::Point{{0.0F, 0.0F, 0.0F}});
        cloud.pushBack(pct::Point{{1.0F, 0.0F, 0.0F}});
        cloud.pushBack(pct::Point{{2.0F, 0.0F, 0.0F}});
        return cloud;
    }

    TEST(BruteForceTest, EmptyCloudReturnsEmpty)
    {
        const pct::PointCloud cloud;
        const auto result = pct::search::knnSearch(cloud, Eigen::Vector3f::Zero(), 3U);
        EXPECT_TRUE(result.empty());
    }

    TEST(BruteForceTest, ZeroReturnEmpty)
    {
        const auto result = pct::search::knnSearch(lineCloud(), Eigen::Vector3f::Zero(), 0U);
        EXPECT_TRUE(result.empty());
    }

    TEST(BruteForceTest, KGreaterThanCloudReturnsAllAvailablePoints)
    {
        const auto result = pct::search::knnSearch(lineCloud(), Eigen::Vector3f::Zero(), 100U);
        EXPECT_EQ(result.size(), 5U);
    }

    TEST(BruteForceTest, ReturnNearestNeighborsInDistanceOrder)
    {
        const auto result = pct::search::knnSearch(lineCloud(), Eigen::Vector3f{0.2F, 0.0F, 0.0F}, 3U);
        ASSERT_EQ(result.size(), 3U);
        EXPECT_EQ(result[0].index, 2U);
        EXPECT_EQ(result[1].index, 3U);
        EXPECT_EQ(result[2].index, 1U);
        EXPECT_NEAR(result[0].squared_distance, 0.04, 1.0e-8);
        EXPECT_NEAR(result[1].squared_distance, 0.64, 1.0e-7);
        EXPECT_NEAR(result[2].squared_distance, 1.44, 1.0e-7);
    }

    TEST(BruteForceTest, BreakDistanceTiesByPointIndex)
    {
        const auto result = pct::search::knnSearch(lineCloud(), Eigen::Vector3f::Zero(), 5U);
        ASSERT_EQ(result.size(), 5U);
        EXPECT_EQ(result[0].index, 2U);
        EXPECT_EQ(result[1].index, 1U);
        EXPECT_EQ(result[2].index, 3U);
        EXPECT_EQ(result[3].index, 0U);
        EXPECT_EQ(result[4].index, 4U);
    }

    TEST(BruteForceTest, MatchesIndependentFullSortBaseline)
    {
        std::mt19937 generator{20260812U};
        std::uniform_real_distribution<float> coordinate{-10.0F, 10.0F};
        pct::PointCloud cloud;
        for (std::size_t index = 0; index < 100U; ++index)
        {
            cloud.pushBack(pct::Point{{coordinate(generator),
                                       coordinate(generator),
                                       coordinate(generator)}});
        }

        const Eigen::Vector3f query{0.25F, -1.5F, 2.0F};
        std::vector<pct::search::Neighbor> expected;
        expected.reserve(cloud.size());
        for (pct::PointCloud::size_type index = 0; index < cloud.size(); ++index)
        {
            expected.push_back(
                {index,
                 (cloud[index].position.cast<double>() - query.cast<double>()).squaredNorm()});
        }
        std::sort(
            expected.begin(),
            expected.end(),
            [](const auto &lhs, const auto &rhs)
            {
                if (lhs.squared_distance != rhs.squared_distance)
                {
                    return lhs.squared_distance < rhs.squared_distance;
                }
                return lhs.index < rhs.index;
            });

        constexpr std::size_t k = 17U;
        const auto actual = pct::search::knnSearch(cloud, query, k);
        ASSERT_EQ(actual.size(), k);
        for (std::size_t index = 0; index < k; ++index)
        {
            EXPECT_EQ(actual[index].index, expected[index].index);
            EXPECT_DOUBLE_EQ(actual[index].squared_distance, expected[index].squared_distance);
        }
    }

    TEST(BruteForceTest, CoordinateQueryIncludesCoincidentPointByDefault)
    {
        const auto result = pct::search::knnSearch(lineCloud(), Eigen::Vector3f::Zero(), 1U);
        ASSERT_EQ(result.size(), 1U);
        EXPECT_EQ(result[0].index, 2U);
        EXPECT_DOUBLE_EQ(result[0].squared_distance, 0.0);
    }

    TEST(BruteForceTest, ExclusionRemovesOnlySpecifiedIndex)
    {
        pct::PointCloud cloud;
        cloud.pushBack(pct::Point{{0.0F, 0.0F, 0.0F}});
        cloud.pushBack(pct::Point{{0.0F, 0.0F, 0.0F}});
        const auto result = pct::search::knnSearch(
            cloud, cloud[0].position, 2U, 0U);
        ASSERT_EQ(result.size(), 1U);
        EXPECT_EQ(result[0].index, 1U);
        EXPECT_DOUBLE_EQ(result[0].squared_distance, 0.0);
    }

    TEST(BruteForceSearchTest, RejectsOutOfRangeExcludedIndex)
    {
        const auto cloud = lineCloud();
        EXPECT_THROW(
            static_cast<void>(
                pct::search::knnSearch(cloud, Eigen::Vector3f::Zero(), 1U, cloud.size())),
            std::out_of_range);
    }

    TEST(BruteForceSearchTest, RejectsNonFiniteQueryAndCloudPoint)
    {
        const auto cloud = lineCloud();
        EXPECT_THROW(
            static_cast<void>(
                pct::search::knnSearch(cloud, {std::numeric_limits<float>::infinity(), 0.0F, 0.0F}, 1U)),
            std::invalid_argument);
        auto invalid_cloud = lineCloud();
        invalid_cloud[0].position.x() =
            std::numeric_limits<float>::quiet_NaN();
        EXPECT_THROW(
            static_cast<void>(
                pct::search::radiusSearch(invalid_cloud, Eigen::Vector3f::Zero(), 1.0F)),
            std::invalid_argument);
    }

    TEST(BruteForceRadiusTest, IncludePointsExactlyOnRadiusBoundary)
    {
        const auto result = pct::search::radiusSearch(
            lineCloud(), Eigen::Vector3f::Zero(), 1.0F, 2U);
        ASSERT_EQ(result.size(), 2U);
        EXPECT_EQ(result[0].index, 1U);
        EXPECT_EQ(result[1].index, 3U);
        EXPECT_DOUBLE_EQ(result[0].squared_distance, 1.0);
        EXPECT_DOUBLE_EQ(result[1].squared_distance, 1.0);
    }

    TEST(BruteForceRadiusTest, ReturnsExpectedNeighborsOnRegularGrid)
    {
        pct::PointCloud grid;
        for (int y = -1; y <= 1; ++y)
        {
            for (int x = -1; x <= 1; ++x)
            {
                grid.pushBack(pct::Point{{static_cast<float>(x),
                                          static_cast<float>(y), 0.0F}});
            }
        }

        const auto result = pct::search::radiusSearch(grid, Eigen::Vector3f::Zero(), 1.0F, 4U);
        ASSERT_EQ(result.size(), 4U);
        EXPECT_EQ(result[0].index, 1U);
        EXPECT_EQ(result[1].index, 3U);
        EXPECT_EQ(result[2].index, 5U);
        EXPECT_EQ(result[3].index, 7U);
    }
    TEST(BruteForceRadiusTest, ZeroRadiusFindsDuplicateAtDifferentIndex)
    {
        pct::PointCloud cloud;
        cloud.pushBack(pct::Point{{1.0F, 2.0F, 3.0F}});
        cloud.pushBack(pct::Point{{1.0F, 2.0F, 3.0F}});
        const auto result =
            pct::search::radiusSearch(cloud, cloud[0].position, 0.0F, 0U);
        ASSERT_EQ(result.size(), 1U);
        EXPECT_EQ(result[0].index, 1U);
    }

    TEST(BruteForceRadiusTest, RejectsNegativeAndNonFiniteRadius)
    {
        const auto cloud = lineCloud();
        EXPECT_THROW(static_cast<void>(pct::search::radiusSearch(
                         cloud, Eigen::Vector3f::Zero(), -1.0F)),
                     std::invalid_argument);
        EXPECT_THROW(static_cast<void>(pct::search::radiusSearch(
                         cloud, Eigen::Vector3f::Zero(),
                         std::numeric_limits<float>::infinity())),
                     std::invalid_argument);
    }

    TEST(BruteForceRadiusTest, HandlesMaximumFiniteRadiusInDoublePrecision)
    {
        const auto result = pct::search::radiusSearch(
            lineCloud(), Eigen::Vector3f::Zero(),
            std::numeric_limits<float>::max());
        EXPECT_EQ(result.size(), 5U);
    }
} // namespace

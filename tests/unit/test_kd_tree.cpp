#include "pct/search/brute_force.hpp"
#include "pct/search/kd_tree.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>

namespace
{
    void expectSame(const std::vector<pct::search::Neighbor> &actual,
                    const std::vector<pct::search::Neighbor> &expected)
    {
        ASSERT_EQ(actual.size(), expected.size());
        for (std::size_t index = 0; index < expected.size(); ++index)
        {
            EXPECT_EQ(actual[index].index, expected[index].index);
            EXPECT_DOUBLE_EQ(actual[index].squared_distance,
                             expected[index].squared_distance);
        }
    }

    pct::PointCloud makeRandomCloud(std::size_t count)
    {
        std::mt19937 generator{20260813U};
        std::uniform_real_distribution<float> coordinate{-50.0F, 50.0F};
        pct::PointCloud cloud;
        cloud.points().reserve(count);
        for (std::size_t index = 0; index < count; ++index)
        {
            cloud.pushBack(pct::Point{{coordinate(generator),
                                       coordinate(generator),
                                       coordinate(generator)}});
        }
        return cloud;
    }

    TEST(KdTreeTest, EmptyCloudHasZeroStatisticsAndEmptyQueries)
    {
        const pct::PointCloud cloud;
        const pct::search::KdTree tree{cloud};
        EXPECT_EQ(tree.stats().node_count, 0U);
        EXPECT_EQ(tree.stats().max_depth, 0U);
        EXPECT_TRUE(tree.knnSearch(Eigen::Vector3f::Zero(), 3U).empty());
        EXPECT_TRUE(tree.radiusSearch(Eigen::Vector3f::Zero(), 1.0F).empty());
    }

    TEST(KdTreeTest, SinglePointQueriesMatchBruteForce)
    {
        pct::PointCloud cloud;
        cloud.pushBack(pct::Point{{1.0F, 2.0F, 3.0F}});
        const pct::search::KdTree tree{cloud};
        expectSame(tree.knnSearch(cloud[0].position, 2U),
                   pct::search::knnSearch(cloud, cloud[0].position, 2U));
        expectSame(tree.radiusSearch(cloud[0].position, 0.0F),
                   pct::search::radiusSearch(cloud, cloud[0].position, 0.0F));
        EXPECT_TRUE(tree.knnSearch(cloud[0].position, 1U, 0U).empty());
    }

    TEST(KdTreeTest, SortedInputBuildsBalancedTree)
    {
        pct::PointCloud cloud;
        for (int x = 0; x < 31; ++x)
        {
            cloud.pushBack(pct::Point{{static_cast<float>(x), 0.0F, 0.0F}});
        }
        const pct::search::KdTree tree{cloud};
        EXPECT_EQ(tree.stats().node_count, cloud.size());
        EXPECT_LE(tree.stats().max_depth, 5U);
        expectSame(tree.knnSearch({14.25F, 0.0F, 0.0F}, 8U),
                   pct::search::knnSearch(
                       cloud, {14.25F, 0.0F, 0.0F}, 8U));
    }

    TEST(KdTreeTest, RandomQueriesMatchBruteForceHundredsOfTimes)
    {
        const auto cloud = makeRandomCloud(500U);
        const pct::search::KdTree tree{cloud};
        std::mt19937 generator{20260814U};
        std::uniform_real_distribution<float> coordinate{-60.0F, 60.0F};
        std::uniform_real_distribution<float> radius{0.0F, 30.0F};
        for (std::size_t query_index = 0; query_index < 300U;
             ++query_index)
        {
            const Eigen::Vector3f query{coordinate(generator),
                                        coordinate(generator),
                                        coordinate(generator)};
            expectSame(tree.knnSearch(query, 12U),
                       pct::search::knnSearch(cloud, query, 12U));
            const float query_radius = radius(generator);
            expectSame(tree.radiusSearch(query, query_radius),
                       pct::search::radiusSearch(
                           cloud, query, query_radius));
        }
    }

    TEST(KdTreeTest, DuplicatePointsKeepIndexTieBreaking)
    {
        pct::PointCloud cloud;
        for (std::size_t index = 0; index < 20U; ++index)
        {
            cloud.pushBack(pct::Point{{1.0F, 1.0F, 1.0F}});
        }
        const pct::search::KdTree tree{cloud};
        EXPECT_EQ(tree.stats().node_count, cloud.size());
        EXPECT_LE(tree.stats().max_depth, 5U);
        expectSame(tree.knnSearch(cloud[0].position, 7U, 0U),
                   pct::search::knnSearch(cloud, cloud[0].position, 7U, 0U));
        expectSame(tree.radiusSearch(cloud[0].position, 0.0F, 0U),
                   pct::search::radiusSearch(
                       cloud, cloud[0].position, 0.0F, 0U));
    }

    TEST(KdTreeTest, CollinearAndCoplanarCloudsMatchBruteForce)
    {
        pct::PointCloud collinear;
        pct::PointCloud coplanar;
        for (int value = -20; value <= 20; ++value)
        {
            collinear.pushBack(
                pct::Point{{static_cast<float>(value), 0.0F, 0.0F}});
            coplanar.pushBack(pct::Point{{static_cast<float>(value),
                                          static_cast<float>(value % 5),
                                          0.0F}});
        }
        const pct::search::KdTree line_tree{collinear};
        const pct::search::KdTree plane_tree{coplanar};
        expectSame(line_tree.knnSearch({0.25F, 0.0F, 0.0F}, 10U),
                   pct::search::knnSearch(
                       collinear, {0.25F, 0.0F, 0.0F}, 10U));
        expectSame(plane_tree.radiusSearch({0.0F, 0.0F, 0.0F}, 5.0F),
                   pct::search::radiusSearch(
                       coplanar, {0.0F, 0.0F, 0.0F}, 5.0F));
    }

    TEST(KdTreeTest, ZeroKAndKAboveAvailableMatchBruteForce)
    {
        const auto cloud = makeRandomCloud(25U);
        const pct::search::KdTree tree{cloud};
        expectSame(tree.knnSearch(Eigen::Vector3f::Zero(), 0U),
                   pct::search::knnSearch(
                       cloud, Eigen::Vector3f::Zero(), 0U));
        expectSame(tree.knnSearch(Eigen::Vector3f::Zero(), 100U),
                   pct::search::knnSearch(
                       cloud, Eigen::Vector3f::Zero(), 100U));
    }

    TEST(KdTreeTest, RadiusBoundaryAndExcludedIndexMatchBruteForce)
    {
        pct::PointCloud cloud;
        cloud.pushBack(pct::Point{{-1.0F, 0.0F, 0.0F}});
        cloud.pushBack(pct::Point{{0.0F, 0.0F, 0.0F}});
        cloud.pushBack(pct::Point{{1.0F, 0.0F, 0.0F}});
        const pct::search::KdTree tree{cloud};
        expectSame(tree.radiusSearch(Eigen::Vector3f::Zero(), 1.0F, 1U),
                   pct::search::radiusSearch(
                       cloud, Eigen::Vector3f::Zero(), 1.0F, 1U));
    }

    TEST(KdTreeTest, ExcludedNodeWithEmptyNearBranchMatchesBruteForce)
    {
        pct::PointCloud cloud;
        cloud.pushBack(pct::Point{{-1.0F, 0.0F, 0.0F}});
        cloud.pushBack(pct::Point{{1.0F, 0.0F, 0.0F}});
        const pct::search::KdTree tree{cloud};

        expectSame(tree.knnSearch(cloud[1].position, 1U, 1U),
                   pct::search::knnSearch(
                       cloud, cloud[1].position, 1U, 1U));
    }

    TEST(KdTreeTest, RejectsInvalidConstructionAndQueries)
    {
        auto invalid_cloud = makeRandomCloud(3U);
        invalid_cloud[1].position.x() =
            std::numeric_limits<float>::quiet_NaN();
        EXPECT_THROW(pct::search::KdTree{invalid_cloud},
                     std::invalid_argument);

        const auto cloud = makeRandomCloud(3U);
        const pct::search::KdTree tree{cloud};
        EXPECT_THROW(static_cast<void>(tree.knnSearch(
                         {std::numeric_limits<float>::infinity(), 0.0F, 0.0F},
                         1U)),
                     std::invalid_argument);
        EXPECT_THROW(static_cast<void>(tree.radiusSearch(
                         Eigen::Vector3f::Zero(), -1.0F)),
                     std::invalid_argument);
        EXPECT_THROW(static_cast<void>(tree.knnSearch(
                         Eigen::Vector3f::Zero(), 1U, cloud.size())),
                     std::out_of_range);
    }
} // namespace

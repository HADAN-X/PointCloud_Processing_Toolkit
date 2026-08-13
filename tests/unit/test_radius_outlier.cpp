#include "pct/filters/radius_outlier.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

namespace
{

    pct::PointCloud clusterWithOutlier()
    {
        pct::PointCloud cloud;
        cloud.pushBack(pct::Point{{0.0F, 0.0F, 0.0F}, {255, 255, 255}});
        cloud.pushBack(pct::Point{{1.0F, 0.0F, 0.0F}, {255, 0, 0}});
        cloud.pushBack(pct::Point{{-1.0F, 0.0F, 0.0F}, {0, 255, 0}});
        cloud.pushBack(pct::Point{{0.0F, 1.0F, 0.0F}, {0, 0, 255}});
        cloud.pushBack(pct::Point{{0.0F, -1.0F, 0.0F}, {255, 255, 0}});
        cloud.pushBack(pct::Point{{10.0F, 10.0F, 10.0F}, {255, 0, 255}});
        return cloud;
    }

    TEST(RadiusOutlierTest, RemovesIsolatedPointAndKeepsDenseCluster)
    {
        const auto cloud = clusterWithOutlier();
        const auto result = pct::filters::radiusOutlierRemoval(cloud, 1.5F, 2U);
        ASSERT_EQ(result.size(), 5U);
        for (pct::PointCloud::size_type index = 0; index < result.size(); ++index)
        {
            EXPECT_TRUE(result[index].position.isApprox(cloud[index].position));
            EXPECT_EQ(result[index].color, cloud[index].color);
        }
    }

    TEST(RadiusOutlierTest, EmptyCloudRemainsEmptyForValidParameters)
    {
        const pct::PointCloud cloud;
        EXPECT_TRUE(pct::filters::radiusOutlierRemoval(cloud, 1.0F, 1U)
                        .empty());
    }

    TEST(RadiusOutlierTest, RejectsZeroMinimumNeighborCount)
    {
        const pct::PointCloud cloud;
        EXPECT_THROW(static_cast<void>(
                         pct::filters::radiusOutlierRemoval(cloud, 1.0F, 0U)),
                     std::invalid_argument);
    }

    TEST(RadiusOutlierTest, RejectsInvalidRadiusEvenForEmptyCloud)
    {
        const pct::PointCloud cloud;
        EXPECT_THROW(static_cast<void>(
                         pct::filters::radiusOutlierRemoval(cloud, -1.0F, 1U)),
                     std::invalid_argument);
    }

    TEST(RadiusOutlierTest, DuplicatePointCountsAsDifferentNeighbor)
    {
        pct::PointCloud cloud;
        cloud.pushBack(pct::Point{{0.0F, 0.0F, 0.0F}});
        cloud.pushBack(pct::Point{{0.0F, 0.0F, 0.0F}});
        // radius 0.0, identify points coordinates same but indices different.
        const auto result = pct::filters::radiusOutlierRemoval(cloud, 0.0F, 1U);
        EXPECT_EQ(result.size(), 2U);
    }

    TEST(RadiusOutlierTest, DemonstratesParameterSensitivity)
    {
        const auto cloud = clusterWithOutlier();
        EXPECT_EQ(pct::filters::radiusOutlierRemoval(cloud, 1.5F, 2U).size(),
                  5U);
        EXPECT_EQ(pct::filters::radiusOutlierRemoval(cloud, 0.5F, 1U).size(),
                  0U);
    }

    TEST(RadiusOutlierTest, MinimumAboveAvailableNeighborsRemovesAllPoints)
    {
        const auto cloud = clusterWithOutlier();
        EXPECT_TRUE(
            pct::filters::radiusOutlierRemoval(cloud, 100.0F, cloud.size())
                .empty());
    }

} // namespace

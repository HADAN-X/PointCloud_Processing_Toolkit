#include "pct/geometry/statistics.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>

namespace
{
    TEST(PointCloudStatisticsTest, EmptyCloudHasNoCentroidOrBoundingBox)
    {
        const pct::PointCloud cloud;
        const auto statistics = pct::geometry::computeStatistics(cloud);
        EXPECT_EQ(statistics.point_count, 0U);
        EXPECT_FALSE(statistics.centroid.has_value());
        EXPECT_FALSE(statistics.bounding_box.has_value());
    }
    TEST(PointCloudStatisticsTest, computeCentroidAndBoundingBox)
    {
        pct::PointCloud cloud;
        cloud.pushBack(pct::Point{Eigen::Vector3f{-2.0F, 1.0F, 4.0F}});
        cloud.pushBack(pct::Point{Eigen::Vector3f{4.0F, 3.0F, -2.0F}});
        cloud.pushBack(pct::Point{Eigen::Vector3f{1.0F, -1.0F, 1.0F}});
        const auto statistics = pct::geometry::computeStatistics(cloud);
        ASSERT_TRUE(statistics.centroid.has_value());
        EXPECT_TRUE(statistics.centroid->isApprox(Eigen::Vector3f{1.0F, 1.0F, 1.0F}));
        ASSERT_TRUE(statistics.bounding_box.has_value());
        EXPECT_TRUE(statistics.bounding_box->minimum.isApprox(Eigen::Vector3f{-2.0F, -1.0F, -2.0F}));
        EXPECT_TRUE(statistics.bounding_box->maximum.isApprox(Eigen::Vector3f{4.0F, 3.0F, 4.0F}));
    }
    TEST(PointCloudStatisticsTest, SinglePointDefinesAllStatistics)
    {
        pct::PointCloud cloud;
        const Eigen::Vector3f point_position{2.5F, -3.0F, 8.0F};
        cloud.pushBack(pct::Point{point_position});
        const auto statistics = pct::geometry::computeStatistics(cloud);
        ASSERT_TRUE(statistics.centroid.has_value());
        EXPECT_EQ(*statistics.centroid, point_position);
        ASSERT_TRUE(statistics.bounding_box.has_value());
        EXPECT_EQ(statistics.bounding_box->minimum, point_position);
        EXPECT_EQ(statistics.bounding_box->maximum, point_position);
    }

    TEST(PointCloudStatisticsTest, RejectsNonFiniteCoordinates)
    {
        pct::PointCloud cloud;
        cloud.pushBack(pct::Point{Eigen::Vector3f{0.0F, std::numeric_limits<float>::infinity(), 0.0F}});
        EXPECT_THROW(static_cast<void>(pct::geometry::computeStatistics(cloud)), std::invalid_argument);
    }

} // namespace
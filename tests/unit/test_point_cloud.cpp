#include "pct/core/point_cloud.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <utility>

namespace pct
{
    namespace
    {

        TEST(PointTest, DefaultPointIsAtOriginWithoutColor)
        {
            const Point point;

            EXPECT_FLOAT_EQ(point.position.x(), 0.0F);
            EXPECT_FLOAT_EQ(point.position.y(), 0.0F);
            EXPECT_FLOAT_EQ(point.position.z(), 0.0F);
            EXPECT_FALSE(point.color.has_value());
        }

        TEST(PointTest, StoresPositionAndOptionalColor)
        {
            const ColorRgb8 color{std::uint8_t{12}, std::uint8_t{34},
                                  std::uint8_t{56}};
            const Point point{Eigen::Vector3f{1.0F, 2.0F, 3.0F}, color};

            // Use isApprox instead of == because floating-point operations have precision loss.
            EXPECT_TRUE(point.position.isApprox(Eigen::Vector3f{1.0F, 2.0F, 3.0F}));
            ASSERT_TRUE(point.color.has_value());
            EXPECT_EQ(*point.color, color);
        }

        TEST(PointCloudTest, NewCloudIsEmpty)
        {
            const PointCloud cloud;

            EXPECT_TRUE(cloud.empty());
            EXPECT_EQ(cloud.size(), 0U);
        }

        TEST(PointCloudTest, PushBackAndAccessPreservePoints)
        {
            PointCloud cloud;
            cloud.pushBack(Point{Eigen::Vector3f{1.0F, 2.0F, 3.0F}});
            Point second{Eigen::Vector3f{-1.0F, -2.0F, -3.0F}};
            cloud.pushBack(std::move(second));

            ASSERT_EQ(cloud.size(), 2U);
            EXPECT_TRUE(cloud[0].position.isApprox(
                Eigen::Vector3f{1.0F, 2.0F, 3.0F}));
            EXPECT_TRUE(cloud.at(1).position.isApprox(
                Eigen::Vector3f{-1.0F, -2.0F, -3.0F}));
        }

        TEST(PointCloudTest, AtThrowsForOutOfRangeIndex)
        {
            const PointCloud cloud;

            EXPECT_THROW(static_cast<void>(cloud.at(0)), std::out_of_range);
        }

        TEST(PointCloudTest, ClearRemovesAllPoints)
        {
            PointCloud cloud;
            cloud.pushBack(Point{Eigen::Vector3f{1.0F, 2.0F, 3.0F}});

            cloud.clear();

            EXPECT_TRUE(cloud.empty());
            EXPECT_EQ(cloud.size(), 0U);
        }

    } // namespace
} // namespace pct
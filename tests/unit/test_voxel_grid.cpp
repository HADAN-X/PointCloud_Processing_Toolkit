#include "pct/filters/voxel_grid.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <random>
#include <stdexcept>

namespace
{

    pct::PointCloud makeCloud(
        std::initializer_list<Eigen::Vector3f> positions)
    {
        pct::PointCloud cloud;
        for (const auto &position : positions)
        {
            cloud.pushBack(pct::Point{position});
        }
        return cloud;
    }

    TEST(VoxelGridTest, EmptyCloudProducesEmptyCloud)
    {
        const pct::PointCloud cloud;
        EXPECT_TRUE(pct::filters::voxelGridDownsample(cloud, 1.0F).empty());
    }

    TEST(VoxelGridTest, RejectsNonPositiveAndNonFiniteLeafSizes)
    {
        const pct::PointCloud cloud;
        EXPECT_THROW(static_cast<void>(
                         pct::filters::voxelGridDownsample(cloud, 0.0F)),
                     std::invalid_argument);
        EXPECT_THROW(static_cast<void>(
                         pct::filters::voxelGridDownsample(cloud, -1.0F)),
                     std::invalid_argument);
        EXPECT_THROW(static_cast<void>(pct::filters::voxelGridDownsample(
                         cloud, std::numeric_limits<float>::infinity())),
                     std::invalid_argument);
        EXPECT_THROW(static_cast<void>(pct::filters::voxelGridDownsample(
                         cloud, std::numeric_limits<float>::quiet_NaN())),
                     std::invalid_argument);
    }

    TEST(VoxelGridTest, AveragesPositionsInsideOneVoxel)
    {
        const auto cloud = makeCloud({{0.1F, 0.2F, 0.3F},
                                      {0.3F, 0.4F, 0.5F}});
        const auto result = pct::filters::voxelGridDownsample(cloud, 1.0F);
        ASSERT_EQ(result.size(), 1U);
        EXPECT_TRUE(result[0].position.isApprox(
            Eigen::Vector3f{0.2F, 0.3F, 0.4F}, 1.0e-6F));
    }

    TEST(VoxelGridTest, KeepsPointsFromDifferentVoxelsSeparate)
    {
        const auto cloud = makeCloud({{0.1F, 0.0F, 0.0F},
                                      {1.1F, 0.0F, 0.0F}});
        const auto result = pct::filters::voxelGridDownsample(cloud, 1.0F);
        ASSERT_EQ(result.size(), 2U);
        EXPECT_TRUE(result[0].position.isApprox(cloud[0].position));
        EXPECT_TRUE(result[1].position.isApprox(cloud[1].position));
    }

    TEST(VoxelGridTest, UsesFloorForNegativeCoordinates)
    {
        const auto cloud = makeCloud({{-0.1F, 0.0F, 0.0F},
                                      {-0.9F, 0.0F, 0.0F},
                                      {0.1F, 0.0F, 0.0F}});
        const auto result = pct::filters::voxelGridDownsample(cloud, 1.0F);
        ASSERT_EQ(result.size(), 2U);
        EXPECT_TRUE(result[0].position.isApprox(
            Eigen::Vector3f{-0.5F, 0.0F, 0.0F}));
        EXPECT_TRUE(result[1].position.isApprox(
            Eigen::Vector3f{0.1F, 0.0F, 0.0F}));
    }

    TEST(VoxelGridTest, TreatsExactBoundaryAsNextVoxel)
    {
        const auto cloud = makeCloud({{0.999F, 0.0F, 0.0F},
                                      {1.0F, 0.0F, 0.0F}});
        EXPECT_EQ(pct::filters::voxelGridDownsample(cloud, 1.0F).size(),
                  2U);
    }

    TEST(VoxelGridTest, LargeLeafProducesOneVoxelWhenPointsShareGridCell)
    {
        const auto cloud = makeCloud({{1.0F, 2.0F, 3.0F},
                                      {4.0F, 5.0F, 6.0F}});
        EXPECT_EQ(pct::filters::voxelGridDownsample(cloud, 10.0F).size(),
                  1U);
    }

    TEST(VoxelGridTest, AveragesColorsAndRoundsHalfUp)
    {
        pct::PointCloud cloud;
        cloud.pushBack(pct::Point{{0.1F, 0.0F, 0.0F}, {10, 20, 30}});
        cloud.pushBack(pct::Point{{0.2F, 0.0F, 0.0F}, {11, 21, 31}});
        const auto result = pct::filters::voxelGridDownsample(cloud, 1.0F);
        ASSERT_EQ(result.size(), 1U);
        ASSERT_TRUE(result[0].color.has_value());
        EXPECT_EQ(*result[0].color, (pct::ColorRgb8{11, 21, 31}));
    }

    TEST(VoxelGridTest, UncoloredInputRemainsUncolored)
    {
        const auto result = pct::filters::voxelGridDownsample(
            makeCloud({{0.1F, 0.0F, 0.0F}, {0.2F, 0.0F, 0.0F}}),
            1.0F);
        ASSERT_EQ(result.size(), 1U);
        EXPECT_FALSE(result[0].color.has_value());
    }

    TEST(VoxelGridTest, RejectsPartiallyColoredCloud)
    {
        pct::PointCloud cloud;
        cloud.pushBack(pct::Point{{0.1F, 0.0F, 0.0F}, {1, 2, 3}});
        cloud.pushBack(pct::Point{{0.2F, 0.0F, 0.0F}});
        EXPECT_THROW(static_cast<void>(
                         pct::filters::voxelGridDownsample(cloud, 1.0F)),
                     std::invalid_argument);
    }

    TEST(VoxelGridTest, RejectsNonFiniteCoordinates)
    {
        const auto cloud = makeCloud(
            {{std::numeric_limits<float>::infinity(), 0.0F, 0.0F}});
        EXPECT_THROW(static_cast<void>(
                         pct::filters::voxelGridDownsample(cloud, 1.0F)),
                     std::invalid_argument);
    }

    TEST(VoxelGridTest, RejectsVoxelIndexOverflow)
    {
        const auto cloud = makeCloud(
            {{std::numeric_limits<float>::max(), 0.0F, 0.0F}});
        EXPECT_THROW(static_cast<void>(pct::filters::voxelGridDownsample(
                         cloud, std::numeric_limits<float>::min())),
                     std::overflow_error);
    }

    TEST(VoxelGridTest, ProducesLexicographicallyOrderedVoxels)
    {
        const auto cloud = makeCloud({{2.1F, 0.0F, 0.0F},
                                      {-0.1F, 0.0F, 0.0F},
                                      {1.1F, 0.0F, 0.0F}});
        const auto result = pct::filters::voxelGridDownsample(cloud, 1.0F);
        ASSERT_EQ(result.size(), 3U);
        EXPECT_LT(result[0].position.x(), result[1].position.x());
        EXPECT_LT(result[1].position.x(), result[2].position.x());
    }

    TEST(VoxelGridTest, ShuffledInputProducesEquivalentOrderedOutput)
    {
        auto cloud = makeCloud({{-1.8F, 0.1F, 0.1F},
                                {-1.2F, 0.3F, 0.5F},
                                {0.1F, 0.2F, 0.3F},
                                {0.8F, 0.6F, 0.4F},
                                {2.1F, 0.0F, 0.0F}});
        const auto expected =
            pct::filters::voxelGridDownsample(cloud, 1.0F);

        std::mt19937 generator{42U};
        std::shuffle(cloud.points().begin(), cloud.points().end(), generator);
        const auto shuffled =
            pct::filters::voxelGridDownsample(cloud, 1.0F);

        ASSERT_EQ(shuffled.size(), expected.size());
        for (pct::PointCloud::size_type index = 0; index < expected.size();
             ++index)
        {
            EXPECT_TRUE(shuffled[index].position.isApprox(
                expected[index].position, 1.0e-6F));
        }
    }

} // namespace
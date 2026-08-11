#include "pct/io/ply_io.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <limits>
#include <string>

namespace pct::io
{
    namespace
    {

        std::filesystem::path fixturePath(const std::string &name)
        {
            return std::filesystem::path{PCT_TEST_FIXTURE_DIR} / name;
        }

        class TemporaryPly
        {
        public:
            explicit TemporaryPly(const std::string &test_name)
            {
                path_ = std::filesystem::temp_directory_path() /
                        ("pct_ply_test_" + test_name + ".ply");
            }

            ~TemporaryPly()
            {
                std::error_code error;
                std::filesystem::remove(path_, error);
            }

            TemporaryPly(const TemporaryPly &) = delete;
            TemporaryPly &operator=(const TemporaryPly &) = delete;

            const std::filesystem::path &path() const noexcept
            {
                return path_;
            }

        private:
            std::filesystem::path path_;
        };

        TEST(PlyReaderTest, ReadsXyzPointCloud)
        {
            const PointCloud cloud = readPly(fixturePath("ascii_xyz.ply"));

            ASSERT_EQ(cloud.size(), 3U);
            EXPECT_TRUE(cloud[0].position.isApprox(Eigen::Vector3f{0.0F, 0.0F, 0.0F}));
            EXPECT_TRUE(
                cloud[1].position.isApprox(Eigen::Vector3f{1.5F, -2.0F, 3.25F}));
            EXPECT_TRUE(cloud[2].position.isApprox(Eigen::Vector3f{-4.0F, 5.0F, 6.0F}));
            for (const auto &point : cloud.points())
                EXPECT_FALSE(point.color.has_value())
                    << "XYZ point cloud should not contain color data!";
        }

        TEST(PlyReaderTest, ReadsRgbPointCloud)
        {
            const PointCloud cloud = readPly(fixturePath("ascii_xyz_rgb.ply"));

            ASSERT_EQ(cloud.size(), 2U);
            ASSERT_TRUE(cloud[0].color.has_value());
            EXPECT_EQ(*cloud[0].color,
                      (ColorRgb8{std::uint8_t{255}, std::uint8_t{0},
                                 std::uint8_t{128}}));
            ASSERT_TRUE(cloud[1].color.has_value());
            EXPECT_EQ(*cloud[1].color,
                      (ColorRgb8{std::uint8_t{4}, std::uint8_t{5}, std::uint8_t{6}}));
        }

        TEST(PlyReaderTest, HandlesReorderedAndUnknownProperties)
        {
            const PointCloud cloud =
                readPly(fixturePath("ascii_reordered_unknown.ply"));

            ASSERT_EQ(cloud.size(), 1U);
            EXPECT_TRUE(cloud[0].position.isApprox(Eigen::Vector3f{1.0F, 2.0F, 3.0F}));
            ASSERT_TRUE(cloud[0].color.has_value());
            EXPECT_EQ(*cloud[0].color,
                      (ColorRgb8{std::uint8_t{10}, std::uint8_t{20},
                                 std::uint8_t{30}}));
        }

        TEST(PlyWriterTest, RoundTripsXyzPointCloud)
        {
            const PointCloud original = readPly(fixturePath("ascii_xyz.ply"));
            const TemporaryPly output{"roundtrip_xyz"};

            writePlyAscii(output.path(), original);
            const PointCloud restored = readPly(output.path());

            ASSERT_EQ(restored.size(), original.size());
            for (PointCloud::size_type index = 0; index < original.size(); ++index)
            {
                EXPECT_TRUE(restored[index].position.isApprox(original[index].position));
                EXPECT_EQ(restored[index].color, original[index].color);
            }
        }

        TEST(PlyWriterTest, RoundTripsRgbPointCloud)
        {
            const PointCloud original = readPly(fixturePath("ascii_xyz_rgb.ply"));
            const TemporaryPly output{"roundtrip_rgb"};

            writePlyAscii(output.path(), original);
            const PointCloud restored = readPly(output.path());

            ASSERT_EQ(restored.size(), original.size());
            for (PointCloud::size_type index = 0; index < original.size(); ++index)
            {
                EXPECT_TRUE(restored[index].position.isApprox(original[index].position));
                EXPECT_EQ(restored[index].color, original[index].color);
            }
        }

        TEST(PlyReaderTest, RejectsMissingFile)
        {
            EXPECT_THROW(readPly(fixturePath("does_not_exist.ply")), PlyError);
        }

        TEST(PlyReaderTest, RejectsBinaryFormat)
        {
            EXPECT_THROW(readPly(fixturePath("unsupported_binary.ply")), PlyError);
        }

        TEST(PlyReaderTest, RejectsMissingCoordinateProperty)
        {
            EXPECT_THROW(readPly(fixturePath("malformed_missing_z.ply")), PlyError);
        }

        TEST(PlyReaderTest, RejectsPointCountMismatch)
        {
            EXPECT_THROW(readPly(fixturePath("malformed_count.ply")), PlyError);
        }

        TEST(PlyReaderTest, RejectsNonFiniteCoordinate)
        {
            EXPECT_THROW(readPly(fixturePath("malformed_nan.ply")), PlyError);
        }

        TEST(PlyReaderTest, RejectsPartialColorProperties)
        {
            EXPECT_THROW(readPly(fixturePath("malformed_partial_color.ply")), PlyError);
        }

        TEST(PlyWriterTest, RejectsPartiallyColoredCloud)
        {
            PointCloud cloud;
            cloud.pushBack(Point{Eigen::Vector3f{0.0F, 0.0F, 0.0F},
                                 ColorRgb8{std::uint8_t{1}, std::uint8_t{2},
                                           std::uint8_t{3}}});
            cloud.pushBack(Point{Eigen::Vector3f{1.0F, 1.0F, 1.0F}});
            const TemporaryPly output{"partial_color"};

            EXPECT_THROW(writePlyAscii(output.path(), cloud), PlyError);
        }

        TEST(PlyWriterTest, RejectsNonFiniteCoordinateBeforeCreatingFile)
        {
            PointCloud cloud;
            cloud.pushBack(Point{Eigen::Vector3f{
                std::numeric_limits<float>::quiet_NaN(), 1.0F, 2.0F}});
            const TemporaryPly output{"non_finite"};

            EXPECT_THROW(writePlyAscii(output.path(), cloud), PlyError);
            EXPECT_FALSE(std::filesystem::exists(output.path()));
        }

    } // namespace
} // namespace pct::io
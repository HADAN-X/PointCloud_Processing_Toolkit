#include "pct/filters/voxel_grid.hpp"
#include "pct/geometry/statistics.hpp"
#include "pct/geometry/transform.hpp"
#include "pct/io/ply_io.hpp"
#include "pct/filters/radius_outlier.hpp"

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string_view>
#include <string>
#include <system_error>

namespace
{

    void printHelp()
    {
        std::cout
            << "PointCloud Processing Toolkit\n\n"
            << "Usage:\n"
            << "  pointcloud_tool --help\n"
            << "  pointcloud_tool --version\n"
            << "  pointcloud_tool info <input.ply>\n"
            << "  pointcloud_tool stats <input.ply>\n"
            << "  pointcloud_tool convert <input.ply> <output.ply>\n"
            << "  pointcloud_tool transform <input.ply> <output.ply> "
               "--translate <tx> <ty> <tz>\n"
            << "  pointcloud_tool transform <input.ply> <output.ply> "
               "--rotate-x <degrees>\n"
            << "  pointcloud_tool transform <input.ply> <output.ply> "
               "--rotate-y <degrees>\n"
            << "  pointcloud_tool transform <input.ply> <output.ply> "
               "--rotate-z <degrees>\n"
            << "  pointcloud_tool transform <input.ply> <output.ply> "
               "--matrix <16 row-major values>\n"
            << "  pointcloud_tool voxel <input.ply> <output.ply> "
               "--leaf-size <size>\n"
            << "  pointcloud_tool radius-filter <input.ply> <output.ply> "
               "--radius <radius> --min-neighbors <count>\n";
    }

    int printUsageError(std::string_view message)
    {
        std::cerr << "Error: " << message << '\n';
        printHelp();
        return 2;
    }

    bool hasCompleteColor(const pct::PointCloud &cloud)
    {
        return !cloud.empty() &&
               std::all_of(cloud.points().begin(), cloud.points().end(),
                           [](const pct::Point &point)
                           {
                               return point.color.has_value();
                           });
    }

    constexpr float kPi = 3.14159265358979323846F;
    float parseFiniteFloat(std::string_view text)
    {
        float value = 0.0F;
        const char *const begin = text.data();
        const char *const end = begin + text.size();
        const auto result = std::from_chars(begin, end, value);
        if (result.ec != std::errc{} || result.ptr != end || !std::isfinite(value))
        {
            throw std::invalid_argument("Invalid float: " + std::string{text});
        }
        return value;
    }

    std::size_t parsePositiveSize(std::string_view text)
    {
        std::size_t value = 0;
        const char *const begin = text.data();
        const char *const end = begin + text.size();
        const auto result = std::from_chars(begin, end, value);
        if (result.ec != std::errc{} || result.ptr != end || value == 0U)
        {
            throw std::invalid_argument(
                "value must be a positive integer: " + std::string{text});
        }
        return value;
    }

    float degreesToRadians(float degrees)
    {
        return degrees * (kPi / 180.0F);
    }

    pct::geometry::Matrix4f parseTransform(int argc, char *argv[])
    {
        const std::string_view option{argv[4]};
        if (option == "--translate")
        {
            if (argc != 8)
            {
                throw std::invalid_argument(
                    "translate requires exactly three float arguments");
            }
            return pct::geometry::makeTranslation({parseFiniteFloat(argv[5]),
                                                   parseFiniteFloat(argv[6]),
                                                   parseFiniteFloat(argv[7])});
        }
        if (option == "--rotate-x" || option == "--rotate-y" || option == "--rotate-z")
        {
            if (argc != 6)
            {
                throw std::invalid_argument(
                    "rotate requires exactly one angle in degrees");
            }
            const float radians = degreesToRadians(parseFiniteFloat(argv[5]));
            if (option == "--rotate-x")
            {
                return pct::geometry::makeRotationX(radians);
            }
            if (option == "--rotate-y")
            {
                return pct::geometry::makeRotationY(radians);
            }
            return pct::geometry::makeRotationZ(radians);
        }
        if (option == "--matrix")
        {
            if (argc != 21)
            {
                throw std::invalid_argument(
                    "--matrix requires exactly 16 row-major arguments");
            }
            pct::geometry::Matrix4f transform;
            for (int row = 0; row < 4; ++row)
            {
                for (int col = 0; col < 4; ++col)
                {
                    transform(row, col) = parseFiniteFloat(argv[5 + row * 4 + col]);
                }
            }
            if (!pct::geometry::isRigidTransform(transform))
            {
                throw std::invalid_argument(
                    "The provided matrix is not a valid rigid transform.");
            }
            return transform;
        }
        throw std::invalid_argument("Unknown transform option: " + std::string{option});
    }
} // namespace

int main(int argc, char *argv[])
{
    if (argc == 1)
    {
        printHelp();
        return 0;
    }

    const std::string_view command{argv[1]};

    if (command == "--help" || command == "-h")
    {
        if (argc == 2)
        {
            printHelp();
            return 0;
        }
        return printUsageError("help takes no extra arguments");
    }

    if (command == "--version")
    {
        if (argc != 2)
        {
            return printUsageError("version takes no extra arguments");
        }
        std::cout << "pointcloud_tool " << PCT_VERSION << '\n';
        return 0;
    }

    if (command == "info")
    {
        if (argc != 3)
        {
            return printUsageError("info requires one input PLY path");
        }

        try
        {
            const auto cloud = pct::io::readPly(argv[2]);
            std::cout << "points: " << cloud.size() << '\n'
                      << "has_color: "
                      << (hasCompleteColor(cloud) ? "true" : "false")
                      << '\n';
            return 0;
        }
        catch (const pct::io::PlyError &error)
        {
            std::cerr << "PLY error: " << error.what() << '\n';
            return 1;
        }
    }

    if (command == "stats")
    {
        if (argc != 3)
        {
            return printUsageError("stats requires one input PLY path");
        }
        try
        {
            const auto cloud = pct::io::readPly(argv[2]);
            const auto statistics = pct::geometry::computeStatistics(cloud);
            std::cout << std::setprecision(9) << "points: " << statistics.point_count << "\n";
            if (!statistics.centroid || !statistics.bounding_box)
            {
                std::cout << "centroid: undefined\n";
                std::cout << "bbox_min: undefined\n";
                std::cout << "bbox_max: undefined\n";
                return 0;
            }
            std::cout << "centroid: " << statistics.centroid->transpose()
                      << '\n'
                      << "bbox_min: "
                      << statistics.bounding_box->minimum.transpose() << '\n'
                      << "bbox_max: "
                      << statistics.bounding_box->maximum.transpose() << '\n';
            return 0;
        }
        catch (const pct::io::PlyError &error)
        {
            std::cerr << "PLY error: " << error.what() << '\n';
            return 1;
        }
        catch (const std::invalid_argument &error)
        {
            std::cerr << "Geometry error: " << error.what() << '\n';
            return 1;
        }
    }

    if (command == "convert")
    {
        if (argc != 4)
        {
            return printUsageError(
                "convert requires input and output PLY paths");
        }

        try
        {
            const auto cloud = pct::io::readPly(argv[2]);
            pct::io::writePlyAscii(std::filesystem::path{argv[3]}, cloud);
            std::cout << "wrote " << cloud.size() << " points to " << argv[3]
                      << '\n';
            return 0;
        }
        catch (const pct::io::PlyError &error)
        {
            std::cerr << "PLY error: " << error.what() << '\n';
            return 1;
        }
    }

    if (command == "transform")
    {
        if (argc < 5)
        {
            return printUsageError(
                "transform requires input, output, and transform arguments");
        }

        try
        {
            const auto transform = parseTransform(argc, argv);
            const auto cloud = pct::io::readPly(argv[2]);
            const auto result =
                pct::geometry::transformPointCloud(cloud, transform);
            pct::io::writePlyAscii(std::filesystem::path{argv[3]}, result);
            std::cout << "wrote " << result.size()
                      << " transformed points to " << argv[3] << '\n';
            return 0;
        }
        catch (const pct::io::PlyError &error)
        {
            std::cerr << "PLY error: " << error.what() << '\n';
            return 1;
        }
        catch (const std::invalid_argument &error)
        {
            std::cerr << "Transform error: " << error.what() << '\n';
            return 1;
        }
    }

    if (command == "voxel")
    {
        if (argc != 6 || std::string_view{argv[4]} != "--leaf-size")
        {
            return printUsageError(
                "voxel requires input, output, and --leaf-size <size>");
        }

        try
        {
            const float leaf_size = parseFiniteFloat(argv[5]);
            const auto cloud = pct::io::readPly(argv[2]);
            const auto start = std::chrono::steady_clock::now();
            const auto result =
                pct::filters::voxelGridDownsample(cloud, leaf_size);
            const auto stop = std::chrono::steady_clock::now();
            const double elapsed_ms =
                std::chrono::duration<double, std::milli>(stop - start)
                    .count();
            const double retention_percent =
                cloud.empty()
                    ? 0.0
                    : 100.0 * static_cast<double>(result.size()) /
                          static_cast<double>(cloud.size());

            pct::io::writePlyAscii(std::filesystem::path{argv[3]}, result);
            std::cout << std::fixed << std::setprecision(3)
                      << "input_points: " << cloud.size() << '\n'
                      << "output_points: " << result.size() << '\n'
                      << "retention_ratio_percent: " << retention_percent
                      << '\n'
                      << "processing_time_ms: " << elapsed_ms << '\n'
                      << "wrote: " << argv[3] << '\n';
            return 0;
        }
        catch (const pct::io::PlyError &error)
        {
            std::cerr << "PLY error: " << error.what() << '\n';
            return 1;
        }
        catch (const std::exception &error)
        {
            std::cerr << "Voxel error: " << error.what() << '\n';
            return 1;
        }
    }

    if (command == "radius-filter")
    {
        if (argc != 8 || std::string_view{argv[4]} != "--radius" || std::string_view{argv[6]} != "--min-neighbors")
        {
            return printUsageError("radius-filter requires input, output, --radius <radius>, "
                                   "and --min-neighbors <count>");
        }

        try
        {
            const float radius = parseFiniteFloat(argv[5]);
            const std::size_t min_neighbors = parsePositiveSize(argv[7]);
            const auto cloud = pct::io::readPly(argv[2]);
            const auto start = std::chrono::steady_clock::now();
            const auto result = pct::filters::radiusOutlierRemoval(cloud, radius, min_neighbors);
            const auto stop = std::chrono::steady_clock::now();
            const double elapsed_ms = std::chrono::duration<double, std::milli>(stop - start).count();
            const std::size_t removed_points = cloud.size() - result.size();
            const double retention_percent = cloud.empty() ? 0.0 : 100 * static_cast<double>(result.size()) / static_cast<double>(cloud.size());
            pct::io::writePlyAscii(std::filesystem::path{argv[3]}, result);
            std::cout << std::fixed << std::setprecision(3)
                      << "input_points: " << cloud.size() << '\n'
                      << "output_points: " << result.size() << '\n'
                      << "removed_points: " << removed_points << '\n'
                      << "retention_ratio_percent: " << retention_percent
                      << '\n'
                      << "radius: " << radius << '\n'
                      << "min_neighbors: " << min_neighbors << '\n'
                      << "processing_time_ms: " << elapsed_ms << '\n'
                      << "wrote: " << argv[3] << '\n';
            return 0;
        }
        catch (const pct::io::PlyError &error)
        {
            std::cerr << "PLY error: " << error.what() << '\n';
            return 1;
        }
        catch (const std::exception &error)
        {
            std::cerr << "Radius filter error: " << error.what() << '\n';
            return 1;
        }
    }

    return printUsageError("unknown command or argument");
}

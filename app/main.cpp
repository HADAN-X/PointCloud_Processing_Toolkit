#include "pct/io/ply_io.hpp"

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <string_view>

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
            << "  pointcloud_tool convert <input.ply> <output.ply>\n";
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

    return printUsageError("unknown command or argument");
}
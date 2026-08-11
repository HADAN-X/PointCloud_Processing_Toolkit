#pragma once

#include "pct/core/point_cloud.hpp"

#include <filesystem>
#include <stdexcept>
#include <string>

namespace pct::io
{

    class PlyError final : public std::runtime_error
    {
    public:
        explicit PlyError(const std::string &message) : std::runtime_error(message) {}
    };

    PointCloud readPly(const std::filesystem::path &path);
    void writePlyAscii(const std::filesystem::path &path, const PointCloud &cloud);

} // namespace pct::io
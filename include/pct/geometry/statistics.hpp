#pragma once
#include "pct/core/point_cloud.hpp"
#include <Eigen/Core>
#include <cstddef>
#include <optional>

namespace pct::geometry
{
    // AABB
    struct AxisAlignedBoundingBox
    {
        Eigen::Vector3f minimum{Eigen::Vector3f::Zero()};
        Eigen::Vector3f maximum{Eigen::Vector3f::Zero()};
    };

    struct PointCloudStatistics
    {
        std::size_t point_count{0};
        std::optional<Eigen::Vector3f> centroid{};
        std::optional<AxisAlignedBoundingBox> bounding_box{};
        // When point clouds are empty, they are mathematically Undefined.
        // Use optional Not (0,0,0),Because it is a completely legal real coordinate!
    };
    [[nodiscard]] PointCloudStatistics computeStatistics(const PointCloud &cloud);

} // namespace pct::geometry
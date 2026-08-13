#pragma once
#include "pct/core/point_cloud.hpp"

#include <Eigen/Core>
#include <cstddef>
#include <optional>
#include <vector>

namespace pct::search
{
    struct Neighbor
    {
        PointCloud::size_type index{0};
        double squared_distance{0.0};
    };

    [[nodiscard]] std::vector<Neighbor>
    knnSearch(const PointCloud &cloud, const Eigen::Vector3f &query, std::size_t k,
              std::optional<PointCloud::size_type> excluded_index = std::nullopt);
    [[nodiscard]] std::vector<Neighbor>
    radiusSearch(const PointCloud &cloud, const Eigen::Vector3f &query, float radius,
                 std::optional<PointCloud::size_type> excluded_index = std::nullopt);
} // namespace pct::search
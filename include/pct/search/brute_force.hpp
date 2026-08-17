#pragma once
#include "pct/search/neighbor.hpp"

#include <Eigen/Core>
#include <cstddef>
#include <optional>
#include <vector>

namespace pct::search
{

    [[nodiscard]] std::vector<Neighbor>
    knnSearch(const PointCloud &cloud, const Eigen::Vector3f &query, std::size_t k,
              std::optional<PointCloud::size_type> excluded_index = std::nullopt);
    [[nodiscard]] std::vector<Neighbor>
    radiusSearch(const PointCloud &cloud, const Eigen::Vector3f &query, float radius,
                 std::optional<PointCloud::size_type> excluded_index = std::nullopt);
} // namespace pct::search
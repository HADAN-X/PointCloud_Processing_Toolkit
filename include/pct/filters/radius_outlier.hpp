#pragma once

#include "pct/core/point_cloud.hpp"

#include <cstddef>

namespace pct::filters
{
    [[nodiscard]] PointCloud
    radiusOutlierRemoval(const PointCloud &cloud, float radius, std::size_t min_neighbors);
} // namespace pct::filters
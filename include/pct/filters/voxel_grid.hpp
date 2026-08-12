#pragma once
#include "pct/core/point_cloud.hpp"

namespace pct::filters
{
    [[nodiscard]] PointCloud voxelGridDownsample(const PointCloud &cloud,
                                                 float leaf_size);
} // namespace pct::filters
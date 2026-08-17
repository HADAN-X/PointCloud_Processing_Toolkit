#pragma once
#include "pct/core/point_cloud.hpp"

namespace pct::search
{
    struct Neighbor
    {
        PointCloud::size_type index{0};
        double squared_distance{0.0};
    };

    // Sort by index if the distances are the same
    [[nodiscard]] inline bool neighborLess(const Neighbor &lhs, const Neighbor &rhs) noexcept
    {
        if (lhs.squared_distance != rhs.squared_distance)
        {
            return lhs.squared_distance < rhs.squared_distance;
        }
        return lhs.index < rhs.index;
    }
} // namespace pct::search

#include "pct/filters/radius_outlier.hpp"

#include "pct/search/brute_force.hpp"

#include <cmath>
#include <stdexcept>

namespace pct::filters
{
    PointCloud radiusOutlierRemoval(const PointCloud &cloud, float radius, std::size_t min_neighbors)
    {
        if (!std::isfinite(radius) || radius < 0.0F)
        {
            throw std::invalid_argument("radius must be finite and greater than or equal to zero");
        }
        if (min_neighbors == 0U)
        {
            throw std::invalid_argument(
                "minimum neighbor count must be greater than zero");
        }
        PointCloud result;
        result.points().reserve(cloud.size());
        for (PointCloud::size_type index = 0; index < cloud.size(); ++index)
        {
            const auto neighbors = search::radiusSearch(
                cloud,
                cloud[index].position,
                radius,
                index);

            if (neighbors.size() >= min_neighbors)
            {
                result.pushBack(cloud[index]);
            }
        }
        return result;
    }
} // namespace pct::filters

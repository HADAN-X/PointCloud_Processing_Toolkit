#include "pct/search/brute_force.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace pct::search
{
    namespace
    {
        void validateQuery(const PointCloud &cloud, const Eigen::Vector3f &query,
                           std::optional<PointCloud::size_type> excluded_index)
        {
            if (!query.allFinite())
            {
                throw std::invalid_argument("query must be finite");
            }
            if (excluded_index && *excluded_index >= cloud.size())
            {
                throw std::out_of_range("excluded index is out of range");
            }
            for (const auto &point : cloud.points())
            {
                if (!point.position.allFinite())
                {
                    throw std::invalid_argument(
                        "point cloud contains a non-finite position");
                }
            }
        }

        bool neighborLess(const Neighbor &lhs, const Neighbor &rhs) noexcept
        {
            if (lhs.squared_distance != rhs.squared_distance)
            {
                return lhs.squared_distance < rhs.squared_distance;
            }
            // Sort by index if the distances are the same
            return lhs.index < rhs.index;
        }

        std::vector<Neighbor>

        // Exclude only the explicitly requested point index.
        collectNeighbors(const PointCloud &cloud, const Eigen::Vector3f &query,
                         std::optional<PointCloud::size_type> excluded_index)
        {
            std::vector<Neighbor> neighbors;
            // If there are indexes to be excluded, the reserved space will be reduced by 1
            neighbors.reserve(cloud.size() - static_cast<PointCloud::size_type>(excluded_index.has_value()));
            for (PointCloud::size_type index = 0; index < cloud.size(); ++index)
            {
                if (excluded_index && index == *excluded_index)
                {
                    continue;
                }

                // get squared_distance
                const double squared_distance =
                    (cloud[index].position.cast<double>() - query.cast<double>()).squaredNorm();
                neighbors.push_back(Neighbor{index, squared_distance});
            }
            return neighbors;
        }

    } // namespace

    // KNN
    std::vector<Neighbor>
    knnSearch(const PointCloud &cloud, const Eigen::Vector3f &query, std::size_t k,
              std::optional<PointCloud::size_type> excluded_index)
    {
        validateQuery(cloud, query, excluded_index);
        if (k == 0U || cloud.empty() || (excluded_index && cloud.size() == 1U))
        {
            return {};
        }

        auto neighbors = collectNeighbors(cloud, query, excluded_index);
        const std::size_t result_count = std::min(k, neighbors.size());
        std::partial_sort(neighbors.begin(),
                          neighbors.begin() + static_cast<std::ptrdiff_t>(result_count),
                          neighbors.end(),
                          neighborLess);
        neighbors.resize(result_count);
        return neighbors;
    }

    // RADIUSSEARCH
    std::vector<Neighbor>
    radiusSearch(const PointCloud &cloud, const Eigen::Vector3f &query, float radius,
                 std::optional<PointCloud::size_type> excluded_index)
    {
        validateQuery(cloud, query, excluded_index);
        if (!std::isfinite(radius) || radius < 0.0F)
        {
            throw std::invalid_argument("radius must be finite and greater than or equal to zero");
        }
        const double squared_radius = static_cast<double>(radius) * static_cast<double>(radius);
        auto candidates = collectNeighbors(cloud, query, excluded_index);
        candidates.erase(
            std::remove_if(
                candidates.begin(), candidates.end(),
                [squared_radius](const Neighbor &neighbor)
                {
                    return neighbor.squared_distance > squared_radius;
                }),
            candidates.end());
        std::sort(candidates.begin(), candidates.end(), neighborLess);
        return candidates;
    }

} // namespace pct::search

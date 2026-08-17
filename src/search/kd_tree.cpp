#include "pct/search/kd_tree.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <queue>
#include <stdexcept>

namespace pct::search
{
    namespace
    {
        double squaredDistance(const Point &point,
                               const Eigen::Vector3f &query)
        {
            return (point.position.cast<double>() - query.cast<double>()).squaredNorm();
        }
    } // namespace

    KdTree::KdTree(const PointCloud &cloud) : cloud_(cloud)
    {
        const auto start = std::chrono::steady_clock::now();
        for (const auto &point : cloud_.points())
        {
            if (!point.position.allFinite())
            {
                throw std::invalid_argument("point cloud contains a non-finite position");
            }
        }
        std::vector<PointCloud::size_type> indices(cloud_.size());
        std::iota(indices.begin(), indices.end(), PointCloud::size_type{0});
        nodes_.reserve(cloud_.size());
        root_ = build(indices, 0U, indices.size(), 0U);
        stats_.node_count = nodes_.size();
        const auto stop = std::chrono::steady_clock::now();
        stats_.build_time_ms = std::chrono::duration<double, std::milli>(stop - start).count();
    }

    std::size_t KdTree::build(
        std::vector<PointCloud::size_type> &indices,
        std::size_t begin,
        std::size_t end,
        std::size_t depth)
    {
        // empty cloud
        if (begin == end)
        {
            return kNoNode;
        }

        const unsigned char axis = static_cast<unsigned char>(depth % 3U);
        const std::size_t middle = begin + (end - begin) / 2U;
        std::nth_element(
            indices.begin() + static_cast<std::ptrdiff_t>(begin),
            indices.begin() + static_cast<std::ptrdiff_t>(middle),
            indices.begin() + static_cast<std::ptrdiff_t>(end),
            [this, axis](PointCloud::size_type lhs,
                         PointCloud::size_type rhs)
            {
                const float lhs_value = cloud_[lhs].position[axis];
                const float rhs_value = cloud_[rhs].position[axis];
                if (lhs_value != rhs_value)
                {
                    return lhs_value < rhs_value;
                }
                return lhs < rhs;
            });
        const std::size_t node_index = nodes_.size();
        nodes_.push_back(Node{indices[middle], kNoNode, kNoNode, axis});
        const std::size_t left = build(indices, begin, middle, depth + 1U);
        const std::size_t right = build(indices, middle + 1U, end, depth + 1U);
        nodes_[node_index].left = left;
        nodes_[node_index].right = right;
        stats_.max_depth = std::max(stats_.max_depth, depth + 1U);
        return node_index;
    }

    void KdTree::validateQuery(const Eigen::Vector3f &query,
                               std::optional<PointCloud::size_type> excluded_index) const
    {
        if (!query.allFinite())
        {
            throw std::invalid_argument("query must be finite");
        }
        if (excluded_index && *excluded_index >= cloud_.size())
        {
            throw std::out_of_range("excluded index is out of range");
        }
    }

    std::vector<Neighbor> KdTree::knnSearch(const Eigen::Vector3f &query, std::size_t k,
                                            std::optional<PointCloud::size_type> excluded_index) const
    {
        validateQuery(query, excluded_index);
        const std::size_t available = cloud_.size() - static_cast<std::size_t>(excluded_index.has_value());
        const std::size_t result_count = std::min(k, available);
        if (result_count == 0U)
        {
            return {};
        }
        std::priority_queue<Neighbor, std::vector<Neighbor>, decltype(&neighborLess)> best(&neighborLess);
        const auto visit = [&](const auto &self, std::size_t node_index) -> void
        {
            if (node_index == kNoNode)
            {
                return;
            }
            const Node &node = nodes_[node_index];
            const Point &point = cloud_[node.point_index];
            const double difference = static_cast<double>(query[node.axis]) -
                                      static_cast<double>(point.position[node.axis]);
            const std::size_t near_child = difference < 0.0 ? node.left : node.right;
            const std::size_t far_child = difference < 0.0 ? node.right : node.left;
            self(self, near_child);
            if (!excluded_index || node.point_index != *excluded_index)
            {
                const Neighbor candidate{node.point_index, squaredDistance(point, query)};
                if (best.size() < result_count)
                {
                    best.push(candidate);
                }
                else if (neighborLess(candidate, best.top()))
                {
                    best.pop();
                    best.push(candidate);
                }
            }
            const double plane_distance = difference * difference;
            if (best.size() < result_count ||
                plane_distance <= best.top().squared_distance)
            {
                self(self, far_child);
            }
        };
        visit(visit, root_);

        std::vector<Neighbor> result;
        result.reserve(best.size());
        while (!best.empty())
        {
            result.push_back(best.top());
            best.pop();
        }
        std::sort(result.begin(), result.end(), neighborLess);
        return result;
    }

    std::vector<Neighbor> KdTree::radiusSearch(
        const Eigen::Vector3f &query, float radius,
        std::optional<PointCloud::size_type> excluded_index) const
    {
        validateQuery(query, excluded_index);
        if (!std::isfinite(radius) || radius < 0.0F)
        {
            throw std::invalid_argument("radius must be finite and greater than or equal to zero");
        }
        const double squared_radius = static_cast<double>(radius) * static_cast<double>(radius);
        std::vector<Neighbor> result;
        const auto visit = [&](const auto &self, const std::size_t node_index) -> void
        {
            if (node_index == kNoNode)
            {
                return;
            }
            const Node &node = nodes_[node_index];
            const Point &point = cloud_[node.point_index];
            const double difference =
                static_cast<double>(query[node.axis]) -
                static_cast<double>(point.position[node.axis]);
            const std::size_t near_child = difference < 0.0 ? node.left : node.right;
            const std::size_t far_child = difference < 0.0 ? node.right : node.left;
            self(self, near_child);

            if (!excluded_index || node.point_index != *excluded_index)
            {
                const double distance = squaredDistance(point, query);
                if (distance <= squared_radius)
                {
                    result.push_back(Neighbor{node.point_index, distance});
                }
            }
            if (difference * difference <= squared_radius)
            {
                self(self, far_child);
            }
        };
        visit(visit, root_);
        std::sort(result.begin(), result.end(), neighborLess);
        return result;
    }

    const KdTreeStats &KdTree::stats() const noexcept
    {
        return stats_;
    }
} // namespace pct::search

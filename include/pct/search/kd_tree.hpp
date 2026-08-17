#pragma once

#include "pct/core/point_cloud.hpp"
#include "pct/search/neighbor.hpp"

#include <Eigen/Core>
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

namespace pct::search
{
    struct KdTreeStats
    {
        std::size_t node_count{0};
        std::size_t max_depth{0};
        double build_time_ms{0.0};
    };

    class KdTree
    {
    public:
        explicit KdTree(const PointCloud &cloud);
        KdTree(PointCloud &&) = delete;

        [[nodiscard]] std::vector<Neighbor>
        knnSearch(const Eigen::Vector3f &query, std::size_t k,
                  std::optional<PointCloud::size_type> excluded_index = std::nullopt) const;

        [[nodiscard]] std::vector<Neighbor>
        radiusSearch(const Eigen::Vector3f &query, float radius,
                     std::optional<PointCloud::size_type> excluded_index = std::nullopt) const;

        [[nodiscard]] const KdTreeStats &stats() const noexcept;

    private:
        // kNoNode means null node
        static constexpr std::size_t kNoNode =
            std::numeric_limits<std::size_t>::max();

        struct Node
        {
            PointCloud::size_type point_index{0};
            std::size_t left{kNoNode};
            std::size_t right{kNoNode};
            unsigned char axis{0};
        };

        // Left-closed, right-open interval
        std::size_t
        build(std::vector<PointCloud::size_type> &indices,
              std::size_t begin,
              std::size_t end,
              std::size_t depth);

        void validateQuery(
            const Eigen::Vector3f &query,
            std::optional<PointCloud::size_type> excluded_index) const;

        const PointCloud &cloud_;
        std::vector<Node> nodes_;
        std::size_t root_{kNoNode};
        KdTreeStats stats_{};
    };
} // namespace pct::search
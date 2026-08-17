#pragma once

#include "pct/core/point_cloud.hpp"

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <cstddef>
#include <optional>
#include <vector>

namespace pct::features
{
    enum class NormalEstimationStatus
    {
        Valid,
        InsufficientNeighbors,
        DegenerateNeighborhood
    };

    struct NormalEstimate
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Vector3f normal{Eigen::Vector3f::Zero()};
        float curvature{0.0F};
        std::size_t neighbor_count{0};
        NormalEstimationStatus status{
            NormalEstimationStatus::InsufficientNeighbors};

        [[nodiscard]] bool valid() const noexcept
        {
            return status == NormalEstimationStatus::Valid;
        }
    };

    using NormalEstimates =
        std::vector<NormalEstimate, Eigen::aligned_allocator<NormalEstimate>>;

    [[nodiscard]] NormalEstimates estimateNormalsKnn(
        const PointCloud &cloud,
        std::size_t k,
        std::size_t minimum_neighbors = 3U,
        std::optional<Eigen::Vector3f> viewpoint = std::nullopt);

    [[nodiscard]] NormalEstimates estimateNormalsRadius(
        const PointCloud &cloud,
        float radius,
        std::size_t minimum_neighbors = 3U,
        std::optional<Eigen::Vector3f> viewpoint = std::nullopt);
} // namespace pct::features

#pragma once

#include "pct/core/point_cloud.hpp"
#include "pct/geometry/transform.hpp"

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <cstddef>
#include <limits>
#include <vector>

namespace pct::registration
{
    // Describes why ICP stopped.
    enum class IcpTermination
    {
        ConvergedTransform,
        ConvergedRmse,
        MaximumIterations,
        InsufficientCorrespondences,
        DegenerateGeometry,
        NumericalFailure
    };

    struct IcpOptions
    {
        std::size_t max_iterations{50};
        float max_correspondence_distance{1.0F};
        std::size_t minimum_correspondences{3};
        double translation_epsilon{1.0e-6};
        double rotation_epsilon_radians{1.0e-6};
        double rmse_epsilon{1.0e-7};
        geometry::Matrix4f initial_transform{geometry::Matrix4f::Identity()};
    };

    struct IcpIteration
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        std::size_t iteration{0};
        std::size_t correspondence_count{0};
        double rmse{std::numeric_limits<double>::infinity()};
        double rmse_change{std::numeric_limits<double>::infinity()};
        double translation_delta{0.0};
        double rotation_delta_radians{0.0};
        geometry::Matrix4f incremental_transform{geometry::Matrix4f::Identity()};
    };

    using IcpHistory = std::vector<IcpIteration, Eigen::aligned_allocator<IcpIteration>>;

    struct IcpResult
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        geometry::Matrix4f transform{geometry::Matrix4f::Identity()};
        IcpHistory history{};
        IcpTermination termination{IcpTermination::MaximumIterations};
        double final_rmse{std::numeric_limits<double>::infinity()};

        [[nodiscard]] bool converged() const noexcept;
    };

    [[nodiscard]] const char *toString(IcpTermination termination) noexcept;

    [[nodiscard]] IcpResult alignPointToPoint(
        const PointCloud &source,
        const PointCloud &target,
        const IcpOptions &options = {});
} // namespace pct::registration

#include "pct/registration/icp.hpp"

#include "pct/search/kd_tree.hpp"

#include <Eigen/LU>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace pct::registration
{
    namespace
    {
        using Vector3dList = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;

        void validateOptions(const IcpOptions &options)
        {
            if (options.max_iterations == 0)
            {
                throw std::invalid_argument("max_iterations must be greater than zero");
            }

            if (!(options.max_correspondence_distance > 0.0F) || !std::isfinite(options.max_correspondence_distance))
            {
                throw std::invalid_argument("max_correspondence_distance must be finite and greater than zero");
            }

            if (options.minimum_correspondences < 3)
            {
                throw std::invalid_argument(
                    "minimum_correspondences must be at least three");
            }
            if (options.translation_epsilon < 0.0 ||
                !std::isfinite(options.translation_epsilon) ||
                options.rotation_epsilon_radians < 0.0 ||
                !std::isfinite(options.rotation_epsilon_radians) ||
                options.rmse_epsilon < 0.0 ||
                !std::isfinite(options.rmse_epsilon))
            {
                throw std::invalid_argument(
                    "ICP convergence tolerances must be finite and non-negative");
            }
            if (!geometry::isRigidTransform(options.initial_transform))
            {
                throw std::invalid_argument("initial_transform must be a finite rigid transform");
            }
        }
        void validateCloud(const PointCloud &cloud, const char *name)
        {
            for (const Point &point : cloud.points())
            {
                if (!point.position.allFinite())
                {
                    throw std::invalid_argument(
                        std::string(name) + " contains a non-finite position");
                }
            }
        }

        geometry::Matrix4f toFloatTransform(const Eigen::Matrix4d &transform)
        {
            return transform.cast<float>();
        }

        double rotationAngle(const Eigen::Matrix3d &rotation)
        {
            // trace(R) = 1 + 2*cos(θ)
            const double cosine = std::clamp((rotation.trace() - 1.0) * 0.5, -1.0, 1.0);
            return std::acos(cosine);
        }
    } // namespace

    bool IcpResult::converged() const noexcept
    {
        return termination == IcpTermination::ConvergedTransform ||
               termination == IcpTermination::ConvergedRmse;
    }

    const char *toString(IcpTermination termination) noexcept
    {
        switch (termination)
        {
        case IcpTermination::ConvergedTransform:
            return "converged_transform";
        case IcpTermination::ConvergedRmse:
            return "converged_rmse";
        case IcpTermination::MaximumIterations:
            return "maximum_iterations";
        case IcpTermination::InsufficientCorrespondences:
            return "insufficient_correspondences";
        case IcpTermination::DegenerateGeometry:
            return "degenerate_geometry";
        case IcpTermination::NumericalFailure:
            return "numerical_failure";
        }
        return "unknown";
    }

    IcpResult alignPointToPoint(
        const PointCloud &source,
        const PointCloud &target,
        const IcpOptions &options)
    {
        validateOptions(options);
        validateCloud(source, "source");
        validateCloud(target, "target");

        IcpResult result;
        Eigen::Matrix4d cumulative = options.initial_transform.cast<double>();
        result.transform = options.initial_transform;
        if (source.size() < options.minimum_correspondences || target.empty())
        {
            result.termination = IcpTermination::InsufficientCorrespondences;
            return result;
        }

        const search::KdTree target_tree{target};
        const double maximum_squared_distance = static_cast<double>(options.max_correspondence_distance) *
                                                static_cast<double>(options.max_correspondence_distance);
        double previous_rmse = std::numeric_limits<double>::infinity();

        Vector3dList transformed_points;
        Vector3dList target_points;
        transformed_points.reserve(source.size());
        target_points.reserve(source.size());

        for (std::size_t iteration = 1; iteration <= options.max_iterations; ++iteration)
        {
            transformed_points.clear();
            target_points.clear();

            const Eigen::Matrix3d cumulative_rotation = cumulative.block<3, 3>(0, 0);
            const Eigen::Vector3d cumulative_translation = cumulative.block<3, 1>(0, 3);

            // Transform each source point and find its closest target point.
            for (const Point &point : source.points())
            {
                const Eigen::Vector3d transformed =
                    cumulative_rotation * point.position.cast<double>() + cumulative_translation;
                const auto neighbors = target_tree.knnSearch(transformed.cast<float>(), 1);
                if (!neighbors.empty() && neighbors.front().squared_distance <= maximum_squared_distance)
                {
                    transformed_points.push_back(transformed);
                    target_points.push_back(target[neighbors.front().index].position.cast<double>());
                }
            }

            if (transformed_points.size() < options.minimum_correspondences)
            {
                result.transform = toFloatTransform(cumulative);
                result.termination = IcpTermination::InsufficientCorrespondences;
                return result;
            }

            // Compute centroids for the accepted correspondence set.
            Eigen::Vector3d source_centroid = Eigen::Vector3d::Zero();
            Eigen::Vector3d target_centroid = Eigen::Vector3d::Zero();
            for (std::size_t i = 0; i < transformed_points.size(); ++i)
            {
                source_centroid += transformed_points[i];
                target_centroid += target_points[i];
            }
            const double inverse_count = 1.0 / static_cast<double>(transformed_points.size());
            source_centroid *= inverse_count;
            target_centroid *= inverse_count;

            // H = Σ (p_i - μ_p) * (q_i - μ_q)^T
            Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
            for (std::size_t i = 0; i < transformed_points.size(); ++i)
            {
                covariance += (transformed_points[i] - source_centroid) * (target_points[i] - target_centroid).transpose();
            }

            const Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
            const Eigen::Vector3d singular_values = svd.singularValues();
            if (!singular_values.allFinite())
            {
                result.transform = toFloatTransform(cumulative);
                result.termination = IcpTermination::NumericalFailure;
                return result;
            }
            // At least two independent directions are required.
            if (singular_values[0] <= std::numeric_limits<double>::epsilon() || singular_values[1] <= singular_values[0] * 1.0e-12)
            {
                result.transform = toFloatTransform(cumulative);
                result.termination = IcpTermination::DegenerateGeometry;
                return result;
            }

            // R = V * U^T
            Eigen::Matrix3d right_vectors = svd.matrixV();
            Eigen::Matrix3d incremental_rotation = right_vectors * svd.matrixU().transpose();
            // Correct a possible reflection so the update remains a rotation.
            if (incremental_rotation.determinant() < 0.0)
            {
                right_vectors.col(2) *= -1.0;
                incremental_rotation = right_vectors * svd.matrixU().transpose();
            }
            const Eigen::Vector3d incremental_translation = target_centroid - incremental_rotation * source_centroid;
            if (!incremental_rotation.allFinite() || !incremental_translation.allFinite() || incremental_rotation.determinant() <= 0.0)
            {
                result.transform = toFloatTransform(cumulative);
                result.termination = IcpTermination::NumericalFailure;
                return result;
            }

            Eigen::Matrix4d incremental = Eigen::Matrix4d::Identity();
            incremental.block<3, 3>(0, 0) = incremental_rotation;
            incremental.block<3, 1>(0, 3) = incremental_translation;
            cumulative = incremental * cumulative;

            // Compute RMSE after applying this iteration's rigid update.
            double squared_error_sum = 0.0;
            for (std::size_t i = 0; i < transformed_points.size(); ++i)
            {
                const Eigen::Vector3d residual =
                    incremental_rotation * transformed_points[i] + incremental_translation - target_points[i];
                squared_error_sum += residual.squaredNorm();
            }
            const double rmse = std::sqrt(squared_error_sum * inverse_count);
            const double rmse_change = std::isfinite(previous_rmse) ? std::abs(previous_rmse - rmse) : std::numeric_limits<double>::infinity();

            IcpIteration record;
            record.iteration = iteration;
            record.correspondence_count = transformed_points.size();
            record.rmse = rmse;
            record.rmse_change = rmse_change;
            record.translation_delta = incremental_translation.norm();
            record.rotation_delta_radians = rotationAngle(incremental_rotation);
            record.incremental_transform = toFloatTransform(incremental);
            result.history.push_back(record);
            result.transform = toFloatTransform(cumulative);
            result.final_rmse = rmse;

            if (!std::isfinite(rmse))
            {
                result.termination = IcpTermination::NumericalFailure;
                return result;
            }
            if (record.translation_delta <= options.translation_epsilon &&
                record.rotation_delta_radians <= options.rotation_epsilon_radians)
            {
                result.termination = IcpTermination::ConvergedTransform;
                return result;
            }
            if (std::isfinite(rmse_change) && rmse_change <= options.rmse_epsilon)
            {
                result.termination = IcpTermination::ConvergedRmse;
                return result;
            }
            previous_rmse = rmse;
        }
        result.termination = IcpTermination::MaximumIterations;
        return result;
    }

} // namespace pct::registration

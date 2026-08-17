#include "pct/features/normal_estimation.hpp"

#include "pct/search/kd_tree.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace pct::features
{
    namespace
    {
        constexpr double kAbsoluteVarianceTolerance = 1.0e-15;
        constexpr double kRelativeRankTolerance = 1.0e-6;

        void validateCommonParameters(
            std::size_t minimum_neighbors,
            const std::optional<Eigen::Vector3f> &viewpoint)
        {
            if (minimum_neighbors < 3U)
            {
                throw std::invalid_argument(
                    "minimum neighbors must be at least three");
            }
            if (viewpoint && !viewpoint->allFinite())
            {
                throw std::invalid_argument("viewpoint must be finite");
            }
        }

        void canonicalizeNormal(Eigen::Vector3d &normal)
        {
            Eigen::Index dominant_axis = 0;
            normal.cwiseAbs().maxCoeff(&dominant_axis);
            if (normal[dominant_axis] < 0.0)
            {
                normal = -normal;
            }
        }

        void orientNormal(
            Eigen::Vector3d &normal,
            const Eigen::Vector3f &point,
            const std::optional<Eigen::Vector3f> &viewpoint)
        {
            if (!viewpoint)
            {
                canonicalizeNormal(normal);
                return;
            }
            const Eigen::Vector3d view_direction =
                viewpoint->cast<double>() - point.cast<double>();
            const double facing = normal.dot(view_direction);
            if (facing < 0.0)
            {
                normal = -normal;
            }
            else if (facing == 0.0)
            {
                canonicalizeNormal(normal);
            }
        }

        NormalEstimate estimateOne(
            const PointCloud &cloud,
            PointCloud::size_type point_index,
            const std::vector<search::Neighbor> &neighbors,
            std::size_t minimum_neighbors,
            const std::optional<Eigen::Vector3f> &viewpoint)
        {
            NormalEstimate estimate;
            estimate.neighbor_count = neighbors.size();
            if (neighbors.size() < minimum_neighbors)
            {
                return estimate;
            }

            Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
            for (const auto &neighbor : neighbors)
            {
                centroid += cloud[neighbor.index].position.cast<double>();
            }
            centroid /= static_cast<double>(neighbors.size());

            Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
            for (const auto &neighbor : neighbors)
            {
                const Eigen::Vector3d centered =
                    cloud[neighbor.index].position.cast<double>() - centroid;
                covariance.noalias() += centered * centered.transpose();
            }
            covariance /= static_cast<double>(neighbors.size());

            const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver{
                covariance};
            if (solver.info() != Eigen::Success ||
                !solver.eigenvalues().allFinite() ||
                !solver.eigenvectors().allFinite())
            {
                estimate.status = NormalEstimationStatus::DegenerateNeighborhood;
                return estimate;
            }

            const Eigen::Vector3d eigenvalues =
                solver.eigenvalues().cwiseMax(0.0);
            const double largest = eigenvalues[2];
            const double rank_tolerance =
                std::max(kAbsoluteVarianceTolerance,
                         largest * kRelativeRankTolerance);
            if (largest <= kAbsoluteVarianceTolerance ||
                eigenvalues[1] <= rank_tolerance)
            {
                estimate.status = NormalEstimationStatus::DegenerateNeighborhood;
                return estimate;
            }

            Eigen::Vector3d normal = solver.eigenvectors().col(0);
            const double normal_length = normal.norm();
            if (!std::isfinite(normal_length) || normal_length <= 1.0e-12)
            {
                estimate.status = NormalEstimationStatus::DegenerateNeighborhood;
                return estimate;
            }

            normal /= normal_length;
            orientNormal(normal, cloud[point_index].position, viewpoint);

            const double eigenvalue_sum = eigenvalues.sum();
            estimate.normal = normal.cast<float>();
            estimate.curvature = static_cast<float>(
                std::clamp(eigenvalues[0] / eigenvalue_sum, 0.0, 1.0));
            estimate.status = NormalEstimationStatus::Valid;
            return estimate;
        }

        template <typename NeighborhoodQuery>
        NormalEstimates estimateNormals(
            const PointCloud &cloud,
            std::size_t minimum_neighbors,
            const std::optional<Eigen::Vector3f> &viewpoint,
            NeighborhoodQuery &&query)
        {
            NormalEstimates estimates;
            estimates.reserve(cloud.size());
            for (PointCloud::size_type point_index = 0;
                 point_index < cloud.size(); ++point_index)
            {
                estimates.push_back(estimateOne(
                    cloud,
                    point_index,
                    query(point_index),
                    minimum_neighbors,
                    viewpoint));
            }
            return estimates;
        }

    } // namespace

    NormalEstimates estimateNormalsKnn(
        const PointCloud &cloud, std::size_t k, std::size_t minimum_neighbors, std::optional<Eigen::Vector3f> viewpoint)
    {
        validateCommonParameters(minimum_neighbors, viewpoint);
        if (k < minimum_neighbors)
        {
            throw std::invalid_argument(
                "k must be greater than or equal to minimum neighbors");
        }
        const search::KdTree tree{cloud};
        return estimateNormals(cloud, minimum_neighbors, viewpoint,
                               [&cloud, &tree, k](PointCloud::size_type point_index)
                               {
                                   return tree.knnSearch(cloud[point_index].position, k);
                               });
    }

    NormalEstimates estimateNormalsRadius(
        const PointCloud &cloud, float radius, std::size_t minimum_neighbors,
        std::optional<Eigen::Vector3f> viewpoint)
    {
        validateCommonParameters(minimum_neighbors, viewpoint);
        if (!std::isfinite(radius) || !(radius > 0.0F))
        {
            throw std::invalid_argument(
                "radius must be finite and greater than zero");
        }

        const search::KdTree tree{cloud};
        return estimateNormals(
            cloud, minimum_neighbors, viewpoint,
            [&cloud, &tree, radius](PointCloud::size_type point_index)
            {
                return tree.radiusSearch(
                    cloud[point_index].position, radius);
            });
    }
} // namespace pct::features

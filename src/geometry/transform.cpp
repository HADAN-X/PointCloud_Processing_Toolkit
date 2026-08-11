#include "pct/geometry/transform.hpp"
#include <Eigen/Geometry>
#include <stdexcept>
#include <cmath>
#include <utility>

namespace pct::geometry
{
    Matrix4f makeRigidTransform(
        const Eigen::Matrix3f &rotation,
        const Eigen::Vector3f &translation)
    {
        Matrix4f transform = Matrix4f::Identity();
        transform.block<3, 3>(0, 0) = rotation;
        transform.block<3, 1>(0, 3) = translation;

        if (!isRigidTransform(transform))
        {
            throw std::invalid_argument(
                "The provided rotation matrix is not a valid rotation matrix.");
        }
        return transform;
    }

    Matrix4f makeTranslation(const Eigen::Vector3f &translation)
    {
        return makeRigidTransform(Eigen::Matrix3f::Identity(), translation);
    }

    Matrix4f makeRotationX(float radians)
    {
        const Eigen::Matrix3f rotation =
            Eigen::AngleAxisf(radians, Eigen::Vector3f::UnitX()).toRotationMatrix();
        return makeRigidTransform(rotation, Eigen::Vector3f::Zero());
    }

    Matrix4f makeRotationY(float radians)
    {
        const Eigen::Matrix3f rotation =
            Eigen::AngleAxisf(radians, Eigen::Vector3f::UnitY()).toRotationMatrix();
        return makeRigidTransform(rotation, Eigen::Vector3f::Zero());
    }

    Matrix4f makeRotationZ(float radians)
    {
        const Eigen::Matrix3f rotation =
            Eigen::AngleAxisf(radians, Eigen::Vector3f::UnitZ()).toRotationMatrix();
        return makeRigidTransform(rotation, Eigen::Vector3f::Zero());
    }

    bool isRigidTransform(const Eigen::Matrix4f &transform, float tolerance)
    {
        if (!(tolerance > 0.0F) || !transform.allFinite())
        {
            return false;
        }

        const Eigen::Vector4f expected_bottom_row{0.0F, 0.0F, 0.0F, 1.0F};
        if (!transform.row(3).transpose().isApprox(expected_bottom_row, tolerance))
        {
            return false;
        }

        const Eigen::Matrix3f rotation = transform.block<3, 3>(0, 0);
        const Eigen::Matrix3f should_be_identity = rotation.transpose() * rotation;
        if (!should_be_identity.isApprox(Eigen::Matrix3f::Identity(), tolerance))
        {
            return false;
        }
        if (std::abs(rotation.determinant() - 1.0F) > tolerance)
        {
            return false;
        }
        return true;
    }
    PointCloud transformPointCloud(const PointCloud &cloud, const Matrix4f &transform)
    {
        if (!isRigidTransform(transform))
        {
            throw std::invalid_argument(
                "transform must be a finite rigid transform");
        }
        const Eigen::Matrix3f rotation = transform.block<3, 3>(0, 0);
        const Eigen::Vector3f translation = transform.block<3, 1>(0, 3);

        PointCloud transformed_cloud;
        for (const auto &point : cloud.points())
        {
            if (!point.position.allFinite())
            {
                throw std::invalid_argument(
                    "point cloud contains a non-finite position");
            }
            Point transformed_point(point);
            transformed_point.position = rotation * point.position + translation;
            transformed_cloud.pushBack(std::move(transformed_point));
        }
        return transformed_cloud;
    }
} // namespace pct::geometry
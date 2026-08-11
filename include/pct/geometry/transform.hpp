#pragma once
#include "pct/core/point_cloud.hpp"
#include <Eigen/Core>

namespace pct::geometry
{
    using Matrix4f = Eigen::Matrix4f;

    [[nodiscard]] Matrix4f makeRigidTransform(
        const Eigen::Matrix3f &rotation,
        const Eigen::Vector3f &translation);
    [[nodiscard]] Matrix4f makeTranslation(const Eigen::Vector3f &translation);
    [[nodiscard]] Matrix4f makeRotationX(float radians);
    [[nodiscard]] Matrix4f makeRotationY(float radians);
    [[nodiscard]] Matrix4f makeRotationZ(float radians);
    [[nodiscard]] bool isRigidTransform(const Matrix4f &transform,
                                        float tolerance = 1.0e-4F);

    [[nodiscard]] PointCloud
    transformPointCloud(const PointCloud &cloud, const Matrix4f &transform);

} // namespace pct::geometry
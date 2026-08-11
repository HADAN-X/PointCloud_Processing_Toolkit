#include "pct/geometry/statistics.hpp"
#include <stdexcept>

namespace pct::geometry
{
    PointCloudStatistics computeStatistics(const PointCloud &cloud)
    {
        PointCloudStatistics statistics;
        statistics.point_count = cloud.size();
        if (cloud.empty())
        {
            return statistics;
        }
        // Use double to reduce cumulative errors
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        Eigen::Vector3f minimum = cloud[0].position;
        Eigen::Vector3f maximum = cloud[0].position;

        for (const auto &point : cloud.points())
        {
            if (!point.position.allFinite())
            {
                throw std::invalid_argument(
                    "point cloud contains a non-finite position");
            }

            sum += point.position.cast<double>();
            // Find the maximum and minimum values for each component.
            minimum = minimum.cwiseMin(point.position);
            maximum = maximum.cwiseMax(point.position);
        }
        statistics.centroid =
            (sum / static_cast<double>(cloud.size())).cast<float>();
        statistics.bounding_box = AxisAlignedBoundingBox{minimum, maximum};
        return statistics;
    }
} // namespace pct::geometry
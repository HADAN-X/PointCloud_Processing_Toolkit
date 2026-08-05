#include "pct/core/point_cloud.hpp"

#include <utility>

namespace pct
{

    PointCloud::PointCloud(Container points) : points_(std::move(points)) {}

    bool PointCloud::empty() const noexcept
    {
        return points_.empty();
    }

    PointCloud::size_type PointCloud::size() const noexcept
    {
        return points_.size();
    }

    void PointCloud::clear() noexcept
    {
        points_.clear();
    }

    void PointCloud::pushBack(const Point &point)
    {
        points_.push_back(point);
    }

    void PointCloud::pushBack(Point &&point)
    {
        points_.push_back(std::move(point));
    }

    Point &PointCloud::at(size_type index)
    {
        return points_.at(index);
    }

    const Point &PointCloud::at(size_type index) const
    {
        return points_.at(index);
    }

    Point &PointCloud::operator[](size_type index) noexcept
    {
        return points_[index];
    }

    const Point &PointCloud::operator[](size_type index) const noexcept
    {
        return points_[index];
    }

    PointCloud::Container &PointCloud::points() noexcept
    {
        return points_;
    }

    const PointCloud::Container &PointCloud::points() const noexcept
    {
        return points_;
    }

} // namespace pct
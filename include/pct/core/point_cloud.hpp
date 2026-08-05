#pragma once

#include "pct/core/point.hpp"

#include <Eigen/StdVector>

#include <cstddef>
#include <vector>

namespace pct
{

    class PointCloud
    {
    public:
        using Container = std::vector<Point, Eigen::aligned_allocator<Point>>;
        using size_type = Container::size_type;

        PointCloud() = default;
        explicit PointCloud(Container points);

        [[nodiscard]] bool empty() const noexcept;
        [[nodiscard]] size_type size() const noexcept;

        void clear() noexcept;
        void pushBack(const Point &point);
        void pushBack(Point &&point);

        Point &at(size_type index);
        const Point &at(size_type index) const;

        Point &operator[](size_type index) noexcept;
        const Point &operator[](size_type index) const noexcept;

        Container &points() noexcept;
        const Container &points() const noexcept;

    private:
        Container points_;
    };

} // namespace pct
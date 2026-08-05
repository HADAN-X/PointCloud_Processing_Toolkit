#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <optional>

namespace pct
{

    struct ColorRgb8
    {
        std::uint8_t red{0};
        std::uint8_t green{0};
        std::uint8_t blue{0};
    };

    inline bool operator==(const ColorRgb8 &lhs, const ColorRgb8 &rhs) noexcept
    {
        return lhs.red == rhs.red && lhs.green == rhs.green &&
               lhs.blue == rhs.blue;
    }

    struct Point
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Vector3f position{Eigen::Vector3f::Zero()};
        std::optional<ColorRgb8> color{};

        Point() = default;

        explicit Point(const Eigen::Vector3f &point_position)
            : position(point_position) {}

        Point(const Eigen::Vector3f &point_position, ColorRgb8 point_color)
            : position(point_position), color(point_color) {}
    };

} // namespace pct
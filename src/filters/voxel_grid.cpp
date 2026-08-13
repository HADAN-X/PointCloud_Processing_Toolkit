#include "pct/filters/voxel_grid.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace pct::filters
{
    namespace
    {
        struct VoxelKey
        {
            std::int64_t x{0};
            std::int64_t y{0};
            std::int64_t z{0};
            [[nodiscard]] bool operator==(const VoxelKey &other) const noexcept
            {
                return x == other.x && y == other.y && z == other.z;
            }

            [[nodiscard]] bool operator<(const VoxelKey &other) const noexcept
            {
                return std::tie(x, y, z) < std::tie(other.x, other.y, other.z);
            }
        };

        struct VoxelKeyHash
        {
            [[nodiscard]] std::size_t operator()(const VoxelKey &key) const noexcept
            {
                std::size_t seed = 0;
                const auto combine = [&seed](std::int64_t value)
                {
                    const std::size_t hash = std::hash<std::int64_t>{}(value);
                    seed ^= hash + static_cast<std::size_t>(0x9e3779b97f4a7c15ULL) + (seed << 6U) + (seed >> 2u);
                };
                combine(key.x);
                combine(key.y);
                combine(key.z);
                return seed;
            }
        };

        struct VoxelAccumulator
        {
            Eigen::Vector3d position_sum{Eigen::Vector3d::Zero()};
            std::array<std::uint64_t, 3> color_sum{0, 0, 0};
            std::size_t count{0};
        };

        std::int64_t coordinateToIndex(float coordinate, float leaf_size)
        {
            const long double scaled = std::floor(
                static_cast<long double>(coordinate) /
                static_cast<long double>(leaf_size));
            const long double minimum = static_cast<long double>(
                std::numeric_limits<std::int64_t>::min());
            const long double maximum = static_cast<long double>(
                std::numeric_limits<std::int64_t>::max());
            if (!std::isfinite(scaled) || scaled < minimum || scaled > maximum)
            {
                throw std::overflow_error("voxel index exceeds the signed 64-bit range");
            }
            return static_cast<std::int64_t>(scaled);
        }

        VoxelKey pointToKey(const Eigen::Vector3f &position, float leaf_size)
        {
            return {
                coordinateToIndex(position.x(), leaf_size),
                coordinateToIndex(position.y(), leaf_size),
                coordinateToIndex(position.z(), leaf_size),
            };
        }

        std::uint8_t roundedColor(std::uint64_t sum, std::size_t count)
        {
            const double mean = static_cast<double>(sum) /
                                static_cast<double>(count);
            return static_cast<std::uint8_t>(std::lround(mean));
        }

    } // namespace

    PointCloud voxelGridDownsample(const PointCloud &cloud, float leaf_size)
    {

        // Determine the validity of a leaf_size
        if (!std::isfinite(leaf_size) || !(leaf_size > 0.0F))
        {
            throw std::invalid_argument("leaf size must be finite and greater than zero");
        }

        // Determine the validity of position and color
        bool any_color = false;
        bool all_color = true;
        for (const auto &point : cloud.points())
        {
            if (!point.position.allFinite())
            {
                throw std::invalid_argument("point cloud contains a non-finite position");
            }
            any_color = any_color || point.color.has_value();
            all_color = all_color && point.color.has_value();
        }
        if (any_color && !all_color)
        {
            throw std::invalid_argument(
                "point cloud must be entirely colored or entirely uncolored");
        }

        // VoxelGridDownsamlpe
        std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash> voxels;
        for (const auto &point : cloud.points())
        {
            const VoxelKey key = pointToKey(point.position, leaf_size);
            auto &accumulator = voxels[key];
            accumulator.position_sum += point.position.cast<double>();
            ++accumulator.count;
            if (all_color)
            {
                accumulator.color_sum[0] += point.color->red;
                accumulator.color_sum[1] += point.color->green;
                accumulator.color_sum[2] += point.color->blue;
            }
        }
        std::vector<VoxelKey> ordered_keys;
        ordered_keys.reserve(voxels.size());
        for (const auto &entry : voxels)
        {
            ordered_keys.push_back(entry.first);
        }
        std::sort(ordered_keys.begin(), ordered_keys.end());
        PointCloud result;
        result.points().reserve(ordered_keys.size());
        for (const auto &key : ordered_keys)
        {
            const auto &accumulator = voxels.at(key);
            const Eigen::Vector3f centroid = (accumulator.position_sum / static_cast<double>(accumulator.count)).cast<float>();
            if (all_color)
            {
                const ColorRgb8 color{
                    roundedColor(accumulator.color_sum[0], accumulator.count),
                    roundedColor(accumulator.color_sum[1], accumulator.count),
                    roundedColor(accumulator.color_sum[2], accumulator.count)};
                result.pushBack(Point{centroid, color});
            }
            else
            {
                result.pushBack(Point{centroid});
            }
        }
        return result;
    }

} // namespace pct::filters

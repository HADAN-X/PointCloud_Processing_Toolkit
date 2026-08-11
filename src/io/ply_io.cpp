#include "pct/io/ply_io.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <unordered_set>
#include <utility>
#include <vector>

namespace pct::io
{
    namespace
    {

        struct Property
        {
            std::string type; // e.g."float", "uchar"
            std::string name; // e.g."x", "red"
        };

        struct Header
        {
            std::size_t vertex_count{0};
            std::vector<Property> vertex_properties;
            bool has_color{false};
        };

        [[noreturn]] void fail(const std::filesystem::path &path,
                               const std::string &message)
        {
            throw PlyError(path.string() + ": " + message);
        }

        void removeCarriageReturn(std::string &line)
        {
            if (!line.empty() && line.back() == '\r')
            {
                line.pop_back();
            }
        }

        std::vector<std::string> splitWhitespace(const std::string &line)
        {
            std::istringstream input(line);
            input.imbue(std::locale::classic());

            std::vector<std::string> tokens;
            for (std::string token; input >> token;)
            {
                tokens.push_back(std::move(token));
            }
            return tokens;
        }

        std::size_t parseSize(const std::filesystem::path &path,
                              std::string_view token,
                              const std::string &field_name)
        {
            if (token.empty() || token.front() == '-')
            {
                fail(path, "invalid " + field_name + ": " + std::string(token));
            }

            std::uint64_t value{0};
            const auto *begin = token.data();
            const auto *end = token.data() + token.size();
            const auto result = std::from_chars(begin, end, value);
            if (result.ec != std::errc{} || result.ptr != end ||
                value > std::numeric_limits<std::size_t>::max())
            {
                fail(path, "invalid " + field_name + ": " + std::string(token));
            }
            return static_cast<std::size_t>(value);
        }

        float parseCoordinate(const std::filesystem::path &path,
                              const std::string &token,
                              const std::string &property_name,
                              std::size_t vertex_index)
        {
            std::istringstream input(token);
            input.imbue(std::locale::classic());

            float value{0.0F};
            input >> value;
            char extra{'\0'};
            if (!input || (input >> extra) || !std::isfinite(value))
            {
                fail(path, "invalid " + property_name + " value at vertex " +
                               std::to_string(vertex_index));
            }
            return value;
        }

        std::uint8_t parseColor(const std::filesystem::path &path,
                                std::string_view token,
                                const std::string &property_name,
                                std::size_t vertex_index)
        {
            int value{0};
            const auto *begin = token.data();
            const auto *end = token.data() + token.size();
            const auto result = std::from_chars(begin, end, value);
            if (result.ec != std::errc{} || result.ptr != end || value < 0 ||
                value > 255)
            {
                fail(path, "invalid " + property_name + " value at vertex " +
                               std::to_string(vertex_index));
            }
            return static_cast<std::uint8_t>(value);
        }

        bool isSupportedScalarType(const std::string &type)
        {
            static const std::unordered_set<std::string> types{
                "char", "uchar", "short", "ushort", "int", "uint",
                "float", "double", "int8", "uint8", "int16", "uint16",
                "int32", "uint32", "float32", "float64"};
            return types.find(type) != types.end();
        }

        std::size_t findUniqueProperty(const std::filesystem::path &path,
                                       const std::vector<Property> &properties,
                                       const std::string &name)
        {
            std::optional<std::size_t> found;
            for (std::size_t index = 0; index < properties.size(); ++index)
            {
                if (properties[index].name == name)
                {
                    if (found.has_value())
                    {
                        fail(path, "duplicate vertex property: " + name);
                    }
                    found = index;
                }
            }
            if (!found.has_value())
            {
                fail(path, "missing required vertex property: " + name);
            }
            return *found;
        }

        std::optional<std::size_t> findOptionalProperty(
            const std::filesystem::path &path,
            const std::vector<Property> &properties,
            const std::string &name)
        {
            std::optional<std::size_t> found;
            for (std::size_t index = 0; index < properties.size(); ++index)
            {
                if (properties[index].name == name)
                {
                    if (found.has_value())
                    {
                        fail(path, "duplicate vertex property: " + name);
                    }
                    found = index;
                }
            }
            return found;
        }

        Header readHeader(std::ifstream &input, const std::filesystem::path &path)
        {
            std::string line;
            if (!std::getline(input, line))
            {
                fail(path, "file is empty");
            }
            removeCarriageReturn(line);
            if (line != "ply")
            {
                fail(path, "missing PLY magic header");
            }

            Header header;
            bool format_found{false};
            bool vertex_element_found{false};
            bool end_header_found{false};
            std::string current_element;

            while (std::getline(input, line))
            {
                removeCarriageReturn(line);
                const auto tokens = splitWhitespace(line);
                if (tokens.empty())
                {
                    fail(path, "blank line inside PLY header");
                }

                if (tokens[0] == "comment" || tokens[0] == "obj_info")
                {
                    continue;
                }

                if (tokens[0] == "format")
                {
                    if (tokens.size() != 3 || format_found)
                    {
                        fail(path, "invalid or duplicate format declaration");
                    }
                    if (tokens[1] != "ascii" || tokens[2] != "1.0")
                    {
                        fail(path, "only ASCII PLY format 1.0 is supported");
                    }
                    format_found = true;
                    continue;
                }

                if (tokens[0] == "element")
                {
                    if (tokens.size() != 3)
                    {
                        fail(path, "invalid element declaration");
                    }
                    current_element = tokens[1];
                    if (current_element == "vertex")
                    {
                        if (vertex_element_found)
                        {
                            fail(path, "duplicate vertex element");
                        }
                        header.vertex_count =
                            parseSize(path, tokens[2], "vertex count");
                        vertex_element_found = true;
                    }
                    else
                    {
                        static_cast<void>(parseSize(path, tokens[2],
                                                    current_element + " count"));
                    }
                    continue;
                }

                if (tokens[0] == "property")
                {
                    if (current_element.empty())
                    {
                        fail(path, "property declared before any element");
                    }
                    if (current_element != "vertex")
                    {
                        continue;
                    }
                    if (tokens.size() != 3 || tokens[1] == "list")
                    {
                        fail(path, "vertex list properties are not supported");
                    }
                    if (!isSupportedScalarType(tokens[1]))
                    {
                        fail(path, "unsupported vertex property type: " + tokens[1]);
                    }
                    header.vertex_properties.push_back({tokens[1], tokens[2]});
                    continue;
                }

                if (tokens[0] == "end_header")
                {
                    if (tokens.size() != 1)
                    {
                        fail(path, "invalid end_header declaration");
                    }
                    end_header_found = true;
                    break;
                }

                fail(path, "unsupported header directive: " + tokens[0]);
            }

            if (!format_found)
            {
                fail(path, "missing format declaration");
            }
            if (!vertex_element_found)
            {
                fail(path, "missing vertex element");
            }
            if (!end_header_found)
            {
                fail(path, "missing end_header declaration");
            }

            static_cast<void>(findUniqueProperty(path, header.vertex_properties, "x"));
            static_cast<void>(findUniqueProperty(path, header.vertex_properties, "y"));
            static_cast<void>(findUniqueProperty(path, header.vertex_properties, "z"));

            const auto red =
                findOptionalProperty(path, header.vertex_properties, "red");
            const auto green =
                findOptionalProperty(path, header.vertex_properties, "green");
            const auto blue =
                findOptionalProperty(path, header.vertex_properties, "blue");
            const auto color_count = static_cast<int>(red.has_value()) +
                                     static_cast<int>(green.has_value()) +
                                     static_cast<int>(blue.has_value());
            if (color_count != 0 && color_count != 3)
            {
                fail(path, "red, green, and blue properties must appear together");
            }
            header.has_color = color_count == 3;
            return header;
        }

    } // namespace

    PointCloud readPly(const std::filesystem::path &path)
    {
        std::ifstream input(path);
        if (!input)
        {
            fail(path, "cannot open file for reading");
        }
        input.imbue(std::locale::classic()); // Use only decimal points

        const Header header = readHeader(input, path);
        const auto x_index =
            findUniqueProperty(path, header.vertex_properties, "x");
        const auto y_index =
            findUniqueProperty(path, header.vertex_properties, "y");
        const auto z_index =
            findUniqueProperty(path, header.vertex_properties, "z");

        std::optional<std::size_t> red_index;
        std::optional<std::size_t> green_index;
        std::optional<std::size_t> blue_index;
        if (header.has_color)
        {
            red_index = findOptionalProperty(path, header.vertex_properties, "red");
            green_index =
                findOptionalProperty(path, header.vertex_properties, "green");
            blue_index =
                findOptionalProperty(path, header.vertex_properties, "blue");
        }

        PointCloud::Container points;
        points.reserve(header.vertex_count);

        std::string line;
        for (std::size_t vertex = 0; vertex < header.vertex_count; ++vertex)
        {
            if (!std::getline(input, line))
            {
                fail(path, "expected " + std::to_string(header.vertex_count) +
                               " vertices but reached end of file at vertex " +
                               std::to_string(vertex));
            }
            removeCarriageReturn(line);
            const auto tokens = splitWhitespace(line);
            if (tokens.size() != header.vertex_properties.size())
            {
                fail(path, "vertex " + std::to_string(vertex) + " has " +
                               std::to_string(tokens.size()) +
                               " values; expected " +
                               std::to_string(header.vertex_properties.size()));
            }

            const Eigen::Vector3f position{
                parseCoordinate(path, tokens[x_index], "x", vertex),
                parseCoordinate(path, tokens[y_index], "y", vertex),
                parseCoordinate(path, tokens[z_index], "z", vertex)};

            if (header.has_color)
            {
                const ColorRgb8 color{
                    parseColor(path, tokens[*red_index], "red", vertex),
                    parseColor(path, tokens[*green_index], "green", vertex),
                    parseColor(path, tokens[*blue_index], "blue", vertex)};
                points.emplace_back(position, color);
            }
            else
            {
                points.emplace_back(position);
            }
        }

        return PointCloud{std::move(points)};
    }

    void writePlyAscii(const std::filesystem::path &path,
                       const PointCloud &cloud)
    {
        const auto colored_count =
            static_cast<std::size_t>(std::count_if(
                cloud.points().begin(), cloud.points().end(),
                [](const Point &point)
                { return point.color.has_value(); }));
        if (colored_count != 0 && colored_count != cloud.size())
        {
            fail(path, "cannot write a cloud with partially missing colors");
        }
        const bool write_color = !cloud.empty() && colored_count == cloud.size();

        for (const Point &point : cloud.points())
        {
            if (!point.position.allFinite())
            {
                fail(path, "cannot write a point with non-finite coordinates");
            }
        }

        std::ofstream output(path, std::ios::trunc);
        if (!output)
        {
            fail(path, "cannot open file for writing");
        }
        output.imbue(std::locale::classic());

        output << "ply\n"
               << "format ascii 1.0\n"
               << "comment generated by PointCloud Processing Toolkit\n"
               << "element vertex " << cloud.size() << '\n'
               << "property float x\n"
               << "property float y\n"
               << "property float z\n";
        if (write_color)
        {
            output << "property uchar red\n"
                   << "property uchar green\n"
                   << "property uchar blue\n";
        }
        output << "end_header\n";

        output << std::setprecision(std::numeric_limits<float>::max_digits10);
        for (const Point &point : cloud.points())
        {
            output << point.position.x() << ' ' << point.position.y() << ' '
                   << point.position.z();
            if (write_color)
            {
                const auto &color = *point.color;
                output << ' ' << static_cast<int>(color.red) << ' '
                       << static_cast<int>(color.green) << ' '
                       << static_cast<int>(color.blue);
            }
            output << '\n';
        }

        if (!output)
        {
            fail(path, "failed while writing file");
        }
    }

} // namespace pct::io
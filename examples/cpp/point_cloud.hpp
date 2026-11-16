#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "ros2/point_cloud2.pb.h"

namespace x2robot::examples::point_cloud {
using PointCloud2 = ::sensor_msgs::PointCloud2;

struct Point {
    float x{}, y{}, z{};
};

struct FieldSpec {
    int offset{-1};
    int datatype{0};
};

// sensor_msgs/PointField datatype values follow the ROS message contract:
// 1..8 = signed/unsigned integers, FLOAT32, FLOAT64.  Decode the declared
// type instead of assuming every cloud stores coordinates as float32.
inline bool ReadNumber(const std::vector<std::uint8_t>& bytes, std::size_t offset, int datatype, bool big_endian,
                       float* out) {
    const std::size_t width = datatype == 1 || datatype == 2                    ? 1
                              : datatype == 3 || datatype == 4                  ? 2
                              : datatype == 5 || datatype == 6 || datatype == 7 ? 4
                              : datatype == 8                                   ? 8
                                                                                : 0;
    if (width == 0 || offset + width > bytes.size()) return false;
    std::uint8_t raw[8]{};
    for (std::size_t i = 0; i < width; ++i) raw[i] = bytes[offset + (big_endian ? width - 1 - i : i)];
    switch (datatype) {
        case 1: {
            std::int8_t v;
            std::memcpy(&v, raw, 1);
            *out = static_cast<float>(v);
            return true;
        }
        case 2: {
            std::uint8_t v;
            std::memcpy(&v, raw, 1);
            *out = static_cast<float>(v);
            return true;
        }
        case 3: {
            std::int16_t v;
            std::memcpy(&v, raw, 2);
            *out = static_cast<float>(v);
            return true;
        }
        case 4: {
            std::uint16_t v;
            std::memcpy(&v, raw, 2);
            *out = static_cast<float>(v);
            return true;
        }
        case 5: {
            std::int32_t v;
            std::memcpy(&v, raw, 4);
            *out = static_cast<float>(v);
            return true;
        }
        case 6: {
            std::uint32_t v;
            std::memcpy(&v, raw, 4);
            *out = static_cast<float>(v);
            return true;
        }
        case 7: {
            float v;
            std::memcpy(&v, raw, 4);
            *out = v;
            return true;
        }
        case 8: {
            double v;
            std::memcpy(&v, raw, 8);
            *out = static_cast<float>(v);
            return true;
        }
        default:
            return false;
    }
}

inline std::vector<std::uint8_t> Bytes(const PointCloud2& cloud) {
    std::vector<std::uint8_t> bytes;
    bytes.reserve(cloud.data_size());
    for (const auto value : cloud.data()) bytes.push_back(static_cast<std::uint8_t>(value));
    return bytes;
}

inline std::vector<Point> Parse(const PointCloud2& cloud) {
    const auto field_width = [](int type) -> std::size_t {
        return type == 1 || type == 2 ? 1 : type == 3 || type == 4 ? 2 : type >= 5 && type <= 7 ? 4 : type == 8 ? 8 : 0;
    };
    const std::uint64_t row_bytes = std::uint64_t(cloud.width()) * cloud.point_step();
    std::uint64_t row_step = cloud.row_step();
    // Some EX001 depth producers compact the cloud into one row but retain the
    // original image stride (SX001-162). Only accept this unambiguous layout:
    // exactly width * point_step bytes, with no missing points or row padding.
    if (cloud.height() == 1 && row_bytes == std::uint64_t(cloud.data_size()) && row_step != row_bytes) {
        std::cerr << "PointCloud2 warning: single-row packed data uses " << row_bytes
                  << " bytes; ignoring stale row_step=" << row_step << '\n';
        row_step = row_bytes;
    }
    const std::uint64_t total_bytes = row_step * cloud.height();
    if (!cloud.width() || !cloud.height() || !cloud.point_step() || row_bytes > row_step ||
        total_bytes != std::uint64_t(cloud.data_size())) {
        std::cerr << "PointCloud2 parse failed: invalid dimensions or byte count\n";
        return {};
    }
    FieldSpec x, y, z;
    for (const auto& field : cloud.fields()) {
        FieldSpec* dst = field.name() == "x" ? &x : field.name() == "y" ? &y : field.name() == "z" ? &z : nullptr;
        if (!dst) continue;
        const auto width = field_width(field.datatype());
        if (dst->offset >= 0 || field.count() != 1 || !width ||
            std::uint64_t(field.offset()) + width > cloud.point_step() ||
            field.offset() > std::uint32_t(std::numeric_limits<int>::max())) {
            std::cerr << "PointCloud2 parse failed: invalid coordinate field\n";
            return {};
        }
        dst->offset = static_cast<int>(field.offset());
        dst->datatype = field.datatype();
    }
    if (x.offset < 0 || y.offset < 0 || z.offset < 0 ||
        std::any_of(cloud.data().begin(), cloud.data().end(), [](auto byte) { return byte > 255; })) {
        std::cerr << "PointCloud2 parse failed: missing coordinates or non-byte payload\n";
        return {};
    }
    const auto bytes = Bytes(cloud);
    std::vector<Point> points;
    points.reserve(static_cast<std::size_t>(cloud.width()) * cloud.height());
    for (std::size_t row = 0; row < cloud.height(); ++row) {
        for (std::size_t column = 0; column < cloud.width(); ++column) {
            const auto start = row * row_step + column * cloud.point_step();
            Point p;
            if (!ReadNumber(bytes, start + x.offset, x.datatype, cloud.is_bigendian(), &p.x) ||
                !ReadNumber(bytes, start + y.offset, y.datatype, cloud.is_bigendian(), &p.y) ||
                !ReadNumber(bytes, start + z.offset, z.datatype, cloud.is_bigendian(), &p.z))
                return {};
            points.push_back(p);
        }
    }
    return points;
}

inline bool Display(const PointCloud2& cloud) {
    const auto points = Parse(cloud);
    if (points.empty()) return false;
    std::cout << "\n Depth Point Cloud Data:\n"
              << std::string(80, '=') << '\n'
              << "  - Dimensions: " << cloud.width() << 'x' << cloud.height() << '\n'
              << "  - Total Points: " << points.size() << '\n'
              << "  - Is Dense: " << (cloud.is_dense() ? "Yes" : "No") << '\n'
              << "  - Point Step: " << cloud.point_step() << " bytes\n"
              << "  - Row Step: " << cloud.row_step() << " bytes\n"
              << "  - Is Big Endian: " << (cloud.is_bigendian() ? "Yes" : "No") << '\n';
    const std::size_t shown = std::min<std::size_t>(10, points.size());
    if (shown) {
        std::cout << "\n Displaying first " << shown << " points (x, y, z):\n";
        for (std::size_t i = 0; i < shown; ++i)
            std::cout << "  - Point " << i + 1 << ": (" << points[i].x << ", " << points[i].y << ", " << points[i].z
                      << ")\n";
    }
    return true;
}

}  // namespace x2robot::examples::point_cloud

#pragma once

#include <cstdint>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace radar_lidar::odin_tune {

/// @brief Odin1 cloud_raw 自定义点格式（x,y,z,intensity,confidence,offset_time）
struct OdinPoint {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    std::uint8_t intensity = 0;
    std::uint16_t confidence = 0;
    float offset_time = 0.0f;
};

} // namespace radar_lidar::odin_tune

POINT_CLOUD_REGISTER_POINT_STRUCT(radar_lidar::odin_tune::OdinPoint,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (std::uint8_t, intensity, intensity)
    (std::uint16_t, confidence, confidence)
    (float, offset_time, offset_time))

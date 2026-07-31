#pragma once

#include <cstdint>
#include <expected>
#include <optional>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace radar_lidar::grid_map {

struct GridMapParams {
    double resolution       = 0.05; // 每格边长 (m)
    double height_threshold = 0.3;  // 格内 z_max - z_min 超过此值判障碍
    int min_points          = 3;    // 判定障碍所需最少点数
    int dilate              = 0;    // 障碍膨胀半径 (格)
};

struct Bounds {
    double x_min = 0.0, y_min = 0.0, x_max = 0.0, y_max = 0.0;
};

struct GridMapResult {
    int width = 0, height = 0;
    double origin_x = 0.0, origin_y = 0.0; // 世界系左下角
    double resolution = 0.05;
    // row-major, data[iy * width + ix], iy=0 对应世界系 y_min
    // 0 = 障碍, 100 = 空闲, -1 = 未知
    std::vector<int8_t> data;
};

// 将稠密全局点云栅格化为 2D 占用网格。
// bounds 为空时使用点云 bbox (扩到分辨率整数倍); 点云为空且无 bounds 时报错。
auto rasterize(const pcl::PointCloud<pcl::PointXYZ>& cloud, const GridMapParams& params,
    const std::optional<Bounds>& bounds) -> std::expected<GridMapResult, std::string>;

} // namespace radar_lidar::grid_map

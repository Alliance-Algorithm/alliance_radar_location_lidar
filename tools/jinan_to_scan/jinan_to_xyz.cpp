// jinan_to_xyz — 把济南真实场馆多字段 binary_compressed PCD 转成简单 xyz binary PCD
//
// pcd_rmuc2026_jinan.pcd 由第三方测绘设备采集，字段:
//   Coord._Z Original_cloud_index intensity rgb x y z (binary_compressed)
// make_synth_scan.py 只认简单 "FIELDS x y z / DATA binary" 格式，需先转换。
//
// 用法:
//   jinan_to_xyz <input.pcd> <output.pcd> [--roi x_min,x_max,y_min,y_max,z_min,z_max]
//
// 默认不裁剪 (保留场地内所有真实点，包括人/机器人)；--roi 可选裁剪掉明显场外的远处结构，
// 减小体积、加速后续 make_synth_scan.py 的 FOV/z-buffer 计算。

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace {

struct Roi {
    float x_min, x_max, y_min, y_max, z_min, z_max;
};

auto parse_roi(std::string_view s) -> std::optional<Roi> {
    std::vector<float> vals;
    std::stringstream ss { std::string(s) };
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        try {
            vals.push_back(std::stof(tok));
        } catch (const std::exception&) {
            return std::nullopt;
        }
    }
    if (vals.size() != 6) return std::nullopt;
    return Roi { vals[0], vals[1], vals[2], vals[3], vals[4], vals[5] };
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0]
                  << " <input.pcd> <output.pcd> [--roi x_min,x_max,y_min,y_max,z_min,z_max]\n";
        return 1;
    }

    const std::string input_path  = argv[1];
    const std::string output_path = argv[2];
    std::optional<Roi> roi;

    for (int i = 3; i < argc; ++i) {
        std::string_view arg = argv[i];
        if (arg == "--roi" && i + 1 < argc) {
            roi = parse_roi(argv[++i]);
            if (!roi) {
                std::cerr << "ERROR: --roi requires 6 comma-separated floats\n";
                return 1;
            }
        }
    }

    std::cout << "[jinan_to_xyz] Loading " << input_path << " ...\n";
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_path, cloud) == -1) {
        std::cerr << "ERROR: failed to load " << input_path << "\n";
        return 1;
    }
    std::cout << "[jinan_to_xyz] Loaded " << cloud.size() << " points\n";

    pcl::PointCloud<pcl::PointXYZ> filtered;
    filtered.points.reserve(cloud.size());
    for (const auto& p : cloud.points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        if (roi) {
            if (p.x < roi->x_min || p.x > roi->x_max) continue;
            if (p.y < roi->y_min || p.y > roi->y_max) continue;
            if (p.z < roi->z_min || p.z > roi->z_max) continue;
        }
        filtered.points.push_back(p);
    }
    filtered.width    = filtered.points.size();
    filtered.height   = 1;
    filtered.is_dense = true;

    if (pcl::io::savePCDFileBinary(output_path, filtered) != 0) {
        std::cerr << "ERROR: failed to write " << output_path << "\n";
        return 1;
    }
    std::cout << "[jinan_to_xyz] Wrote " << filtered.size() << " points -> " << output_path
              << (roi ? " (ROI filtered)" : " (no filter)") << "\n";
    return 0;
}

// extract_by_index — 按 Original_cloud_index 字段提取 jinan PCD 中的指定子集
//
// pcd_rmuc2026_jinan.pcd 由多个源点云合并而成 (字段 Original_cloud_index 标记来源):
//   index=0  屋顶/悬挂结构高处点 (z 13~44m)
//   index=1  全场馆背景环境扫描 (±61m)
//   index=2,3  蓝色场地3D模型 (含围墙, 与工作系场地边界几乎重合: x±14.1 y±7.6 z-0.2~3.48)
//   index=4  场地内部真实测量点云 (白色, 含比赛现场真实目标)
//
// 用法:
//   extract_by_index <input.pcd> <output.pcd> --index <int> [--index <int> ...]
//
// 输出简单 binary xyz PCD (PointXYZ)，可直接被 MapData::load / make_synth_scan.py /
// offline_detection_node 等现有工具链读取。

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <cmath>
#include <cstring>
#include <iostream>
#include <set>
#include <string>
#include <vector>

int main(int argc, char** argv) {
    if (argc < 5) {
        std::cerr << "Usage: " << argv[0]
                  << " <input.pcd> <output.pcd> --index <int> [--index <int> ...]\n";
        return 1;
    }

    const std::string input_path  = argv[1];
    const std::string output_path = argv[2];
    std::set<int> wanted_indices;
    for (int i = 3; i < argc; ++i) {
        if (std::string(argv[i]) == "--index" && i + 1 < argc) {
            ++i;
            try {
                wanted_indices.insert(std::stoi(argv[i]));
            } catch (const std::exception&) {
                std::cerr << "ERROR: invalid index value: " << argv[i] << "\n";
                return 1;
            }
        }
    }
    if (wanted_indices.empty()) {
        std::cerr << "ERROR: no --index given\n";
        return 1;
    }

    std::cout << "[extract_by_index] Loading " << input_path << " ...\n";
    pcl::PCLPointCloud2 blob;
    pcl::PCDReader reader;
    if (reader.read(input_path, blob) < 0) {
        std::cerr << "ERROR: failed to read " << input_path << "\n";
        return 1;
    }

    int off_x = -1, off_y = -1, off_z = -1, off_idx = -1;
    for (const auto& f : blob.fields) {
        if (f.name == "x") off_x = static_cast<int>(f.offset);
        else if (f.name == "y") off_y = static_cast<int>(f.offset);
        else if (f.name == "z") off_z = static_cast<int>(f.offset);
        else if (f.name == "Original_cloud_index") off_idx = static_cast<int>(f.offset);
    }
    if (off_x < 0 || off_y < 0 || off_z < 0 || off_idx < 0) {
        std::cerr << "ERROR: input PCD missing x/y/z/Original_cloud_index fields\n";
        return 1;
    }

    const std::size_t npts = static_cast<std::size_t>(blob.width) * blob.height;
    const std::size_t step = blob.point_step;
    const std::uint8_t* data = blob.data.data();

    pcl::PointCloud<pcl::PointXYZ> out;
    out.points.reserve(npts);
    for (std::size_t i = 0; i < npts; ++i) {
        const std::uint8_t* p = data + i * step;
        float x, y, z, idx_f;
        std::memcpy(&x, p + off_x, 4);
        std::memcpy(&y, p + off_y, 4);
        std::memcpy(&z, p + off_z, 4);
        std::memcpy(&idx_f, p + off_idx, 4);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

        const int idx = static_cast<int>(std::lround(idx_f));
        if (wanted_indices.count(idx) == 0) continue;
        out.points.emplace_back(x, y, z);
    }
    out.width    = out.points.size();
    out.height   = 1;
    out.is_dense = true;

    if (pcl::io::savePCDFileBinary(output_path, out) != 0) {
        std::cerr << "ERROR: failed to write " << output_path << "\n";
        return 1;
    }
    std::cout << "[extract_by_index] Wrote " << out.size() << " points -> " << output_path << "\n";
    return 0;
}

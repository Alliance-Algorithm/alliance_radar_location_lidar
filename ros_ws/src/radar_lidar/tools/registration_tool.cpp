#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <small_gicp/registration/registration_helper.hpp>

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "radar_lidar/localization.hpp"
#include "radar_lidar/map_data.hpp"
#include "radar_lidar/types.hpp"

namespace {

struct Args {
    std::string map_path;
    std::string scan_path;
    double voxel_leaf      = 0.1;
    double max_corr        = 1.0;
    int max_iter           = 48;
    int num_threads        = 4;
    double init_x          = 0.0;
    double init_y          = 0.0;
    double init_z          = 0.0;
    double init_yaw_deg    = 0.0;
    std::string output_pcd = "aligned.pcd";
    std::string pose_out   = "pose.json";
    bool verbose           = false;
};

auto parse_args(int argc, char** argv) -> Args {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <map.pcd> <scan.pcd> [options]\n"
                  << "Options:\n"
                  << "  --voxel <float>       map voxel size (default 0.1)\n"
                  << "  --max-corr <float>    GICP max correspondence distance (default 1.0)\n"
                  << "  --max-iter <int>      GICP max iterations (default 48)\n"
                  << "  --num-threads <int>   parallel threads (default 4)\n"
                  << "  --init-x/y/z <float>  initial pose translation (default 0)\n"
                  << "  --init-yaw <float>    initial pose yaw in degrees (default 0)\n"
                  << "  --output <path>       aligned PCD output (default aligned.pcd)\n"
                  << "  --pose-out <path>     pose JSON output (default pose.json)\n"
                  << "  --verbose             verbose GICP output\n";
        std::exit(1);
    }

    Args args;
    args.map_path  = argv[1];
    args.scan_path = argv[2];

    for (int i = 3; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--voxel" && i + 1 < argc) args.voxel_leaf = std::stod(argv[++i]);
        else if (arg == "--max-corr" && i + 1 < argc) args.max_corr = std::stod(argv[++i]);
        else if (arg == "--max-iter" && i + 1 < argc) args.max_iter = std::stoi(argv[++i]);
        else if (arg == "--num-threads" && i + 1 < argc) args.num_threads = std::stoi(argv[++i]);
        else if (arg == "--init-x" && i + 1 < argc) args.init_x = std::stod(argv[++i]);
        else if (arg == "--init-y" && i + 1 < argc) args.init_y = std::stod(argv[++i]);
        else if (arg == "--init-z" && i + 1 < argc) args.init_z = std::stod(argv[++i]);
        else if (arg == "--init-yaw" && i + 1 < argc) args.init_yaw_deg = std::stod(argv[++i]);
        else if (arg == "--output" && i + 1 < argc) args.output_pcd = argv[++i];
        else if (arg == "--pose-out" && i + 1 < argc) args.pose_out = argv[++i];
        else if (arg == "--verbose") args.verbose = true;
        else {
            std::cerr << "Unknown argument: " << arg << "\n";
            std::exit(1);
        }
    }
    return args;
}

void write_pose_json(const std::string& path, const radar::types::PoseEstimate& pose) {
    const auto& T    = pose.T;
    const auto trans = T.translation();
    const Eigen::Quaterniond q(T.rotation());

    std::ofstream f(path);
    f << std::fixed << std::setprecision(8);
    f << "{\n";
    f << "  \"converged\": " << (pose.converged ? "true" : "false") << ",\n";
    f << "  \"fitness_score\": " << pose.fitness_score << ",\n";
    f << "  \"translation\": [" << trans.x() << ", " << trans.y() << ", " << trans.z() << "],\n";
    f << "  \"rotation_quaternion\": [" << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w()
      << "],\n";
    f << "  \"rotation_euler_xyz_deg\": ["
      << q.toRotationMatrix().eulerAngles(0, 1, 2).x() * 180.0 / M_PI << ", "
      << q.toRotationMatrix().eulerAngles(0, 1, 2).y() * 180.0 / M_PI << ", "
      << q.toRotationMatrix().eulerAngles(0, 1, 2).z() * 180.0 / M_PI << "],\n";
    f << "  \"covariance\": [\n";
    for (int i = 0; i < 6; ++i) {
        f << "    [";
        for (int j = 0; j < 6; ++j) {
            f << pose.covariance(i, j);
            if (j < 5) f << ", ";
        }
        f << "]";
        if (i < 5) f << ",";
        f << "\n";
    }
    f << "  ]\n";
    f << "}\n";
}

} // namespace

int main(int argc, char** argv) {
    const auto args = parse_args(argc, argv);

    // ── 1. 加载地图 ──────────────────────────────────────────────
    std::cout << "[registration_tool] Loading map: " << args.map_path << std::endl;
    auto map_result = radar::MapData::Load(args.map_path, args.voxel_leaf);
    if (!map_result) {
        std::cerr << "[registration_tool] ERROR: " << map_result.error() << std::endl;
        return 1;
    }
    auto map = *map_result;
    std::cout << "[registration_tool] Map loaded: " << map->size() << " points" << std::endl;

    // ── 2. 加载扫描 ──────────────────────────────────────────────
    std::cout << "[registration_tool] Loading scan: " << args.scan_path << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_pcl(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(args.scan_path, *scan_pcl) == -1) {
        std::cerr << "[registration_tool] ERROR: Failed to load scan PCD" << std::endl;
        return 1;
    }

    // 过滤 NaN/Inf 和零点（和 pipeline.cpp 一致）
    radar::types::Frame frame;
    frame.points.reserve(scan_pcl->size());
    for (const auto& pt : scan_pcl->points) {
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)
            && (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z) > 1e-12) {
            frame.points.emplace_back(pt.x, pt.y, pt.z);
        }
    }
    std::cout << "[registration_tool] Scan loaded: " << frame.points.size() << " points"
              << std::endl;

    if (frame.points.size() < 100) {
        std::cerr << "[registration_tool] ERROR: Too few scan points: " << frame.points.size()
                  << std::endl;
        return 1;
    }

    // ── 3. 配准 ──────────────────────────────────────────────────
    radar::config::LocalizationConfig cfg;
    cfg.voxel_leaf_size   = args.voxel_leaf;
    cfg.max_corr_distance = args.max_corr;
    cfg.max_iterations    = args.max_iter;
    cfg.num_threads       = args.num_threads;
    cfg.verbose           = args.verbose;

    // 如果给了初始位姿，通过 set_initial_pose 传入
    auto localization = radar::LocalizationStage(map, cfg);

    if (args.init_x != 0.0 || args.init_y != 0.0 || args.init_z != 0.0
        || args.init_yaw_deg != 0.0) {
        Eigen::Isometry3d init_pose = Eigen::Isometry3d::Identity();
        init_pose.translation()     = Eigen::Vector3d(args.init_x, args.init_y, args.init_z);
        double yaw_rad              = args.init_yaw_deg * M_PI / 180.0;
        init_pose.linear() =
            (Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ())).toRotationMatrix();
        localization.set_initial_pose(init_pose);
        std::cout << "[registration_tool] Initial pose set: x=" << args.init_x
                  << " y=" << args.init_y << " z=" << args.init_z << " yaw=" << args.init_yaw_deg
                  << "deg" << std::endl;
    }

    std::cout << "[registration_tool] Running GICP..." << std::endl;
    auto result = localization.process(frame);

    if (!result) {
        std::cerr << "[registration_tool] GICP FAILED: " << result.error() << std::endl;
        return 2;
    }

    const auto& pose = *result;
    const auto& T    = pose.T;
    const auto trans = T.translation();
    const Eigen::Quaterniond q(T.rotation());

    std::cout << "[registration_tool] === Result ===" << std::endl;
    std::cout << "  converged:      " << (pose.converged ? "true" : "false") << std::endl;
    std::cout << "  fitness_score:  " << pose.fitness_score << std::endl;
    std::cout << "  translation:    [" << trans.x() << ", " << trans.y() << ", " << trans.z() << "]"
              << std::endl;
    std::cout << "  quaternion:     [" << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w()
              << "]" << std::endl;

    // ── 4. 写位姿 JSON ───────────────────────────────────────────
    write_pose_json(args.pose_out, pose);
    std::cout << "[registration_tool] Pose written: " << args.pose_out << std::endl;

    // ── 5. 变换扫描并写叠加 PCD ──────────────────────────────────
    // 用配准结果变换扫描点云，和地图合并成一个 PCD
    pcl::PointCloud<pcl::PointXYZI>::Ptr merged(new pcl::PointCloud<pcl::PointXYZI>());

    // 地图点 intensity=0
    for (const auto& pt : map->pcl_cloud().points) {
        pcl::PointXYZI p;
        p.x         = pt.x;
        p.y         = pt.y;
        p.z         = pt.z;
        p.intensity = 0.0f;
        merged->push_back(p);
    }

    // 变换后的扫描点 intensity=1
    for (const auto& pt : frame.points) {
        Eigen::Vector3d transformed = T * pt;
        pcl::PointXYZI p;
        p.x         = transformed.x();
        p.y         = transformed.y();
        p.z         = transformed.z();
        p.intensity = 1.0f;
        merged->push_back(p);
    }

    merged->width    = merged->size();
    merged->height   = 1;
    merged->is_dense = true;

    if (pcl::io::savePCDFileBinary(args.output_pcd, *merged) != 0) {
        std::cerr << "[registration_tool] ERROR: Failed to write aligned PCD" << std::endl;
        return 1;
    }

    std::cout << "[registration_tool] Aligned PCD written: " << args.output_pcd << " ("
              << merged->size() << " points, intensity 0=map 1=scan)" << std::endl;

    return 0;
}

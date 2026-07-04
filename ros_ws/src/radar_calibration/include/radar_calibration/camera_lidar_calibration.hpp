#pragma once

#include <Eigen/Geometry>
#include <expected>
#include <filesystem>
#include <string>

namespace radar::calibration {

// Reads the `results.T_lidar_camera` field from a direct_visual_lidar_calibration
// `calib.json` (camera-map registration mode: the "lidar" frame is the map frame).
auto load_t_map_camera(const std::filesystem::path& calib_json_path)
    -> std::expected<Eigen::Isometry3d, std::string>;

// Writes t_map_camera as translation + quaternion into a YAML file for radar_camera to load.
auto write_extrinsic_yaml(const std::filesystem::path& yaml_path,
    const Eigen::Isometry3d& t_map_camera) -> std::expected<bool, std::string>;

// Reads a rough extrinsic guess (initial_tx/ty/tz + initial_roll/pitch/yaw, same rotation
// convention as radar_lidar's initial_pose: Rz(yaw) * Ry(pitch) * Rx(roll)) from a YAML file
// and writes it into `calib.json`'s `results.init_T_lidar_camera` field, so `calibrate` can
// run fully unattended (no SuperGlue, no manual point picking).
auto inject_initial_guess(const std::filesystem::path& calib_json_path,
    const std::filesystem::path& initial_guess_yaml_path) -> std::expected<bool, std::string>;

} // namespace radar::calibration

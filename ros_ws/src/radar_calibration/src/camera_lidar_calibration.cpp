#include "radar_calibration/camera_lidar_calibration.hpp"

#include <fstream>

#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

namespace radar::calibration {

auto load_t_map_camera(const std::filesystem::path& calib_json_path)
    -> std::expected<Eigen::Isometry3d, std::string> {
    std::error_code ec;
    if (!std::filesystem::exists(calib_json_path, ec)) {
        return std::unexpected("calib.json not found: " + calib_json_path.string());
    }

    std::ifstream in(calib_json_path);
    nlohmann::json calib;
    try {
        in >> calib;
    } catch (const std::exception& e) {
        return std::unexpected("Failed to parse calib.json: " + std::string(e.what()));
    }

    if (!calib.contains("results") || !calib["results"].contains("T_lidar_camera")) {
        return std::unexpected("calib.json missing results.T_lidar_camera");
    }

    const auto t = calib["results"]["T_lidar_camera"].get<std::vector<double>>();
    if (t.size() != 7) {
        return std::unexpected("results.T_lidar_camera must have 7 elements [x,y,z,qx,qy,qz,qw]");
    }

    Eigen::Isometry3d t_map_camera = Eigen::Isometry3d::Identity();
    t_map_camera.translation()     = Eigen::Vector3d(t[0], t[1], t[2]);
    t_map_camera.linear()          = Eigen::Quaterniond(t[6], t[3], t[4], t[5]).toRotationMatrix();
    return t_map_camera;
}

auto write_extrinsic_yaml(const std::filesystem::path& yaml_path,
    const Eigen::Isometry3d& t_map_camera) -> std::expected<bool, std::string> {
    const Eigen::Vector3d translation = t_map_camera.translation();
    const Eigen::Quaterniond orientation(t_map_camera.rotation());

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "t_map_camera" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "translation" << YAML::Value << YAML::Flow
        << std::vector { translation.x(), translation.y(), translation.z() };
    out << YAML::Key << "quaternion" << YAML::Value << YAML::Flow
        << std::vector { orientation.x(), orientation.y(), orientation.z(), orientation.w() };
    out << YAML::EndMap;
    out << YAML::EndMap;

    std::ofstream file(yaml_path);
    if (!file) {
        return std::unexpected("Failed to open output YAML: " + yaml_path.string());
    }
    file << out.c_str() << '\n';
    return true;
}

auto inject_initial_guess(const std::filesystem::path& calib_json_path,
    const std::filesystem::path& initial_guess_yaml_path) -> std::expected<bool, std::string> {
    std::error_code ec;
    if (!std::filesystem::exists(calib_json_path, ec)) {
        return std::unexpected("calib.json not found: " + calib_json_path.string());
    }
    if (!std::filesystem::exists(initial_guess_yaml_path, ec)) {
        return std::unexpected("initial guess YAML not found: " + initial_guess_yaml_path.string());
    }

    YAML::Node guess;
    try {
        guess = YAML::LoadFile(initial_guess_yaml_path.string());
    } catch (const std::exception& e) {
        return std::unexpected("Failed to parse initial guess YAML: " + std::string(e.what()));
    }

    const double tx    = guess["initial_tx"].as<double>(0.0);
    const double ty    = guess["initial_ty"].as<double>(0.0);
    const double tz    = guess["initial_tz"].as<double>(0.0);
    const double roll  = guess["initial_roll"].as<double>(0.0);
    const double pitch = guess["initial_pitch"].as<double>(0.0);
    const double yaw   = guess["initial_yaw"].as<double>(0.0);

    Eigen::Isometry3d t_map_camera = Eigen::Isometry3d::Identity();
    t_map_camera.translation()     = Eigen::Vector3d(tx, ty, tz);
    t_map_camera.linear()          = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
                                         .toRotationMatrix();

    nlohmann::json calib;
    {
        std::ifstream in(calib_json_path);
        try {
            in >> calib;
        } catch (const std::exception& e) {
            return std::unexpected("Failed to parse calib.json: " + std::string(e.what()));
        }
    }

    const Eigen::Vector3d translation = t_map_camera.translation();
    const Eigen::Quaterniond orientation(t_map_camera.rotation());
    calib["results"]["init_T_lidar_camera"] = { translation.x(), translation.y(), translation.z(),
        orientation.x(), orientation.y(), orientation.z(), orientation.w() };

    std::ofstream out(calib_json_path);
    if (!out) {
        return std::unexpected(
            "Failed to open calib.json for writing: " + calib_json_path.string());
    }
    out << calib.dump(2) << '\n';
    return true;
}

} // namespace radar::calibration

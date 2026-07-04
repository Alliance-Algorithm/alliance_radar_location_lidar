#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "radar_calibration/camera_lidar_calibration.hpp"

namespace {

class CameraLidarCalibrationTest : public ::testing::Test {
protected:
    void TearDown() override {
        std::filesystem::remove(calib_json_path_);
        std::filesystem::remove(yaml_path_);
        std::filesystem::remove(initial_guess_yaml_path_);
    }

    static constexpr const char* calib_json_path_          = "/tmp/radar_test_calib.json";
    static constexpr const char* yaml_path_                = "/tmp/radar_test_extrinsic.yaml";
    static constexpr const char* initial_guess_yaml_path_   = "/tmp/radar_test_initial_guess.yaml";
};

auto write_calib_json(const std::string& path, const std::string& results_body) -> void {
    std::ofstream file(path);
    file << "{\"results\":{" << results_body << "}}";
}

} // namespace

TEST_F(CameraLidarCalibrationTest, LoadValidTransform) {
    write_calib_json(calib_json_path_, R"("T_lidar_camera":[1.0,2.0,3.0,0.0,0.0,0.0,1.0])");

    auto result = radar::calibration::load_t_map_camera(calib_json_path_);
    ASSERT_TRUE(result.has_value()) << result.error();
    EXPECT_DOUBLE_EQ(result->translation().x(), 1.0);
    EXPECT_DOUBLE_EQ(result->translation().y(), 2.0);
    EXPECT_DOUBLE_EQ(result->translation().z(), 3.0);
    EXPECT_TRUE(result->rotation().isApprox(Eigen::Matrix3d::Identity()));
}

TEST_F(CameraLidarCalibrationTest, LoadMissingFile) {
    auto result = radar::calibration::load_t_map_camera("/tmp/radar_test_calib_missing.json");
    EXPECT_FALSE(result.has_value());
    EXPECT_NE(result.error().find("not found"), std::string::npos);
}

TEST_F(CameraLidarCalibrationTest, LoadMissingResultsField) {
    std::ofstream file(calib_json_path_);
    file << "{}";
    file.close();

    auto result = radar::calibration::load_t_map_camera(calib_json_path_);
    EXPECT_FALSE(result.has_value());
    EXPECT_NE(result.error().find("results.T_lidar_camera"), std::string::npos);
}

TEST_F(CameraLidarCalibrationTest, LoadWrongElementCount) {
    write_calib_json(calib_json_path_, R"("T_lidar_camera":[1.0,2.0,3.0])");

    auto result = radar::calibration::load_t_map_camera(calib_json_path_);
    EXPECT_FALSE(result.has_value());
    EXPECT_NE(result.error().find("7 elements"), std::string::npos);
}

TEST_F(CameraLidarCalibrationTest, WriteAndRoundTrip) {
    Eigen::Isometry3d t_map_camera = Eigen::Isometry3d::Identity();
    t_map_camera.translation()     = Eigen::Vector3d(0.5, -0.5, 4.0);
    t_map_camera.linear() = Eigen::Quaterniond(0.98, 0.1, 0.1, 0.1).normalized().toRotationMatrix();

    auto write_result = radar::calibration::write_extrinsic_yaml(yaml_path_, t_map_camera);
    ASSERT_TRUE(write_result.has_value()) << write_result.error();
    EXPECT_TRUE(std::filesystem::exists(yaml_path_));

    std::ifstream file(yaml_path_);
    std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    EXPECT_NE(content.find("t_map_camera"), std::string::npos);
    EXPECT_NE(content.find("translation"), std::string::npos);
    EXPECT_NE(content.find("quaternion"), std::string::npos);
}

TEST_F(CameraLidarCalibrationTest, InjectInitialGuess) {
    write_calib_json(calib_json_path_, R"("T_lidar_camera":[0.0,0.0,0.0,0.0,0.0,0.0,1.0])");

    {
        std::ofstream file(initial_guess_yaml_path_);
        file << "initial_tx: 1.5\n"
             << "initial_ty: 2.5\n"
             << "initial_tz: 3.5\n"
             << "initial_roll: 0.0\n"
             << "initial_pitch: 0.0\n"
             << "initial_yaw: 1.5707963267948966\n"; // pi/2
    }

    auto result = radar::calibration::inject_initial_guess(calib_json_path_, initial_guess_yaml_path_);
    ASSERT_TRUE(result.has_value()) << result.error();

    std::ifstream file(calib_json_path_);
    nlohmann::json calib;
    file >> calib;

    ASSERT_TRUE(calib["results"].contains("init_T_lidar_camera"));
    const auto init = calib["results"]["init_T_lidar_camera"].get<std::vector<double>>();
    ASSERT_EQ(init.size(), 7u);
    EXPECT_DOUBLE_EQ(init[0], 1.5);
    EXPECT_DOUBLE_EQ(init[1], 2.5);
    EXPECT_DOUBLE_EQ(init[2], 3.5);
    // yaw = pi/2 around Z: quaternion should be approximately (0, 0, sin(pi/4), cos(pi/4))
    EXPECT_NEAR(init[5], std::sin(M_PI / 4), 1e-9);
    EXPECT_NEAR(init[6], std::cos(M_PI / 4), 1e-9);
}

TEST_F(CameraLidarCalibrationTest, InjectInitialGuessMissingCalibJson) {
    {
        std::ofstream file(initial_guess_yaml_path_);
        file << "initial_tx: 0.0\n";
    }

    auto result = radar::calibration::inject_initial_guess(
        "/tmp/radar_test_calib_missing.json", initial_guess_yaml_path_);
    EXPECT_FALSE(result.has_value());
    EXPECT_NE(result.error().find("not found"), std::string::npos);
}

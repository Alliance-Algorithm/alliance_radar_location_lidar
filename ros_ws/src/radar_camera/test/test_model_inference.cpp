#include <cstdlib>
#include <filesystem>
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

#include "radar_camera/model_inference.hpp"

namespace {

auto make_config() -> radar_camera::inference_config::InferenceConfig {
    radar_camera::inference_config::InferenceConfig cfg;
    cfg.model_input_width       = 1280;
    cfg.model_input_height      = 1280;
    cfg.conf_threshold          = 0.6f;
    cfg.min_length_width_rate   = 0.8f;
    cfg.max_length_width_rate   = 1.5f;
    cfg.device_name             = "CPU";
    return cfg;
}

auto make_row(float x1, float y1, float x2, float y2, float conf, float cls)
    -> std::vector<float> {
    return { x1, y1, x2, y2, conf, cls };
}

auto resolve_model_path() -> std::string {
    if (const char* env = std::getenv("RADAR_CAMERA_TEST_MODEL"); env != nullptr && env[0] != '\0') {
        return env;
    }
    const auto pkg =
        std::filesystem::path(RADAR_CAMERA_TEST_SOURCE_DIR).parent_path() / "config"
        / "camera_inference_model.onnx";
    return pkg.string();
}

auto resolve_image_path() -> std::string {
    if (const char* env = std::getenv("RADAR_CAMERA_TEST_IMAGE"); env != nullptr && env[0] != '\0') {
        return env;
    }
    return (std::filesystem::path(RADAR_CAMERA_TEST_SOURCE_DIR) / "fixtures" / "real_scene.jpg")
        .string();
}

} // namespace

TEST(FilterDetectionsTest, DropsBelowConfidence) {
    auto cfg  = make_config();
    auto row  = make_row(100.f, 100.f, 200.f, 200.f, 0.5f, 1.f);
    auto dets = radar_camera::model_inference::filter_detections(row, 1, 6, 1280, 1280, cfg);
    ASSERT_TRUE(dets.has_value()) << dets.error();
    EXPECT_TRUE(dets->empty());
}

TEST(FilterDetectionsTest, KeepsValidBox) {
    auto cfg  = make_config();
    auto row  = make_row(100.f, 100.f, 200.f, 200.f, 0.9f, 3.f);
    auto dets = radar_camera::model_inference::filter_detections(row, 1, 6, 1280, 1280, cfg);
    ASSERT_TRUE(dets.has_value()) << dets.error();
    ASSERT_EQ(dets->size(), 1u);
    EXPECT_EQ(dets->front().id, 3);
    EXPECT_FLOAT_EQ(dets->front().confidence, 0.9f);
    EXPECT_NEAR(dets->front().center.x, 150.0, 1e-3);
    EXPECT_NEAR(dets->front().center.y, 150.0, 1e-3);
}

TEST(FilterDetectionsTest, DropsBadAspectRatio) {
    auto cfg = make_config();
    // 100x10 after scale -> ratio 10 > 1.5
    auto row  = make_row(0.f, 0.f, 100.f, 10.f, 0.95f, 1.f);
    auto dets = radar_camera::model_inference::filter_detections(row, 1, 6, 1280, 1280, cfg);
    ASSERT_TRUE(dets.has_value()) << dets.error();
    EXPECT_TRUE(dets->empty());
}

TEST(FilterDetectionsTest, DropsTinyBox) {
    auto cfg = make_config();
    auto row = make_row(0.f, 0.f, 0.5f, 0.5f, 0.99f, 1.f);
    auto dets =
        radar_camera::model_inference::filter_detections(row, 1, 6, 1280, 1280, cfg);
    ASSERT_TRUE(dets.has_value()) << dets.error();
    EXPECT_TRUE(dets->empty());
}

TEST(FilterDetectionsTest, ScalesCenterToSourceImage) {
    auto cfg = make_config();
    // model space box centered at (640, 640); source 640x480 => scale 0.5, 0.375
    auto row  = make_row(540.f, 540.f, 740.f, 740.f, 0.95f, 5.f);
    auto dets = radar_camera::model_inference::filter_detections(row, 1, 6, 640, 480, cfg);
    ASSERT_TRUE(dets.has_value()) << dets.error();
    ASSERT_EQ(dets->size(), 1u);
    EXPECT_NEAR(dets->front().center.x, 320.0, 1e-2);
    EXPECT_NEAR(dets->front().center.y, 240.0, 1e-2);
}

TEST(FilterDetectionsTest, RejectsShortBuffer) {
    auto cfg  = make_config();
    std::vector<float> short_buf { 1.f, 2.f, 3.f };
    auto dets = radar_camera::model_inference::filter_detections(short_buf, 1, 6, 1280, 1280, cfg);
    EXPECT_FALSE(dets.has_value());
}

TEST(ModelInferencePreprocessTest, BuildsNchwTensorInUnitRange) {
    radar_camera::model_inference::ModelInference engine;
    cv::Mat image(480, 640, CV_8UC3, cv::Scalar(0, 128, 255));
    auto tensor = engine.infer_preprocess(image, 320, 240);
    ASSERT_TRUE(tensor.has_value()) << tensor.error();
    const auto& t = tensor->get();
    ASSERT_EQ(t.get_shape().size(), 4u);
    EXPECT_EQ(t.get_shape()[0], 1u);
    EXPECT_EQ(t.get_shape()[1], 3u);
    EXPECT_EQ(t.get_shape()[2], 240u);
    EXPECT_EQ(t.get_shape()[3], 320u);
    EXPECT_EQ(t.get_element_type(), ov::element::f32);

    const float* data = static_cast<const float*>(t.data());
    const size_t n    = t.get_size();
    for (size_t i = 0; i < n; ++i) {
        EXPECT_GE(data[i], 0.0f);
        EXPECT_LE(data[i], 1.0f + 1e-5f);
    }
}

TEST(ModelInferencePreprocessTest, EmptyImageFails) {
    radar_camera::model_inference::ModelInference engine;
    cv::Mat empty;
    auto tensor = engine.infer_preprocess(empty, 1280, 1280);
    EXPECT_FALSE(tensor.has_value());
}

TEST(ModelInferenceInitTest, MissingModelFails) {
    radar_camera::model_inference::ModelInference engine;
    auto cfg       = make_config();
    cfg.model_path = "/tmp/radar_camera_missing_model_does_not_exist.onnx";
    auto ret       = engine.infer_init(cfg);
    ASSERT_FALSE(ret.has_value());
    EXPECT_NE(ret.error().find("OpenVINO"), std::string::npos);
}

TEST(ModelInferenceSmokeTest, FullPipelineWhenAssetsPresent) {
    const auto model_path = resolve_model_path();
    if (!std::filesystem::exists(model_path)) {
        GTEST_SKIP() << "ONNX missing: set RADAR_CAMERA_TEST_MODEL or place "
                     << model_path;
    }

    radar_camera::model_inference::ModelInference engine;
    auto cfg       = make_config();
    cfg.model_path = model_path;
    auto init      = engine.infer_init(cfg);
    ASSERT_TRUE(init.has_value()) << init.error();

    cv::Mat image;
    const auto image_path = resolve_image_path();
    if (std::filesystem::exists(image_path)) {
        image = cv::imread(image_path, cv::IMREAD_COLOR);
    }
    if (image.empty()) {
        image = cv::Mat(1280, 1280, CV_8UC3, cv::Scalar(32, 64, 128));
    }

    auto tensor = engine.infer_preprocess(
        image, static_cast<size_t>(cfg.model_input_width),
        static_cast<size_t>(cfg.model_input_height));
    ASSERT_TRUE(tensor.has_value()) << tensor.error();

    auto async_ret = engine.infer_runtime_async(tensor->get());
    ASSERT_TRUE(async_ret.has_value()) << async_ret.error();

    auto raw = engine.infer_runtime_wait();
    ASSERT_TRUE(raw.has_value()) << raw.error();
    EXPECT_FALSE(raw->get().empty());

    auto dets = engine.infer_postprocess(raw->get(), image.cols, image.rows);
    ASSERT_TRUE(dets.has_value()) << dets.error();
}

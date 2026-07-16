#pragma once
#include <expected>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>
#include <vector>

#include "radar_camera/data_format.hpp"

namespace radar_camera::model_inference {

class ModelInference {
public:
    explicit ModelInference(const inference_config::InferenceConfig& config);
    ~ModelInference() = default;

    auto infer_preprocess(const cv::Mat& image, size_t width, size_t height)
        -> std::expected<ov::Tensor, std::string>;

    auto infer_runtime_async(const ov::Tensor& input_tensor) -> std::expected<void, std::string>;
    auto infer_runtime_wait() -> std::expected<std::vector<float>, std::string>;

    auto infer_postprocess(const std::vector<float>& raw_output)
        -> std::expected<std::vector<detection::Detection>, std::string>;

private:
    auto infer_init(const inference_config::InferenceConfig& config) -> std::expected<void, std::string>;

    inference_config::InferenceConfig config_;
    ov::Core core_;
    ov::CompiledModel compiled_model_;
    ov::InferRequest infer_request_;
};

} // namespace radar_camera::model_inference

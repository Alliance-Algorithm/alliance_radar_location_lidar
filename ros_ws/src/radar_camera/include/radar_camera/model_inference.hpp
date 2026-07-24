#pragma once
#include <expected>
#include <functional>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>
#include <vector>

#include "radar_camera/data_format.hpp"

namespace radar_camera::model_inference {

auto filter_detections(const std::vector<float>& raw_output, int num_detections, int stride,
    int src_width, int src_height, const inference_config::InferenceConfig& config)
    -> std::expected<std::vector<detection::Detection>, std::string>;

class ModelInference {
public:
    ModelInference()  = default;
    ~ModelInference() = default;

    auto infer_init(const inference_config::InferenceConfig& config)
        -> std::expected<void, std::string>;

    auto infer_preprocess(const cv::Mat& image, size_t width, size_t height)
        -> std::expected<std::reference_wrapper<const ov::Tensor>, std::string>;

    auto infer_runtime_async(const ov::Tensor& input_tensor) -> std::expected<void, std::string>;
    auto infer_runtime_wait()
        -> std::expected<std::reference_wrapper<const std::vector<float>>, std::string>;

    auto infer_postprocess(const std::vector<float>& raw_output, int src_width, int src_height)
        -> std::expected<std::reference_wrapper<const std::vector<detection::Detection>>,
            std::string>;

private:
    ov::Tensor input_tensor_;
    std::vector<float> raw_buffer_;
    std::vector<detection::Detection> postprocess_buffer_;
    ov::Shape last_output_shape_;

    inference_config::InferenceConfig config_;
    ov::Core core_;
    ov::CompiledModel compiled_model_;
    ov::InferRequest infer_request_;
};

} // namespace radar_camera::model_inference

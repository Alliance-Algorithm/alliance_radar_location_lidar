#pragma once
#include <expected>
#include <openvino/openvino.hpp>
#include <string>
#include <vector>

#include "radar_camera/data_format.hpp"

namespace radar_camera::model_inference {

class ModelInference {
public:
    explicit ModelInference(const inference_config::InferenceConfig& config);
    ~ModelInference() = default;

    auto infer_runtime(const ov::Tensor& input_tensor) -> std::expected<std::vector<float>, std::string>;
    auto infer_runtime_async(const ov::Tensor& input_tensor) -> std::expected<void, std::string>;
    auto infer_runtime_wait() -> std::expected<std::vector<float>, std::string>;
    auto infer_filter() -> std::expected<std::vector<float>, std::string>;

private:
    auto infer_init(const inference_config::InferenceConfig& config) -> std::expected<void, std::string>;

    struct Inference_Result {
        std::vector<float> boxes;
        std::vector<float> confidences;
        std::vector<int> classes;
    };

    inference_config::InferenceConfig config_;
    ov::Core core_;
    ov::CompiledModel compiled_model_;
    ov::InferRequest infer_request_;
    Inference_Result result_;
};

} // namespace radar_camera::model_inference

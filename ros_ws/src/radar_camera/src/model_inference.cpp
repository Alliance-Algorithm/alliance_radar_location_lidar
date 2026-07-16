#include "radar_camera/model_inference.hpp"

namespace radar_camera::model_inference {

ModelInference::ModelInference(const inference_config::InferenceConfig& config) {
    auto ret = infer_init(config);
    if (!ret) {
        throw std::runtime_error(ret.error());
    }
}

auto ModelInference::infer_init(const inference_config::InferenceConfig& config) -> std::expected<void, std::string> {
    config_ = config;
    try {
        core_           = ov::Core();
        compiled_model_ = core_.compile_model(config_.model_path, config_.device_name);
        infer_request_  = compiled_model_.create_infer_request();
        return {};
    } catch (const std::exception& e) {
        return std::unexpected(std::string("OpenVINO init failed: ") + e.what());
    }
}

auto ModelInference::infer_runtime(const ov::Tensor& input_tensor)
    -> std::expected<std::vector<float>, std::string> {
    try {
        infer_request_.set_input_tensor(input_tensor);
        infer_request_.start_async();
        infer_request_.wait();

        auto output_tensor = infer_request_.get_output_tensor();
        auto* output_data  = output_tensor.data<float>();
        auto output_size   = output_tensor.get_size();
        return std::vector<float>(output_data, output_data + output_size);
    } catch (const std::exception& e) {
        return std::unexpected(std::string("Inference failed: ") + e.what());
    }
}

auto ModelInference::infer_runtime_async(const ov::Tensor& input_tensor)
    -> std::expected<void, std::string> {
    try {
        infer_request_.set_input_tensor(input_tensor);
        infer_request_.start_async();
        return {};
    } catch (const std::exception& e) {
        return std::unexpected(std::string("Async inference start failed: ") + e.what());
    }
}

auto ModelInference::infer_runtime_wait() -> std::expected<std::vector<float>, std::string> {
    try {
        infer_request_.wait();

        auto output_tensor = infer_request_.get_output_tensor();
        auto* output_data  = output_tensor.data<float>();
        auto output_size   = output_tensor.get_size();
        return std::vector<float>(output_data, output_data + output_size);
    } catch (const std::exception& e) {
        return std::unexpected(std::string("Async inference wait failed: ") + e.what());
    }
}

auto ModelInference::infer_filter() -> std::expected<std::vector<float>, std::string> {
    return { result_.boxes };
}

} // namespace radar_camera::model_inference

#include "radar_camera/model_inference.hpp"
#include <cmath>

namespace radar_camera::model_inference {

ModelInference::ModelInference(const inference_config::InferenceConfig& config) {
    auto ret = infer_init(config);
    if (!ret) {
        throw std::runtime_error(ret.error());
    }
}

auto ModelInference::infer_init(const inference_config::InferenceConfig& config)
    -> std::expected<void, std::string> {
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

auto ModelInference::infer_filter(const std::vector<float>& raw_output)
    -> std::expected<std::vector<float>, std::string> {
    try {
        auto output_tensor = infer_request_.get_output_tensor();
        auto shape         = output_tensor.get_shape();

        int num_detections;
        int stride;

        if (shape.size() == 3) {
            num_detections = static_cast<int>(shape[1]);
            stride         = static_cast<int>(shape[2]);
        } else if (shape.size() == 2) {
            num_detections = static_cast<int>(shape[0]);
            stride         = static_cast<int>(shape[1]);
        } else {
            return std::unexpected("Unsupported output shape");
        }

        if (stride < 6) {
            return std::unexpected("Output stride too small: expected >= 6");
        }

        std::vector<float> results;
        results.reserve(static_cast<size_t>(num_detections) * 6);

        for (int i = 0; i < num_detections; ++i) {
            const float* ptr = &raw_output[i * stride];

            float x1 = ptr[0];
            float y1 = ptr[1];
            float x2 = ptr[2];
            float y2 = ptr[3];
            float conf = ptr[4];
            int cls    = static_cast<int>(ptr[5]);

            if (conf < config_.conf_threshold) continue;

            float box_w = x2 - x1;
            float box_h = y2 - y1;
            if (box_w < 1.0f || box_h < 1.0f) continue;

            float ratio = std::max(box_w, box_h) / std::min(box_w, box_h);
            if (ratio < config_.min_length_width_rate || ratio > config_.max_length_width_rate) continue;

            results.push_back(x1);
            results.push_back(y1);
            results.push_back(x2);
            results.push_back(y2);
            results.push_back(conf);
            results.push_back(static_cast<float>(cls));
        }

        return results;
    } catch (const std::exception& e) {
        return std::unexpected(std::string("Filter failed: ") + e.what());
    }
}

} // namespace radar_camera::model_inference

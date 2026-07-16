#include "radar_camera/model_inference.hpp"
#include <algorithm>
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

auto ModelInference::parse_output(const ov::Tensor& output_tensor) -> std::expected<void, std::string> {
    auto shape = output_tensor.get_shape();

    if (shape.size() != 3 && shape.size() != 2) {
        return std::unexpected("Unsupported output shape: expected 2D or 3D tensor");
    }

    auto* data = output_tensor.data<float>();

    int num_detections;
    int stride;

    if (shape.size() == 3) {
        num_detections = static_cast<int>(shape[1]);
        stride         = static_cast<int>(shape[2]);
    } else {
        num_detections = static_cast<int>(shape[0]);
        stride         = static_cast<int>(shape[1]);
    }

    int expected_stride = 4 + config_.num_classes;
    if (stride != expected_stride) {
        return std::unexpected("Output stride mismatch: expected " + std::to_string(expected_stride)
                               + ", got " + std::to_string(stride));
    }

    return {};
}

auto ModelInference::infer_filter(const std::vector<float>& raw_output)
    -> std::expected<std::vector<float>, std::string> {
    try {
        auto output_tensor = infer_request_.get_output_tensor();
        auto ret = parse_output(output_tensor);
        if (!ret) {
            return std::unexpected(ret.error());
        }

        auto shape         = output_tensor.get_shape();
        int num_detections = (shape.size() == 3) ? static_cast<int>(shape[1])
                                                 : static_cast<int>(shape[0]);
        int stride         = (shape.size() == 3) ? static_cast<int>(shape[2])
                                                 : static_cast<int>(shape[1]);

        float input_w = static_cast<float>(config_.model_input_width);
        float input_h = static_cast<float>(config_.model_input_height);

        std::vector<Box> candidates;

        for (int i = 0; i < num_detections; ++i) {
            const float* ptr = &raw_output[i * stride];

            float cx   = ptr[0];
            float cy   = ptr[1];
            float w    = ptr[2];
            float h    = ptr[3];

            int best_class = -1;
            float max_prob = 0.0f;
            for (int c = 0; c < config_.num_classes; ++c) {
                float prob = ptr[4 + c];
                if (prob > max_prob) {
                    max_prob = prob;
                    best_class = c;
                }
            }

            if (max_prob < config_.conf_threshold) continue;

            float x1 = std::max(0.0f, (cx - w / 2) * input_w);
            float y1 = std::max(0.0f, (cy - h / 2) * input_h);
            float x2 = std::min(input_w - 1, (cx + w / 2) * input_w);
            float y2 = std::min(input_h - 1, (cy + h / 2) * input_h);

            float box_w = x2 - x1;
            float box_h = y2 - y1;
            if (box_w < 1.0f || box_h < 1.0f) continue;

            float ratio = std::max(box_w, box_h) / std::min(box_w, box_h);
            if (ratio < config_.min_length_width_rate || ratio > config_.max_length_width_rate) continue;

            candidates.push_back({ x1, y1, x2, y2, max_prob, best_class });
        }

        std::sort(candidates.begin(), candidates.end(),
            [](const Box& a, const Box& b) { return a.confidence > b.confidence; });

        std::vector<bool> suppressed(candidates.size(), false);

        for (size_t i = 0; i < candidates.size(); ++i) {
            if (suppressed[i]) continue;
            for (size_t j = i + 1; j < candidates.size(); ++j) {
                if (suppressed[j]) continue;
                if (candidates[i].class_id != candidates[j].class_id) continue;
                float xi1 = std::max(candidates[i].x1, candidates[j].x1);
                float yi1 = std::max(candidates[i].y1, candidates[j].y1);
                float xi2 = std::min(candidates[i].x2, candidates[j].x2);
                float yi2 = std::min(candidates[i].y2, candidates[j].y2);
                float inter = std::max(0.0f, xi2 - xi1) * std::max(0.0f, yi2 - yi1);
                float area_i = (candidates[i].x2 - candidates[i].x1)
                             * (candidates[i].y2 - candidates[i].y1);
                float area_j = (candidates[j].x2 - candidates[j].x1)
                             * (candidates[j].y2 - candidates[j].y1);
                float iou = inter / (area_i + area_j - inter);
                if (iou > config_.nms_threshold) {
                    suppressed[j] = true;
                }
            }
        }

        std::vector<float> results;
        for (size_t i = 0; i < candidates.size(); ++i) {
            if (suppressed[i]) continue;
            results.push_back(candidates[i].x1);
            results.push_back(candidates[i].y1);
            results.push_back(candidates[i].x2);
            results.push_back(candidates[i].y2);
            results.push_back(candidates[i].confidence);
            results.push_back(static_cast<float>(candidates[i].class_id));
        }
        return results;
    } catch (const std::exception& e) {
        return std::unexpected(std::string("Filter failed: ") + e.what());
    }
}

} // namespace radar_camera::model_inference

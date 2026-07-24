#include "radar_camera/model_inference.hpp"
#include <algorithm>
#include <cmath>

namespace radar_camera::model_inference {

auto filter_detections(const std::vector<float>& raw_output, int num_detections, int stride,
    int src_width, int src_height, const inference_config::InferenceConfig& config)
    -> std::expected<std::vector<detection::Detection>, std::string> {
    constexpr int kMinStride = 6;
    if (num_detections < 0 || stride < kMinStride) {
        return std::unexpected("Output stride too small: expected >= 6");
    }
    if (src_width <= 0 || src_height <= 0) {
        return std::unexpected("Invalid source image size");
    }
    if (config.model_input_width <= 0 || config.model_input_height <= 0) {
        return std::unexpected("Invalid model input size");
    }
    const auto need = static_cast<size_t>(num_detections) * static_cast<size_t>(stride);
    if (raw_output.size() < need) {
        return std::unexpected("raw_output shorter than num_detections * stride");
    }

    const float scale_x =
        static_cast<float>(src_width) / static_cast<float>(config.model_input_width);
    const float scale_y =
        static_cast<float>(src_height) / static_cast<float>(config.model_input_height);

    std::vector<detection::Detection> out;
    out.reserve(static_cast<size_t>(num_detections));

    for (int i = 0; i < num_detections; ++i) {
        const float* ptr = &raw_output[static_cast<size_t>(i) * static_cast<size_t>(stride)];

        const float x1   = ptr[0];
        const float y1   = ptr[1];
        const float x2   = ptr[2];
        const float y2   = ptr[3];
        const float conf = ptr[4];
        const int cls    = static_cast<int>(ptr[5]);

        if (conf < config.conf_threshold) continue;

        const float box_w = (x2 - x1) * scale_x;
        const float box_h = (y2 - y1) * scale_y;
        if (box_w < 1.0f || box_h < 1.0f) continue;

        const float ratio   = std::max(box_w, box_h) / std::min(box_w, box_h);
        const bool is_drone = std::find(config.drone_class_ids.begin(),
                                  config.drone_class_ids.end(), static_cast<std::int64_t>(cls))
            != config.drone_class_ids.end();
        const float min_rate =
            is_drone ? config.drone_min_length_width_rate : config.min_length_width_rate;
        const float max_rate =
            is_drone ? config.drone_max_length_width_rate : config.max_length_width_rate;
        if (ratio < min_rate || ratio > max_rate) continue;

        out.push_back(detection::Detection {
            .center     = cv::Point2d((x1 + x2) * 0.5f * scale_x, (y1 + y2) * 0.5f * scale_y),
            .id         = cls,
            .confidence = conf });
    }
    return out;
}

auto ModelInference::infer_preprocess(const cv::Mat& image, size_t width, size_t height)
    -> std::expected<std::reference_wrapper<const ov::Tensor>, std::string> {
    try {
        if (image.empty()) {
            return std::unexpected("infer_preprocess failed: empty image");
        }
        cv::Mat blob = cv::dnn::blobFromImage(image, 1.0 / 255.0,
            cv::Size(static_cast<int>(width), static_cast<int>(height)), cv::Scalar(), false,
            false);

        ov::Shape expected_shape { 1, 3, height, width };
        if (!input_tensor_ || input_tensor_.get_shape() != expected_shape) {
            input_tensor_ = ov::Tensor(ov::element::f32, expected_shape);
        }

        std::memcpy(input_tensor_.data<float>(), blob.data, blob.total() * sizeof(float));

        return std::ref(input_tensor_);
    } catch (const std::exception& e) {
        return std::unexpected(std::string("infer_preprocess failed: ") + e.what());
    }
}

auto ModelInference::infer_init(const inference_config::InferenceConfig& config)
    -> std::expected<void, std::string> {
    config_ = config;
    try {
        core_           = ov::Core();
        compiled_model_ = core_.compile_model(config_.model_path, config_.device_name);
        infer_request_  = compiled_model_.create_infer_request();
        return { };
    } catch (const std::exception& e) {
        return std::unexpected(std::string("OpenVINO init failed: ") + e.what());
    }
}

auto ModelInference::infer_runtime_async(const ov::Tensor& input_tensor)
    -> std::expected<void, std::string> {
    try {
        infer_request_.set_input_tensor(input_tensor);
        infer_request_.start_async();
        return { };
    } catch (const std::exception& e) {
        return std::unexpected(std::string("Async inference start failed: ") + e.what());
    }
}

auto ModelInference::infer_runtime_wait()
    -> std::expected<std::reference_wrapper<const std::vector<float>>, std::string> {
    try {
        infer_request_.wait();

        auto output_tensor = infer_request_.get_output_tensor();
        last_output_shape_ = output_tensor.get_shape();
        auto* output_data  = output_tensor.data<float>();
        auto output_size   = output_tensor.get_size();
        raw_buffer_.assign(output_data, output_data + output_size);
        return std::ref(raw_buffer_);
    } catch (const std::exception& e) {
        return std::unexpected(std::string("Async inference wait failed: ") + e.what());
    }
}

auto ModelInference::infer_postprocess(
    const std::vector<float>& raw_output, int src_width, int src_height)
    -> std::expected<std::reference_wrapper<const std::vector<detection::Detection>>, std::string> {
    try {
        auto shape = last_output_shape_;
        if (shape.empty()) {
            try {
                shape = infer_request_.get_output_tensor().get_shape();
            } catch (const std::exception&) {
                return std::unexpected("No output shape available; run wait() first");
            }
        }

        int num_detections = 0;
        int stride         = 0;
        if (shape.size() == 3) {
            num_detections = static_cast<int>(shape[1]);
            stride         = static_cast<int>(shape[2]);
        } else if (shape.size() == 2) {
            num_detections = static_cast<int>(shape[0]);
            stride         = static_cast<int>(shape[1]);
        } else {
            return std::unexpected("Unsupported output shape");
        }

        auto dets =
            filter_detections(raw_output, num_detections, stride, src_width, src_height, config_);
        if (!dets) {
            return std::unexpected(dets.error());
        }
        postprocess_buffer_ = std::move(*dets);
        return std::ref(postprocess_buffer_);
    } catch (const std::exception& e) {
        return std::unexpected(std::string("Filter failed: ") + e.what());
    }
}

} // namespace radar_camera::model_inference

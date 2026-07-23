#include "radar_camera/model_inference.hpp"
#include <cmath>

namespace radar_camera::model_inference {

auto ModelInference::infer_preprocess(const cv::Mat& image, size_t width, size_t height)
    -> std::expected<std::reference_wrapper<const ov::Tensor>, std::string> {
    try {
        cv::Mat blob = cv::dnn::blobFromImage(image, 1.0 / 255.0,
            cv::Size(static_cast<int>(width), static_cast<int>(height)), cv::Scalar(), false, false);

        // FIXME(TEMP): uncomment after model file is available
        // ov::Shape expected_shape { 1, 3, height, width };
        // if (input_tensor_.get_shape() != expected_shape) {
        //     input_tensor_ = ov::Tensor(ov::element::f32, expected_shape);
        // }
        // std::memcpy(input_tensor_.data<float>(), blob.data, blob.total() * sizeof(float));

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

        constexpr int kMinStride = 6;
        if (stride < kMinStride) {
            return std::unexpected("Output stride too small: expected >= 6");
        }

        float scale_x =
            static_cast<float>(src_width) / static_cast<float>(config_.model_input_width);
        float scale_y =
            static_cast<float>(src_height) / static_cast<float>(config_.model_input_height);

        postprocess_buffer_.clear();

        for (int i = 0; i < num_detections; ++i) {
            const float* ptr = &raw_output[i * stride];

            float x1   = ptr[0];
            float y1   = ptr[1];
            float x2   = ptr[2];
            float y2   = ptr[3];
            float conf = ptr[4];
            int cls    = static_cast<int>(ptr[5]);

            if (conf < config_.conf_threshold) continue;

            float box_w = (x2 - x1) * scale_x;
            float box_h = (y2 - y1) * scale_y;
            if (box_w < 1.0f || box_h < 1.0f) continue;

            float ratio = std::max(box_w, box_h) / std::min(box_w, box_h);
            if (ratio < config_.min_length_width_rate || ratio > config_.max_length_width_rate)
                continue;

            postprocess_buffer_.push_back(detection::Detection {
                .center     = cv::Point2d((x1 + x2) * 0.5f * scale_x, (y1 + y2) * 0.5f * scale_y),
                .id         = cls,
                .confidence = conf });
        }

        return std::ref(postprocess_buffer_);
    } catch (const std::exception& e) {
        return std::unexpected(std::string("Filter failed: ") + e.what());
    }
}

} // namespace radar_camera::model_inference

#include "radar_camera/model_inference.hpp"
#include <cmath>

namespace radar_camera::model_inference {

auto ModelInference::infer_preprocess(const cv::Mat& image, size_t width, size_t height)
    -> std::expected<ov::Tensor, std::string> {
    try {
        cv::Mat resized_img;
        cv::resize(image, resized_img, cv::Size(static_cast<int>(width), static_cast<int>(height)));

        cv::Mat rgb_img;
        cv::cvtColor(resized_img, rgb_img, cv::COLOR_BGR2RGB);

        cv::Mat float_img;
        rgb_img.convertTo(float_img, CV_32FC3, 1.0 / 255.0);

        ov::Tensor input_tensor(ov::element::f32, ov::Shape{ 1, 3, height, width });

        float* data        = input_tensor.data<float>();
        const int channels = 3;
        const int h        = static_cast<int>(height);
        const int w        = static_cast<int>(width);

        for (int c = 0; c < channels; ++c) {
            for (int row = 0; row < h; ++row) {
                for (int col = 0; col < w; ++col) {
                    data[c * h * w + row * w + col] = float_img.at<cv::Vec3f>(row, col)[c];
                }
            }
        }

        return input_tensor;
    } catch (const std::exception& e) {
        return std::unexpected(std::string("infer_preprocess failed: ") + e.what());
    }
}

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

auto ModelInference::infer_runtime_async(const cv::Mat& image)
    -> std::expected<void, std::string> {
    try {
        auto tensor = infer_preprocess(image, config_.model_input_width, config_.model_input_height);
        if (!tensor) {
            return std::unexpected(tensor.error());
        }
        infer_request_.set_input_tensor(*tensor);
        infer_request_.start_async();
        return {};
    } catch (const std::exception& e) {
        return std::unexpected(std::string("Async inference start failed: ") + e.what());
    }
}

auto ModelInference::infer_runtime_wait() -> std::expected<std::vector<detection::Detection>, std::string> {
    try {
        infer_request_.wait();

        auto output_tensor = infer_request_.get_output_tensor();
        auto* output_data  = output_tensor.data<float>();
        auto output_size   = output_tensor.get_size();
        std::vector<float> raw(output_data, output_data + output_size);
        return infer_filter(raw);
    } catch (const std::exception& e) {
        return std::unexpected(std::string("Async inference wait failed: ") + e.what());
    }
}

auto ModelInference::infer_filter(const std::vector<float>& raw_output)
    -> std::expected<std::vector<detection::Detection>, std::string> {
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

        std::vector<detection::Detection> results;
        results.reserve(static_cast<size_t>(num_detections));

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

            results.push_back(detection::Detection{
                .center     = cv::Point2d((x1 + x2) * 0.5f, (y1 + y2) * 0.5f),
                .id         = cls,
                .confidence = conf
            });
        }

        return results;
    } catch (const std::exception& e) {
        return std::unexpected(std::string("Filter failed: ") + e.what());
    }
}

} // namespace radar_camera::model_inference

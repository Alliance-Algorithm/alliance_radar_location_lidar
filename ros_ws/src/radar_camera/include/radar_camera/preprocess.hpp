#pragma once
#include <expected>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>

namespace radar_camera::preprocess {

auto image_to_tensor(const cv::Mat& image, size_t width, size_t height)
    -> std::expected<ov::Tensor, std::string>;

} // namespace radar_camera::preprocess

#include "radar_camera/preprocess.hpp"

namespace radar_camera::preprocess {

auto image_to_tensor(const cv::Mat& image, size_t width, size_t height)
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
        return std::unexpected(std::string("image_to_tensor failed: ") + e.what());
    }
}

} // namespace radar_camera::preprocess

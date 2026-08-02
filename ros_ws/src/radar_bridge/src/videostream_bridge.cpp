#include "radar_bridge/videostream_bridge.hpp"

#include <algorithm>
#include <iostream>
#include <print>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace radar_bridge::videostream_bridge {

namespace {
    void draw_text(cv::Mat& image, const std::string& value, cv::Point point, cv::Scalar color,
        double scale = 0.8) {
        cv::putText(image, value, point, cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(0, 0, 0), 4,
            cv::LINE_AA);
        cv::putText(image, value, point, cv::FONT_HERSHEY_SIMPLEX, scale, color, 2, cv::LINE_AA);
    }
    auto pct(float value) -> std::string {
        return std::to_string(static_cast<int>(value * 100.0f)) + "%";
    }
} // namespace

VideoBridge::~VideoBridge() { auto _ = video_thread_stop(); }

auto VideoBridge::video_init(const std::string& shm_name, const std::string& pub_address,
    std::shared_ptr<radar_camera::armor_infer::ArmorInfer> infer)
    -> std::expected<void, std::string> {
    shm_name_    = shm_name;
    pub_address_ = pub_address;
    infer_       = std::move(infer);

    auto open_ret = reader_.open(shm_name_.c_str());
    if (!open_ret) return std::unexpected("SharedFrameReader open failed: " + open_ret.error());

    try {
        pub_.bind(pub_address_);
    } catch (const zmq::error_t& e) {
        return std::unexpected("zmq bind failed: " + std::string(e.what()));
    }
    pub_.set(zmq::sockopt::conflate, 1);
    return { };
}

void VideoBridge::draw_overlay(
    cv::Mat& bgr, const std::vector<radar_camera::armor_infer::ArmorResult>& results) {
    using radar_camera::armor_infer::l1_names;
    using radar_camera::armor_infer::l3_names;
    const cv::Scalar l1_color(255, 180, 0), l2_color(0, 255, 255), miss_color(0, 0, 255);
    for (const auto& result : results) {
        const bool l3_match = result.l3.has_value();
        const cv::Scalar box_color =
            (result.final_id < 6) ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
        const auto draw_box = [&](cv::Rect2f box, cv::Scalar color, int thickness) {
            cv::rectangle(bgr, { int(box.x), int(box.y) },
                { int(box.x + box.width), int(box.y + box.height) }, color, thickness);
        };
        draw_box(result.l1_box, box_color, 8);
        if (result.l2 && result.l2->corners.size() == 4) {
            std::vector<cv::Point> poly;
            for (const auto& p : result.l2->corners)
                poly.emplace_back(int(p.x), int(p.y));
            cv::polylines(bgr, poly, true, l2_color, 6, cv::LINE_AA);
        } else if (result.l2) {
            draw_box(result.l2->box, l2_color, 6);
        }
        if (l3_match) {
            const cv::Rect2f roi = result.l2 ? result.l2->box : result.l1_box;
            draw_box(roi, l2_color, 6);
        }
        const int label_x = std::clamp(static_cast<int>(result.l1_box.x), 10, bgr.cols - 650);
        const int label_y = std::max(150, static_cast<int>(result.l1_box.y) - 110);
        draw_text(bgr, "L1 " + std::string(l1_names(result.l1_id)) + " " + pct(result.l1_conf),
            { label_x, label_y }, l1_color, 0.9);
        if (result.l2) {
            const std::string color = result.l2->color == 2 ? "BLUE"
                : result.l2->color == 1                     ? "RED"
                                                            : "UNK";
            draw_text(bgr,
                "L2 armor " + color + " genre=" + std::to_string(result.l2->genre) + " "
                    + pct(result.l2->conf),
                { label_x, label_y + 32 }, l2_color, 0.9);
        } else {
            draw_text(bgr, "L2 MISS", { label_x, label_y + 32 }, miss_color, 0.9);
        }
        if (result.l3) {
            draw_text(bgr,
                "L3 " + std::string(l3_names(result.l3->index)) + " " + pct(result.l3->conf),
                { label_x, label_y + 64 }, l2_color, 0.9);
        } else {
            draw_text(bgr, "L3 MISS", { label_x, label_y + 64 }, miss_color, 0.9);
        }
        draw_text(bgr,
            "FINAL " + std::string(l1_names(result.final_id)) + " [" + result.decision + "] "
                + result.match_state,
            { label_x, label_y + 96 },
            result.match_state == "MATCH" ? cv::Scalar(0, 255, 0) : miss_color, 0.9);
    }
}

auto VideoBridge::video_thread() -> std::expected<void, std::string> {
    video_thread_running_ = true;
    video_thread_         = std::thread([this]() {
        constexpr auto kWaitTimeout = std::chrono::milliseconds { 500 };
        while (video_thread_running_) {
            auto frame = reader_.wait_next(kWaitTimeout);
            if (!frame || !frame->valid()) continue;

            // mat() 是 SHM 上的 RGB8 非拥有视图（hikcamera SDK 输出 RGB8，与模型训练一致）；
            // clone 保证短生命周期内数据安全。imencode/overlay 按 OpenCV BGR 惯例，编码前转回。
            cv::Mat rgb = frame->mat().clone();
            cv::Mat bgr;
            cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
            if (infer_) {
                try {
                    const auto results = infer_->infer(rgb);
                    draw_overlay(bgr, results);
                } catch (const std::exception& e) {
                    std::println(std::cerr, "[VideoBridge] inference failed, passthrough frame: {}",
                        e.what());
                }
            }

            std::vector<uchar> jpeg;
            if (!cv::imencode(".jpg", bgr, jpeg, { cv::IMWRITE_JPEG_QUALITY, 85 })
                || jpeg.empty()) {
                std::println(std::cerr, "[VideoBridge] JPEG encode failed");
                video_thread_running_ = false;
                break;
            }

            try {
                auto send_ret =
                    pub_.send(zmq::message_t(jpeg.data(), jpeg.size()), zmq::send_flags::none);
                if (!send_ret) {
                    std::println(std::cerr, "[VideoBridge] ZMQ send failed");
                    video_thread_running_ = false;
                    break;
                }
            } catch (const zmq::error_t& e) {
                std::println(std::cerr, "[VideoBridge] ZMQ send error: {}", e.what());
                video_thread_running_ = false;
                break;
            }
        }
    });
    return { };
}

auto VideoBridge::video_thread_stop() -> std::expected<void, std::string> {
    video_thread_running_ = false;
    if (video_thread_.joinable()) video_thread_.join();
    return { };
}

} // namespace radar_bridge::videostream_bridge

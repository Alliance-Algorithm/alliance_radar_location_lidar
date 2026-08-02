#include <algorithm>
#include <chrono>
#include <expected>
#include <iostream>
#include <print>
#include <string>
#include <thread>

#include <opencv2/opencv.hpp>

#include "radar_camera/armor_infer.hpp"

using radar_camera::armor_infer::ArmorInfer;
using radar_camera::armor_infer::ArmorResult;
using radar_camera::armor_infer::l1_names;
using radar_camera::armor_infer::l3_names;

namespace {

void draw_text(cv::Mat& image, const std::string& value, cv::Point point,
    cv::Scalar color, double scale = 0.8) {
    cv::putText(image, value, point, cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(0, 0, 0), 4, cv::LINE_AA);
    cv::putText(image, value, point, cv::FONT_HERSHEY_SIMPLEX, scale, color, 2, cv::LINE_AA);
}

auto pct(float value) -> std::string {
    return std::to_string(static_cast<int>(value * 100.0f)) + "%";
}

void draw_overlay(cv::Mat& bgr, const std::vector<ArmorResult>& results) {
    const cv::Scalar l1_color(255, 180, 0), l2_color(0, 255, 255), miss_color(0, 0, 255);
    for (const auto& result : results) {
        const bool l3_match = result.l3.has_value();
        const cv::Scalar box_color = (result.final_id < 6)
            ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
        const auto draw_box = [&](cv::Rect2f box, cv::Scalar color, int thickness) {
            cv::rectangle(bgr, { int(box.x), int(box.y) },
                { int(box.x + box.width), int(box.y + box.height) }, color, thickness);
        };
        draw_box(result.l1_box, box_color, 8);
        if (result.l2 && result.l2->corners.size() == 4) {
            std::vector<cv::Point> poly;
            for (const auto& p : result.l2->corners) poly.emplace_back(int(p.x), int(p.y));
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
                : result.l2->color == 1 ? "RED" : "UNK";
            draw_text(bgr, "L2 armor " + color + " genre=" + std::to_string(result.l2->genre)
                    + " " + pct(result.l2->conf), { label_x, label_y + 32 }, l2_color, 0.9);
        } else {
            draw_text(bgr, "L2 MISS", { label_x, label_y + 32 }, miss_color, 0.9);
        }
        if (result.l3) {
            draw_text(bgr, "L3 " + std::string(l3_names(result.l3->index)) + " "
                    + pct(result.l3->conf), { label_x, label_y + 64 }, l2_color, 0.9);
        } else {
            draw_text(bgr, "L3 MISS", { label_x, label_y + 64 }, miss_color, 0.9);
        }
        draw_text(bgr, "FINAL " + std::string(l1_names(result.final_id)) + " ["
                + result.decision + "] " + result.match_state,
            { label_x, label_y + 96 }, result.match_state == "MATCH"
                ? cv::Scalar(0, 255, 0) : miss_color, 0.9);
    }
}

} // namespace

auto main(int argc, char** argv) -> int {
    if (argc < 3) {
        std::println(std::cerr, "usage: video_annotate <input.mp4> <output.mp4> [model_dir]");
        std::println(std::cerr, "       video_annotate --stats <input.mp4> [model_dir]");
        return 2;
    }
    const bool stats_only = std::string(argv[1]) == "--stats";
    const std::string input_path  = stats_only ? argv[2] : argv[1];
    const std::string output_path = stats_only ? "" : argv[2];
    const std::string model_dir   = stats_only
        ? (argc >= 4 ? argv[3] : "/workspace/ros_ws/src/radar_camera/model")
        : (argc >= 4 ? argv[3] : "/workspace/ros_ws/src/radar_camera/model");

    auto infer = ArmorInfer::create(model_dir);
    if (!infer) {
        std::println(std::cerr, "ArmorInfer init failed: {}", infer.error());
        return 1;
    }

    cv::VideoCapture cap(input_path);
    if (!cap.isOpened()) {
        std::println(std::cerr, "failed to open video: {}", input_path);
        return 1;
    }
    const double fps  = cap.get(cv::CAP_PROP_FPS);
    const int width   = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    const int height  = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    const int total   = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));

    cv::VideoWriter writer;
    if (!stats_only) {
        writer.open(output_path, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), fps,
            cv::Size(width, height));
        if (!writer.isOpened()) {
            std::println(std::cerr, "failed to open writer: {}", output_path);
            return 1;
        }
    }

    std::println("[video_annotate] {} {}x{} @ {} fps, {} frames{}",
        input_path, width, height, fps, total,
        stats_only ? " (stats only)" : " -> " + output_path);

    // per-class stats: 0-11 L1 class ids
    struct ClassStat {
        int frames_seen = 0;      // frames with >=1 detection of this class
        int total_dets  = 0;      // total detection count
        double conf_sum = 0.0;    // confidence sum
        double x_sum = 0.0, y_sum = 0.0;  // L1 box center sum (pixels)
    };
    std::vector<ClassStat> stats(12);

    cv::Mat bgr;
    int frame_idx = 0;
    const auto t0 = std::chrono::steady_clock::now();
    while (cap.read(bgr)) {
        if (bgr.empty()) break;
        ++frame_idx;
        try {
            cv::Mat rgb;
            cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
            const auto results = (*infer)->infer(rgb);
            if (stats_only) {
                bool class_seen[12] = { false };
                for (const auto& r : results) {
                    if (r.l1_id >= 0 && r.l1_id < 12) {
                        class_seen[r.l1_id] = true;
                        auto& s             = stats[r.l1_id];
                        ++s.total_dets;
                        s.conf_sum += r.l1_conf;
                        s.x_sum += r.l1_box.x + r.l1_box.width / 2.0;
                        s.y_sum += r.l1_box.y + r.l1_box.height / 2.0;
                    }
                }
                for (int i = 0; i < 12; ++i) {
                    if (class_seen[i]) ++stats[i].frames_seen;
                }
            } else {
                draw_overlay(bgr, results);
            }
        } catch (const std::exception& e) {
            std::println(std::cerr, "frame {} inference failed: {}", frame_idx, e.what());
        }
        if (!stats_only) {
            writer.write(bgr);
        }

        if (frame_idx % 30 == 0) {
            const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            const double done_pct = total > 0 ? 100.0 * frame_idx / total : 0.0;
            const double fps_eff  = frame_idx / elapsed;
            std::print("\r[{}/{}] {:.1f}%  {:.1f} fps", frame_idx, total, done_pct, fps_eff);
            std::flush(std::cout);
        }
    }
    const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::println("\n[video_annotate] done: {} frames in {} s ({} fps)",
        frame_idx, elapsed, frame_idx / std::max(elapsed, 0.001));

    if (stats_only) {
        std::println("\n=== per-class stats (fps={}, frames={}) ===", fps, frame_idx);
        std::println("class        frames   seconds   dets    mean_conf   mean_x   mean_y");
        for (int i = 0; i < 12; ++i) {
            const auto& s = stats[i];
            if (s.frames_seen == 0) continue;
            const double secs = s.frames_seen / std::max(fps, 1.0);
            std::println("{}  {}    {:.1f}s      {}     {:.2f}      {:.0f}   {:.0f}",
                l1_names(i), s.frames_seen, secs, s.total_dets,
                s.conf_sum / s.total_dets, s.x_sum / s.total_dets, s.y_sum / s.total_dets);
        }
    }
    return 0;
}

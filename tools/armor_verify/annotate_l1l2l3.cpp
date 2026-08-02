#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <print>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "radar_camera/armor_infer.hpp"
#include "radar_camera/tensorrt_inference.hpp"

namespace fs = std::filesystem;
using radar_camera::model_inference::TensorRtInference;
using radar_camera::armor_infer::ArmorInfer;
using radar_camera::armor_infer::l1_names;
using radar_camera::armor_infer::l3_names;

namespace {

auto crop(const cv::Mat& frame, cv::Rect2f box) -> cv::Mat {
    int x = std::clamp(static_cast<int>(box.x), 0, frame.cols - 1);
    int y = std::clamp(static_cast<int>(box.y), 0, frame.rows - 1);
    int w = std::clamp(static_cast<int>(box.width), 1, frame.cols - x);
    int h = std::clamp(static_cast<int>(box.height), 1, frame.rows - y);
    return frame(cv::Rect(x, y, w, h));
}

void text(cv::Mat& image, const std::string& value, cv::Point point, cv::Scalar color, double scale = 0.8) {
    cv::putText(image, value, point, cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(0, 0, 0), 4, cv::LINE_AA);
    cv::putText(image, value, point, cv::FONT_HERSHEY_SIMPLEX, scale, color, 2, cv::LINE_AA);
}

auto pct(float value) -> std::string { return std::to_string(static_cast<int>(value * 100.0f)) + "%"; }

auto main_impl(int argc, char** argv) -> int {
    if (argc < 4) {
        std::println(std::cerr, "usage: annotate_l1l2l3 <frames> <output> <model_dir>");
        return 2;
    }
    const fs::path frames = argv[1], output = argv[2];
    const auto infer = ArmorInfer::create(argv[3]);
    if (!infer) {
        std::println(std::cerr, "ArmorInfer init failed: {}", infer.error());
        return 1;
    }
    fs::create_directories(output);
    std::ofstream csv(output / "results.csv");
    csv << "frame,det_idx,l1_id,l1_conf,l1_x,l1_y,l1_w,l1_h,l2_match,l2_genre,l2_color,l2_conf,l2_x,l2_y,l2_w,l2_h,l3_plate,l3_plate_conf,decision,final_id,match_state\n";
    std::vector<fs::path> files;
    for (const auto& entry : fs::directory_iterator(frames)) if (entry.path().extension() == ".jpg") files.push_back(entry.path());
    std::sort(files.begin(), files.end());
    const cv::Scalar l1_color(255, 180, 0), l2_color(0, 255, 255), miss_color(0, 0, 255);
    for (const auto& file : files) {
        cv::Mat bgr = cv::imread(file.string()); if (bgr.empty()) continue;
        cv::Mat rgb; cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
        constexpr int panel_width = 1100;
        cv::Mat output_image(bgr.rows, bgr.cols + panel_width, CV_8UC3, cv::Scalar(28, 28, 28));
        bgr.copyTo(output_image(cv::Rect(0, 0, bgr.cols, bgr.rows)));
        cv::rectangle(output_image, { 0, 0 }, { output_image.cols - 1, 92 }, { 25, 25, 25 }, cv::FILLED);
        text(output_image, "L1 orange", { 25, 52 }, l1_color, 1.0);
        text(output_image, "L2 plate poly", { 245, 52 }, l2_color, 1.0);
        text(output_image, "L3 number 224", { 540, 52 }, l2_color, 1.0);
        text(output_image, "green=MATCH", { 880, 52 }, { 0, 255, 0 }, 1.0);
        text(output_image, "red=MISS", { 1120, 52 }, miss_color, 1.0);
        const auto results = (*infer)->infer(rgb);
        int index = 0;
        for (const auto& result : results) {
            const bool l2_match = result.l2.has_value();
            const bool l3_match = result.l3.has_value();
            const auto l3_value = result.l3;
            const cv::Scalar box_color = (result.final_id < 6) ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
            const auto draw_box = [&](cv::Rect2f box, cv::Scalar color) {
                cv::rectangle(output_image, { int(box.x), int(box.y) },
                    { int(box.x + box.width), int(box.y + box.height) }, color, 8);
            };
            draw_box(result.l1_box, box_color);
            if (result.l2 && result.l2->corners.size() == 4) {
                std::vector<cv::Point> poly;
                for (const auto& p : result.l2->corners) poly.emplace_back(int(p.x), int(p.y));
                cv::polylines(output_image, poly, true, l2_color, 6, cv::LINE_AA);
            } else if (result.l2) {
                draw_box(result.l2->box, l2_color);
            }
            if (l3_value) {
                const cv::Rect2f roi = result.l2 ? result.l2->box : result.l1_box;
                draw_box(roi, l2_color);
            }
            const int label_x = std::clamp(static_cast<int>(result.l1_box.x), 10, bgr.cols - 650);
            const int label_y = std::max(150, static_cast<int>(result.l1_box.y) - 110);
            text(output_image, "L1 " + std::string(l1_names(result.l1_id)) + " " + pct(result.l1_conf),
                { label_x, label_y }, l1_color, 0.9);
            if (result.l2) {
                const std::string color = result.l2->color == 2 ? "BLUE" : result.l2->color == 1 ? "RED" : "UNK";
                text(output_image, "L2 armor " + color + " genre=" + std::to_string(result.l2->genre)
                        + " " + pct(result.l2->conf), { label_x, label_y + 32 }, l2_color, 0.9);
            } else {
                text(output_image, "L2 MISS", { label_x, label_y + 32 }, miss_color, 0.9);
            }
            if (l3_value) {
                text(output_image, "L3 " + std::string(l3_names(l3_value->index)) + " " + pct(l3_value->conf),
                    { label_x, label_y + 64 }, l2_color, 0.9);
            } else {
                text(output_image, "L3 MISS", { label_x, label_y + 64 }, miss_color, 0.9);
            }
            text(output_image, "FINAL " + std::string(l1_names(result.final_id)) + " [" + result.decision + "] " + result.match_state,
                { label_x, label_y + 96 }, result.match_state == "MATCH"
                    ? cv::Scalar(0, 255, 0) : miss_color, 0.9);

            const int card_y = 125 + index * 620;
            if (card_y + 570 < output_image.rows) {
                const int panel_x = bgr.cols + 35;
                text(output_image, "TARGET " + std::to_string(index + 1), { panel_x, card_y }, { 255, 255, 255 }, 0.9);
                const cv::Rect2f armor_box = result.l2 ? result.l2->box : result.l1_box;
                cv::Mat armor_crop = crop(bgr, armor_box);
                cv::Mat armor_preview;
                cv::resize(armor_crop, armor_preview, { panel_width - 70, 400 });
                armor_preview.copyTo(output_image(cv::Rect(panel_x, card_y + 20, armor_preview.cols, armor_preview.rows)));
                text(output_image, result.l2 ? "L2 armor crop" : "L1 ROI (L2 MISS)",
                    { panel_x, card_y + 450 }, result.l2 ? l2_color : miss_color, 0.8);
                if (l3_value) {
                    text(output_image, "L3 " + std::string(l3_names(l3_value->index)) + " " + pct(l3_value->conf),
                        { panel_x, card_y + 485 }, l2_color, 0.8);
                } else {
                    text(output_image, "L3 MISS", { panel_x, card_y + 485 }, miss_color, 0.8);
                }
            }
            csv << file.filename().string() << "," << index << "," << result.l1_id << "," << result.l1_conf << ","
                << result.l1_box.x << "," << result.l1_box.y << "," << result.l1_box.width << "," << result.l1_box.height << ","
                << (result.l2 ? 1 : 0) << "," << (result.l2 ? result.l2->genre : -1) << "," << (result.l2 ? result.l2->color : -1) << "," << (result.l2 ? result.l2->conf : 0) << ","
                << (result.l2 ? result.l2->box.x : 0) << "," << (result.l2 ? result.l2->box.y : 0) << "," << (result.l2 ? result.l2->box.width : 0) << "," << (result.l2 ? result.l2->box.height : 0) << ","
                << (result.l3 ? result.l3->index : -1) << "," << (result.l3 ? result.l3->conf : 0) << ","
                << result.decision << "," << result.final_id << "," << result.match_state << "\n";
            ++index;
        }
        const std::string stem = file.stem().string();
        cv::imwrite((output / (stem + "_l123.jpg")).string(), output_image);
    }
    return 0;
}

} // namespace

auto main(int argc, char** argv) -> int { return main_impl(argc, argv); }

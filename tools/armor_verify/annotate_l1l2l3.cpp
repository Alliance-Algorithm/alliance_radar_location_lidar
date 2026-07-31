#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "radar_camera/tensorrt_inference.hpp"

namespace fs = std::filesystem;
using radar_camera::model_inference::TensorRtInference;

namespace {
constexpr float kL1Conf = 0.20f;
constexpr float kL2Conf = 0.50f;
constexpr float kL3Conf = 0.80f;
constexpr float kL2Nms = 0.30f;
constexpr int kSideL1 = 1280;
constexpr int kSideL2 = 640;
constexpr int kSideL3 = 224;
const char* names[] = { "hero-R", "eng-R", "inf3-R", "inf4-R", "sentry-R", "drone-R",
    "hero-B", "eng-B", "inf3-B", "inf4-B", "sentry-B", "drone-B" };
const char* number_names[] = { "B1", "B2", "B3", "B4", "BS", "R1", "R2", "R3", "R4" };

struct Det { int id; float conf; cv::Rect2f box; };
struct Plate { cv::Rect2f box; std::vector<cv::Point2f> corners; int genre; int color; float conf; };
struct Number { int index; float conf; };
struct Result {
    Det l1;
    std::optional<Plate> l2;
    std::optional<Number> plate_l3;
    int final_id;
    std::string decision;
};

auto sigmoid(float x) -> float { return 1.0f / (1.0f + std::exp(-x)); }

auto letterbox(const cv::Mat& src, int side, bool center, float& scale, int& px, int& py) -> cv::Mat {
    scale = std::min(static_cast<float>(side) / src.cols, static_cast<float>(side) / src.rows);
    const int w = std::max(1, static_cast<int>(std::round(src.cols * scale)));
    const int h = std::max(1, static_cast<int>(std::round(src.rows * scale)));
    px = center ? (side - w) / 2 : 0;
    py = center ? (side - h) / 2 : 0;
    cv::Mat out(side, side, src.type(), cv::Scalar::all(0));
    cv::Mat resized;
    cv::resize(src, resized, { w, h });
    resized.copyTo(out(cv::Rect(px, py, w, h)));
    return out;
}

auto blob(const cv::Mat& rgb) -> std::vector<float> {
    cv::Mat b = cv::dnn::blobFromImage(rgb, 1.0 / 255.0, {}, {}, false, false);
    std::vector<float> out(b.total());
    std::memcpy(out.data(), b.ptr<float>(), out.size() * sizeof(float));
    return out;
}

auto infer(TensorRtInference& engine, const cv::Mat& input)
    -> std::optional<std::vector<float>> {
    auto data = blob(input);
    if (!engine.start(data.data(), data.size())) return std::nullopt;
    auto output = engine.wait();
    if (!output) return std::nullopt;
    return output->get();
}

auto iou(const cv::Rect2f& a, const cv::Rect2f& b) -> float {
    const auto inter = a & b;
    const float union_area = a.area() + b.area() - inter.area();
    return union_area <= 0.0f ? 0.0f : inter.area() / union_area;
}

auto l1(TensorRtInference& engine, const cv::Mat& rgb) -> std::vector<Det> {
    float scale; int px; int py;
    auto input = letterbox(rgb, kSideL1, false, scale, px, py);
    auto raw = infer(engine, input);
    if (!raw) return {};
    std::vector<Det> best;
    for (size_t i = 0; i + 5 < raw->size(); i += 6) {
        const float conf = (*raw)[i + 4];
        const int id = static_cast<int>((*raw)[i + 5]);
        if (conf < kL1Conf) continue;
        const float x1 = (*raw)[i] / scale;
        const float y1 = (*raw)[i + 1] / scale;
        const float x2 = (*raw)[i + 2] / scale;
        const float y2 = (*raw)[i + 3] / scale;
        cv::Rect2f box(x1, y1, x2 - x1, y2 - y1);
        if (box.width < 1 || box.height < 1) continue;
        const float ratio = std::max(box.width, box.height) / std::min(box.width, box.height);
        const bool drone = id == 5 || id == 11;
        if (ratio < (drone ? 2.0f : 0.5f) || ratio > (drone ? 10.0f : 3.0f)) continue;
        auto it = std::find_if(best.begin(), best.end(), [id](const Det& d) { return d.id == id; });
        Det candidate { id, conf, box };
        if (it == best.end()) best.push_back(candidate);
        else if (conf > it->conf) *it = candidate;
    }
    return best;
}

auto crop(const cv::Mat& frame, cv::Rect2f box) -> cv::Mat {
    int x = std::clamp(static_cast<int>(box.x), 0, frame.cols - 1);
    int y = std::clamp(static_cast<int>(box.y), 0, frame.rows - 1);
    int w = std::clamp(static_cast<int>(box.width), 1, frame.cols - x);
    int h = std::clamp(static_cast<int>(box.height), 1, frame.rows - y);
    return frame(cv::Rect(x, y, w, h));
}

auto run_l3(TensorRtInference& engine, const cv::Mat& frame, cv::Rect2f box) -> std::optional<Number> {
    float scale; int px; int py;
    auto input = letterbox(crop(frame, box), kSideL3, true, scale, px, py);
    auto raw = infer(engine, input);
    if (!raw || raw->size() < 9) return std::nullopt;
    // Model output is already softmax probabilities — use directly.
    const int index = static_cast<int>(std::max_element(raw->begin(), raw->begin() + 9) - raw->begin());
    const float confidence = (*raw)[index];
    if (confidence < kL3Conf) return std::nullopt;
    return Number { index, confidence };
}

auto run_l2(TensorRtInference& engine, const cv::Mat& frame, cv::Rect2f roi) -> std::optional<Plate> {
    float scale; int px; int py;
    const cv::Rect2f source_roi = roi;
    auto input = letterbox(crop(frame, roi), kSideL2, false, scale, px, py);
    auto raw = infer(engine, input);
    if (!raw) return std::nullopt;
    std::vector<Plate> candidates;
    for (size_t i = 0; i + 21 < raw->size(); i += 22) {
        const float confidence = sigmoid((*raw)[i + 8]);
        if (confidence < kL2Conf) continue;
        float min_x = (*raw)[i], max_x = (*raw)[i];
        float min_y = (*raw)[i + 1], max_y = (*raw)[i + 1];
        for (int p = 0; p < 4; ++p) {
            min_x = std::min(min_x, (*raw)[i + p * 2]);
            max_x = std::max(max_x, (*raw)[i + p * 2]);
            min_y = std::min(min_y, (*raw)[i + p * 2 + 1]);
            max_y = std::max(max_y, (*raw)[i + p * 2 + 1]);
        }
        cv::Rect2f box((min_x - px) / scale + source_roi.x, (min_y - py) / scale + source_roi.y,
            (max_x - min_x) / scale, (max_y - min_y) / scale);
        if (box.width < 1 || box.height < 1) continue;
        int color = static_cast<int>(std::max_element(raw->begin() + i + 9, raw->begin() + i + 13) - (raw->begin() + i + 9));
        int genre = static_cast<int>(std::max_element(raw->begin() + i + 13, raw->begin() + i + 22) - (raw->begin() + i + 13));
        std::vector<cv::Point2f> corners;
        for (int p = 0; p < 4; ++p) {
            corners.emplace_back(((*raw)[i + p * 2] - px) / scale + source_roi.x,
                ((*raw)[i + p * 2 + 1] - py) / scale + source_roi.y);
        }
        candidates.push_back({ box, corners, genre, color, confidence });
    }
    std::sort(candidates.begin(), candidates.end(), [](const Plate& a, const Plate& b) { return a.conf > b.conf; });
    for (size_t i = 0; i < candidates.size(); ++i) {
        bool suppressed = false;
        for (size_t j = 0; j < i; ++j) if (iou(candidates[i].box, candidates[j].box) > kL2Nms) suppressed = true;
        if (!suppressed) return candidates[i];
    }
    return std::nullopt;
}

auto l3_id(int index) -> std::optional<int> {
    // The L3 ONNX names are B1..BS,R1..R4, while L1 IDs are red-first.
    if (index >= 0 && index <= 4) return index + 6;
    if (index >= 5 && index <= 8) return index - 5;
    return std::nullopt;
}

auto l2_id(int genre, int color) -> std::optional<int> {
    const int ids[] = { -1, 0, 1, 2, 3, -1, 4 };
    if (genre < 0 || genre >= 7 || color == 0 || ids[genre] < 0) return std::nullopt;
    return color == 1 ? ids[genre] : ids[genre] + 6;
}

void text(cv::Mat& image, const std::string& value, cv::Point point, cv::Scalar color, double scale = 0.8) {
    cv::putText(image, value, point, cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(0, 0, 0), 4, cv::LINE_AA);
    cv::putText(image, value, point, cv::FONT_HERSHEY_SIMPLEX, scale, color, 2, cv::LINE_AA);
}

auto main_impl(int argc, char** argv) -> int {
    if (argc < 4) {
        std::cerr << "usage: annotate_l1l2l3 <frames> <output> <model_dir>\n";
        return 2;
    }
    const fs::path frames = argv[1], output = argv[2], models = argv[3];
    fs::create_directories(output);
    TensorRtInference l1_engine, l2_engine, l3_engine;
    if (!l1_engine.init((models / "best_fixed_names_1280_fp16.engine").string())) return 1;
    if (!l2_engine.init((models / "shenzhen-0708_fp16.engine").string())) return 1;
    if (!l3_engine.init((models / "armor-number_fp16.engine").string())) return 1;
    std::ofstream csv(output / "results.csv");
    csv << "frame,det_idx,l1_id,l1_conf,l1_x,l1_y,l1_w,l1_h,l2_match,l2_genre,l2_color,l2_conf,l2_x,l2_y,l2_w,l2_h,l3_plate,l3_plate_conf,decision,final_id,match_state\n";
    std::vector<fs::path> files;
    for (const auto& entry : fs::directory_iterator(frames)) if (entry.path().extension() == ".jpg") files.push_back(entry.path());
    std::sort(files.begin(), files.end());
    const cv::Scalar l1_color(255, 180, 0), l2_color(0, 255, 255), l3_color(0, 255, 255), miss_color(0, 0, 255);
    for (const auto& file : files) {
        cv::Mat bgr = cv::imread(file.string()); if (bgr.empty()) continue;
        cv::Mat rgb; cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
        constexpr int panel_width = 1100;
        cv::Mat output_image(bgr.rows, bgr.cols + panel_width, CV_8UC3, cv::Scalar(28, 28, 28));
        bgr.copyTo(output_image(cv::Rect(0, 0, bgr.cols, bgr.rows)));
        cv::rectangle(output_image, { 0, 0 }, { output_image.cols - 1, 92 }, { 25, 25, 25 }, cv::FILLED);
        text(output_image, "L1 orange", { 25, 52 }, l1_color, 1.0);
        text(output_image, "L2 plate poly", { 245, 52 }, l2_color, 1.0);
        text(output_image, "L3 number 224", { 540, 52 }, l3_color, 1.0);
        text(output_image, "green=MATCH", { 880, 52 }, { 0, 255, 0 }, 1.0);
        text(output_image, "red=MISS", { 1120, 52 }, miss_color, 1.0);
        const auto detections = l1(l1_engine, rgb);
        int index = 0;
        for (const auto& detection : detections) {
            Result result { detection, std::nullopt, std::nullopt, detection.id, "L1" };
            if (detection.id != 5 && detection.id != 11) {
                // L2 plate detector always runs for the plate-polygon annotation.
                result.l2 = run_l2(l2_engine, rgb, detection.box);
                if (result.l2) {
                    // L3 number classifier on the tighter plate crop only.
                    result.plate_l3 = run_l3(l3_engine, rgb, result.l2->box);
                    if (result.plate_l3 && l3_id(result.plate_l3->index)) {
                        result.final_id = *l3_id(result.plate_l3->index); result.decision = "L3-plate";
                    } else if (l2_id(result.l2->genre, result.l2->color)) {
                        result.final_id = *l2_id(result.l2->genre, result.l2->color); result.decision = "L2";
                    }
                }
            }
            const bool l2_match = result.l2.has_value();
            const bool l3_match = result.plate_l3.has_value();
            const std::string state = (l2_match || l3_match || detection.id == 5 || detection.id == 11) ? "MATCH" : "MISS";
            const auto l3_value = result.plate_l3;
            // L1 boxes are colored by the FINAL class color: 0-5 red, 6-11 blue.
            const cv::Scalar box_color = (result.final_id < 6) ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
            const auto draw_box = [&](cv::Rect2f box, cv::Scalar color) {
                cv::rectangle(output_image, { int(box.x), int(box.y) },
                    { int(box.x + box.width), int(box.y + box.height) }, color, 8);
            };
            draw_box(detection.box, box_color);
            if (result.l2 && result.l2->corners.size() == 4) {
                std::vector<cv::Point> poly;
                for (const auto& p : result.l2->corners) poly.emplace_back(int(p.x), int(p.y));
                cv::polylines(output_image, poly, true, l2_color, 6, cv::LINE_AA);
            } else if (result.l2) {
                draw_box(result.l2->box, l2_color);
            }
            if (l3_value) {
                const cv::Rect2f roi = result.l2 ? result.l2->box : detection.box;
                draw_box(roi, l3_color);
            }
            const auto pct = [](float value) { return std::to_string(static_cast<int>(value * 100.0f)) + "%"; };
            const int label_x = std::clamp(static_cast<int>(detection.box.x), 10, bgr.cols - 650);
            const int label_y = std::max(150, static_cast<int>(detection.box.y) - 110);
            text(output_image, "L1 " + std::string(names[detection.id]) + " " + pct(detection.conf),
                { label_x, label_y }, l1_color, 0.9);
            if (result.l2) {
                const std::string color = result.l2->color == 2 ? "BLUE" : result.l2->color == 1 ? "RED" : "UNK";
                text(output_image, "L2 armor " + color + " genre=" + std::to_string(result.l2->genre)
                        + " " + pct(result.l2->conf), { label_x, label_y + 32 }, l2_color, 0.9);
            } else {
                text(output_image, "L2 MISS", { label_x, label_y + 32 }, miss_color, 0.9);
            }
            if (l3_value) {
                text(output_image, "L3 " + std::string(number_names[l3_value->index]) + " " + pct(l3_value->conf),
                    { label_x, label_y + 64 }, l3_color, 0.9);
            } else {
                text(output_image, "L3 MISS", { label_x, label_y + 64 }, miss_color, 0.9);
            }
            text(output_image, "FINAL " + std::string(names[result.final_id]) + " [" + result.decision + "] " + state,
                { label_x, label_y + 96 }, state == "MATCH" ? box_color : miss_color, 0.9);

            const int card_y = 125 + index * 620;
            if (card_y + 570 < output_image.rows) {
                const int panel_x = bgr.cols + 35;
                text(output_image, "TARGET " + std::to_string(index + 1), { panel_x, card_y }, { 255, 255, 255 }, 0.9);
                const cv::Rect2f armor_box = result.l2 ? result.l2->box : detection.box;
                cv::Mat armor_crop = crop(bgr, armor_box);
                cv::Mat armor_preview;
                cv::resize(armor_crop, armor_preview, { panel_width - 70, 400 });
                armor_preview.copyTo(output_image(cv::Rect(panel_x, card_y + 20, armor_preview.cols, armor_preview.rows)));
                text(output_image, result.l2 ? "L2 armor crop" : "L1 ROI (L2 MISS)",
                    { panel_x, card_y + 450 }, result.l2 ? l2_color : miss_color, 0.8);
                if (l3_value) {
                    text(output_image, "L3 " + std::string(number_names[l3_value->index]) + " " + pct(l3_value->conf),
                        { panel_x, card_y + 485 }, l3_color, 0.8);
                } else {
                    text(output_image, "L3 MISS", { panel_x, card_y + 485 }, miss_color, 0.8);
                }
            }
            csv << file.filename().string() << "," << index << "," << detection.id << "," << detection.conf << ","
                << detection.box.x << "," << detection.box.y << "," << detection.box.width << "," << detection.box.height << ","
                << (result.l2 ? 1 : 0) << "," << (result.l2 ? result.l2->genre : -1) << "," << (result.l2 ? result.l2->color : -1) << "," << (result.l2 ? result.l2->conf : 0) << ","
                << (result.l2 ? result.l2->box.x : 0) << "," << (result.l2 ? result.l2->box.y : 0) << "," << (result.l2 ? result.l2->box.width : 0) << "," << (result.l2 ? result.l2->box.height : 0) << ","
                << (result.plate_l3 ? result.plate_l3->index : -1) << "," << (result.plate_l3 ? result.plate_l3->conf : 0) << ","
                << result.decision << "," << result.final_id << "," << state << "\n";
            ++index;
        }
        const std::string stem = file.stem().string();
        cv::imwrite((output / (stem + "_l123.jpg")).string(), output_image);
    }
    return 0;
}

} // namespace

auto main(int argc, char** argv) -> int { return main_impl(argc, argv); }

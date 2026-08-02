#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <print>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "radar_camera/tensorrt_inference.hpp"

namespace fs = std::filesystem;
using radar_camera::model_inference::TensorRtInference;

namespace {
constexpr int kSide = 640;
constexpr float kConfidence = 0.8f;
const cv::Scalar kBoxColor(0, 165, 255);

struct Plate {
    cv::Rect2f box;
    int genre;
    int color;
    float confidence;
};

auto sigmoid(float value) -> float { return 1.0f / (1.0f + std::exp(-value)); }

auto crop(const cv::Mat& image, cv::Rect2f box) -> cv::Mat {
    const int x = std::clamp(static_cast<int>(box.x), 0, image.cols - 1);
    const int y = std::clamp(static_cast<int>(box.y), 0, image.rows - 1);
    const int w = std::clamp(static_cast<int>(box.width), 1, image.cols - x);
    const int h = std::clamp(static_cast<int>(box.height), 1, image.rows - y);
    return image(cv::Rect(x, y, w, h));
}

auto infer(TensorRtInference& engine, const cv::Mat& input) -> std::vector<float> {
    cv::Mat blob = cv::dnn::blobFromImage(input, 1.0 / 255.0, {}, {}, false, false);
    std::vector<float> data(blob.total());
    std::memcpy(data.data(), blob.ptr<float>(), data.size() * sizeof(float));
    if (!engine.start(data.data(), data.size())) return {};
    const auto output = engine.wait();
    return output ? output->get() : std::vector<float> {};
}

auto detect(TensorRtInference& engine, const cv::Mat& rgb) -> std::vector<Plate> {
    float scale = std::min(static_cast<float>(kSide) / rgb.cols, static_cast<float>(kSide) / rgb.rows);
    const int width = std::max(1, static_cast<int>(std::round(rgb.cols * scale)));
    const int height = std::max(1, static_cast<int>(std::round(rgb.rows * scale)));
    cv::Mat input(kSide, kSide, rgb.type(), cv::Scalar::all(0));
    cv::Mat resized;
    cv::resize(rgb, resized, { width, height });
    resized.copyTo(input(cv::Rect(0, 0, width, height)));
    const auto raw = infer(engine, input);
    std::vector<Plate> plates;
    for (size_t i = 0; i + 21 < raw.size(); i += 22) {
        const float confidence = sigmoid(raw[i + 8]);
        if (confidence < kConfidence) continue;
        float min_x = raw[i], max_x = raw[i];
        float min_y = raw[i + 1], max_y = raw[i + 1];
        for (int point = 0; point < 4; ++point) {
            min_x = std::min(min_x, raw[i + point * 2]);
            max_x = std::max(max_x, raw[i + point * 2]);
            min_y = std::min(min_y, raw[i + point * 2 + 1]);
            max_y = std::max(max_y, raw[i + point * 2 + 1]);
        }
        cv::Rect2f box(min_x / scale, min_y / scale, (max_x - min_x) / scale, (max_y - min_y) / scale);
        if (box.width < 1.0f || box.height < 1.0f) continue;
        const int color = static_cast<int>(std::max_element(raw.begin() + i + 9, raw.begin() + i + 13)
            - (raw.begin() + i + 9));
        const int genre = static_cast<int>(std::max_element(raw.begin() + i + 13, raw.begin() + i + 22)
            - (raw.begin() + i + 13));
        plates.push_back({ box, genre, color, confidence });
    }
    std::sort(plates.begin(), plates.end(), [](const Plate& a, const Plate& b) {
        return a.confidence > b.confidence;
    });
    std::vector<Plate> kept;
    for (const auto& plate : plates) {
        bool overlap = false;
        for (const auto& previous : kept) {
            const auto intersection = plate.box & previous.box;
            const float union_area = plate.box.area() + previous.box.area() - intersection.area();
            if (union_area > 0.0f && intersection.area() / union_area > 0.3f) {
                overlap = true;
                break;
            }
        }
        if (!overlap) kept.push_back(plate);
    }
    return kept;
}

auto main_impl(int argc, char** argv) -> int {
    if (argc < 4) {
        std::println(std::cerr, "usage: annotate_l2 <frames> <output> <model_dir>");
        return 2;
    }
    const fs::path frames = argv[1], output = argv[2], models = argv[3];
    fs::create_directories(output);
    TensorRtInference engine;
    if (!engine.init((models / "shenzhen-0708_fp16.engine").string())) return 1;
    std::ofstream csv(output / "results.csv");
    csv << "frame,plate_idx,genre,color,confidence,x,y,w,h\n";
    std::vector<fs::path> files;
    for (const auto& entry : fs::directory_iterator(frames)) {
        if (entry.path().extension() == ".jpg") files.push_back(entry.path());
    }
    std::sort(files.begin(), files.end());
    for (const auto& file : files) {
        cv::Mat bgr = cv::imread(file.string());
        if (bgr.empty()) continue;
        cv::Mat rgb;
        cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
        cv::Mat annotated = bgr.clone();
        cv::putText(annotated, "L2 armor  conf>=0.80", { 25, 58 }, cv::FONT_HERSHEY_SIMPLEX,
            1.0, cv::Scalar::all(0), 5, cv::LINE_AA);
        cv::putText(annotated, "L2 armor  conf>=0.80", { 25, 58 }, cv::FONT_HERSHEY_SIMPLEX,
            1.0, kBoxColor, 2, cv::LINE_AA);
        const auto plates = detect(engine, rgb);
        int index = 0;
        for (const auto& plate : plates) {
            cv::rectangle(annotated, { static_cast<int>(plate.box.x), static_cast<int>(plate.box.y) },
                { static_cast<int>(plate.box.x + plate.box.width), static_cast<int>(plate.box.y + plate.box.height) },
                kBoxColor, 8);
            const std::string label = "genre=" + std::to_string(plate.genre)
                + " color=" + std::to_string(plate.color) + " "
                + std::to_string(static_cast<int>(plate.confidence * 100.0f)) + "%";
            cv::putText(annotated, label, { static_cast<int>(plate.box.x),
                    std::max(120, static_cast<int>(plate.box.y) - 18) },
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar::all(0), 5, cv::LINE_AA);
            cv::putText(annotated, label, { static_cast<int>(plate.box.x),
                    std::max(120, static_cast<int>(plate.box.y) - 18) },
                cv::FONT_HERSHEY_SIMPLEX, 1.0, kBoxColor, 2, cv::LINE_AA);
            csv << file.filename().string() << "," << index++ << "," << plate.genre << ","
                << plate.color << "," << plate.confidence << "," << plate.box.x << ","
                << plate.box.y << "," << plate.box.width << "," << plate.box.height << "\n";
        }
        cv::imwrite((output / (file.stem().string() + "_L2.jpg")).string(), annotated);
    }
    return 0;
}
} // namespace

auto main(int argc, char** argv) -> int { return main_impl(argc, argv); }

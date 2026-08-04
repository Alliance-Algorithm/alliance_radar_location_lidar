// validate_3layer.cpp
// Three-layer (L1->L2->L3) TensorRT fusion validation on a folder of frames.
// Writes annotated images to ./output/ and prints per-layer latency stats.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>
#include <optional>
#include <print>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "radar_camera/armor_refine.hpp"
#include "radar_camera/tensorrt_inference.hpp"

namespace fs = std::filesystem;
using namespace radar_camera;

// ─── constants ──────────────────────────────────────────────────────────────
static constexpr float L1_CONF                   = 0.30f;
static constexpr int L1_SIDE                     = 1280;
static const std::vector<std::int64_t> DRONE_IDS = { 5, 11 };

static const char* CLASS_NAMES[] = { "hero-B", "eng-B", "inf3-B", "inf4-B", "sentry-B", "drone-B",
    "hero-R", "eng-R", "inf3-R", "inf4-R", "sentry-R", "drone-R" };

// ─── helpers ─────────────────────────────────────────────────────────────────
static cv::Mat letterbox_topleft(const cv::Mat& src, int side, float& scale) {
    scale  = std::min(float(side) / src.cols, float(side) / src.rows);
    int nw = std::max(1, int(std::round(src.cols * scale)));
    int nh = std::max(1, int(std::round(src.rows * scale)));
    cv::Mat canvas(side, side, src.type(), cv::Scalar(0, 0, 0));
    cv::Mat resized;
    cv::resize(src, resized, { nw, nh });
    resized.copyTo(canvas(cv::Rect(0, 0, nw, nh)));
    return canvas;
}

static std::vector<float> to_blob(const cv::Mat& rgb) {
    cv::Mat blob = cv::dnn::blobFromImage(rgb, 1.0 / 255.0, { }, { }, false, false);
    std::vector<float> v(blob.total());
    std::memcpy(v.data(), blob.ptr<float>(), v.size() * sizeof(float));
    return v;
}

struct Detection {
    float x1, y1, x2, y2, conf;
    int cls;
    cv::Rect2f bbox() const { return { x1, y1, x2 - x1, y2 - y1 }; }
};

static std::vector<Detection> parse_l1(
    const std::vector<float>& out, float scale, float threshold) {
    // output: [1,300,6] stored flat -> 1800 floats
    std::vector<Detection> dets;
    constexpr int stride = 6;
    const int num        = static_cast<int>(out.size()) / stride;
    for (int i = 0; i < num; ++i) {
        const float* r = out.data() + i * stride;
        float conf     = r[4];
        int cls        = static_cast<int>(r[5]);
        if (conf < threshold) continue;
        dets.push_back({ r[0] / scale, r[1] / scale, r[2] / scale, r[3] / scale, conf, cls });
    }
    return dets;
}

struct Latency {
    std::vector<double> samples;
    void add(double ms) { samples.push_back(ms); }
    void print(const std::string& name) const {
        if (samples.empty()) {
            std::println("{}: (no calls)", name);
            return;
        }
        auto s = samples;
        std::sort(s.begin(), s.end());
        double mean = std::accumulate(s.begin(), s.end(), 0.0) / s.size();
        std::println("{}  n={}  mean={}ms  p50={}ms  p95={}ms  p99={}ms",
            name, s.size(), mean, s[s.size() * 50 / 100], s[s.size() * 95 / 100],
            s[s.size() * 99 / 100]);
    }
};

static double now_ms() {
    using namespace std::chrono;
    return duration<double, std::milli>(high_resolution_clock::now().time_since_epoch()).count();
}

// ─── main ────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    std::string frames_dir = argc > 1 ? argv[1] : "/workspace/tools/verify_frames";
    std::string out_dir    = argc > 2 ? argv[2] : "/workspace/tools/verify_output";
    std::string model_dir =
        argc > 3 ? argv[3] : "/workspace/ros_ws/install/radar_camera/share/radar_camera/model";

    fs::create_directories(out_dir);

    // ── init engines ──────────────────────────────────────────────────────────
    model_inference::TensorRtInference l1_trt;
    std::string l1_path = model_dir + "/best_fixed_names_1280_fp16.engine";
    if (auto r = l1_trt.init(l1_path); !r) {
        std::println(std::cerr, "L1 init failed: {}", r.error());
        return 1;
    }
    std::println("L1 engine loaded: {}", l1_path);
    std::println("  input_elements={}  output_elements={}", l1_trt.input_elements(),
        l1_trt.output_elements());

    armor_refine::ArmorRefiner refiner;
    armor_refine::ArmorRefineConfig acfg;
    armor_refine::NumberRefineConfig ncfg;
    acfg.armor_model_path  = model_dir + "/shenzhen-0708_fp16.engine";
    acfg.score_threshold   = 0.80f;
    acfg.nms_threshold     = 0.30f;
    ncfg.number_model_path = model_dir + "/armor-number_fp16.engine";
    ncfg.conf_threshold    = 0.80f;
    if (auto r = refiner.init(acfg, ncfg); !r) {
        std::println(std::cerr, "ArmorRefiner init failed: {}", r.error());
        return 1;
    }
    std::println("L2/L3 engines loaded");

    // ── collect frames ────────────────────────────────────────────────────────
    std::vector<fs::path> files;
    for (auto& e : fs::directory_iterator(frames_dir))
        if (e.path().extension() == ".jpg" || e.path().extension() == ".png")
            files.push_back(e.path());
    std::sort(files.begin(), files.end());
    std::println("Processing {} frames...", files.size());

    Latency lat_l1, lat_refine;
    int total_dets = 0, drone_dets = 0;

    // ── CSV ───────────────────────────────────────────────────────────────────
    std::ofstream csv(std::string(out_dir) + "/results.csv");
    csv << "frame,det_idx,l1_class,final_class,l1_conf,u,v\n";

    // ── per-frame loop ────────────────────────────────────────────────────────
    for (size_t fi = 0; fi < files.size(); ++fi) {
        cv::Mat bgr = cv::imread(files[fi].string());
        if (bgr.empty()) {
            std::println(std::cerr, "skip (empty): {}", files[fi].string());
            continue;
        }
        cv::Mat rgb;
        cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);

        // ── L1 ──────────────────────────────────────────────────────────────
        float scale = 1.0f;
        cv::Mat lb  = letterbox_topleft(rgb, L1_SIDE, scale);
        auto blob   = to_blob(lb);

        double t0 = now_ms();
        if (auto r = l1_trt.start(blob.data(), blob.size()); !r) {
            std::println(std::cerr, "L1 start: {}", r.error());
            continue;
        }
        auto l1_wait = l1_trt.wait();
        double l1_ms = now_ms() - t0;
        if (!l1_wait) {
            std::println(std::cerr, "L1 wait: {}", l1_wait.error());
            continue;
        }
        lat_l1.add(l1_ms);

        auto dets = parse_l1(l1_wait->get(), scale, L1_CONF);

        // ── refine + annotate ────────────────────────────────────────────────
        cv::Mat vis = bgr.clone();
        for (size_t di = 0; di < dets.size(); ++di) {
            auto& d = dets[di];
            // Build a Detection-like struct for ArmorRefiner
            detection::Detection det;
            det.id         = d.cls;
            det.confidence = d.conf;
            det.bbox       = d.bbox();

            bool is_drone =
                std::find(DRONE_IDS.begin(), DRONE_IDS.end(), static_cast<std::int64_t>(d.cls))
                != DRONE_IDS.end();

            double rt0 = now_ms();
            refiner.refine(rgb, det, DRONE_IDS, 1.0f, 1.0f);
            lat_refine.add(now_ms() - rt0);

            if (is_drone) ++drone_dets;
            ++total_dets;

            const int l1_cls    = d.cls;
            const int final_cls = det.id;
            const char* l1_name = (l1_cls >= 0 && l1_cls < 12) ? CLASS_NAMES[l1_cls] : "?";
            const char* final_name =
                (final_cls >= 0 && final_cls < 12) ? CLASS_NAMES[final_cls] : "?";

            // color: green = changed, gray = unchanged
            cv::Scalar color =
                (final_cls != l1_cls) ? cv::Scalar(0, 220, 0) : cv::Scalar(160, 160, 160);

            int x1 = std::max(0, int(d.x1));
            int y1 = std::max(0, int(d.y1));
            int x2 = std::min(bgr.cols - 1, int(d.x2));
            int y2 = std::min(bgr.rows - 1, int(d.y2));
            cv::rectangle(vis, { x1, y1 }, { x2, y2 }, color, 2);

            std::string label = std::string(final_name)
                + (final_cls != l1_cls ? std::string(" (L1:") + l1_name + ")" : "") + " "
                + std::to_string(int(d.conf * 100)) + "%";
            cv::putText(
                vis, label, { x1, std::max(0, y1 - 6) }, cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);

            float cx = (d.x1 + d.x2) / 2.0f;
            float cy = (d.y1 + d.y2) / 2.0f;
            csv << files[fi].filename().string() << "," << di << "," << l1_name << "," << final_name
                << "," << d.conf << "," << cx << "," << cy << "\n";
        }

        cv::imwrite(out_dir + "/" + files[fi].filename().string(), vis);

        if ((fi + 1) % 10 == 0) {
            std::print("  {}/{}\n", fi + 1, files.size());
            std::flush(std::cout);
        }
    }

    // ── report ────────────────────────────────────────────────────────────────
    std::println("\n=== latency (GPU TensorRT, H2D+infer+D2H) ===");
    lat_l1.print("L1 (1280x1280)  ");
    lat_refine.print("L2+L3 per-det   ");
    std::println("\n=== detection stats ===\ntotal_dets={}  drones={}  robots={}",
        total_dets, drone_dets, total_dets - drone_dets);
    std::println("annotated images -> {}", out_dir);
    std::println("CSV              -> {}/results.csv", out_dir);
    return 0;
}

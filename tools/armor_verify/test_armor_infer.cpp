#include <cassert>
#include <cmath>
#include <iostream>
#include <print>

#include <opencv2/core.hpp>

#include "radar_camera/armor_infer.hpp"

using namespace radar_camera::armor_infer;

void test_l2_id() {
    assert(l2_id(1, 1) == 0);   // eng-R
    assert(l2_id(1, 2) == 6);   // eng-B
    assert(l2_id(6, 1) == 4);   // sentry-R
    assert(l2_id(0, 1) == std::nullopt); // genre 0 无映射
    assert(l2_id(2, 0) == std::nullopt); // color 0 无效
    assert(l2_id(7, 1) == std::nullopt); // genre 越界
}

void test_l3_id() {
    assert(l3_id(0) == 6);      // B1 -> hero-B
    assert(l3_id(4) == 10);     // BS -> sentry-B
    assert(l3_id(5) == 0);      // R1 -> hero-R
    assert(l3_id(8) == 3);      // R4 -> inf4-R
    assert(l3_id(9) == std::nullopt);
}

void test_letterbox() {
    cv::Mat src(100, 200, CV_8UC3);
    float scale; int px; int py;
    auto lb = letterbox(src, 1280, false, scale, px, py);
    assert(lb.cols == 1280 && lb.rows == 1280);
    assert(px == 0 && py == 0);
    assert(std::abs(scale - 6.4f) < 1e-4f);
    auto lbc = letterbox(src, 224, true, scale, px, py);
    assert(lbc.cols == 224 && lbc.rows == 224);
    assert(px == 0 && py == 56); // scale=1.12, w=224 占满, h=112 居中偏移 56
}

void test_iou() {
    assert(iou(cv::Rect2f(0, 0, 10, 10), cv::Rect2f(0, 0, 10, 10)) == 1.0f);
    assert(iou(cv::Rect2f(0, 0, 10, 10), cv::Rect2f(20, 20, 10, 10)) == 0.0f);
    assert(std::abs(iou(cv::Rect2f(0, 0, 10, 10), cv::Rect2f(5, 0, 10, 10)) - 0.333333f) < 1e-4f);
}

void test_decode_l1() {
    // scale=1: 两个同类检测取高分；宽高比过滤；低置信度过滤
    std::vector<float> raw {
        10, 10, 110, 210, 0.9f, 0,   // hero-R, ratio 2.0 通过
        20, 20, 120, 220, 0.8f, 0,   // 同类低分被丢弃
        0, 0, 100, 50, 0.99f, 5,     // drone-R, ratio 2.0 通过
        0, 0, 100, 20, 0.99f, 7,     // ratio 5.0 > 3.0 被过滤 (非无人机)
        0, 0, 10, 10, 0.1f, 1,       // 低置信度被过滤
    };
    auto dets = decode_l1(raw, 1.0f);
    assert(dets.size() == 2);
    assert(dets[0].id == 0 && std::abs(dets[0].conf - 0.9f) < 1e-5f);
    assert(dets[1].id == 5 && std::abs(dets[1].conf - 0.99f) < 1e-5f);
}

void test_decode_l2() {
    // 手工构造一条 22 列检测: 角点(5,5)(15,5)(15,10)(5,10), obj=0.9, color=[0.1,0.8,0.05,0.05] -> red(1), genre=[0.9,...] -> genre 0
    std::vector<float> raw(22, 0.0f);
    raw[0] = 5; raw[1] = 5; raw[2] = 15; raw[3] = 5; raw[4] = 15; raw[5] = 10; raw[6] = 5; raw[7] = 10;
    raw[8] = 0.9f;
    raw[9] = 0.1f; raw[10] = 0.8f; raw[11] = 0.05f; raw[12] = 0.05f;
    raw[13] = 0.9f; // genre 0
    auto plate = decode_l2(raw, cv::Rect2f(100, 100, 0, 0), 1.0f, 0, 0);
    assert(plate.has_value());
    assert(plate->color == 1 && plate->genre == 0);
    assert(std::abs(plate->box.x - 105.0f) < 1e-3f && std::abs(plate->box.y - 105.0f) < 1e-3f);
    assert(std::abs(plate->box.width - 10.0f) < 1e-3f && std::abs(plate->box.height - 5.0f) < 1e-3f);
    assert(plate->corners.size() == 4);
    // roi 偏移 + 缩放: scale=2, px=10, py=20, roi=(100,100)
    auto plate2 = decode_l2(raw, cv::Rect2f(100, 100, 0, 0), 2.0f, 10, 20);
    assert(plate2.has_value());
    // box: min corner (5,5) -> ((5-10)/2+100, (5-20)/2+100) = (97.5, 92.5), 尺寸 (10/2, 5/2) = (5, 2.5)
    assert(std::abs(plate2->box.x - 97.5f) < 1e-3f && std::abs(plate2->box.y - 92.5f) < 1e-3f);
    assert(std::abs(plate2->box.width - 5.0f) < 1e-3f && std::abs(plate2->box.height - 2.5f) < 1e-3f);
    assert(std::abs(plate2->corners[0].x - 97.5f) < 1e-3f && std::abs(plate2->corners[0].y - 92.5f) < 1e-3f);
    // 低置信度: sigmoid(-10) ~= 4.5e-5 << kL2Conf
    std::vector<float> raw_low = raw;
    raw_low[8] = -10.0f;
    assert(!decode_l2(raw_low, cv::Rect2f(0, 0, 0, 0), 1.0f, 0, 0).has_value());
}

void test_names() {
    assert(std::string(l1_names(0)) == "hero-R");
    assert(std::string(l1_names(11)) == "drone-B");
    assert(std::string(l3_names(0)) == "B1");
    assert(std::string(l3_names(8)) == "R4");
}

int main() {
    test_l2_id(); test_l3_id(); test_letterbox(); test_iou();
    test_decode_l1(); test_decode_l2(); test_names();
    std::println("all armor_infer tests passed");
    return 0;
}

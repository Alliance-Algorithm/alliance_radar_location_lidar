// test_color_voxel_map.cpp — Unit tests for ColorVoxelMap
//
// Follows the Task 2 brief: best-observation replacement logic and
// PCL PointXYZRGB export. Additional edge-case tests cover multiple
// voxels, lower-quality rejection, empty-map behaviour, and PCD save.

#include <gtest/gtest.h>

#include <atomic>
#include <cstdio>
#include <fstream>
#include <set>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// The class under test — will fail to compile before implementation.
#include "radar_fast_livo2_rgb/color_voxel_map.hpp"

using namespace radar::fast_livo2::rgb;

// ════════════════════════════════════════════════════════════════════════
// Best-observation replacement (brief Step 1)
// ════════════════════════════════════════════════════════════════════════

TEST(ColorVoxelMap, ReplacesVoxelColorOnlyWithHigherQualityObservation)
{
    ColorVoxelMap map(0.10);  // 10 cm voxels

    // All three points fall into the same voxel (0.02, 0.02, 0.02)
    // because floor(0.02/0.10)=0, floor(0.03/0.10)=0, floor(0.04/0.10)=0.
    map.insert_if_better(Eigen::Vector3d{0.02, 0.02, 0.02}, 0x112233U, 0.4);
    map.insert_if_better(Eigen::Vector3d{0.03, 0.02, 0.02}, 0x445566U, 0.3);
    map.insert_if_better(Eigen::Vector3d{0.04, 0.02, 0.02}, 0x778899U, 0.9);

    const auto cloud = map.to_point_cloud();
    ASSERT_EQ(cloud.size(), 1U);
    // The best quality (0.9) has colour 0x778899 — that should be stored.
    EXPECT_EQ(unpack_rgb(cloud.front()), 0x778899U);
}

// ════════════════════════════════════════════════════════════════════════
// PCL PointXYZRGB export (brief Step 1)
// ════════════════════════════════════════════════════════════════════════

TEST(ColorVoxelMap, ExportsPclPointXyzRgb)
{
    ColorVoxelMap map(0.10);
    map.insert_if_better(Eigen::Vector3d{1.0, 2.0, 3.0}, 0xA0B0C0U, 1.0);

    const auto cloud = map.to_point_cloud();
    ASSERT_EQ(cloud.size(), 1U);
    EXPECT_FLOAT_EQ(cloud.front().x, 1.0F);
    EXPECT_FLOAT_EQ(cloud.front().y, 2.0F);
    EXPECT_FLOAT_EQ(cloud.front().z, 3.0F);
    EXPECT_EQ(unpack_rgb(cloud.front()), 0xA0B0C0U);
}

// ════════════════════════════════════════════════════════════════════════
// Edge cases
// ════════════════════════════════════════════════════════════════════════

TEST(ColorVoxelMap, LowerQualityDoesNotReplaceBetterObservation)
{
    ColorVoxelMap map(0.10);

    map.insert_if_better(Eigen::Vector3d{0.05, 0.05, 0.05}, 0xFF0000U, 0.8);
    // Lower quality — should NOT replace.
    map.insert_if_better(Eigen::Vector3d{0.06, 0.05, 0.05}, 0x00FF00U, 0.5);

    const auto cloud = map.to_point_cloud();
    ASSERT_EQ(cloud.size(), 1U);
    EXPECT_EQ(unpack_rgb(cloud.front()), 0xFF0000U);
}

TEST(ColorVoxelMap, EqualQualityDoesNotReplace)
{
    ColorVoxelMap map(0.10);

    map.insert_if_better(Eigen::Vector3d{0.05, 0.05, 0.05}, 0xFF0000U, 0.8);
    // Equal quality — should NOT replace (strictly greater required).
    map.insert_if_better(Eigen::Vector3d{0.06, 0.05, 0.05}, 0x00FF00U, 0.8);

    const auto cloud = map.to_point_cloud();
    ASSERT_EQ(cloud.size(), 1U);
    EXPECT_EQ(unpack_rgb(cloud.front()), 0xFF0000U);
}

TEST(ColorVoxelMap, MultipleVoxelsInDifferentCells)
{
    ColorVoxelMap map(0.10);

    // Three points in different voxels (separated by >= voxel size).
    map.insert_if_better(Eigen::Vector3d{0.01, 0.01, 0.01}, 0x111111U, 0.5);
    map.insert_if_better(Eigen::Vector3d{0.11, 0.01, 0.01}, 0x222222U, 0.5);
    map.insert_if_better(Eigen::Vector3d{0.01, 0.11, 0.01}, 0x333333U, 0.5);

    const auto cloud = map.to_point_cloud();
    ASSERT_EQ(cloud.size(), 3U);
    // Each voxel should have distinct colour.
    std::set<uint32_t> colours;
    for (const auto& pt : cloud) {
        colours.insert(unpack_rgb(pt));
    }
    EXPECT_EQ(colours.size(), 3U);
}

TEST(ColorVoxelMap, EmptyMapReturnsEmptyCloud)
{
    ColorVoxelMap map(0.10);
    const auto cloud = map.to_point_cloud();
    EXPECT_EQ(cloud.size(), 0U);
}

TEST(ColorVoxelMap, SaveBinaryPcdProducesNonEmptyFile)
{
    ColorVoxelMap map(0.10);
    map.insert_if_better(Eigen::Vector3d{1.0, 2.0, 3.0}, 0xAABBCCU, 1.0);

    const std::string tmp_file = "/tmp/test_color_voxel_map.pcd";
    int save_result = map.save_binary_pcd(tmp_file);

    // save_binary_pcd must report success (0).
    EXPECT_EQ(save_result, 0);

    // Verify the file exists and has reasonable size.
    std::ifstream f(tmp_file, std::ios::binary | std::ios::ate);
    ASSERT_TRUE(f.good());
    std::streamsize file_size = f.tellg();
    f.close();
    EXPECT_GT(file_size, 0);

    // Clean up.
    std::remove(tmp_file.c_str());
}

// ════════════════════════════════════════════════════════════════════════
// Thread safety: concurrent insert and export must not crash
// ════════════════════════════════════════════════════════════════════════

TEST(ColorVoxelMap, ConcurrentInsertAndExportDoesNotCrash)
{
    ColorVoxelMap map(0.10);
    std::atomic<bool> running { true };
    std::atomic<int> insert_count { 0 };

    // Thread 1: continuously inserts unique points
    std::thread inserter([&]() {
        int i = 0;
        while (running.load(std::memory_order_relaxed)) {
            double x = static_cast<double>(i) * 0.01;
            double y = static_cast<double>(i % 100) * 0.01;
            double z = static_cast<double>(i / 100) * 0.01;
            uint32_t colour = 0xFF0000U | (static_cast<uint32_t>(i) & 0xFFFF);
            map.insert_if_better(Eigen::Vector3d{x, y, z}, colour, 0.5);
            ++i;
            insert_count.store(i, std::memory_order_relaxed);
        }
    });

    // Thread 2: repeatedly exports point cloud while insertions happen
    std::thread exporter([&]() {
        for (int iter = 0; iter < 200; ++iter) {
            const auto cloud = map.to_point_cloud();
            // Must not crash; verify cloud is well-formed
            ASSERT_NO_THROW(cloud.size());
        }
    });

    // Let threads run concurrently for ~50 ms worth of iterations
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    running.store(false, std::memory_order_relaxed);

    inserter.join();
    exporter.join();

    // Final cloud should be valid and contain at least some points
    const auto final_cloud = map.to_point_cloud();
    EXPECT_GT(final_cloud.size(), 0U);
    EXPECT_LE(final_cloud.size(), static_cast<std::size_t>(insert_count.load()));
}

// ════════════════════════════════════════════════════════════════════════
// Regression: raw PCL byte layout preserves R/G/B channels correctly
// ════════════════════════════════════════════════════════════════════════

TEST(ColorVoxelMap, PclByteLayoutPreservesPureRed)
{
    // Manually construct a PointXYZRGB whose float stores the uint32
    // 0x00FF0000 — PCL convention: R at bits 16-23, G at bits 8-15,
    // B at bits 0-7 of the underlying uint32. 0x00FF0000 thus means
    // R=255, G=0, B=0 (pure red).
    pcl::PointXYZRGB pt;
    pt.x = 0.0f; pt.y = 0.0f; pt.z = 0.0f;
    uint32_t raw_rgba = 0x00FF0000U;
    std::memcpy(&pt.rgb, &raw_rgba, sizeof(pt.rgb));

    // unpack_rgb must see pure red, i.e. 0x00FF0000 in our convention.
    EXPECT_EQ(unpack_rgb(pt), 0x00FF0000U);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

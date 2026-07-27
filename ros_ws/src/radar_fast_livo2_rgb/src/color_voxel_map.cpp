// color_voxel_map.cpp — Best-quality RGB voxel storage
//
// Implements a separate RGB voxel map that stores the best-quality colour
// observation per voxel. Uses a hash key based on floor-divided world
// coordinates. Thread-safe for concurrent insert_if_better calls.

#include "radar_fast_livo2_rgb/color_voxel_map.hpp"

#include <cmath>
#include <cstdint>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace radar::fast_livo2::rgb {

// -----------------------------------------------------------------------
// Internal: Teschner-style hash combination using large primes.
// These constants are implementation details, not part of the interface.
// -----------------------------------------------------------------------
namespace {

constexpr int64_t kHashP1 = 73856093;
constexpr int64_t kHashP2 = 19349663;
constexpr int64_t kHashP3 = 83492791;

} // anonymous namespace

// -----------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------

ColorVoxelMap::ColorVoxelMap(double voxel_size)
    : voxel_size_(voxel_size)
{
    // voxel_size is assumed positive; no runtime check in release builds.
}

// -----------------------------------------------------------------------
// Key computation
// -----------------------------------------------------------------------

VoxelKey ColorVoxelMap::make_key(const Eigen::Vector3d& position) const
{
    // Floor-divide each coordinate to obtain integer voxel indices.
    // Using std::floor ensures deterministic negative-coordinate handling.
    int64_t ix = static_cast<int64_t>(std::floor(position.x() / voxel_size_));
    int64_t iy = static_cast<int64_t>(std::floor(position.y() / voxel_size_));
    int64_t iz = static_cast<int64_t>(std::floor(position.z() / voxel_size_));

    // Standard Teschner et al. hash for spatial hashing.
    return (ix * kHashP1) ^ (iy * kHashP2) ^ (iz * kHashP3);
}

// -----------------------------------------------------------------------
// Insert / update
// -----------------------------------------------------------------------

void ColorVoxelMap::insert_if_better(
    const Eigen::Vector3d& position,
    uint32_t rgb,
    double quality)
{
    VoxelKey key = make_key(position);

    std::lock_guard<std::mutex> lock(mutex_);

    auto it = voxels_.find(key);
    if (it == voxels_.end()) {
        // No existing voxel — insert unconditionally.
        voxels_.emplace(key, ColorVoxel {
            .position = position.cast<float>(),
            .rgb = rgb,
            .quality = quality
        });
    } else if (quality > it->second.quality) {
        // New observation is strictly better: replace colour, quality,
        // and position (keeps the cloud fresh as poses drift).
        it->second.position = position.cast<float>();
        it->second.rgb = rgb;
        it->second.quality = quality;
    }
    // Otherwise: keep the existing (better) observation.
}

// -----------------------------------------------------------------------
// PCL export (unlocked implementation — caller must hold mutex_)
// -----------------------------------------------------------------------

pcl::PointCloud<pcl::PointXYZRGB> ColorVoxelMap::to_point_cloud_impl() const
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    cloud.reserve(voxels_.size());

    for (const auto& [key, voxel] : voxels_) {
        (void)key; // unused in the loop body
        pcl::PointXYZRGB pt;
        pt.x = voxel.position.x();
        pt.y = voxel.position.y();
        pt.z = voxel.position.z();
        pt.rgb = pack_rgb_for_pcl(voxel.rgb);
        cloud.push_back(pt);
    }

    return cloud;
}

// -----------------------------------------------------------------------
// PCL export (public, thread-safe)
// -----------------------------------------------------------------------

pcl::PointCloud<pcl::PointXYZRGB> ColorVoxelMap::to_point_cloud() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return to_point_cloud_impl();
}

// -----------------------------------------------------------------------
// PCD save (public, thread-safe)
// -----------------------------------------------------------------------

int ColorVoxelMap::save_binary_pcd(const std::string& filename) const
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cloud = to_point_cloud_impl();
    }
    return pcl::io::savePCDFileBinary(filename, cloud);
}

// -----------------------------------------------------------------------
// Size query
// -----------------------------------------------------------------------

std::size_t ColorVoxelMap::size() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return voxels_.size();
}

} // namespace radar::fast_livo2::rgb

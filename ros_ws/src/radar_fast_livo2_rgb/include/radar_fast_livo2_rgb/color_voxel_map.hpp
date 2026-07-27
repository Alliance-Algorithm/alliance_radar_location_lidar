// color_voxel_map.hpp — Best-quality RGB voxel storage
//
// Task 2 of the FAST-LIVO2 RGB map/replay plan.
// A completely separate RGB voxel map that stores the best-quality colour
// observation per voxel. Uses a hash key based on floor-divided world
// coordinates. Never touches FAST-LIVO2's own voxel map structures.
//
// Consumes: valid projected BGR candidate + scalar quality (from Task 1).
// Produces: pcl::PointCloud<pcl::PointXYZRGB> export + binary PCD save.

#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <unordered_map>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace radar::fast_livo2::rgb {

/// A single colour voxel storing the best-quality observation.
struct ColorVoxel {
    Eigen::Vector3f position;  ///< World position (float for dense storage)
    uint32_t rgb;              ///< Packed 0xRRGGBB colour
    double quality;            ///< Best quality score seen so far
};

/// Hash key for voxel indexing (int64_t from floor-divided coordinates).
using VoxelKey = int64_t;

/// Separate RGB voxel map — never touches FAST-LIVO2 voxel map.
/// Thread-safe: all public methods lock the mutex internally.
class ColorVoxelMap {
public:
    /// Construct with a configured voxel size in metres.
    explicit ColorVoxelMap(double voxel_size);

    /// Move constructor — transfers all voxels; mutex is default-constructed
    /// in the moved-from object (which remains in a valid empty state).
    ColorVoxelMap(ColorVoxelMap&& other) noexcept
        : voxel_size_(other.voxel_size_)
        , voxels_(std::move(other.voxels_))
        , mutex_()   // default-constructed; moved-from object has its own mutex
    {}

    /// Insert or update a colour observation at the given world position.
    ///
    /// If a voxel already exists at that coordinate, the colour is replaced
    /// only if \p quality is strictly greater than the stored quality.
    /// The position stored is the latest observation's position.
    ///
    /// \param position  World position of the point (double for precision).
    /// \param rgb       Packed 0xRRGGBB colour (e.g. 0xA0B0C0).
    /// \param quality   Quality score for this observation.
    void insert_if_better(const Eigen::Vector3d& position,
                          uint32_t rgb, double quality);

    /// Export the entire voxel map as a PCL point cloud.
    ///
    /// Each voxel becomes one pcl::PointXYZRGB with xyz from the stored
    /// position and rgb from the packed colour.  Thread-safe.
    pcl::PointCloud<pcl::PointXYZRGB> to_point_cloud() const;

    /// Export and immediately save as binary PCD.
    ///
    /// Convenience wrapper around to_point_cloud() +
    /// pcl::io::savePCDFileBinary().  Thread-safe.
    /// \return 0 on success, negative on I/O failure (from pcl::io::savePCDFileBinary).
    int save_binary_pcd(const std::string& filename) const;

    /// Return the number of occupied voxels (for diagnostic use).
    std::size_t size() const;

    /// Visit every voxel under the internal mutex, preserving per-voxel
    /// quality scores.  The callback receives (position, packed_rgb, quality).
    /// This is the safe way to merge per-voxel qualities from one map into
    /// another.
    template <typename Visitor>
    void for_each_voxel(Visitor&& visitor) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto& [key, voxel] : voxels_) {
            (void)key;
            visitor(voxel.position.cast<double>(),
                    voxel.rgb,
                    voxel.quality);
        }
    }

private:
    /// Compute the hash key for a world position by floor-dividing each
    /// coordinate by voxel_size_ and combining with Teschner-style hashing.
    VoxelKey make_key(const Eigen::Vector3d& position) const;

    /// Build point cloud without locking (caller must hold mutex_).
    pcl::PointCloud<pcl::PointXYZRGB> to_point_cloud_impl() const;

    double voxel_size_;
    std::unordered_map<VoxelKey, ColorVoxel> voxels_;
    mutable std::mutex mutex_;
};

// -----------------------------------------------------------------------
// Free utility functions for RGB packing/unpacking
// -----------------------------------------------------------------------

/// Pack 0xRRGGBB into the PCL rgb float.
///
/// PCL stores the `rgba` uint32 with R at bits 16-23, G at bits 8-15,
/// B at bits 0-7 — bit-identical to our 0xRRGGBB convention, so the
/// value is memcpy'd directly with no channel swapping.
inline float pack_rgb_for_pcl(uint32_t rgb)
{
    float result;
    std::memcpy(&result, &rgb, sizeof(result));
    return result;
}

/// Unpack a PCL PointXYZRGB rgb float back into 0xRRGGBB.
/// Intended for test assertions.  Inverse of pack_rgb_for_pcl.
inline uint32_t unpack_rgb(const pcl::PointXYZRGB& pt)
{
    uint32_t rgb;
    std::memcpy(&rgb, &pt.rgb, sizeof(rgb));
    return rgb;
}

} // namespace radar::fast_livo2::rgb

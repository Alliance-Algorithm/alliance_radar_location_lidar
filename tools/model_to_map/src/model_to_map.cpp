#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <expected>
#include <fcntl.h>
#include <iostream>
#include <numeric>
#include <optional>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

namespace {

struct Roi {
    Eigen::Vector3f min;
    Eigen::Vector3f max;
};

struct Args {
    std::string input_path;
    std::string output_path;
    double density        = 10000.0;
    double scale          = 1.0;
    double voxel_leaf     = 0.0;
    std::optional<Roi> roi;
    bool write_shm        = false;
};

struct TriangleRecord {
    Eigen::Vector3f a;
    Eigen::Vector3f b;
    Eigen::Vector3f c;
    float area = 0.0f;
};

float triangle_area(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c) {
    return 0.5f * ((b - a).cross(c - a)).norm();
}

auto parse_args(int argc, char** argv) -> std::expected<Args, std::string> {
    if (argc < 3) {
        return std::unexpected(
            "Usage: " + std::string(argv[0]) + " <input.fbx> <output.pcd> [options]\n"
            "Options:\n"
            "  --density <float>   points per m^2 (default 10000.0)\n"
            "  --scale <float>     coordinate scale factor (e.g. 0.01 for cm→m, default 1.0)\n"
            "  --voxel <float>     VoxelGrid leaf size, 0=skip (default 0)\n"
            "  --roi <6 floats>    x_min,x_max,y_min,y_max,z_min,z_max\n"
            "  --shm               write point cloud to /pointcloud_frame SHM\n");
    }

    Args args;
    args.input_path  = argv[1];
    args.output_path = argv[2];

    for (int i = 3; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--density" && i + 1 < argc) {
            args.density = std::stod(argv[++i]);
        } else if (arg == "--voxel" && i + 1 < argc) {
            args.voxel_leaf = std::stod(argv[++i]);
        } else if (arg == "--scale" && i + 1 < argc) {
            args.scale = std::stod(argv[++i]);
        } else if (arg == "--shm") {
            args.write_shm = true;
        } else if (arg == "--roi" && i + 6 < argc) {
            args.roi = Roi {
                .min = Eigen::Vector3f(
                    std::stof(argv[i + 1]), std::stof(argv[i + 3]), std::stof(argv[i + 5])),
                .max = Eigen::Vector3f(
                    std::stof(argv[i + 2]), std::stof(argv[i + 4]), std::stof(argv[i + 6])),
            };
            i += 6;
        } else {
            return std::unexpected("Unknown argument: " + arg);
        }
    }
    return args;
}

Eigen::Matrix4f to_matrix(const aiMatrix4x4& matrix) {
    Eigen::Matrix4f result;
    result << matrix.a1, matrix.a2, matrix.a3, matrix.a4, matrix.b1, matrix.b2, matrix.b3,
        matrix.b4, matrix.c1, matrix.c2, matrix.c3, matrix.c4, matrix.d1, matrix.d2, matrix.d3,
        matrix.d4;
    return result;
}

Eigen::Vector3f transform_point(const Eigen::Matrix4f& transform, const aiVector3D& point) {
    const Eigen::Vector4f homogeneous(point.x, point.y, point.z, 1.0f);
    const Eigen::Vector4f transformed = transform * homogeneous;
    return transformed.head<3>() / transformed.w();
}

void collect_triangles(const aiNode* node, const aiScene* scene,
                       const Eigen::Matrix4f& parent_transform,
                       std::vector<TriangleRecord>& triangles) {
    const Eigen::Matrix4f current_transform = parent_transform * to_matrix(node->mTransformation);
    for (unsigned int i = 0; i < node->mNumMeshes; ++i) {
        const auto* mesh = scene->mMeshes[node->mMeshes[i]];
        for (unsigned int f = 0; f < mesh->mNumFaces; ++f) {
            const aiFace& face = mesh->mFaces[f];
            if (face.mNumIndices != 3) {
                continue;
            }
            const Eigen::Vector3f a = transform_point(current_transform, mesh->mVertices[face.mIndices[0]]);
            const Eigen::Vector3f b = transform_point(current_transform, mesh->mVertices[face.mIndices[1]]);
            const Eigen::Vector3f c = transform_point(current_transform, mesh->mVertices[face.mIndices[2]]);
            const float area = triangle_area(a, b, c);
            if (area > 0.0f) {
                triangles.push_back({a, b, c, area});
            }
        }
    }
    for (unsigned int i = 0; i < node->mNumChildren; ++i) {
        collect_triangles(node->mChildren[i], scene, current_transform, triangles);
    }
}

auto grid_point(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c,
    int subdivision, int i, int j) -> Eigen::Vector3f {
    const float fi = static_cast<float>(i) / static_cast<float>(subdivision);
    const float fj = static_cast<float>(j) / static_cast<float>(subdivision);
    return a + (b - a) * fi + (c - a) * fj;
}

/// Pack absolute normal components into PCL rgb float.
/// Mapping: r=|nx|, g=|nz|, b=|ny| → ground (n·up) green, walls red/blue.
inline float pack_rgb(const Eigen::Vector3f& normal) {
    const auto r = static_cast<uint8_t>(std::abs(normal.x()) * 255.0f);
    const auto g = static_cast<uint8_t>(std::abs(normal.z()) * 255.0f);
    const auto b = static_cast<uint8_t>(std::abs(normal.y()) * 255.0f);
    const uint32_t packed = (static_cast<uint32_t>(r) << 16)
                          | (static_cast<uint32_t>(g) << 8)
                          | static_cast<uint32_t>(b);
    float result;
    std::memcpy(&result, &packed, sizeof(float));
    return result;
}

void append_subdivided_triangle_samples(const TriangleRecord& triangle, std::size_t sample_count,
    float scale, pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud) {
    if (sample_count == 0) {
        return;
    }

    const Eigen::Vector3f a = triangle.a * scale;
    const Eigen::Vector3f b = triangle.b * scale;
    const Eigen::Vector3f c = triangle.c * scale;

    // Face normal (direction preserved under uniform scale; area>0 ensures non-degenerate)
    const Eigen::Vector3f normal = ((b - a).cross(c - a)).normalized();
    const float packed = pack_rgb(normal);

    auto make_point = [&](const Eigen::Vector3f& pos) -> pcl::PointXYZRGBNormal {
        pcl::PointXYZRGBNormal pt;
        pt.x = pos.x();
        pt.y = pos.y();
        pt.z = pos.z();
        pt.rgb       = packed;
        pt.normal_x  = normal.x();
        pt.normal_y  = normal.y();
        pt.normal_z  = normal.z();
        return pt;
    };

    if (sample_count == 1) {
        cloud.push_back(make_point((a + b + c) / 3.0f));
        return;
    }

    const int subdivision = std::max(1,
        static_cast<int>(std::ceil(std::sqrt(static_cast<double>(sample_count)))));
    std::vector<Eigen::Vector3f> candidates;
    candidates.reserve(static_cast<std::size_t>(subdivision * subdivision));

    for (int i = 0; i < subdivision; ++i) {
        for (int j = 0; j < subdivision - i; ++j) {
            const Eigen::Vector3f p00 = grid_point(a, b, c, subdivision, i, j);
            const Eigen::Vector3f p10 = grid_point(a, b, c, subdivision, i + 1, j);
            const Eigen::Vector3f p01 = grid_point(a, b, c, subdivision, i, j + 1);
            candidates.push_back((p00 + p10 + p01) / 3.0f);

            if (i + j + 1 < subdivision) {
                const Eigen::Vector3f p11 = grid_point(a, b, c, subdivision, i + 1, j + 1);
                candidates.push_back((p10 + p11 + p01) / 3.0f);
            }
        }
    }

    if (candidates.size() <= sample_count) {
        for (const auto& point : candidates) {
            cloud.push_back(make_point(point));
        }
        return;
    }

    const double stride = static_cast<double>(candidates.size()) / static_cast<double>(sample_count);
    for (std::size_t i = 0; i < sample_count; ++i) {
        const auto index = std::min(
            static_cast<std::size_t>(std::floor((static_cast<double>(i) + 0.5) * stride)),
            candidates.size() - 1);
        cloud.push_back(make_point(candidates[index]));
    }
}

// ── Shared memory writer ───────────────────────────────────────────

/// Per-point layout in SHM: 28 bytes.
struct ShmPoint {
    float x, y, z;
    uint8_t r, g, b, a;
    float nx, ny, nz;
};
static_assert(sizeof(ShmPoint) == 28, "ShmPoint must be 28 bytes");

/// Header layout: 64 bytes, with atomic counters for lock-free consumer reads.
struct ShmHeader {
    uint32_t magic       = 0x50434446;  // "PCDF"
    uint32_t version     = 1;
    uint32_t point_count = 0;
    uint32_t max_points  = 0;
    std::atomic<uint32_t> frame_seq{0};
    std::atomic<uint32_t> write_idx{0};
    uint32_t stride      = sizeof(ShmPoint);
    uint8_t _pad[36]     = {};
};
static_assert(sizeof(ShmHeader) == 64, "ShmHeader must be 64 bytes");

/// Write point cloud data to /pointcloud_frame shared memory.
/// Coordinate mapping: SHM(x,y,z) = PCD(x,z,y); normals: (nx, nz, ny).
int write_to_shm(const pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud) {
    constexpr const char* shm_name = "/pointcloud_frame";

    if (cloud.empty()) {
        std::cerr << "[model_to_map] SHM: point cloud is empty" << std::endl;
        return 1;
    }

    const uint32_t n_points = static_cast<uint32_t>(cloud.size());
    const size_t shm_size = sizeof(ShmHeader) + 2ULL * n_points * sizeof(ShmPoint);

    shm_unlink(shm_name);
    const int fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
    if (fd < 0) {
        std::cerr << "[model_to_map] SHM: shm_open failed: " << std::strerror(errno) << std::endl;
        return 1;
    }
    if (ftruncate(fd, static_cast<off_t>(shm_size)) != 0) {
        std::cerr << "[model_to_map] SHM: ftruncate failed: " << std::strerror(errno) << std::endl;
        close(fd);
        shm_unlink(shm_name);
        return 1;
    }

    void* addr = mmap(nullptr, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (addr == MAP_FAILED) {
        std::cerr << "[model_to_map] SHM: mmap failed: " << std::strerror(errno) << std::endl;
        close(fd);
        shm_unlink(shm_name);
        return 1;
    }
    close(fd);

    auto* header = static_cast<ShmHeader*>(addr);
    header->magic       = 0x50434446;
    header->version     = 1;
    header->point_count = n_points;
    header->max_points  = n_points;
    header->stride      = sizeof(ShmPoint);
    std::memset(header->_pad, 0, sizeof(header->_pad));
    header->write_idx.store(0, std::memory_order_relaxed);

    auto* buf = reinterpret_cast<ShmPoint*>(static_cast<char*>(addr) + sizeof(ShmHeader));
    for (uint32_t i = 0; i < n_points; ++i) {
        const auto& pt = cloud.points[i];

        uint32_t rgb_packed;
        std::memcpy(&rgb_packed, &pt.rgb, sizeof(uint32_t));

        buf[i].x  = pt.x;
        buf[i].y  = pt.z;
        buf[i].z  = pt.y;
        buf[i].r  = static_cast<uint8_t>((rgb_packed >> 16) & 0xFF);
        buf[i].g  = static_cast<uint8_t>((rgb_packed >> 8)  & 0xFF);
        buf[i].b  = static_cast<uint8_t>(rgb_packed         & 0xFF);
        buf[i].a  = 255;
        buf[i].nx = pt.normal_x;
        buf[i].ny = pt.normal_z;
        buf[i].nz = pt.normal_y;
    }

    std::atomic_thread_fence(std::memory_order_release);
    header->frame_seq.store(1, std::memory_order_release);

    munmap(addr, shm_size);
    std::cout << "[model_to_map] SHM: wrote " << n_points << " points to "
              << shm_name << " (" << shm_size << " bytes)" << std::endl;
    return 0;
}

/// Load PCD from disk and write to SHM (standalone PCD→SHM path).
int write_to_shm_file(const std::string& pcd_path) {
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(pcd_path, *cloud) != 0) {
        std::cerr << "[model_to_map] SHM: failed to load " << pcd_path << std::endl;
        return 1;
    }
    return write_to_shm(*cloud);
}

} // namespace

int main(int argc, char** argv) {
    const auto args_result = parse_args(argc, argv);
    if (!args_result) {
        std::cerr << args_result.error() << std::endl;
        return 1;
    }
    const auto args = *args_result;

    // ── PCD-only path: load existing PCD → SHM ──────────────────
    const bool is_pcd_input = args.input_path.size() >= 4
        && args.input_path.compare(args.input_path.size() - 4, 4, ".pcd") == 0;

    if (is_pcd_input) {
        if (!args.write_shm) {
            std::cerr << "[model_to_map] PCD input requires --shm" << std::endl;
            return 1;
        }
        return write_to_shm_file(args.input_path);
    }

    // ── 1. 加载 FBX ──────────────────────────────────────────────
    std::cout << "[model_to_map] Loading: " << args.input_path << std::endl;
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(
        args.input_path,
        aiProcess_Triangulate
        | aiProcess_JoinIdenticalVertices
        | aiProcess_GenNormals);

    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
        std::cerr << "[model_to_map] ERROR: Assimp failed to load: "
                  << importer.GetErrorString() << std::endl;
        return 1;
    }

    std::vector<TriangleRecord> triangles;
    collect_triangles(scene->mRootNode, scene, Eigen::Matrix4f::Identity(), triangles);
    std::cout << "[model_to_map] Triangles: " << triangles.size() << std::endl;

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>());

    double total_area = 0.0;
    for (const auto& triangle : triangles) {
        total_area += triangle.area;
    }

    const double effective_density = args.density * args.scale * args.scale;
    std::size_t total_samples = 0;
    std::vector<std::size_t> num_samples_per_triangle;
    num_samples_per_triangle.reserve(triangles.size());
    std::vector<double> fractional_parts;
    fractional_parts.reserve(triangles.size());

    for (const auto& triangle : triangles) {
        const double expected = effective_density * static_cast<double>(triangle.area);
        std::size_t num_samples = static_cast<std::size_t>(std::floor(expected));
        num_samples_per_triangle.push_back(num_samples);
        fractional_parts.push_back(expected - std::floor(expected));
        total_samples += static_cast<std::size_t>(num_samples);
    }

    if (total_samples < static_cast<std::size_t>(std::round(effective_density * total_area))) {
        const std::size_t target_samples = static_cast<std::size_t>(std::round(effective_density * total_area));
        std::vector<std::size_t> order(fractional_parts.size());
        std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(), [&](std::size_t lhs, std::size_t rhs) {
            return fractional_parts[lhs] > fractional_parts[rhs];
        });
        for (std::size_t index : order) {
            if (total_samples >= target_samples) {
                break;
            }
            ++num_samples_per_triangle[index];
            ++total_samples;
        }
    }

    for (std::size_t i = 0; i < triangles.size(); ++i) {
        const auto& triangle = triangles[i];
        append_subdivided_triangle_samples(
            triangle, num_samples_per_triangle[i], static_cast<float>(args.scale), *cloud);
    }

    std::cout << "[model_to_map] Total triangles: " << triangles.size() << std::endl;
    const double total_area_m2 = total_area * args.scale * args.scale;
    std::cout << "[model_to_map] Total surface area: " << total_area_m2 << " m^2" << std::endl;
    std::cout << "[model_to_map] Sampled points: " << cloud->size() << std::endl;

    if (cloud->empty()) {
        std::cerr << "[model_to_map] ERROR: No points generated. Check FBX file." << std::endl;
        return 1;
    }

    if (args.roi) {
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
        for (const auto& pt : cloud->points) {
            if (pt.x >= args.roi->min.x() && pt.x <= args.roi->max.x()
                && pt.y >= args.roi->min.y() && pt.y <= args.roi->max.y()
                && pt.z >= args.roi->min.z() && pt.z <= args.roi->max.z()) {
                filtered->push_back(pt);
            }
        }
        *cloud = *filtered;
        std::cout << "[model_to_map] After ROI filter: " << cloud->size() << " points" << std::endl;
    }

    if (args.voxel_leaf > 0.0) {
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
        pcl::VoxelGrid<pcl::PointXYZRGBNormal> vg;
        vg.setLeafSize(args.voxel_leaf, args.voxel_leaf, args.voxel_leaf);
        vg.setInputCloud(cloud);
        vg.filter(*downsampled);
        *cloud = *downsampled;
        std::cout << "[model_to_map] After VoxelGrid " << args.voxel_leaf << "m: "
                  << cloud->size() << " points" << std::endl;
    }

    cloud->width    = cloud->size();
    cloud->height   = 1;
    cloud->is_dense = true;

    if (pcl::io::savePCDFileBinary(args.output_path, *cloud) != 0) {
        std::cerr << "[model_to_map] ERROR: Failed to write PCD: " << args.output_path << std::endl;
        return 1;
    }

    std::cout << "[model_to_map] Done. Output: " << args.output_path
              << " (" << cloud->size() << " points)" << std::endl;

    if (args.write_shm) {
        const int shm_rc = write_to_shm(*cloud);
        if (shm_rc != 0) {
            return shm_rc;
        }
    }

    return 0;
}

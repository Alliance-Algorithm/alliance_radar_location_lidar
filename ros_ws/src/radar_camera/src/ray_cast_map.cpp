#include "radar_camera/ray_cast_map.hpp"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <filesystem>

namespace radar_camera::projection {

auto RayCastMap::map_init(const std::string& mesh_path) -> std::expected<void, std::string> {
    if (!std::filesystem::exists(mesh_path)) {
        return std::unexpected("Mesh file not found: " + mesh_path);
    }

    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(mesh_path,
        aiProcess_Triangulate | aiProcess_FlipUVs | aiProcess_GenNormals | aiProcess_OptimizeMeshes);

    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
        return std::unexpected("Assimp error: " + std::string(importer.GetErrorString()));
    }

    struct CollectContext {
        std::vector<Triangle>& triangles;
    };
    CollectContext ctx{ triangles_ };

    std::function<void(aiNode*, const aiScene*)> collect_triangles;
    collect_triangles = [&ctx, &collect_triangles](aiNode* node, const aiScene* scene) {
        for (unsigned int i = 0; i < node->mNumMeshes; ++i) {
            aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
            for (unsigned int f = 0; f < mesh->mNumFaces; ++f) {
                const aiFace& face = mesh->mFaces[f];
                if (face.mNumIndices != 3) continue;

                Eigen::Vector3f v0(mesh->mVertices[face.mIndices[0]].x,
                                    mesh->mVertices[face.mIndices[0]].y,
                                    mesh->mVertices[face.mIndices[0]].z);
                Eigen::Vector3f v1(mesh->mVertices[face.mIndices[1]].x,
                                    mesh->mVertices[face.mIndices[1]].y,
                                    mesh->mVertices[face.mIndices[1]].z);
                Eigen::Vector3f v2(mesh->mVertices[face.mIndices[2]].x,
                                    mesh->mVertices[face.mIndices[2]].y,
                                    mesh->mVertices[face.mIndices[2]].z);
                ctx.triangles.push_back({ v0, v1, v2 });
            }
        }
        for (unsigned int i = 0; i < node->mNumChildren; ++i) {
            collect_triangles(node->mChildren[i], scene);
        }
    };

    triangles_.clear();
    collect_triangles(scene->mRootNode, scene);

    if (triangles_.empty()) {
        return std::unexpected("No triangles found in mesh");
    }

    return {};
}

auto RayCastMap::map_intersect(const Eigen::Vector3f& origin, const Eigen::Vector3f& direction) const
    -> std::optional<Eigen::Vector3f> {
    constexpr float kEpsilon = 1e-8f;
    float nearest = std::numeric_limits<float>::max();
    std::optional<Eigen::Vector3f> hit_point;

    for (const auto& tri : triangles_) {
        Eigen::Vector3f e1 = tri.v1 - tri.v0;
        Eigen::Vector3f e2 = tri.v2 - tri.v0;

        Eigen::Vector3f pvec = direction.cross(e2);
        float det = e1.dot(pvec);
        if (std::fabs(det) < kEpsilon) continue;

        float inv_det = 1.0f / det;
        Eigen::Vector3f tvec = origin - tri.v0;
        float u = tvec.dot(pvec) * inv_det;
        if (u < 0.0f || u > 1.0f) continue;

        Eigen::Vector3f qvec = tvec.cross(e1);
        float v = direction.dot(qvec) * inv_det;
        if (v < 0.0f || u + v > 1.0f) continue;

        float t = e2.dot(qvec) * inv_det;
        if (t < kEpsilon || t >= nearest) continue;

        nearest = t;
        hit_point = origin + direction * t;
    }

    return hit_point;
}

} // namespace radar_camera::projection

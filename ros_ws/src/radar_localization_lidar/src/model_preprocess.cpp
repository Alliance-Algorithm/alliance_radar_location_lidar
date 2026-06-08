#include "model_preprocess.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
using namespace Radar::process::model;
class ModelProcess::Impl {
public:
    struct Vertex {
        Eigen::Vector3f position;
        Eigen::Vector3f normal;
        Eigen::Vector2f uv = Eigen::Vector2f::Zero();
    };
    struct Indices {
        unsigned int vetrex_index;
    };
    struct ModelData {
        std::string name;
        std::string path;
        std::string model_type;
        std::vector<Vertex> vertices;
        std::vector<Indices> indices;
        std::vector<Eigen::Vector3f> point_cloud;
    };
    ModelData model_data;
    Assimp::Importer importer;

    auto ConfigLoader(const YAML::Node& model_config) -> std::expected<bool, std::string> {
        model_data = ModelData { .name = model_config["Model"]["model_name"].as<std::string>(),
            .path                      = model_config["Model"]["model_path"].as<std::string>(),
            .model_type                = model_config["Model"]["model_type"].as<std::string>() };
        if (model_data.model_type != "fbx") {
            return std::unexpected("Unsupported model type: " + model_data.model_type);
        }
        if (model_data.path.empty()) {
            return std::unexpected("Model path is empty");
        }
        return true;
    }

    static constexpr unsigned int kAssimpFlags =
        aiProcess_Triangulate | aiProcess_FlipUVs | aiProcess_GenNormals | aiProcess_OptimizeMeshes;

    auto LoadModel() -> std::expected<const aiScene*, std::string> {
        const aiScene* scene = importer.ReadFile(model_data.path, kAssimpFlags);
        if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
            return std::unexpected("Assimp error: " + std::string(importer.GetErrorString()));
        }
        return scene;
    }

    auto ProcessMeshes(const aiMesh* mesh) -> std::expected<bool, std::string> {
        model_data.vertices.reserve(model_data.vertices.size() + mesh->mNumVertices);
        for (unsigned int j = 0; j < mesh->mNumVertices; j++) {
            Vertex vertex;
            vertex.position = Eigen::Vector3f::Map(&mesh->mVertices[j].x);
            vertex.normal   = Eigen::Vector3f::Map(&mesh->mNormals[j].x);
            if (mesh->mTextureCoords[0]) {
                vertex.uv =
                    Eigen::Vector2f(mesh->mTextureCoords[0][j].x, mesh->mTextureCoords[0][j].y);
            } else {
                vertex.uv = Eigen::Vector2f::Zero();
            }
            model_data.vertices.push_back(vertex);
        }
        model_data.indices.reserve(model_data.indices.size() + mesh->mNumFaces * 3);
        for (unsigned int j = 0; j < mesh->mNumFaces; j++) {
            const aiFace& face = mesh->mFaces[j];
            for (unsigned int k = 0; k < face.mNumIndices; k++) {
                Indices index;
                index.vetrex_index = face.mIndices[k];
                model_data.indices.push_back(index);
            }
        }
        return true;
    }

    auto ProcessNode(aiNode* node, const aiScene* scene) -> std::expected<bool, std::string> {
        for (unsigned int i = 0; i < node->mNumMeshes; i++) {
            aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
            auto result  = ProcessMeshes(mesh);
            if (!result) {
                return std::unexpected(result.error());
            }
        }
        for (unsigned int i = 0; i < node->mNumChildren; i++) {
            auto result = ProcessNode(node->mChildren[i], scene);
            if (!result) {
                return std::unexpected(result.error());
            }
        }
        return true;
    }

    auto ProcessModel() -> std::expected<bool, std::string> {
        auto scene_result = LoadModel();
        if (!scene_result) {
            return std::unexpected(scene_result.error());
        }
        const aiScene* scene = scene_result.value();
        return ProcessNode(scene->mRootNode, scene);
    }

    auto TransformModeltoPointCloud() -> std::expected<bool, std::string> {
        model_data.point_cloud.reserve(model_data.vertices.size());
        for (const auto& vertex : model_data.vertices) {
            model_data.point_cloud.push_back(vertex.position);
        }
        return true;
    }

    auto GetPointCloud() const noexcept -> const std::vector<Eigen::Vector3f>& {
        return model_data.point_cloud;
    }
};

auto ModelProcess::ConfigLoader(const YAML::Node& model_config)
    -> std::expected<bool, std::string> {
    return impl_->ConfigLoader(model_config);
}
ModelProcess::ModelProcess() noexcept
    : impl_ { std::make_unique<Impl>() } { }

auto ModelProcess::ProcessModel() -> std::expected<bool, std::string> {
    return impl_->ProcessModel();
}
auto ModelProcess::TransformModeltoPointCloud() -> std::expected<bool, std::string> {
    return impl_->TransformModeltoPointCloud();
}
auto ModelProcess::GetPointCloud() const noexcept -> const std::vector<Eigen::Vector3f>& {
    return impl_->GetPointCloud();
}
ModelProcess::~ModelProcess() noexcept = default;

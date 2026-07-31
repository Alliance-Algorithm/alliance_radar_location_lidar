#include "radar_camera/tensorrt_inference.hpp"

#include <NvInferRuntime.h>
#include <cuda_runtime_api.h>

#include <fstream>
#include <sstream>
#include <utility>

namespace radar_camera::model_inference {

namespace {

    class Logger final : public nvinfer1::ILogger {
    public:
        void log(Severity severity, const char* message) noexcept override {
            if (severity <= Severity::kERROR) last_message_ = message == nullptr ? "" : message;
        }

        [[nodiscard]] auto last_message() const -> const std::string& { return last_message_; }

    private:
        std::string last_message_;
    };

    auto trt_error(cudaError_t code, const char* operation) -> std::string {
        return std::string(operation) + ": " + cudaGetErrorString(code);
    }

} // namespace

struct TensorRtInference::Impl {
    Logger logger;
    nvinfer1::IRuntime* runtime { nullptr };
    nvinfer1::ICudaEngine* engine { nullptr };
    nvinfer1::IExecutionContext* context { nullptr };
    cudaStream_t stream { nullptr };
    void* device_input { nullptr };
    void* device_output { nullptr };
    std::size_t input_count { 0 };
    std::size_t output_count { 0 };
    std::string input_name;
    std::string output_name;
    std::vector<float> output;
    bool enqueued { false };

    ~Impl() {
        if (stream != nullptr) cudaStreamDestroy(stream);
        if (device_input != nullptr) cudaFree(device_input);
        if (device_output != nullptr) cudaFree(device_output);
        delete context;
        delete engine;
        delete runtime;
    }
};

TensorRtInference::TensorRtInference()
    : impl_(std::make_unique<Impl>()) { }

TensorRtInference::~TensorRtInference() = default;

auto TensorRtInference::init(const std::string& engine_path) -> std::expected<void, std::string> {
    std::ifstream file(engine_path, std::ios::binary | std::ios::ate);
    if (!file) return std::unexpected("TensorRT engine cannot be opened: " + engine_path);

    const auto file_size = file.tellg();
    if (file_size <= 0) return std::unexpected("TensorRT engine is empty: " + engine_path);
    file.seekg(0, std::ios::beg);
    std::vector<char> serialized(static_cast<std::size_t>(file_size));
    if (!file.read(serialized.data(), file_size)) {
        return std::unexpected("TensorRT engine read failed: " + engine_path);
    }

    impl_->runtime = nvinfer1::createInferRuntime(impl_->logger);
    if (impl_->runtime == nullptr) {
        return std::unexpected(
            "TensorRT createInferRuntime failed: " + impl_->logger.last_message());
    }
    impl_->engine = impl_->runtime->deserializeCudaEngine(serialized.data(), serialized.size());
    if (impl_->engine == nullptr) {
        return std::unexpected(
            "TensorRT deserializeCudaEngine failed: " + impl_->logger.last_message());
    }
    impl_->context = impl_->engine->createExecutionContext();
    if (impl_->context == nullptr) return std::unexpected("TensorRT createExecutionContext failed");

    if (impl_->engine->getNbIOTensors() != 2) {
        return std::unexpected("TensorRT engine must have exactly one input and one output tensor");
    }
    for (int32_t i = 0; i < impl_->engine->getNbIOTensors(); ++i) {
        const char* name = impl_->engine->getIOTensorName(i);
        if (name == nullptr) return std::unexpected("TensorRT IO tensor has no name");
        if (impl_->engine->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
            impl_->input_name = name;
        } else {
            impl_->output_name = name;
        }
    }
    if (impl_->input_name.empty() || impl_->output_name.empty()) {
        return std::unexpected("TensorRT engine must expose one input and one output");
    }

    const auto input_dims  = impl_->engine->getTensorShape(impl_->input_name.c_str());
    const auto output_dims = impl_->engine->getTensorShape(impl_->output_name.c_str());
    if (input_dims.nbDims != 4 || input_dims.d[0] != 1 || input_dims.d[1] != 3
        || input_dims.d[2] != 1280 || input_dims.d[3] != 1280) {
        return std::unexpected("TensorRT input must be static 1x3x1280x1280");
    }
    if (output_dims.nbDims != 3 || output_dims.d[0] != 1 || output_dims.d[1] != 300
        || output_dims.d[2] != 6) {
        return std::unexpected("TensorRT output must be static 1x300x6");
    }
    if (impl_->engine->getTensorDataType(impl_->input_name.c_str()) != nvinfer1::DataType::kFLOAT
        || impl_->engine->getTensorDataType(impl_->output_name.c_str())
            != nvinfer1::DataType::kFLOAT) {
        return std::unexpected("TensorRT backend requires float32 IO tensors");
    }

    impl_->input_count  = 1u * 3u * 1280u * 1280u;
    impl_->output_count = 1u * 300u * 6u;
    impl_->output.resize(impl_->output_count);

    if (auto code = cudaStreamCreate(&impl_->stream); code != cudaSuccess) {
        return std::unexpected(trt_error(code, "cudaStreamCreate"));
    }
    if (auto code = cudaMalloc(&impl_->device_input, impl_->input_count * sizeof(float));
        code != cudaSuccess) {
        return std::unexpected(trt_error(code, "cudaMalloc input"));
    }
    if (auto code = cudaMalloc(&impl_->device_output, impl_->output_count * sizeof(float));
        code != cudaSuccess) {
        return std::unexpected(trt_error(code, "cudaMalloc output"));
    }
    if (!impl_->context->setTensorAddress(impl_->input_name.c_str(), impl_->device_input)
        || !impl_->context->setTensorAddress(impl_->output_name.c_str(), impl_->device_output)) {
        return std::unexpected("TensorRT setTensorAddress failed");
    }
    return { };
}

auto TensorRtInference::start(const float* input, std::size_t input_elements)
    -> std::expected<void, std::string> {
    if (impl_->context == nullptr || impl_->stream == nullptr) {
        return std::unexpected("TensorRT inference is not initialized");
    }
    if (input == nullptr || input_elements != impl_->input_count) {
        return std::unexpected("TensorRT input size mismatch");
    }
    if (auto code = cudaMemcpyAsync(impl_->device_input, input, impl_->input_count * sizeof(float),
            cudaMemcpyHostToDevice, impl_->stream);
        code != cudaSuccess) {
        return std::unexpected(trt_error(code, "cudaMemcpyAsync input"));
    }
    if (!impl_->context->enqueueV3(impl_->stream)) {
        return std::unexpected("TensorRT enqueueV3 failed");
    }
    impl_->enqueued = true;
    return { };
}

auto TensorRtInference::wait()
    -> std::expected<std::reference_wrapper<const std::vector<float>>, std::string> {
    if (!impl_->enqueued) return std::unexpected("TensorRT wait called before start");
    if (auto code = cudaMemcpyAsync(impl_->output.data(), impl_->device_output,
            impl_->output_count * sizeof(float), cudaMemcpyDeviceToHost, impl_->stream);
        code != cudaSuccess) {
        return std::unexpected(trt_error(code, "cudaMemcpyAsync output"));
    }
    if (auto code = cudaStreamSynchronize(impl_->stream); code != cudaSuccess) {
        return std::unexpected(trt_error(code, "cudaStreamSynchronize"));
    }
    impl_->enqueued = false;
    return std::ref(impl_->output);
}

auto TensorRtInference::input_elements() const noexcept -> std::size_t {
    return impl_->input_count;
}

auto TensorRtInference::output_elements() const noexcept -> std::size_t {
    return impl_->output_count;
}

} // namespace radar_camera::model_inference

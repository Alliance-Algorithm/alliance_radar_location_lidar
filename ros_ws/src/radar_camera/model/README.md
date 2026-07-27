# Radar Camera Model Artifacts

The competition model is `best_fixed_names_1280_fp16.engine`.

## Source model

- Source: `/home/yukikaze/Downloads/final_export (1)/final_export/best_fixed_names/onnx/1280/best_fixed_names_1280.onnx`
- Repository ONNX SHA-256: `5971f6c197ec9c3b54a66eef3e16e041940fa11bd7657f23f6ad436cbe606b4d`
- Input: `images`, `1x3x1280x1280`, `float32`
- Output: `output0`, `1x300x6`, `float32`
- Row format: `x1,y1,x2,y2,confidence,class_id`
- Class order: `0-5` blue (`hero`, `engineer`, `infantry3`, `infantry4`, `sentry`, `drone`), `6-11` red in the same order
- Live SHM preprocessing: RGB8 bytes, resize to `1280x1280`, divide by `255`, NCHW
- Offline JPEG preprocessing: `cv::imread` BGR is converted to RGB first to match SHM bytes
- `blobFromImage` uses `swapRB=false` because the live SHM buffer is already RGB

## TensorRT engine

The checked-in engine was built on the target host with:

- GPU: NVIDIA GeForce RTX 4060 Laptop GPU
- Compute capability: 8.9
- CUDA: 13.3
- TensorRT: 11.1.0
- Engine build: fixed batch 1, fixed input `1x3x1280x1280`
- Engine size: about 6.8 MiB
- Repository engine SHA-256: `09ad9e7d2263b2e94489a5c0f8a19277c3d9a20eb6af2fe423d643d6c59103b5`

The engine is hardware and TensorRT-version specific. Rebuild it with
`.script/build-radar-camera-tensorrt-engine` when the GPU architecture or
TensorRT runtime changes.

The final host benchmark recorded 266.613 qps, 4.05005 ms median latency with
H2D enabled, 1.58984 ms median H2D, 2.45117 ms median GPU compute, and
0.00390625 ms median D2H. TensorRT reported unstable GPU compute timing during
this run because the GPU clock was not locked. These are pure engine
measurements and exclude SHM, resize, normalization, postprocessing,
projection, and ROS publication.

YOLO26 is exported end-to-end and NMS-free. The camera path uses a confidence
threshold of `0.3`, applies the existing size and aspect-ratio filters, and
then keeps only the highest-confidence detection for each class. No additional
IoU/NMS suppression is performed.

## Container runtime

The development container uses NVIDIA device nodes plus a read-only host
staging directory at `/opt/radar_camera_trt`. Prepare it before recreating the
container:

```bash
.script/prepare-radar-camera-tensorrt-runtime
docker compose -f .devcontainer/docker-compose.yml up -d --force-recreate radar-develop
```

The staging directory contains the TensorRT 11.1 runtime, CUDA 13 headers and
runtime, and the host NVIDIA driver libraries needed by CUDA initialization.
The engine must be rebuilt if the target GPU compute capability or TensorRT
major/minor runtime changes.

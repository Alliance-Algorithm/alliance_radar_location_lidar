# radar_camera YOLO / Algorithm Unit Tests Design

Date: 2026-07-23  
Package: `ros_ws/src/radar_camera`  
Status: design (awaiting user review before implementation plan)

## Goal

Add **image-backed unit tests** that verify:

1. YOLO / OpenVINO **model access** (`ModelInference`: init, preprocess, async runtime, postprocess).
2. Algorithm pieces that consume detections (`Projector` mapping + class-id → `RobotPose` slots).

Success means: local `colcon test --packages-select radar_camera` (or the package gtest target) is green without requiring committed ONNX weights or GPU; when a real model + fixture image are present, an optional smoke path exercises full inference.

## Current Codebase Facts

| Item | Detail |
|------|--------|
| Inference API | `ModelInference::{infer_init, infer_preprocess, infer_runtime_async, infer_runtime_wait, infer_postprocess}` |
| Runtime | OpenVINO `compile_model(model_path, device_name)`; default CPU |
| Preprocess | `cv::dnn::blobFromImage` → NCHW f32, scale 1/255, swapRB |
| Postprocess | Expect raw rows `[x1,y1,x2,y2,conf,cls,...]`; conf + aspect-ratio filters; center scaled to source image |
| Node wiring | `RadarCameraNode::ImageCallback` chains inference → projector → `CameraDetectionPose` publish |
| Tests today | **None** (CMake only lint under `BUILD_TESTING`) |
| Model in repo | Config points to `config/camera_inference_model.onnx` — **file not present** |
| Peer pattern | `radar_lidar` / `radar_calibration`: static `*_core` lib + `ament_add_gtest` |
| Dev image (pre-change) | `Dockerfile` had **no** OpenVINO; CMake uses `openvino.utils.get_cmake_path()` |
| Dev image (design) | Install `openvino==2025.4.1` via pip in Dockerfile; verified hot-install in running container |

## Model format contract (locked for tests + export)

Derived from `model_inference.cpp` + `radar_camera.yaml`. L3 smoke and any export tooling must match this.

### File / runtime

- **Primary path format:** ONNX (`.onnx`) via OpenVINO ONNX frontend.
- **Also acceptable:** OpenVINO IR (`.xml` + `.bin`) if `model_path` points at the `.xml`.
- **Device for CI/smoke:** `CPU` (`device_name` in YAML).

### Input

| Field | Value |
|-------|--------|
| Layout | NCHW `float32` |
| Shape | `[1, 3, model_input_height, model_input_width]` default **1280×1280** |
| Color | RGB after `swapRB=true` |
| Scale | pixels / 255 → roughly `[0, 1]` |
| Mean/std | **none** in code — export must not double-normalize |

### Output (decoded detections, not raw YOLO head)

| Field | Value |
|-------|--------|
| Shape | rank-2 `[N, S]` or rank-3 `[1, N, S]` with **S ≥ 6** |
| Row layout | **`x1, y1, x2, y2, conf, class_id`** (xyxy) |
| Coordinate frame | Model input pixel space (e.g. 1280×1280); code scales to source image size |
| `class_id` | Discrete id at index 5 (**not** argmax over `num_classes` scores) |
| Classes | 12 ids: even red / odd blue for hero, engineer, infantry3/4, sentry, drone |

**Implication:** Ultralytics default multi-score head export does **not** match this postprocess. Export must emit NMS-decoded rows, or production postprocess must be changed (out of scope for unit-test v1 except documenting the contract).

### Filters applied in postprocess (L1 tests lock these)

- Drop if `conf < conf_threshold` (default 0.6).
- Drop if scaled box width or height `< 1`.
- Drop if `max(w,h)/min(w,h)` outside `[min_length_width_rate, max_length_width_rate]` (default 0.8–1.5).

## Approaches Considered

### A — End-to-end true YOLO (real image + real ONNX)

- **Pros:** Closest to production; catches export/layout bugs.
- **Cons:** Weights large / not in git; flaky thresholds across devices; slow CI; needs committed or downloaded artifacts.
- **Verdict:** Not default. Keep as optional offline smoke only.

### B — Smoke-only (run if assets exist, else skip)

- **Pros:** Proves real OpenVINO load when available.
- **Cons:** Alone does not lock postprocess filters or projection math when assets missing.
- **Verdict:** Useful **secondary** layer, not sole strategy.

### C — Pure unit tests + synthetic / fixture images (recommended core)

- **Pros:** CI-stable; no weights; tests filter logic and tensor shapes with controlled inputs; matches other packages’ gtest style.
- **Cons:** Does not prove a specific trained model’s accuracy.
- **Verdict:** **Primary** approach.

### Recommendation: **C + B (optional smoke)**

| Layer | Depends on ONNX? | Depends on real photo? | Always runs? |
|-------|------------------|------------------------|--------------|
| L1 Preprocess / postprocess pure | No | Synthetic `cv::Mat` only | Yes |
| L2 Projector pure | No | No (detections as numbers) | Yes |
| L3 Optional full inference smoke | Yes (env or path) | Optional fixture image | Skip if missing |

Default success criterion for CI: **L1 + L2 pass**. L3 is best-effort.

## Design

### 1. Build layout (match `radar_calibration` / `radar_lidar`)

Split algorithm code into a static library so gtests do not spin up the full ROS node:

```
radar_camera_core (STATIC)
  src/model_inference.cpp
  src/projector.cpp

radar_camera_node (executable)
  src/runtime.cpp
  src/radar_camera_node.cpp
  → links radar_camera_core + OpenCV + OpenVINO + Eigen + assimp + ROS deps
```

CMake under `BUILD_TESTING`:

```cmake
find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(radar_camera_tests
  test/test_model_inference.cpp
  test/test_projector.cpp
  # optional: test/test_model_inference_smoke.cpp  (or same file with GTEST_SKIP)
)
target_link_libraries(radar_camera_tests radar_camera_core ...)
```

`package.xml`: add `test_depend` on `ament_cmake_gtest` (and keep OpenVINO / OpenCV as build deps already used by core).

**Out of scope for this design:** full ROS node spin tests (`RadarCameraNode` + topics). Those can be a later integration test.

### 2. Fixtures layout

```
ros_ws/src/radar_camera/
  test/
    test_model_inference.cpp
    test_projector.cpp
    fixtures/                    # small assets only
      synthetic_blank.png        # optional: generate in SetUp instead of committing
      # real_scene.jpg           # optional, gitignored or LFS — not required for CI
  config/
    camera_inference_model.onnx  # NOT required in CI; path overridable
```

Prefer **runtime-generated** synthetic images (`cv::Mat` filled with known patterns) over large binary fixtures. If a real photo is needed for L3, resolve path via:

1. Env `RADAR_CAMERA_TEST_IMAGE`
2. Else `test/fixtures/real_scene.jpg` if exists
3. Else `GTEST_SKIP`

Model path for L3:

1. Env `RADAR_CAMERA_TEST_MODEL`
2. Else package share / source `config/camera_inference_model.onnx` if exists
3. Else `GTEST_SKIP`

### 3. Test cases — `ModelInference` (L1 + L3)

#### L1a — `infer_preprocess` shape and finite values

- Input: BGR `cv::Mat` e.g. 640×480 solid color or gradient (no model init required for tensor creation path that only builds NCHW — note: current API does not require `infer_init` for preprocess; it reallocates `input_tensor_` by shape).
- Call: `infer_preprocess(img, 1280, 1280)` (or smaller if we only care shape).
- Assert: `has_value()`, tensor shape `{1,3,H,W}`, element type f32, values in `[0,1]` (within float noise).

#### L1b — `infer_postprocess` confidence filter

**Constraint:** Current `infer_postprocess` reads `infer_request_.get_output_tensor().get_shape()` for shape, while values come from the `raw_output` argument. That couples postprocess to a live InferRequest.

**Design decision (required for pure unit tests):**

- **Refactor postprocess slightly** so shape is either:
  - taken from `raw_output` + explicit `(num_detections, stride)` args, or
  - stored from last wait, or
  - inferred as `raw_output.size() / stride` with stride from config / argument.

Recommended minimal API change (production-compatible):

```cpp
// Prefer: shape derived without depending on live OV request when testing.
// Option chosen in plan phase: extract pure function, e.g.
// filter_detections(raw, num_det, stride, src_w, src_h, config) -> vector<Detection>
// ModelInference::infer_postprocess calls that pure function after reading shape from OV.
```

Tests then call the pure filter (or postprocess after injecting shape) with hand-built float buffers:

| Case | Input | Expected |
|------|--------|----------|
| Below conf | conf `< conf_threshold` | empty detections |
| Above conf | conf `≥` threshold, valid box | one detection, correct id/center |
| Aspect ratio out of range | ratio `< min` or `> max` | filtered out |
| Tiny box | w or h `< 1` after scale | filtered out |
| Scale mapping | model coords → src image center | `EXPECT_NEAR` center |

Default config in tests: `conf_threshold=0.6`, `min_length_width_rate=0.8`, `max_length_width_rate=1.5`, `model_input_width/height=1280`.

#### L1c — `infer_init` error path

- Path to missing file → `infer_init` returns `unexpected` with message containing OpenVINO / fail wording.
- No skip; always runnable.

#### L3 — Optional full pipeline smoke (image + model)

When model (and optional image) present:

1. `infer_init` with `device_name=CPU`.
2. Load image (fixture or synthetic 1280×1280 if only proving “runs”).
3. preprocess → async → wait → postprocess.
4. Assert: all steps `has_value()`; if using synthetic blank, **do not** assert specific class IDs (model-dependent). Optional: assert runtime finishes under a generous timeout.
5. If using a **labeled** fixture (future): document expected class set in a sidecar JSON; not required for v1.

### 4. Test cases — `Projector` (L2)

No mesh required for pure camera + pose mapping tests where mesh is empty / optional.

| Case | Setup | Expected |
|------|--------|----------|
| Camera init rejects bad matrix size | `camera_matrix` not length 9 | error |
| Camera init OK | identity K, zero dist, identity extrinsics | success |
| `proj_postprocess` class mapping | detections with blue/red class ids matching `CameraConfig` | correct slot filled, others zero/default |
| Ray + flat mesh (optional) | minimal one-triangle mesh file or in-memory if API allows | hit point near expected map xy |

**Mesh note:** `proj_init_map` requires a filesystem mesh via Assimp. For ray-hit tests, either:

- Ship a tiny OBJ under `test/fixtures/ground_plane.obj` (few lines of text), or
- Leave mesh intersection as optional if Assimp path is awkward in CI.

Recommendation: **include a tiny OBJ** (plane under camera) so L2 is complete without depending on `model/` FBX.

`proj_postprocess` last-wins for duplicate class ids (document current behavior; test it).

### 5. What we do **not** test in v1

- Live Hik camera / SHM.
- Full `RadarCameraNode` lifecycle and topic contracts.
- GPU / OpenVINO NPU devices.
- Trained-model mAP / qualitative accuracy gates.
- Fusion with `radar_fusion` (separate todo T3.7).

### 6. Error handling & skip policy

| Situation | Behavior |
|-----------|----------|
| Missing ONNX | L3 `GTEST_SKIP`; L1/L2 still pass |
| Missing real photo | L3 uses synthetic image or skip only if model also missing |
| OpenVINO not found at configure time | package already `find_package(OpenVINO REQUIRED)` — same as production build |
| Postprocess refactor needed | Required for L1b; keep node call site behavior identical |

### 7. Run commands (implementation target)

```bash
cd /workspace/ros_ws   # or host ros_ws
colcon build --packages-select radar_camera --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select radar_camera --event-handlers console_direct+
# optional smoke:
RADAR_CAMERA_TEST_MODEL=/path/to/model.onnx \
RADAR_CAMERA_TEST_IMAGE=/path/to/scene.jpg \
  colcon test --packages-select radar_camera ...
```

### 8. Implementation order (for later writing-plans)

1. Extract `radar_camera_core` in CMake; keep node linking core.
2. Extract pure postprocess filter for testability (minimal production change).
3. RED: write L1/L2 gtests first; watch fail.
4. GREEN: only enough production/test glue to pass.
5. Add L3 smoke with skip guards.
6. Tiny OBJ fixture for projector ray test if in scope.
7. Verify with `colcon test`.

TDD rule: no production-only changes without a failing test first (except pure CMake library split required to link tests).

## Decisions Locked

1. **Strategy:** pure unit tests (C) + optional ONNX/image smoke (B).
2. **Framework:** gtest + `ament_add_gtest`, same as sibling packages.
3. **No ROS node unit test in v1.**
4. **No committing large ONNX into git for CI green.**
5. **Postprocess must be unit-testable without a live successful inference** (small extract/refactor allowed).
6. **Images:** synthetic primary; real images via env/path optional.
7. **OpenVINO in Dockerfile:** Intel apt `openvino-2025.4.1` (CXX11 ABI=1). **Do not use pip manylinux wheels** — they force ABI=0 and break link against system OpenCV/rclcpp.
8. **Model contract:** ONNX (or IR), 1280 NCHW 0–1 RGB, output NMS-decoded `xyxy+conf+cls` rows as above.

## Environment / rebuild notes

```bash
# Persist OpenVINO (after Dockerfile change)
docker compose -f .devcontainer/docker-compose.yml build radar-develop
docker compose -f .devcontainer/docker-compose.yml up -d --force-recreate

# Verify
docker exec <container> python3 -c \
  "import openvino as ov; from openvino.utils import get_cmake_path; print(ov.__version__, get_cmake_path())"
```

GHCR `ghcr.io/harrypotter1tech/radar:develop` may be unreachable or outdated relative to this Dockerfile; prefer local rebuild until a new image is published with OpenVINO.

## Open Items for Plan Phase (not blockers)

- Exact pure-function signature for postprocess (keep public API of `ModelInference` stable if possible).
- Whether to install `test/fixtures` via CMake or only use source-tree relative paths via `TEST_SOURCE_DIR`.
- Whether `use_openvino` flag (unused in cpp today) should be tested or ignored (**ignore** until implemented).
- Optional export helper script for Ultralytics → contract-compliant ONNX (nice-to-have, not required for L1/L2).

## Spec self-review (2026-07-23)

| Check | Result |
|-------|--------|
| Placeholders / TBD | None remaining in locked decisions |
| Consistency | L1/L2 always-on vs L3 skip matches model-missing fact |
| Scope | Single package tests + minimal postprocess extract; no node/fusion |
| Ambiguity | Output layout fixed to xyxy+conf+cls; raw multi-score head explicitly out of format |
| OpenVINO | Documented as env prerequisite, not as a test case itself |

## Acceptance Criteria

- [ ] `radar_camera_tests` target exists and is wired under `BUILD_TESTING`.
- [ ] L1 preprocess + L1 postprocess filter + L1 init-failure tests pass without ONNX.
- [ ] L2 projector config + class mapping tests pass without mesh/ONNX.
- [ ] L3 smoke skips cleanly without assets; runs when env/paths set.
- [ ] Node runtime behavior unchanged for existing YAML-driven path.
- [ ] Dockerfile (or published image) provides OpenVINO for build/link.
- [ ] Design reviewed by user before implementation plan / code.

## Status

Design written to this file. **Awaiting user review/approval** before invoking `writing-plans` and any production/test code beyond the Dockerfile OpenVINO install already applied.

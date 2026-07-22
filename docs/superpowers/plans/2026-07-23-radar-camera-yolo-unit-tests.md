# radar_camera YOLO Unit Tests Implementation Plan

> **For agentic workers:** Inline execution in session. Steps use checkbox syntax.

**Goal:** Add gtest coverage for `ModelInference` (preprocess/postprocess/init) and `Projector` without requiring ONNX in CI; optional L3 smoke when assets exist.

**Architecture:** Static `radar_camera_core` library + pure `filter_detections` + `ament_add_gtest`.

**Tech Stack:** C++23, gtest, OpenVINO, OpenCV, ament_cmake, Eigen, assimp.

## Global Constraints

- No committed ONNX required for green CI.
- Keep `ModelInference` public methods behavior-compatible for the ROS node.
- Match `radar_lidar` / `radar_calibration` test layout.

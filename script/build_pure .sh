#!/bin/bash
BUILD_TYPE=${1:-Release}
cd /workspace/ros_ws
source /opt/ros/humble/setup.bash
rm -rf build/radar_localization_lidar install/radar_localization_lidar
echo "[PURE BUILD] Build type: ${BUILD_TYPE}"
colcon build --packages-select radar_localization_lidar --cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE} -DCMAKE_EXPORT_COMPILE_COMMANDS=1

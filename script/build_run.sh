#!/bin/bash
BUILD_TYPE=${1:-Release}
cd /workspace/ros_ws
source /opt/ros/humble/setup.bash
echo "[BUILD & RUN] Build type: ${BUILD_TYPE}"
colcon build --packages-select radar_localization_lidar --cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE} -DCMAKE_EXPORT_COMPILE_COMMANDS=1
source /workspace/ros_ws/install/setup.bash
ros2 run radar_localization_lidar radar_localization_lidar_node

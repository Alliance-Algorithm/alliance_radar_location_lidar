#!/bin/bash
# env_setup.bash - Environment setup for RADAR-LOCATION-LIDAR container
# Sourced by both bash and zsh

# ROS2
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Workspace
export RADAR_WS="/workspace"
export RADAR_SRC="${RADAR_WS}/ros_ws/src/radar_localization_lidar"

# Add build scripts to PATH
if [ -d "${HOME}/.script" ]; then
    export PATH="${HOME}/.script:${PATH}"
fi

# Add opencode to PATH
if [ -d "${HOME}/.opencode/bin" ]; then
    export PATH="${HOME}/.opencode/bin:${PATH}"
fi

# Add mimocode to PATH
if [ -d "${HOME}/.mimocode/bin" ]; then
    export PATH="${HOME}/.mimocode/bin:${PATH}"
fi

# CMake
export CMAKE_BUILD_PARALLEL_LEVEL=$(nproc)

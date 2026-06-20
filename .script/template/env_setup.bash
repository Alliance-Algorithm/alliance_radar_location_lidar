#!/bin/bash
# env_setup.bash - Environment setup for RADAR-LOCATION-LIDAR container
# Sourced by both bash and zsh (via env_setup.zsh)

: "${RADAR_WS:=/workspace}"

# ── ROS 2 ────────────────────────────────────────────────────────
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null
else
    echo "[ERROR] ROS2 Jazzy not found at /opt/ros/jazzy/setup.bash" >&2
fi

# ── Workspace ────────────────────────────────────────────────────
export RADAR_ROS_WS="${RADAR_WS}/ros_ws"
export RADAR_SRC="${RADAR_ROS_WS}/src"
export RADAR_THIRD_PARTY="${RADAR_ROS_WS}/third-party"
export RADAR_MODEL_DIR="${RADAR_WS}/model"

# ── Build scripts ────────────────────────────────────────────────
if [ -d "${HOME}/.script" ]; then
    if [[ ":${PATH}:" != *":${HOME}/.script:"* ]]; then
        export PATH="${HOME}/.script:${PATH}"
    fi
fi

# ── Tools ────────────────────────────────────────────────────────
for tool in opencode codex mimocode; do
    dir="${HOME}/.${tool}/bin"
    if [ -d "${dir}" ] && [[ ":${PATH}:" != *":${dir}:"* ]]; then
        export PATH="${dir}:${PATH}"
    fi
done

# ── CMake ────────────────────────────────────────────────────────
export CMAKE_BUILD_PARALLEL_LEVEL=$(nproc)

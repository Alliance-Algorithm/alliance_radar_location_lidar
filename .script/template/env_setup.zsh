#!/bin/zsh
# env_setup.zsh - Zsh environment setup for RADAR-LOCATION-LIDAR container
# Reference: Alliance-Algorithm/RMCS .script/template/env_setup.zsh

: "${RADAR_WS:=/workspace}"

# ── ROS 2 discovery ──────────────────────────────────────────────
# Odin driver requires localhost-only to avoid DDS broadcast issues
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOCALHOST_ONLY=1
export RCUTILS_COLORIZED_OUTPUT=1

# ── ROS 2 zsh setup ──────────────────────────────────────────────
if [ -f /opt/ros/jazzy/setup.zsh ]; then
    source /opt/ros/jazzy/setup.zsh 2>/dev/null
else
    echo "[ERROR] ROS2 Jazzy not found at /opt/ros/jazzy/setup.zsh" >&2
fi

# ── Workspace local_setup (conditional) ──────────────────────────
if [ -f "${RADAR_WS}/ros_ws/install/local_setup.zsh" ]; then
    source "${RADAR_WS}/ros_ws/install/local_setup.zsh"
fi

# ── Bash-compatible env ──────────────────────────────────────────
source ~/env_setup.bash 2>/dev/null || true

# ── Python argcomplete ───────────────────────────────────────────
if command -v register-python-argcomplete &>/dev/null; then
    eval "$(register-python-argcomplete ros2)"
    eval "$(register-python-argcomplete colcon)"
fi

# ── Sensor type ──────────────────────────────────────────────────
: "${RADAR_SENSOR_TYPE:=mid70}"
export RADAR_SENSOR_TYPE

# ── Zsh completions ──────────────────────────────────────────────
if [ -d "${HOME}/.script/complete" ]; then
    fpath=("${HOME}/.script/complete" $fpath)
fi
if [ -d "${HOME}/.opencode/bin" ]; then
    fpath+=("${HOME}/.opencode/bin")
fi

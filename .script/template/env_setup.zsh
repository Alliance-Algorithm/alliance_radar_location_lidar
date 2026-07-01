#!/bin/zsh
# env_setup.zsh - Zsh environment setup for RADAR-LOCATION-LIDAR container
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
if [ -f "${HOME}/env_setup.bash" ]; then
    source "${HOME}/env_setup.bash" 2>/dev/null || \
        echo "[WARN] Failed to source ${HOME}/env_setup.bash" >&2
fi

# ── Prefer workspace scripts over the synced home copy ───────────
if [ -d "${RADAR_WS}/.script" ]; then
    case ":${PATH}:" in
        *":${RADAR_WS}/.script:"*) ;;
        *) PATH="${RADAR_WS}/.script:${PATH}" ;;
    esac
fi

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
    if [[ " ${fpath[*]} " != *" ${HOME}/.script/complete "* ]]; then
        fpath=("${HOME}/.script/complete" "${fpath[@]}")
    fi
fi
if [ -d "${HOME}/.opencode/bin" ]; then
    if [[ " ${fpath[*]} " != *" ${HOME}/.opencode/bin "* ]]; then
        fpath+=("${HOME}/.opencode/bin")
    fi
fi

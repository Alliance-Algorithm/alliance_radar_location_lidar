#!/bin/bash
# post-create.sh - First-time container setup
set -euo pipefail

echo "========================================="
echo "  RADAR-LOCATION-LIDAR post-create setup"
echo "========================================="

# Check host opencode config
if [ -f /home/ubuntu/.opencode/bin/opencode ]; then
    echo "[OK] opencode binary found from host mount"
fi

# Check host codex config
if [ -f /home/ubuntu/.codex/config.toml ]; then
    echo "[OK] codex config found from host mount"
fi

# Initialize git submodules if needed
cd /workspace
if [ -f .gitmodules ] && [ ! -f ros_ws/third-party/small_gicp/.git ]; then
    echo "[..] Initializing git submodules..."
    git submodule update --init --recursive
fi

# Build third-party packages if not already built
if [ ! -d /workspace/ros_ws/install ]; then
    echo "[..] First-time build: third-party packages..."
    cd /workspace/ros_ws
    if [ -f /opt/ros/jazzy/setup.bash ]; then
        source /opt/ros/jazzy/setup.bash
    elif [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    else
        echo "[ERROR] ROS2 environment not found." >&2
        exit 1
    fi
    colcon build --packages-select small_gicp hikcamera \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev
fi

echo ""
echo "[OK] post-create complete. Ready to develop."

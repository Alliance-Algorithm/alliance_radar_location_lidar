#!/bin/bash
set -euo pipefail

echo "========================================="
echo "  RADAR-LOCATION-LIDAR post-create setup"
echo "========================================="

# Sync host opencode config into container
# The bind mount already makes ~/.opencode available,
# but we ensure the binary is in PATH
if [ -f /home/ubuntu/.opencode/bin/opencode ]; then
    echo "[OK] opencode binary found from host mount"
fi

# Sync host codex config
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
    source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/$ROS_DISTRO/setup.bash
    colcon build --packages-select small_gicp hikcamera \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev 2>/dev/null || true
fi

echo ""
echo "[OK] post-create complete. Ready to develop."

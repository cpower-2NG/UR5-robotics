#!/usr/bin/env bash
# ============================================================
# 一键启动 Gazebo 仿真（含 ros_gz_bridge）
# 用法: bash scripts/launch_sim.sh
# ============================================================
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="${HERE%/scripts}"

echo "=== 启动 Gazebo Harmonic 仿真 ==="

# 1. Source ROS2 Jazzy
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "[ERROR] ROS2 Jazzy not found at /opt/ros/jazzy/" >&2
    exit 1
fi

# 2. Source 本工作区
if [ -f "${ROOT}/ros2_ws/install/setup.bash" ]; then
    source "${ROOT}/ros2_ws/install/setup.bash"
else
    echo "[WARN] 工作区未编译，先执行 colcon build..."
    cd "${ROOT}/ros2_ws"
    colcon build --symlink-install
    source install/setup.bash
fi

# 3. 启动
ros2 launch yolo_bringup simulation.launch.py "$@"

#!/usr/bin/env bash
# ============================================================
# 一键启动完整系统（Gazebo + 感知流水线）
# 用法: bash scripts/launch_all.sh
# ============================================================
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="${HERE%/scripts}"

echo "=== 启动 YOLO+ROS2 完整抓取系统 ==="

# 1. Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# 2. Source 本工作区
if [ -f "${ROOT}/ros2_ws/install/setup.bash" ]; then
    source "${ROOT}/ros2_ws/install/setup.bash"
else
    echo "[ERROR] 工作区未编译，请先执行: cd ros2_ws && colcon build --symlink-install" >&2
    exit 1
fi

# 3. 启动
ros2 launch yolo_bringup bringup.launch.py "$@"

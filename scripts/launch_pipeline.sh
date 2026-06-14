#!/usr/bin/env bash
# ============================================================
# 一键启动感知流水线（yolo_detector + perception_3d + grasp_planner）
# 用法: bash scripts/launch_pipeline.sh [mode:=mock]
# ============================================================
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="${HERE%/scripts}"

echo "=== 启动感知流水线 ==="

# 1. Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# 2. Source 本工作区
if [ -f "${ROOT}/ros2_ws/install/setup.bash" ]; then
    source "${ROOT}/ros2_ws/install/setup.bash"
else
    echo "[ERROR] 工作区未编译，请先执行: cd ros2_ws && colcon build --symlink-install" >&2
    exit 1
fi

# 3. 启动（默认 mock 模式）
MODE="${1:-mode:=mock}"
ros2 launch yolo_bringup pipeline.launch.py "${MODE}"

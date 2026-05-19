#!/usr/bin/env bash
set -euo pipefail

# 创建 ROS2 工作区并添加占位包（需系统已安装 ROS2）
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="${ROOT_DIR}/ros2_ws"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

echo "Creating ROS2 workspace at: ${WS_DIR} (ROS2 distro: ${ROS_DISTRO})"
mkdir -p "${WS_DIR}/src"
cd "${WS_DIR}"

if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  echo "Warning: /opt/ros/${ROS_DISTRO}/setup.bash not found. Please install ROS2 ${ROS_DISTRO} and source it." >&2
  echo "Proceeding to create workspace folders only."
else
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  echo "Creating placeholder ROS2 Python packages (if ros2 CLI available)"
  if command -v ros2 >/dev/null 2>&1; then
    ros2 pkg create --build-type ament_python yolo_detector || true
    ros2 pkg create --build-type ament_python perception_3d || true
    ros2 pkg create --build-type ament_python ik_solver || true
  else
    echo "ros2 CLI not found in PATH despite /opt/ros/${ROS_DISTRO} existing." >&2
  fi
fi

echo "You can build the workspace with:"
echo "  cd ${WS_DIR} && colcon build"

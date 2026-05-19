#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# 一键环境配置脚本
# ============================================================

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="${HERE%/scripts}"

echo "========================================"
echo " YOLO+ROS2 抓取系统 — 一键配置"
echo "========================================"

# 1. 安装系统依赖
echo ""
read -p "是否安装系统依赖 (ROS2+Gazebo+MoveIt2)? [y/N] " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    bash "${HERE}/install_deps.sh"
fi

# 2. 创建 Conda 环境
echo ""
read -p "是否创建/更新 Conda 环境? [y/N] " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    bash "${HERE}/create_env.sh"
fi

# 3. 运行最小验证
echo ""
echo "--- 运行最小验证测试 ---"
conda run -n yolo_ros python "${HERE}/test_minimal.py"

echo ""
echo "========================================"
echo " 配置完成！"
echo ""
echo " 构建 ROS2 工作区:"
echo "   conda activate yolo_ros"
echo "   cd ${ROOT}/ros2_ws"
echo "   colcon build --symlink-install"
echo "   source install/setup.bash"
echo ""
echo " 启动仿真 (需先安装系统依赖):"
echo "   ros2 launch yolo_bringup bringup.launch.py"
echo "========================================"

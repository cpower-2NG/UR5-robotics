#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# 安装 ROS2 Jazzy + Gazebo Harmonic + MoveIt2 系统依赖
# 适用于: Ubuntu 24.04 (Noble Numbat)
# ============================================================

echo "========================================"
echo " YOLO+ROS2 抓取系统 — 系统依赖安装脚本"
echo " 目标: Ubuntu 24.04 + ROS2 Jazzy"
echo "========================================"

# ---- 1. 配置 ROS2 Jazzy 仓库 ----
echo ""
echo "[1/5] 配置 ROS2 Jazzy 仓库..."

if ! dpkg -l | grep -q ros-jazzy-desktop 2>/dev/null; then
    sudo apt update
    sudo apt install -y software-properties-common curl gnupg lsb-release

    # 添加 ROS2 GPG key
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

    # 添加 ROS2 源
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update
    echo "ROS2 仓库已配置。"
else
    echo "ROS2 仓库已存在。"
fi

# ---- 2. 安装 ROS2 Jazzy ----
echo ""
echo "[2/5] 安装 ROS2 Jazzy Desktop..."

sudo apt install -y \
    ros-jazzy-desktop \
    ros-jazzy-ros-base \
    python3-rosdep \
    python3-colcon-common-extensions

echo "ROS2 Jazzy 安装完成。"

# ---- 3. 安装 Gazebo Harmonic ----
echo ""
echo "[3/5] 安装 Gazebo Harmonic..."

# Gazebo Harmonic 通过 ROS2 vendor 包拉取（无需额外 gz-harmonic）
sudo apt install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-image

echo "Gazebo Harmonic 安装完成。"

# ---- 4. 安装 MoveIt2 及相关包 ----
echo ""
echo "[4/5] 安装 MoveIt2 及感知依赖..."

sudo apt install -y \
    ros-jazzy-moveit \
    ros-jazzy-moveit-visual-tools \
    ros-jazzy-vision-msgs \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-image-geometry \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-rviz2 \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro

echo "MoveIt2 及相关包安装完成。"

# ---- 5. 初始化 rosdep ----
echo ""
echo "[5/5] 初始化 rosdep..."

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

echo ""
echo "========================================"
echo " 系统依赖安装完成！"
echo ""
echo " 下一步:"
echo "   1. conda env create -f environment.yml"
echo "   2. conda activate yolo_ros"
echo "   3. cd ros2_ws && colcon build --symlink-install"
echo "   4. source install/setup.bash"
echo "   5. ros2 launch yolo_bringup bringup.launch.py"
echo "========================================"

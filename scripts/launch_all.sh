#!/usr/bin/env bash
# ============================================================
# 一键启动完整系统（Gazebo + 感知流水线）
# 用法: bash scripts/launch_all.sh
# ============================================================
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="${HERE%/scripts}"

echo "=== 启动 YOLO+ROS2 完整抓取系统 ==="

# 1. Source ROS2（本机是 Humble；用 ROS_DISTRO 可覆盖）
# ⚠️ 这里曾写死 /opt/ros/jazzy/setup.bash —— 该路径在本机**不存在**，而本脚本
# 第 6 行是 `set -eo pipefail`，source 失败会**当场中止**，于是永远走不到下面
# 第 25 行那个正确的 bringup 入口。工作区路径都做了存在性检查，唯独 ROS 路径没有。
ROS_DISTRO="${ROS_DISTRO:-humble}"
if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    echo "[ERROR] 找不到 /opt/ros/${ROS_DISTRO}/setup.bash（本机为 Humble）" >&2
    exit 1
fi
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# 2. Source 本工作区
if [ -f "${ROOT}/ros2_ws/install/setup.bash" ]; then
    source "${ROOT}/ros2_ws/install/setup.bash"
else
    # 注意：**不要加 --symlink-install**。叠加 Python 依赖环境（venv/conda）后
    # 它会触发 setuptools 的 `setup.py develop --uninstall`，而新版 setuptools
    # 已移除该子命令，构建必失败（见 RESTRUCTURE_PLAN 的 E2）。构建应在
    # **纯净 ROS 环境**下做，运行期才叠加 Python 依赖。
    echo "[ERROR] 工作区未编译，请先执行: cd ros2_ws && colcon build" >&2
    exit 1
fi

# 3. 启动
ros2 launch yolo_bringup bringup.launch.py "$@"

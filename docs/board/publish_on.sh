#!/bin/bash
# 把手眼标定发成 TF：`tool0 → camera_link`（腕部相机的树与臂的树靠这一条接上）。
#
# ✅ 2026-10-25 起**不再依赖** `~/handeye_ws/scripts/publish_on.sh` 与
#    easy_handeye2 的 `handeye_publisher`：直接跑仓库内的
#    `scripts/wrist_tf_publish.py`，标定值取自
#    `docs/calib/wrist_eih_camlink-2026-10-24.json`（纯数字，无序列号）。
#
# ⚠️ 为什么是 `camera_link` 而不是 `camera_color_optical_frame`：
#    RealSense 驱动自己会发 `camera_link → camera_color_optical_frame`。发到
#    光学帧会让它**有两个父节点**，而且绕过驱动内部那一段平移（两块板的实测
#    间距约 15 mm；俯视相机上它造成过 17.46 mm 的落地误差，见
#    `docs/当前状态.md` §3）。
#
# ⚠️ 标定文件是**派生量**（由 `wrist_eih.calib` 的 tool0→光学帧合成），
#    不是一次独立标定。重标之后要重新生成这份 JSON，别只替换一半。
#
# ⚠️ 别同时起第二个发同一对帧的东西（`real.launch.py publish_camera_tf:=true`、
#    easy_handeye2 的 `handeye_publisher`）—— `camera_link` 两个父节点，
#    TF 树会时对时错。
#
# 用法（板上，仓库根目录）：
#     ./docs/board/publish_on.sh                   # 发 TF，前台常驻
#     python3 scripts/wrist_tf_publish.py --check  # 只验标定文件（不碰 ROS）
#     python3 scripts/wrist_tf_publish.py --verify # 直查 TF，对不上就退 1
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CAL="$REPO/docs/calib/wrist_eih_camlink-2026-10-24.json"

if [ ! -f "$CAL" ]; then
    echo "❌ 标定文件不在：$CAL"
    echo "   ⇒ 别用默认值往下跑（那会发一条**错的** TF，而且不报错）。"
    exit 1
fi

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"

exec python3 "$REPO/scripts/wrist_tf_publish.py" --calib "$CAL" "$@"

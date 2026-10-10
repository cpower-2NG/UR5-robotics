#!/bin/bash
# 把手眼标定发成 TF：`tool0 → camera_color_optical_frame`。
#
# ⚠️ 这条 TF 就是**腕部相机能不能用**的开关。没有它，相机那棵树和机械臂那棵树
#    是**两棵断开的树**（`tf2_echo` 会报 "two or more unconnected trees"），
#    相机看到的东西算出来的坐标落不进基座系，下游一步都走不了。
#    它由 easy_handeye2 的 `handeye_publisher` 发，读的是
#    `~/.ros2/easy_handeye2/calibrations/wrist_eih.calib` —— 2026-10-10 标定，
#    算法 **Tsai-Lenz**（算法名**不进**存盘文件，所以记在这里）。
#
# ⚠️ **不要**改用 `calibrate.launch.py` 起服务端：它第 14-17 行会发一条拿
#    `tracking_base_frame` 当子帧的**假 TF**，正好和这条打架。
#
# ⚠️ `handeye_publisher` 只发**静态** TF（就一条），所以它是「起一次就不用管」的；
#    但也正因为是静态的，标定文件**换了它不会自己重读** —— 重标之后必须重启它。
source /opt/ros/humble/setup.bash
source ~/handeye_ws/install/setup.bash
export ROS_DOMAIN_ID=99

CAL=~/.ros2/easy_handeye2/calibrations/wrist_eih.calib
if [ ! -f "$CAL" ]; then
    echo "❌ 标定文件不在：$CAL"
    echo "   ⇒ 别用默认值往下跑（那会发一条**错的** TF，而且不报错）。"
    echo "     先确认标定做没做、文件名对不对。"
    exit 1
fi

exec ros2 launch easy_handeye2 publish.launch.py name:=wrist_eih

#!/bin/bash
# 起腕部相机（Intel RealSense）。
#
# ⚠️ **参数文件必须放在持久位置。** 2026-10-10 之前它一直在
#    `/tmp/wrist_test/wrist_swapped.yaml` 下，而这块板子的开机规则是
#    `/usr/lib/tmpfiles.d/tmp.conf` 里的 `D /tmp` —— **开机清空**。
#    所以断一次电，相机就再也起不来。而且它是「文件找不到」这种**直白的**失败，
#    不是静默的 —— 但也足够让「腕部相机为主」这条路当天开不了工。
#    真值现在在 `~/.config/ur5-vision-grasp/wrist_camera.yaml`（**仓库外**，红线 3：
#    里面是相机序列号，不能进仓）。
#
# ⚠️ **认相机只认外壳标签上的 SDK 号**（红线 5）。参数文件里就是拿这个号选相机的；
#    用 ASIC 号**不报错，只是静默打开另一台相机**。
#
# ⚠️ 相机必须插 **USB3** 口。插 USB2 上「能枚举、能绑、能起节点，但 **0 帧**」，
#    日志里刷 XU Protocol error。下面顺手报一下速度：
#    5000 = USB3，480 = USB2。
source /opt/ros/humble/setup.bash
source ~/ur5-vision-grasp/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=99

CFG=~/.config/ur5-vision-grasp/wrist_camera.yaml
if [ ! -f "$CFG" ]; then
    echo "❌ 相机参数文件不在：$CFG"
    echo "   ⇒ **不要**改成不带 --params-file 硬起：那样会按默认值打开"
    echo "     「碰巧先枚举到」的那台相机，而且**不报错**。"
    exit 1
fi

# 只看速度，不印序列号（红线 3）
for d in /sys/bus/usb/devices/*/; do
    [ -r "$d/idVendor" ] || continue
    [ "$(cat "$d/idVendor" 2>/dev/null)" = "8086" ] || continue
    sp=$(cat "$d/speed" 2>/dev/null)
    [ "$sp" = "480" ] && echo "⚠️ 有一台 RealSense 挂在 USB2（speed=$sp）⇒ 大概率 0 帧"
done

exec ros2 run realsense2_camera realsense2_camera_node \
    --ros-args -r __ns:=/wrist --params-file "$CFG"

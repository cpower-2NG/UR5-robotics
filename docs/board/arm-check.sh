#!/usr/bin/env bash
# arm-check.sh — 机械臂控制链路快速体检 (不需要 sudo)
CUR=$(( $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq 2>/dev/null || echo 0) / 1000 ))
MAXF=$(( $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq 2>/dev/null || echo 0) / 1000 ))
echo "== CPU =="
echo "核数: $(nproc) | 调度器: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null) | 当前: ${CUR} MHz (上限 ${MAXF})"
echo "== 温度 (°C, 持续超过 75 需关注) =="
for z in /sys/class/thermal/thermal_zone*/; do
  [ -f "$z/type" ] && t=$(cat $z/temp 2>/dev/null) && [ -n "$t" ] && echo "  $(cat $z/type): $((t/1000))"
done | sort -t: -k2 -rn | head -6
echo "== 内存/交换 =="; free -h | sed -n '1,2p'
echo "== 磁盘 =="; df -h / | tail -1 | awk '{print "  已用 " $5 "  剩余 " $4}'
echo "== LLM 远程服务 <ip>:8000 =="
t0=$(date +%s%N)
if curl -s -m 5 http://<ip>:8000/v1/models >/dev/null 2>&1; then
  t1=$(date +%s%N); echo "  可达, 延迟 $(((t1-t0)/1000000)) ms"
else echo "  !! 不可达 (检查 10.108.16 网段连通性)"; fi
echo "== 串口 (CH340/机械臂) =="
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "  未检测到串口设备 (线未插或 ch34x 驱动未加载)"
echo "== ROS2 环境 =="
env | grep -E '^ROS' | sort | sed 's/^/  /'
if [ -z "$(env | grep '^ROS_DISTRO')" ]; then echo "  (当前 shell 未 source ROS2)"; fi

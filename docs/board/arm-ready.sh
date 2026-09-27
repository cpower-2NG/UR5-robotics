#!/usr/bin/env bash
# arm-ready.sh — 把 Jetson Orin 调到「机械臂控制/开发」高性能状态
# 用法: sudo ~/arm-ready.sh        (调优)
#       sudo ~/arm-ready.sh check  (只查看当前状态, 不做修改)
set -e
FAN=/sys/class/hwmon/hwmon0/pwm1
SHOW() {
  echo "== 当前状态 =="
  echo "CPU 核数: $(nproc)   调度器: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null)"
  echo "CPU 最大频: $(( $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq 2>/dev/null || echo 0) / 1000 )) MHz"
  [ -f "$FAN" ] && echo "风扇 PWM: $(cat $FAN)  (nvfancontrol 闭环管理, 无需手动干预)"
  T=$(cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null || true)
  [ -n "$T" ] && echo "温度(C): cpu=$((T/1000))"
  free -h | sed -n '1p'
  df -h / | tail -1
  echo "swappiness: $(cat /proc/sys/vm/swappiness)"
}

if [ "$1" = "check" ]; then SHOW; exit 0; fi

[ "$(id -u)" = 0 ] || { echo "请用 sudo 运行: sudo $0"; exit 1; }

echo ">> 1/3 CPU 调度器 -> performance (固定高频, 降低控制环路抖动)"
for c in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
  echo performance > "$c" 2>/dev/null || true
done
echo "   现在: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor)"
# 开机持久化
cat > /etc/systemd/system/arm-cpufreq.service <<'SVC'
[Unit]
Description=Set CPU governor to performance for arm control
After=multi-user.target
[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/bin/sh -c 'for c in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do echo performance > $c; done'
[Install]
WantedBy=multi-user.target
SVC
systemctl enable arm-cpufreq.service >/dev/null 2>&1 && echo "   已设为开机自动生效"

# 说明: 风扇由 nvfancontrol 闭环温控, 手动写 pwm1 会在 2 秒内被覆盖, 故不在此设置
echo ">> 2/3 内存: swappiness=10 (优先用内存, 减少换页抖动)"
sysctl -w vm.swappiness=10
if [ ! -f /etc/sysctl.d/99-arm-tuning.conf ]; then
  echo "vm.swappiness=10" > /etc/sysctl.d/99-arm-tuning.conf
  echo "   已写入 /etc/sysctl.d/99-arm-tuning.conf (开机持久)"
fi

echo ">> 3/3 完成, 当前状态:"
SHOW
echo
echo "可选(更激进的实时性): 关闭透明大页对控制节点进程有帮助, 如需可执行:"
echo "  echo never > /sys/kernel/mm/transparent_hugepage/enabled"

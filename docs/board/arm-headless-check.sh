#!/usr/bin/env bash
# arm-headless-check.sh — 无头运行（只接电源+网线）可行性体检
# 不需要 sudo。拔显示器前后各跑一次，对比输出即可判断要不要买显示假负载。
#
#   ~/arm-headless-check.sh

echo "===== 1. 自动登录（决定重启后远程桌面能否自己起来）====="
if grep -qs 'AutomaticLoginEnable=True' /etc/gdm3/custom.conf; then
  echo "  [ok] 已开启自动登录: $(grep '^AutomaticLogin=' /etc/gdm3/custom.conf)"
else
  echo "  [!!] 未开启自动登录 —— 重启后会停在登录界面，远程桌面连不上（SSH 仍可用）"
fi

echo
echo "===== 2. 图形会话 ====="
sess=$(loginctl list-sessions --no-pager 2>/dev/null | awk '$3=="jetson"{print $1; exit}')
if [ -n "$sess" ]; then
  printf '  会话 %s: type=%s state=%s remote=%s\n' "$sess" \
    "$(loginctl show-session "$sess" -p Type   --value 2>/dev/null)" \
    "$(loginctl show-session "$sess" -p State  --value 2>/dev/null)" \
    "$(loginctl show-session "$sess" -p Remote --value 2>/dev/null)"
else
  echo "  [!!] 没有活动的 jetson 图形会话"
fi

echo
echo "===== 3. 显示输出（关键：拔线后这里是否还 connected）====="
for s in /sys/class/drm/card*-*/status; do
  [ -f "$s" ] || continue
  printf '  %-16s %s\n' "$(basename "$(dirname "$s")")" "$(cat "$s")"
done

echo
echo "===== 4. X 当前分辨率（拔线后若塌到 640x480 就需要假负载）====="
disp=":0"; xauth="/run/user/1000/gdm/Xauthority"
if [ -f "$xauth" ]; then
  line=$(DISPLAY=$disp XAUTHORITY=$xauth xrandr 2>/dev/null | sed -n '1p')
  if [ -n "$line" ]; then
    echo "  $line"
    cur=$(echo "$line" | sed -n 's/.*current \([0-9]*\) x \([0-9]*\).*/\1x\2/p')
    w=$(echo "$cur" | cut -dx -f1)
    if [ -n "$w" ] && [ "$w" -ge 1280 ] 2>/dev/null; then
      echo "  [ok] 分辨率正常，远程桌面可用"
    else
      echo "  [!!] 分辨率偏低（$cur），远程桌面会很难用 -> 需要假负载或保持显示器连接"
    fi
  else
    echo "  [!!] 取不到 xrandr 输出，X 可能已无可用输出"
  fi
else
  echo "  (找不到 Xauthority，跳过)"
fi

echo
echo "===== 5. 远程入口 ====="
printf '  SSH   22   : %s\n' "$(ss -tlnH 2>/dev/null | grep -q ':22 ' && echo listening || echo '未监听')"
printf '  RDP   3389 : %s\n' "$(ss -tlnH 2>/dev/null | grep -q ':3389 ' && echo listening || echo '未监听')"
printf '  VNC   5900 : %s\n' "$(ss -tlnH 2>/dev/null | grep -q ':5900 ' && echo listening || echo '未监听')"
printf '  网口地址   : %s\n' "$(ip -br addr show eno1 2>/dev/null | awk '{print $3}')"

echo
echo "===== 6. 救砖通道（无显示器时的保命手段）====="
for u in serial-getty@ttyTCU0.service serial-getty@ttyGS0.service; do
  printf '  %-28s %s\n' "$u" "$(systemctl is-active "$u" 2>/dev/null)"
done
[ -c /dev/ttyTCU0 ] && echo "  [ok] /dev/ttyTCU0 存在（调试串口，接 USB-TTL 即可看开机过程）"

echo
echo "===== 结论 ====="
echo "  电源 + 网线即可远程使用，前提是第 3、4 项保持正常。"
echo "  键鼠和 USB Hub 可以直接拔；显示器是否必须留着，看第 4 项拔线后的结果。"

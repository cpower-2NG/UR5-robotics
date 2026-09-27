#!/usr/bin/env bash
# arm-isolation-sudo.sh — 资源隔离（体检清单 D4）：给机械臂控制栈留出专用 CPU / 内存
#
#   sudo ~/arm-isolation-sudo.sh check     只查看当前设置
#   sudo ~/arm-isolation-sudo.sh apply     应用（立刻生效，且开机保持）
#   sudo ~/arm-isolation-sudo.sh revert    撤销，恢复不限
#
# 策略:
#   控制栈   -> arm-control.slice     固定 4-5 号核 | 内存保底 1G | CPU/IO 高权重
#   桌面会话 -> user-1000.slice       压到 0-3 号核 | 内存软限 4.5G 硬限 6G
#
# 为什么现在不自动应用: apply 后你正在用的桌面被限制到 4 个核，
# 浏览器/编译会略慢；等真正开始跑 UR5 控制节点时再执行更合适。
set -u

[ "$(id -u)" = 0 ] || { echo "请用 sudo 运行: sudo $0"; exit 1; }

CTRL_SLICE=/etc/systemd/system/arm-control.slice
USER_DROPIN=/etc/systemd/system/user-1000.slice.d/50-arm-limits.conf

show_state(){
  echo "== arm-control.slice =="
  if [ -f "$CTRL_SLICE" ]; then
    grep -E '^(CPUWeight|IOWeight|MemoryMin|AllowedCPUs|CPUAffinity)=' "$CTRL_SLICE" | sed 's/^/  /'
    systemctl show arm-control.slice -p AllowedCPUs -p MemoryMin -p CPUWeight 2>/dev/null | sed 's/^/  /'
  else
    echo "  (未创建)"
  fi
  echo "== user-1000.slice =="
  if [ -f "$USER_DROPIN" ]; then
    grep -E '^(AllowedCPUs|CPUAffinity|CPUQuota|MemoryHigh|MemoryMax)=' "$USER_DROPIN" | sed 's/^/  /'
  else
    echo "  (未限制)"
  fi
  systemctl show user-1000.slice -p AllowedCPUs -p CPUQuotaPerSecUSec -p MemoryHigh -p MemoryMax -p MemoryCurrent 2>/dev/null | sed 's/^/  /'
  echo "== 当前内存 =="
  free -h | head -2
}

case "${1:-check}" in
  check|"")
    show_state
    ;;

  apply)
    echo ">> 创建 arm-control.slice（控制栈专用）"
    cat > "$CTRL_SLICE" <<'EOF'
[Unit]
Description=Slice for robot arm control (UR5 / ROS2)
Before=slices.target

[Slice]
CPUWeight=1000
IOWeight=1000
MemoryMin=1G
AllowedCPUs=4-5
CPUAffinity=4-5
EOF
    systemctl daemon-reload
    echo "   ok: $CTRL_SLICE"

    echo ">> 限制桌面会话（user-1000.slice）"
    install -d "$(dirname "$USER_DROPIN")"
    cat > "$USER_DROPIN" <<'EOF'
[Slice]
AllowedCPUs=0-3
MemoryHigh=4500M
MemoryMax=6000M
EOF
    systemctl daemon-reload
    # 立刻在运行中的会话上生效（daemon-reload 不会自动重设已有 slice 的 cgroup 属性）
    systemctl set-property user-1000.slice AllowedCPUs=0-3 MemoryHigh=4500M MemoryMax=6000M --runtime 2>/dev/null \
      || systemctl set-property user-1000.slice AllowedCPUs=0-3 MemoryHigh=4500M MemoryMax=6000M
    echo "   ok: $USER_DROPIN（已即时生效）"
    echo
    echo ">> 控制节点请这样启动，才会落在专用切片里："
    echo "   systemd-run --slice=arm-control.slice -p AllowedCPUs=4-5 --pty bash"
    echo "   或者把你自己的 launch 脚本包成一个 systemd 服务，Slice=arm-control.slice"
    echo
    show_state
    ;;

  revert)
    echo ">> 撤销限制"
    [ -f "$CTRL_SLICE" ] && rm "$CTRL_SLICE" && echo "   已删除 $CTRL_SLICE"
    [ -f "$USER_DROPIN" ] && rm "$USER_DROPIN" && echo "   已删除 $USER_DROPIN"
    systemctl daemon-reload
    systemctl set-property user-1000.slice AllowedCPUs= MemoryHigh=infinity MemoryMax=infinity 2>/dev/null || true
    show_state
    ;;

  *)
    echo "用法: sudo $0 {check|apply|revert}"
    exit 1
    ;;
esac

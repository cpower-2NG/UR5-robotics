#!/usr/bin/env bash
# arm-cleanup-sudo.sh — 需要 root 的部分（对应体检清单 A2~A6 / A8 / B1 / B7 / B8 / D2）
#   sudo ~/arm-cleanup-sudo.sh         执行（扩容那一步会单独询问你）
#   sudo ~/arm-cleanup-sudo.sh check   只查看现状，不改动任何东西
#
# 保留不动: ssh(22)、gnome-remote-desktop(3389/5900)、clash 代理、ollama 服务、docker
set -u

[ "$(id -u)" = 0 ] || { echo "请用 sudo 运行: sudo $0"; exit 1; }

hdr(){ printf '\n===== %s =====\n' "$*"; }
ok(){ printf '  [ok] %s\n' "$*"; }
skip(){ printf '  [跳过] %s\n' "$*"; }

hdr "0. 执行前现状"
df -h / | tail -1
swapon --show --raw
echo "透明大页: $(cat /sys/kernel/mm/transparent_hugepage/enabled)"
echo "防火墙:   $(ufw status 2>/dev/null | head -1)"
for u in jupyterlab rpcbind snap.cups.cupsd ModemManager bluetooth packagekit avahi-daemon ssh gnome-remote-desktop; do
  printf '  %-24s enable=%-9s active=%s\n' "$u" \
    "$(systemctl is-enabled "$u" 2>/dev/null || echo '-')" \
    "$(systemctl is-active "$u" 2>/dev/null || echo '-')"
done

[ "${1:-}" = "check" ] && exit 0

# ---------------------------------------------------------------- A2~A6
hdr "A2~A6 关停与机械臂开发无关的服务"
for u in jupyterlab.service rpcbind.service rpcbind.socket \
         snap.cups.cupsd.service snap.cups.cups-browsed.service lpd.service \
         ModemManager.service bluetooth.service packagekit.service \
         avahi-daemon.service avahi-daemon.socket; do
  if systemctl list-unit-files "$u" 2>/dev/null | grep -q "$u"; then
    if systemctl disable --now "$u" >/dev/null 2>&1; then ok "已停用 $u"; else skip "$u 停用失败"; fi
  else
    skip "$u 不存在"
  fi
done

# ---------------------------------------------------------------- A8
hdr "A8 启用 ufw（默认拒绝入站，放行 ssh / RDP / VNC）"
if ufw status 2>/dev/null | grep -q "Status: active"; then
  skip "ufw 已处于启用状态，不重复配置"
else
  ufw default deny incoming >/dev/null
  ufw default allow outgoing >/dev/null
  ufw allow 22/tcp   comment 'ssh' >/dev/null
  ufw allow 3389/tcp comment 'rdp' >/dev/null
  ufw allow 5900/tcp comment 'vnc' >/dev/null
  ufw --force enable >/dev/null
  ok "ufw 已启用"
fi
ufw status verbose | head -12
echo "  提示: 以后要多机 ROS2 联调时，需要放行 DDS 端口，例如"
echo "        ufw allow from <ip>/23 to any port 7400:7500 proto udp"

# ---------------------------------------------------------------- B1
hdr "B1 清理 apt 缓存"
apt-get clean
ok "apt 缓存已清（/var/cache/apt 现为 $(du -sh /var/cache/apt 2>/dev/null | cut -f1)）"

# ---------------------------------------------------------------- B7
hdr "B7 磁盘 swap 瘦身 8G -> 2G，并修正 fstab 重复行"
if swapon --show --raw 2>/dev/null | awk '{print $1}' | grep -qx '/var/8GB.swap'; then
  swapoff /var/8GB.swap && ok "已 swapoff /var/8GB.swap"
fi
if [ -f /var/8GB.swap ]; then
  truncate -s 2G /var/8GB.swap
  chmod 0600 /var/8GB.swap
  mkswap /var/8GB.swap >/dev/null
  swapon /var/8GB.swap && ok "swap 文件已重建为 2G 并重新启用"
fi
cp -a /etc/fstab "/etc/fstab.bak-$(date +%Y%m%d)"
awk '!/^\/var\/8GB\.swap/ {print} END {print "/var/8GB.swap none swap sw,pri=-2 0 0"}' \
    "/etc/fstab.bak-$(date +%Y%m%d)" > /etc/fstab
if mount -a 2>/dev/null && swapon --show --raw | grep -q zram; then
  ok "fstab 中 swap 行已收敛为 1 行（备份 /etc/fstab.bak-$(date +%Y%m%d)）"
else
  cp -a "/etc/fstab.bak-$(date +%Y%m%d)" /etc/fstab
  skip "fstab 校验未通过，已回滚"
fi
swapon --show --raw
echo "  说明: zram 优先级 5 高于 swap 文件 -2，内核优先用 zram，本来就是对的，无需调整。"

# ---------------------------------------------------------------- B8
hdr "B8 根分区扩容"
DISK=/dev/nvme0n1
PART=${DISK}p1
disk_sz=$(cat /sys/class/block/nvme0n1/size)
p1_start=$(cat /sys/class/block/nvme0n1p1/start)
p1_sz=$(cat /sys/class/block/nvme0n1p1/size)
tail_mb=$(( (disk_sz - p1_start - p1_sz) * 512 / 1024 / 1024 ))
echo "  根分区后可扩展空间: ${tail_mb} MiB"
if [ "$tail_mb" -lt 1024 ]; then
  skip "没有可扩展空间"
else
  echo "  ! 这是在线扩容根文件系统，过程中断电会有风险，建议确保供电稳定。"
  read -r -p "  现在扩容吗? 输入 yes 继续，其它任意键跳过: " ans
  if [ "$ans" = "yes" ]; then
    command -v growpart >/dev/null 2>&1 || apt-get install -y cloud-guest-utils >/dev/null 2>&1 || true
    if command -v growpart >/dev/null 2>&1; then
      growpart "$DISK" 1 || parted -s "$DISK" resizepart 1 100%
    else
      parted -s "$DISK" resizepart 1 100%
    fi
    partx -u "$DISK" 2>/dev/null || partprobe "$DISK" 2>/dev/null || true
    sleep 1
    if [ "$(cat /sys/class/block/nvme0n1p1/size)" -gt "$p1_sz" ]; then
      resize2fs "$PART" && ok "文件系统已扩容"
      df -h / | tail -1
    else
      echo "  [注意] 内核尚未读取到新的分区表，重启后执行下面这条即可："
      echo "         sudo resize2fs $PART"
    fi
  else
    skip "已跳过扩容（随时可重跑本脚本）"
  fi
fi

# ---------------------------------------------------------------- D2
hdr "D2 透明大页 always -> madvise（开机持久）"
cat > /etc/systemd/system/arm-thp.service <<'SVC'
[Unit]
Description=Set transparent hugepage to madvise for lower control-loop latency
After=multi-user.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/bin/sh -c 'echo madvise > /sys/kernel/mm/transparent_hugepage/enabled'

[Install]
WantedBy=multi-user.target
SVC
systemctl daemon-reload
systemctl enable --now arm-thp.service >/dev/null 2>&1 && ok "已生效并设为开机自动"
echo "  透明大页: $(cat /sys/kernel/mm/transparent_hugepage/enabled)"

# ---------------------------------------------------------------- 收尾
hdr "完成"
df -h / | tail -1
free -h | head -2
echo
echo "资源隔离（CPU 核绑定 / 内存上限）没有放进本脚本，"
echo "因为那会限制你当前正在使用的桌面会话；等接机械臂时再跑 ~/arm-isolation-sudo.sh。"

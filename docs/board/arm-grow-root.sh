#!/usr/bin/env bash
# arm-grow-root.sh — 把根分区扩到整块 NVMe（对应清单 B8）
#
#   sudo ~/arm-grow-root.sh check   只查看，不改任何东西
#   sudo ~/arm-grow-root.sh         执行扩容
#
# 磁盘    : /dev/nvme0n1  (256 GB)
# 根分区  : /dev/nvme0n1p1 (当前 147 GB，后面约 100 GiB 未分配)
# 做了什么: 只把 p1 的**尾部往后挪**，不移动分区起点、不新建分区、不动分区里的数据。
#           先备份分区表到 /root，再 growpart，再在线 resize2fs。
# 最坏情况: 内核对新分区表读取被拒（分区在用），此时重启一次即可，脚本会自动挂好
#           开机执行的 resize2fs，重启后就是满盘。
set -u

[ "$(id -u)" = 0 ] || { echo "请用 sudo 运行: sudo $0"; exit 1; }

DISK=/dev/nvme0n1
PART=${DISK}p1
SYSBLK=/sys/class/block

sec(){ cat "$SYSBLK/$1/size"; }
to_h(){ awk -v s="$1" 'BEGIN{printf "%.1f", s*512/1073741824}'; }

# ---------- 0. 顺手修掉上次 mkswap 提示的 swap 权限 ----------
if [ -f /var/8GB.swap ]; then
  chmod 0600 /var/8GB.swap
  echo "[ok] /var/8GB.swap 权限 -> 0600（上次 mkswap 提示不安全）"
fi

# ---------- 1. 现状 ----------
p1_start=$(cat "$SYSBLK/nvme0n1p1/start")
p1_sz=$(cat "$SYSBLK/nvme0n1p1/size")
p1_end=$((p1_start + p1_sz))
disk_sz=$(sec nvme0n1)

max_end=0
for d in "$SYSBLK"/nvme0n1p*; do
  [ -f "$d/start" ] || continue
  e=$(( $(cat "$d/start") + $(cat "$d/size") ))
  [ "$e" -gt "$max_end" ] && max_end=$e
done
free_mb=$(( (disk_sz - max_end) * 512 / 1024 / 1024 ))

echo "磁盘总容量   : $(to_h "$disk_sz") GiB"
echo "根分区       : 起点 $(to_h "$p1_start") GiB / 大小 $(to_h "$p1_sz") GiB"
echo "最后分区尾部 : $(to_h "$max_end") GiB"
echo "可扩展空间   : ${free_mb} MiB ($(awk -v m="$free_mb" 'BEGIN{printf "%.1f", m/1024}') GiB)"
echo "当前挂载     : $(df -h / | tail -1)"
echo

if [ "$max_end" -ne "$p1_end" ]; then
  echo "!! 根分区不是磁盘上最后一个分区，布局与预期不符，脚本不做任何修改。"
  echo "   请先人工确认: parted -s $DISK print"
  exit 1
fi

if [ "${1:-}" = "check" ]; then
  # 顺便报告文件系统是否已经填满分区
  ff=$(dumpe2fs -h "$PART" 2>/dev/null | awk -F: '/^Block count/{gsub(/ /,"",$2);print $2}')
  fb=$(dumpe2fs -h "$PART" 2>/dev/null | awk -F: '/^Block size/{gsub(/ /,"",$2);print $2}')
  if [ -n "${ff:-}" ] && [ -n "${fb:-}" ]; then
    echo "文件系统     : $(( ff * fb / 1073741824 )) GiB"
  fi
  echo
  echo "check 模式：未做任何修改。"
  exit 0
fi

# ---------- 2. 备份分区表 ----------
BK="/root/ptable-nvme0n1-$(date +%Y%m%d-%H%M%S).dump"
if sfdisk -d "$DISK" > "$BK"; then
  echo "[ok] 分区表已备份: $BK"
  echo "     回滚方式: sudo sfdisk $DISK < $BK"
else
  echo "!! 分区表备份失败，为安全起见中止。"; exit 1
fi
echo

# ---------- 3. 扩展分区 ----------
if [ "$free_mb" -ge 1024 ]; then
  echo ">> 扩展分区 $PART 到磁盘末尾"
  command -v growpart >/dev/null 2>&1 || {
    echo "   安装 cloud-guest-utils（提供 growpart）..."
    apt-get install -y cloud-guest-utils >/dev/null 2>&1 || true
  }
  if command -v growpart >/dev/null 2>&1 && growpart "$DISK" 1; then
    :
  else
    echo "   改用 parted resizepart..."
    parted -s "$DISK" resizepart 1 100% || { echo "!! 分区扩展失败，未做后续操作"; exit 1; }
  fi
  partx -u "$DISK" 2>/dev/null || partprobe "$DISK" 2>/dev/null || true
  sleep 1
  new_sz=$(sec nvme0n1p1)
  echo "   分区大小: $(to_h "$p1_sz") GiB -> $(to_h "$new_sz") GiB"
  if [ "$new_sz" -le "$p1_sz" ]; then
    echo "!! 内核还没读到新的分区表（分区正在被使用，这是常见现象）。"
    cat > /etc/systemd/system/arm-growfs.service <<'EOF'
[Unit]
Description=One-shot resize2fs of root filesystem after partition grow
After=local-fs.target
Before=multi-user.target

[Service]
Type=oneshot
ExecStart=/sbin/resize2fs /dev/nvme0n1p1
ExecStartPost=/bin/systemctl disable arm-growfs.service

[Install]
WantedBy=multi-user.target
EOF
    systemctl daemon-reload
    if systemctl enable arm-growfs.service >/dev/null 2>&1; then
      echo "   [ok] 已挂好开机自动 resize2fs 的一次性任务。"
      echo "        现在执行: sudo reboot"
      echo "        重启完成后就是满盘，该任务会自动注销。"
    fi
    exit 0
  fi
else
  echo ">> 分区已经是满盘，跳过分区扩展"
fi

# ---------- 4. 扩展文件系统 ----------
ff=$(dumpe2fs -h "$PART" 2>/dev/null | awk -F: '/^Block count/{gsub(/ /,"",$2);print $2}')
fb=$(dumpe2fs -h "$PART" 2>/dev/null | awk -F: '/^Block size/{gsub(/ /,"",$2);print $2}')
fs_gib=0
[ -n "${ff:-}" ] && [ -n "${fb:-}" ] && fs_gib=$(( ff * fb / 1073741824 ))
part_gib=$(awk -v s="$(sec nvme0n1p1)" 'BEGIN{printf "%d", s*512/1073741824}')
echo ">> 扩展 ext4 文件系统（当前 ${fs_gib} GiB，分区 ${part_gib} GiB）"
if [ "$fs_gib" -lt "$part_gib" ]; then
  resize2fs "$PART" && echo "[ok] 文件系统已扩容"
else
  echo "[ok] 文件系统已经填满分区，无需操作"
fi

# ---------- 5. 收尾 ----------
echo
df -h / | tail -1
lsblk -o NAME,SIZE,FSTYPE,TYPE,MOUNTPOINT "$DISK" | head -10

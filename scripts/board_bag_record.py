#!/usr/bin/env python3
"""板端录包 → 回开发机回放。**在板子上跑。**

**为什么走这条路**（第三十一轮实测的结论）
----------------------------------------
板子桌面是关的（省内存），但**看**很重要。想把板上的画面/数据弄到开发机来看，
有三条路，实测后只剩这一条：

| 路线 | 实测结论 |
|---|---|
| 板子跑 Gazebo | ❌ 与本题无关，且会给「板子扛不扛得住」这个结论**添加仿真才有的负载** |
| 跨机 DDS 实时看 | ❌ 不通。板子 `<host-ip>/23` 与开发机 `<ip>/23` **不同子网**，DDS 默认发现用**多播 239.255.0.1**（不过路由）；换 Fast DDS Discovery Server 转单播后发现能通，但**板子→开发机的 UDP 被 Windows Hyper-V 防火墙 `DefaultInboundAction=Block` 丢掉** |
| **录包 + rsync + 本机回放** | ✅ **零 sudo、不依赖那条网络、可暂停倒放反复看** —— 最适合分析 |

**这一条能成立的根据**：已建立的 TCP 连接上实测 **7.4 MB/s**（出站发起不受
那条 Block 规则影响）。rsync 走的就是已建立 TCP，所以**搬得动**。

三种档位
--------
| profile | 录什么 | 30 秒大概多大 | 用途 |
|---|---|---|---|
| `plan`（默认） | 状态/检测/目标/日志，**不含图像** | **几 MB** | 查规划器状态机、TF、目标点 —— 最常用 |
| `perception` | 上面的 + **JPEG 压缩**的图像 | **约 50–100 MB** | 要看画面又不想搬原始图 |
| `full` | 上面的 + **原始**图像 | **约 2.5 GB** | 只在需要原始像素做算法分析时用 |

> `plan` 小到可以直接 `scp`，几乎不用等；`full` 的原始彩图 1280×720×3 ≈ 2.76 MB/帧，
> rsync 回开发机约 **6 分钟/30 秒录像** —— 脚本会在开跑前把预估打出来。

`perception` 档怎么来的
-----------------------
板上**没装** `image_transport_plugins`（第三十一轮实测），所以用不了现成的
`republish`；但 **cv2 板上现成**。本脚本因此会自己拉起
`scripts/image_jpeg_relay.py`（零安装），录它发布的 `<话题>/compressed`。
**开发机**那边装了 image_transport，RViz 直接 `Transport Hint: compressed` 就能解。

用法
----
    # 默认档，录 30 秒（板子上的链路要先用别的方式起着，见 README）
    python3 scripts/board_bag_record.py --seconds 30

    # 要看画面
    python3 scripts/board_bag_record.py --profile perception --seconds 60 \\
        --out ~/bags/pick_demo

    # 先列出「会录哪些、缺哪些」，不真录
    python3 scripts/board_bag_record.py --dry-run

⚠️ 本脚本**只录，不起链路**。它要和正在跑的链路并存，所以：
  · 无臂预检：另开一个终端跑 `scripts/board_chain_check.py`（或它起的那套 launch）
  · 真机调试：正常情况下是 `real.launch.py` 已经在跑
这样设计是因为真机场景下链路是驱动带着起来的，录制器只能挂上去。

退出码：0 = 录完；2 = 环境不满足；3 = 录完了但有关键话题一条消息都没有。
"""

import argparse
import os
import re
import shutil
import signal
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
QOS_OVERRIDES = os.path.join(REPO, "scripts/bag_qos_overrides.yaml")
RELAY = os.path.join(REPO, "scripts/image_jpeg_relay.py")

# 实测链路速率（MB/s）。est_seconds() 用它换算 rsync 预估时间。
# 来源：第三十一轮，板子当服务端、开发机当客户端，已建立 TCP 连接上测得。
DEFAULT_LINK_MBS = 7.4

# ⚠️ 这些话题名是**从代码里核实的**，不是猜的：
#   /detections       detector_node.py:171        vision_msgs/Detection2DArray
#   /detections_image detector_node.py:177        sensor_msgs/Image（publish_annotated 默认 true）
#   /targets_3d       projector_node.py:139       vision_msgs/Detection3DArray
#   /target_3d        projector_node.py:141       geometry_msgs/PoseStamped
#   /camera/...       RealSense 驱动 / 仿真桥
# ⚠️ `/target_3d_marker` **不在这里** —— 它只在 `grasping.rviz` 里被订阅，
#    全仓库**没有任何地方发布它**（已核实）。录它只会得到空话题。
# ⚠️ `planner_node` **一个发布者都没有**（全文件无 create_publisher），所以
#    规划器的决策（轨迹、分区）在 bag 里**看不到** —— 这是已知的可观测性缺口。
PLAN_TOPICS = [
    "/tf",
    "/tf_static",
    "/joint_states",
    "/detections",
    "/targets_3d",
    "/target_3d",
    "/robot_description",
    "/rosout",
]
# 图像话题。`plan` 档不录；`perception` 档录它们的 `/compressed` 版本；
# `full` 档录原始。
IMAGE_TOPICS = [
    "/camera/color/image_raw",
    "/detections_image",
]
# 深度图：只在 `full` 档带上（原始 16 位，比彩图还大，且**没有**压缩中继）。
# 投影的数值正确性靠它复核，所以不能完全不录 —— 但代价要说清楚。
DEPTH_TOPICS = [
    "/camera/aligned_depth_to_color/image_raw",   # 真机（pipeline.launch.py:41）
    "/camera/depth/image_raw",                    # 仿真（pipeline.launch.py:42）
]

PROFILES = {
    "plan":       {"images": "none",        "depth": False},
    "perception": {"images": "compressed",  "depth": False},
    "full":       {"images": "raw",         "depth": True},
}


def sh(cmd, timeout=60, **kw):
    try:
        return subprocess.run(cmd, shell=True, capture_output=True, text=True,
                              timeout=timeout, cwd=REPO, **kw)
    except subprocess.TimeoutExpired as e:
        out = e.stdout or ""
        if isinstance(out, bytes):
            out = out.decode(errors="replace")
        return subprocess.CompletedProcess(cmd, 124, out, "TIMEOUT")


def topic_types():
    """`ros2 topic list -t` → {话题: 类型}。拿不到就返回空字典。"""
    r = sh("ros2 topic list -t", timeout=30)
    out = {}
    for line in r.stdout.splitlines():
        m = re.match(r"^(\S+)\s+\[(.+)\]$", line.strip())
        if m:
            out[m.group(1)] = m.group(2)
    return out


def du_mb(path):
    total = 0
    for root, _, files in os.walk(path):
        for f in files:
            try:
                total += os.path.getsize(os.path.join(root, f))
            except OSError:
                pass
    return total / (1024 * 1024)


def est_seconds(mb, link_mbs):
    return mb / link_mbs if link_mbs > 0 else 0.0


def human(sec):
    if sec < 90:
        return f"{sec:.0f} 秒"
    return f"{sec / 60:.1f} 分钟"


def main():
    ap = argparse.ArgumentParser(
        description="板端录包（录完给 rsync 回开发机的命令）")
    ap.add_argument("--profile", choices=sorted(PROFILES), default="plan")
    ap.add_argument("--seconds", type=float, default=30.0,
                    help="录多久；0 = 一直录到 Ctrl-C")
    ap.add_argument("--out", default=None,
                    help="bag 目录（默认 ~/bags/<profile>_<时间戳>）")
    ap.add_argument("--link-mbs", type=float, default=DEFAULT_LINK_MBS,
                    help=f"回传速率 MB/s，用于估时（默认 {DEFAULT_LINK_MBS}，实测值）")
    ap.add_argument("--extra-topics", default="",
                    help="额外要录的话题，逗号分隔")
    ap.add_argument("--jpeg-quality", type=int, default=85)
    ap.add_argument("--jpeg-scale", type=float, default=1.0)
    ap.add_argument("--jpeg-max-rate", type=float, default=10.0,
                    help="压缩图像每话题最大频率 Hz，默认 10 —— 彩图原生 30 Hz，"
                         "而分析看不出 10 与 30 的差别，bag 却小三倍；0 = 不限")
    ap.add_argument("--dry-run", action="store_true",
                    help="只列出会录哪些、缺哪些，不真录")
    args = ap.parse_args()

    if not os.environ.get("ROS_DISTRO"):
        print("环境未就绪：先 source /opt/ros/humble/setup.bash 和 "
              "ros2_ws/install/setup.bash（约定：环境不满足退 2）")
        return 2
    if shutil.which("ros2") is None or sh("ros2 bag --help", timeout=20).returncode != 0:
        print("`ros2 bag` 不可用 —— 板上应已装 ros-humble-rosbag2（退 2）")
        return 2
    if not os.path.exists(QOS_OVERRIDES):
        print(f"缺 {QOS_OVERRIDES}（退 2）—— 没有它 /tf_static 与 /robot_description "
              "会录成空话题，回放时 TF 树断根")
        return 2

    prof = PROFILES[args.profile]
    stamp = time.strftime("%Y%m%d_%H%M%S")
    out = os.path.abspath(os.path.expanduser(
        args.out or f"~/bags/{args.profile}_{stamp}"))

    # ---- 1) 决定要录什么 -------------------------------------------------
    tmap = topic_types()
    if not tmap:
        print("`ros2 topic list -t` 没有任何输出 —— 链路没起来？"
              "（本脚本只录，不起链路）退 2")
        return 2
    print(f"板上共 {len(tmap)} 个话题。")

    wanted, missing = [], []
    for t in PLAN_TOPICS:
        (wanted if t in tmap else missing).append(t)

    relay_topics = []
    if prof["images"] == "compressed":
        for t in IMAGE_TOPICS:
            if t in tmap:
                relay_topics.append(t)
                wanted.append(t + "/compressed")
            else:
                missing.append(t)
    elif prof["images"] == "raw":
        for t in IMAGE_TOPICS:
            (wanted if t in tmap else missing).append(t)

    if prof["depth"]:
        present = [t for t in DEPTH_TOPICS if t in tmap]
        if present:
            wanted.extend(present)
        else:
            missing.extend(DEPTH_TOPICS)

    extra = [t.strip() for t in args.extra_topics.split(",") if t.strip()]
    for t in extra:
        (wanted if t in tmap else missing).append(t)

    print(f"\n档位 {args.profile}：录 {len(wanted)} 个话题")
    for t in wanted:
        print(f"  ✓ {t:52s} {tmap.get(t, '(中继发布)')}")
    if missing:
        print("\n⚠️ 以下话题**当前不存在**，录不到（不是 QoS 问题，是没在发）：")
        for t in missing:
            print(f"  ✗ {t}")
    if prof["images"] == "raw" and not args.dry_run:
        print("\n⚠️ full 档录原始图像：1280×720×3 ≈ 2.76 MB/帧，"
              "30 秒约 2.5 GB、回传约 6 分钟。只要看画面就用 perception 档。")

    if args.dry_run:
        print("\n--dry-run：到此为止，没有真录。")
        return 0

    if os.path.exists(out):
        print(f"\n输出目录已存在，不覆盖：{out}（退 2）")
        return 2

    # ---- 2) 需要压缩图像就先拉中继 ---------------------------------------
    relay = None
    procs_extra = []
    try:
        if relay_topics:
            cmd = [sys.executable, RELAY,
                   "--image-topic", ",".join(relay_topics),
                   "--quality", str(args.jpeg_quality),
                   "--scale", str(args.jpeg_scale),
                   "--max-rate", str(args.jpeg_max_rate)]
            print("\n压缩中继：" + " ".join(cmd))
            relay = subprocess.Popen(cmd, cwd=REPO, start_new_session=True)
            procs_extra.append(relay)
            # 等中继把 /compressed 话题挂上去，否则 rosbag2 可能错过开头。
            # （话题已存在时不会漏，但等一拍没有代价。）
            time.sleep(4.0)
            if relay.poll() is not None:
                print("压缩中继**启动即退出** —— 见上面的报错（退 2）")
                return 2
            for t in relay_topics:
                if t + "/compressed" not in topic_types():
                    print(f"⚠️ {t}/compressed 还没出现，"
                          "bag 里这个话题可能是空的")

        # ---- 3) 录 --------------------------------------------------------
        os.makedirs(os.path.dirname(out), exist_ok=True)
        cmd = ["ros2", "bag", "record", "-o", out,
               "--qos-profile-overrides-path", QOS_OVERRIDES] + wanted
        print(f"\n录制中 → {out}")
        print("  " + " ".join(cmd))
        if args.seconds > 0:
            print(f"  {args.seconds:.0f} 秒后自动停（SIGINT，让 bag 正常收尾）")

        rec = subprocess.Popen(cmd, cwd=REPO, start_new_session=True)
        procs_extra.append(rec)
        try:
            if args.seconds > 0:
                # ⚠️ `ros2 bag record` 必须**收 SIGINT**（Ctrl-C）才会把 bag
                # 正常收尾；SIGTERM 在 Humble 上会留下没写 metadata.yaml 的
                # 半成品目录，回放时报「不是一个 bag」。
                deadline = time.time() + args.seconds
                while time.time() < deadline and rec.poll() is None:
                    time.sleep(0.5)
                if rec.poll() is None:
                    rec.send_signal(signal.SIGINT)
            else:
                print("  录到 Ctrl-C 为止。")
            rec.wait(timeout=45)
        except KeyboardInterrupt:
            print("\n收到 Ctrl-C，正在让 bag 收尾……")
            rec.send_signal(signal.SIGINT)
            try:
                rec.wait(timeout=45)
            except subprocess.TimeoutExpired:
                print("⚠️ 30 秒内没退出，强制杀掉 —— bag 可能没写完 metadata")
                os.killpg(rec.pid, signal.SIGKILL)
    finally:
        for p in reversed(procs_extra):
            if p.poll() is None:
                try:
                    os.killpg(p.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
        time.sleep(1)

    # ---- 4) 报告 ---------------------------------------------------------
    if not os.path.isdir(out):
        print(f"\n没有产出 {out} —— 录制失败（退 2）")
        return 2

    info = sh(f"ros2 bag info {out}", timeout=120)
    print("\n=== ros2 bag info ===")
    print(info.stdout.strip() or info.stderr.strip()[:500])

    mb = du_mb(out)
    print(f"\n=== 体积 / 回传 ===")
    print(f"  bag 总大小   {mb:.1f} MB")
    print(f"  回传预估     {human(est_seconds(mb, args.link_mbs))}"
          f"（按实测 {args.link_mbs} MB/s）")
    print(f"  板子剩余空间 "
          f"{shutil.disk_usage(out).free / 1024**3:.0f} GB")

    print("\n=== 搬回开发机（在**开发机**上执行）===")
    print("  mkdir -p ~/bags && rsync -avP --info=progress2 \\")
    print(f"      jetson:{out} ~/bags/")
    if prof["images"] != "none":
        print("  # 注意：sqlite3 存储把整个 bag 装在一个 .db3 里，"
              "**没法按话题 --exclude**。")
        print("  # 嫌大：perception 档加 `--jpeg-scale 0.5` 降分辨率，"
              "或改用 plan 档（不含图像）。")

    name = os.path.basename(out)
    print("\n=== 本机回放（开发机）===")
    print(f"  ros2 bag info ~/bags/{name}     # 先确认话题/条数")
    print()
    print("  # 1) 放包。⚠️ `--clock` 不是可选项：不加的话 TF 时间戳还是录制当时")
    print("  #    的，RViz 会把它们当过期数据丢掉（实测 TF_OLD_DATA 警告 252 条），")
    print("  #    表现为**机械臂模型僵住不动**。")
    print(f"  ros2 bag play -l --clock ~/bags/{name} &")
    if prof["images"] == "compressed":
        print()
        print("  # 2) 本 bag 的图像是 JPEG，先解回原始话题。")
        print("  #    ⚠️ **不能**靠 RViz 的 Transport Hint —— 实测 RViz2(Humble) 从配置")
        print("  #    文件加载时会忽略它，只订阅你写的那个原始话题名（那里没有发布者）。")
        print("  #    用开发机现成的 image_transport 插件桥一下即可：")
        for t in relay_topics:
            print("  ros2 run image_transport republish compressed raw --ros-args \\")
            print(f"      -r in/compressed:={t}/compressed -r out:={t} &")
    print()
    print("  # 3) RViz。⚠️ `use_sim_time:=true` 必须与第 1 步的 `--clock` 配对，")
    print("  #    否则上一条的 TF 问题照旧（实测：配对后警告 252 → 0）。")
    print("  rviz2 -d ros2_ws/src/yolo_bringup/config/grasping.rviz \\")
    print("      --ros-args -p use_sim_time:=true")

    # ---- 5) 空话题告警（最容易把「录坏了」和「本来就没发」搞混）-----------
    empty = []
    for line in info.stdout.splitlines():
        m = re.match(r"^Topic information:\s*$", line)
        if not m:
            m2 = re.search(r"Topic:\s*(\S+)\s*\|\s*Type:\s*\S+\s*\|\s*Count:\s*(\d+)", line)
            if m2 and int(m2.group(2)) == 0:
                empty.append(m2.group(1))
    # rosbag2 的 info 有时把 Count 放在下一行，兜底再扫一遍原始文本
    for t in wanted:
        if t in info.stdout and t not in empty:
            blk = info.stdout.split(t, 1)[1][:200]
            if re.search(r"Count:\s*0\b", blk):
                empty.append(t)

    if empty:
        print("\n⚠️ 以下话题**在 bag 里 0 条消息**：")
        for t in sorted(set(empty)):
            print(f"  · {t}")
        print("  `/tf_static` 或 `/robot_description` 为空 ⇒ 回放时 TF 树断根、"
              "RViz 看不到机器人模型 —— 先查发布者是否真的起了。")
        crit = [t for t in empty if t in ("/tf_static", "/robot_description")]
        if crit:
            return 3

    print("\n录完了。")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""A1 真机验收：**抓 4 放 4 + 「槽位用尽」收工**，一条命令跑完并判分。

**在板子上跑。** 它自己起 `bringup.launch.py`（`carry_objects:=true`，
臂会真的动）、把日志写到文件、盯到收工、然后**按验收判据判分**。

它判什么
--------
两条**都必须成立**才算通过（见 `grade()`）：

  1. 规划器日志出现 `已放置 4/4 个`；
  2. 收工原因是 `放置槽位已用尽（4 个）—— 没有地方可放了`。

🔴 为什么第 2 条**需要抓取区里摆 5 块**（而不是 4 块）
----------------------------------------------------
这是 2026-10-05 真机实测才暴露的，**不是设计文档里写的**。
4 块方块配 4 个槽位时，第 4 块放下的**同一刻**抓取区也空了，
于是 `_select_and_approach` 先命中 `if best is None:` 分支
（`planner_node.py:1339`），**根本走不到**下面那道槽位闸门
（`:1345`）——收工于是走 `idle_finish_timeout`（日志 `空闲 20s …`）。

那道闸门要求「**有一个够分的新候选**」与「**没有空槽位**」**同时**成立：

    if best is None:                 # ← 4 块/4 槽 时先命中这里
        ...; return
    # ③ 槽位闸门：起步之前先确认还有地方可放
    if self._next_place_slot() is None:
        self._finish("放置槽位已用尽（…）—— 没有地方可放了")

⇒ 摆 **5 块**、槽位留 4 个，第 5 块就会在放完第 4 块之后撞上这道闸门。
本脚本**不替你摆件**，但它会在起栈后第一时间读第一行 `候选 N 个`，
若 `N <= 槽位数` 就**大声警告**「这一轮拿不到第 2 条」，别等跑完才发现。

前置检查（任一不过 ⇒ 退出码 2，**不起栈**）
------------------------------------------
  · 没有残留进程（否则端口/串口会打架）；
  · `/dev/ttyUSB0` 没被占；
  · **AG-95 已初始化**（读 `0x0200 == 1`）——未初始化时规划器会在
    启动序列的 `open()` 上被闸门拒掉，然后**静默卡住 ~15 分钟**；
  · 示教器 `programState = PLAYING`、`robotmode = RUNNING`（**只读查询**）。

⚠️ 对控制柜**只发只读查询**（`robotmode` / `programState` / `safetystatus`），
绝不发 `play` / `stop` / `power` —— 手搓协议打死过 URControl。

用法
====
    cd ~/yolo_ros_original && source /opt/ros/humble/setup.bash \\
        && source ros2_ws/install/setup.bash
    ROS_DOMAIN_ID=99 python3 scripts/board_a1.py \\
        --robot-ip <控制箱IP> --params $HOME/site_real.yaml

  `--robot-ip` **必填**（或设环境变量 `UR_ROBOT_IP`）—— 仓库是公开的，
  现场 IP 不写进源码。`--params` 默认 `~/site_real.yaml`。

  `--skip-preflight` 跳过前置检查（调试用，**别在有臂在旁边时用**）。
  `--no-launch` 只对已有日志判分（`--log` 指过去），不起栈、不动臂。

退出码：0 = 两条判据都过；1 = 判据没过；2 = 环境/前置不满足。
"""

import argparse
import os
import re
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

# 前置四件（残留进程 / 串口 / 夹爪 / 控制柜）与「谁是残留进程」的判据都搬进了
# `board_lib.py`（那里还有站点配置）。这里 **re-export** 一次：既有调用点与
# `test_board_a1.py` 一行都不用改，而真值只有一份。
import board_lib                                             # noqa: E402
from board_lib import (dashboard, gripper_state, is_playing,  # noqa: E402
                       leftovers, port_busy, teardown)

# ⚠️ **作废的孤本记录 —— 程序里一个字都不用，也不许用它兜底。**
#
# 这是 2026-10-05 那次验收当天敲的值，**故意不更新**。理由是一条设计决策：
# 现行外参只许有**一份**真值，就在站点配置 `~/.config/ur5-vision-grasp/site.yaml`。
# 在仓库里再放一份「更新的」＝造第二份真值，两份迟早对不上，而且**对不上的时候
# 不会报错**。所以它留在这里只为一件事：让人一眼看出「仓库里的常量不可信」。
#
# `build_cmd` 没有外参就**抛**；`main` 没有 `--site` 就**退 2**。
# 换句话说：**这份常量读起来像配置，其实是一块纪念碑。**
CAM = {
    "camera_x": "0.34694477757210807",
    "camera_y": "0.8693043936314018",
    "camera_z": "0.6454397556505314",
    "camera_qx": "-0.32913450910630504",
    "camera_qy": "-0.316374655430605",
    "camera_qz": "0.6185281368827056",
    "camera_qw": "-0.6395314661538762",
}

# 满行程跑 5 个循环 ≈ 5×40 s，再加收尾；给足余量。
RUN_TIMEOUT = 900.0


# ---------------------------------------------------------------- 判分（纯函数）

def grade(lines, slots):
    """只吃日志行，返回判定字典。**不碰任何进程/设备**，所以能离线测。

    `slots` 是期望的槽位数（= 现场 yaml 的 `place_slots` 长度）。
    """
    placed = []        # (已放置, 槽位共)
    finish = None
    summary = None
    cands = []         # (候选数, 当时已放置) —— 只用来看趋势，**不当判据**

    for ln in lines:
        m = re.search(r"已放置 (\d+)/(\d+) 个", ln)
        if m:
            placed.append((int(m.group(1)), int(m.group(2))))
        m = re.search(r"收工：(.+)$", ln)
        if m:
            finish = m.group(1).strip()
        m = re.search(r"已成功放置 (\d+) 个 / 槽位共 (\d+) 个；已抓取记账 (\d+) 个", ln)
        if m:
            summary = (int(m.group(1)), int(m.group(2)), int(m.group(3)))
        m = re.search(r"候选 (\d+) 个 .*?已放置 (\d+)/(\d+)", ln)
        if m:
            cands.append((int(m.group(1)), int(m.group(2))))

    last_placed = placed[-1] if placed else (0, slots)
    full = last_placed[0] == slots and last_placed[1] == slots

    # ⚠️ 用**子串**判，不用 == ：收工语后面还可能接解释性尾巴。
    slots_gone = bool(finish) and "放置槽位已用尽" in finish
    m = re.search(r"放置槽位已用尽（(\d+) 个）", finish or "")
    reported_slots = int(m.group(1)) if m else None

    # ⚠️ 曾想用「槽位放满那批的候选数 ≥ 1」当前置判据，**实测不成立**：
    #    抓取区空了之后检测批次是空的，`_targets_cb` 在
    #    `if latest is None or not latest.detections: self._reset_busy(); return`
    #    （planner_node.py:1181）就返回了，`_select_and_approach` **根本不被调用**
    #    —— 于是日志里连 `候选 0 个` 都不会出现。**"没有这行" ≠ "前置不满足"。**
    #    所以这里只当趋势留档，判据不依赖它。
    return {
        "placed": last_placed,
        "placed_ok": full,
        "finish": finish,
        "slots_exhausted_ok": slots_gone,
        "reported_slots": reported_slots,
        "summary": summary,
        "cands": cands,
        "ok": bool(full and slots_gone),
    }


def explain(v, slots):
    """把判定字典讲成人话（成功和失败都要能一眼看懂）。"""
    out = []
    if v["summary"]:
        p, s, g = v["summary"]
        out.append(f"  收工自述：已成功放置 {p} 个 / 槽位共 {s} 个；已抓取记账 {g} 个")
    out.append(f"  判据 1  已放置 {v['placed'][0]}/{v['placed'][1]} 个"
               f"（期望 {slots}/{slots}）  ->  {'✅ 过' if v['placed_ok'] else '❌ 不过'}")
    f = v["finish"] or "（日志里没有「收工：」这一行 —— 是超时被中断的？）"
    out.append(f"  收工原因：{f}")
    out.append(f"  判据 2  「放置槽位已用尽」        ->  "
               f"{'✅ 过' if v['slots_exhausted_ok'] else '❌ 不过'}")

    if v["reported_slots"] is not None and v["reported_slots"] != slots:
        out.append(f"  ⚠️ 日志说用尽的是 {v['reported_slots']} 个槽位，与期望的 {slots} 不符")

    if not v["slots_exhausted_ok"]:
        if v["finish"] and "idle_finish_timeout" in (v["finish"] or ""):
            out.append("")
            out.append("  🔴 走的是**空闲看门狗**这条路：放满的那一刻抓取区里")
            out.append("     **已经没有候选**了，闸门根本没被碰到。")
            out.append(f"     ⇒ 抓取区里的方块数必须 **> {slots}**（即 ≥ {slots + 1} 块），")
            out.append("       让第 %d 块在放完第 %d 块之后还留在区里。" % (slots + 1, slots))
        elif v["finish"] and "连续" in (v["finish"] or "") and "没放下" in (v["finish"] or ""):
            out.append("")
            out.append("  🔴 走的是 `max_attempts_without_place`（卡住）。")
            out.append("     先看日志里的轨迹失败/容差报错 —— 抓取本身没成功，不是槽位问题。")
        elif v["finish"] and "失败恢复" in (v["finish"] or ""):
            out.append("")
            out.append("  🔴 失败恢复流程自己失败了，交给人处理。")

    if v["cands"]:
        trend = " → ".join(f"{c}(放置{p})" for c, p in v["cands"])
        out.append("")
        out.append(f"  候选数轨迹：{trend}")
        out.append("  （趋势只作参考 —— 区空了之后 `_targets_cb` 会因为"
                   "「检测批次为空」提前返回，")
        out.append("    连 `候选 0 个` 都不会打，所以**没有这行不代表前置不满足**。）")
    return "\n".join(out)


# ---------------------------------------------------------------- 前置检查
# `leftovers` / `port_busy` / `gripper_state` / `dashboard` 见上方 re-export
# （实现在 `board_lib.py`，都有离线自检）。

def wait_for_playing(ip, timeout):
    """起栈**之后**轮询示教器直到 PLAYING。返回 `(到了没, 最后一次回复)`。

    ⚠️ 次序是「**起栈 → 按播放 → 拉回控制器**」，不能倒过来：URCap 的
    External Control 需要一个对端，先按播放再起栈，示教器上就是
    「连接被拒绝」，程序停在 STOPPED。本函数只读查询，**不发 play/power/stop**。

    ⚠️ 判据走 `board_lib.is_playing`（剥标签 + 取首词），**不是**子串、更不是
    与 `'PLAYING'` 精确比 —— 实测原文是 `'PLAYING ur5ros.urp'`（缀着程序名）。
    这里原来写的是 `"PLAYING" in last["programState"]`：它对真机**碰巧**成立，
    但同一个问题在 `board_run.py` 里被写成了精确比，于是那边恒 False
    （2026-10-06 合并成一份）。
    """
    t0, last = time.time(), None
    while time.time() - t0 < timeout:
        last = dashboard(ip)
        if last and is_playing(last):
            return True, last
        time.sleep(2.0)
    return False, last


# ---------------------------------------------------------------- 起栈

def build_cmd(a):
    args = [
        "ros2", "launch", "yolo_bringup", "bringup.launch.py",
        "camera_source:=realsense",
        f"robot_ip:={a.robot_ip}",
        f"planner_params_file:={a.params}",
        "gripper_backend:=ag95",
        "carry_objects:=true",
        "tf_required:=true",
        "use_target_plane:=true", "target_plane_z:=0.0285",
        # ── 抓取点取**顶面中心**，不是检测框中心 ──────────────────────────
        # 检测框框住的是方块在图像上的**整个侧影**（顶面 + 朝着相机的那些竖面），
        # 不是顶面。斜视 α 角下侧影中心比顶面中心低 (S·sinα·f)/(2D) 像素 ——
        # 2026-10-08 在真帧上量定：这帧 **6.44 px ≈ base 方向 13.7 mm**，而方块
        # 才 28.5 mm 宽，夹爪正好咬在顶面靠相机那一侧。**两棱中点法**把它修回来
        # （侧影最高点必在 target_plane_z、最低点必在 table_plane_z，取两点中点的
        # x/y —— 边长 S 与投影比例自动消掉，不需要知道方块多大或相机多斜）。
        # 细节见 `projector_node.py` 的 `_top_face_xy()`。
        #
        # `table_plane_z` 是近端下棱所在的台面高度，必须**小于** target_plane_z；
        # 写反了 projector_node 构造时就记 ERROR（不静默）。0.0 与
        # target_plane_z=0.0285「台面 + 方块高」的口径一致。
        # ⚠️ 只在 `use_target_plane=true` 时有意义：深度模式高度未知，两条棱
        #    所在的平面无从谈起（那时本键被忽略，不报错）。
        # ⚠️ 这两行必须与 `docs/待执行命令.md` A1 一节那三块**同时**改：
        #    `scripts/test_startup_docs.py` 拿本函数当**真值**比对文档，
        #    只改一边就判红（那正是要的）。同理，改了这里要回头看
        #    `board_run.py` 的 `LAUNCHED_ONLY_OK` —— 两边都有这个键时，
        #    那边就得把理由撤掉（表会自己查过时）。
        "grasp_point_mode:=top_face_center", "table_plane_z:=0.0",
        "publish_camera_tf:=true",
        # 规划器推迟启动：`start_delay` 包的是 [realsense, detector, perception,
        # planner]，**不含 UR 驱动**。驱动立刻起、抢在规划器之前把 5000x 连接建立好，
        # 启动 HOME 才不会撞上 `State tolerances failed`（实测不加时开局就是
        # `连续未放置 2/10`，加了是 0/10）。
        f"start_delay:={a.start_delay:g}",
    ]
    # ⚠️ **绝不回落 `CAM`**（理由见模块顶部那段注释）。没有站点外参就**当场抛**,
    # 让调用方在**起栈之前**失败 —— 拿一份旧几何跑真机是**静默**失效：
    # 症状是候选落到臂够不到的地方，读起来像「检测坏了」。
    # `getattr` 是因为离线测试用 `Namespace` 造参数、可能不带 `cam`。
    cam = getattr(a, "cam", None)
    if not cam:
        raise ValueError(
            "build_cmd 没有外参：需要 a.cam（站点配置的 topcam.extrinsics）。"
            "本函数**不会**回落到 board_a1.CAM —— 那是 2026-10-05 的作废孤本。")
    args += [f"{k}:={v}" for k, v in cam.items()]
    return args


def wait_for_finish(log_path, deadline):
    """盯日志直到出现「收工：」或超时。返回已读到的全部行。"""
    lines = []
    seen = 0
    warned = False
    while time.time() < deadline:
        try:
            with open(log_path, errors="replace") as fh:
                lines = fh.read().splitlines()
        except OSError:
            lines = []
        if any("收工：" in l for l in lines[seen:]):
            return lines, True
        # 起栈后第一时间看第一行「候选 N 个」，早警告早止损。
        if not warned:
            for l in lines:
                m = re.search(r"候选 (\d+) 个", l)
                if m:
                    print(f"  ℹ️ 首次检测到候选数 = {m.group(1)}")
                    warned = True
                    break
        seen = len(lines)
        time.sleep(2.0)
    return lines, False


def main(argv=None):
    ap = argparse.ArgumentParser(description="A1 真机验收（抓 4 放 4 + 槽位用尽）")
    # ⚠️ **没有默认值**：这是公开仓，现场控制柜 IP 不进源码。
    #    现场配一次 `export UR_ROBOT_IP=<控制箱IP>` 即可（或每次显式传）。
    ap.add_argument("--robot-ip", default=os.environ.get("UR_ROBOT_IP"))
    ap.add_argument("--site", default=None,
                    help="站点配置路径（默认取 $UR5VG_SITE）—— 起栈时**必给**，"
                         "不给就退 2。外参只许有一份真值（站点配置的 "
                         "topcam.extrinsics）；仓库里那份 CAM 是作废的孤本，"
                         "**不作兜底**")
    ap.add_argument("--override-freshness", default=None, metavar="理由",
                    help="外参超期时由人显式破例，**必须写下为什么**（原样打印出来）")
    ap.add_argument("--play-timeout", type=float, default=300.0,
                    help="起栈后等人在示教器按「播放」的秒数上限（默认 300）")
    ap.add_argument("--params", default=None,
                    help="规划器参数文件（默认：站点配置的 planner_params，"
                         "再默认 ~/site_real.yaml）")
    ap.add_argument("--slots", type=int, default=4, help="期望的放置槽位数（place_slots 长度）")
    ap.add_argument("--log", default=os.path.expanduser("~/a1_board.log"))
    ap.add_argument("--no-launch", action="store_true", help="只对已有 --log 判分，不起栈")
    ap.add_argument("--skip-preflight", action="store_true")
    ap.add_argument("--start-delay", type=float, default=25.0,
                    help="起栈后推迟多少秒再启动规划器（默认 25：留给 UR 驱动建立 "
                         "5000x 连接，避免启动 HOME 被状态容差拒掉。设 0 可关掉）")
    ap.add_argument("--timeout", type=float, default=RUN_TIMEOUT)
    a = ap.parse_args(argv)

    # ---- 站点配置（给了才读；读不合法**绝不放行**）----
    site = None
    if a.site or os.environ.get(board_lib.SITE_ENV):
        p = board_lib.site_path(a.site)
        try:
            site = board_lib.load_site(path=p)
        except board_lib.SiteError as exc:
            print(f"❌ {exc}")
            return 2
        probs = board_lib.validate_site(site)
        if probs:
            print(f"❌ 站点配置有 {len(probs)} 条毛病（{site.path}）：")
            for i, x in enumerate(probs, 1):
                print(f"   {i}. {x}")
            return 2
        print(f"—— 站点配置 {site.path} ——")
        st, det = board_lib.freshness_gate(site, override=a.override_freshness)
        print(f"  外参时效：[{st}] {det}")
        if st != board_lib.PASS:
            print("   外参过期时点云整体偏，症状是 /targets_3d 的候选落到臂够不到的地方；")
            print("   跑 python3 scripts/topcam_drift_check.py 重新核对，或"
                  "--override-freshness \"理由\" 显式破例。")
            return 2
        print()
    elif not a.no_launch:
        # ⚠️ **不给站点配置就退 2，绝不回落到 `CAM`**（2026-10-06 用户拍板）。
        # 回落的代价是**静默**的：拿一份旧几何起栈，症状是候选落到臂够不到的地方，
        # 读起来像「检测坏了」—— 这一轮照样能跑完，于是「外参作废了」被掩盖成
        # 「检测不稳定」。而相机每动一次外参就作废一次 ⇒ **常量只要还在兜底，
        # 迟早有人用它**。所以这里不是警告，是拒绝。
        print("❌ 没给站点配置（--site / $UR5VG_SITE）⇒ **退 2，不起栈**。")
        print("   外参只许有一份真值：站点配置的 topcam.extrinsics。")
        print("   仓库里那份 `CAM` 是**2026-10-05 的作废孤本**，本脚本不作兜底 ——")
        print("   拿旧几何起栈是**静默**失效（候选落到臂够不到的地方，看起来像"
              "检测坏了）。")
        print("   cp config/site.example.yaml ~/.config/ur5-vision-grasp/site.yaml")
        print("   填好之后 --site 指过去（或 export UR5VG_SITE=<路径>）。")
        return 2
    else:
        # `--no-launch` 只对现成日志判分：不起栈、不碰外参，所以不需要站点配置。
        print("（--no-launch：只判分，不起栈 ⇒ 不需要站点配置与外参）\n")

    a.robot_ip = a.robot_ip or (site.robot_ip if site else None)
    a.params = (a.params or (site.planner_params if site else None)
                or os.path.expanduser("~/site_real.yaml"))
    a.cam = site.extrinsics() if site else None
    gripper_port = (site.gripper_port if site else None) or "/dev/ttyUSB0"

    if not a.no_launch and not a.robot_ip:
        print("❌ 缺 --robot-ip（环境变量 UR_ROBOT_IP 也没设）")
        print("   公开仓不写现场 IP，所以这里没有默认值：")
        print("     export UR_ROBOT_IP=<控制箱IP>        # 配一次")
        print("     或 python3 scripts/board_a1.py --robot-ip <控制箱IP> ...")
        return 2

    # ---- 前置检查（任一不过 ⇒ 2，不起栈）----
    if not a.skip_preflight and not a.no_launch:
        print("—— 前置检查 ——")
        stale = leftovers()
        if stale:
            print("❌ 有残留进程（先收干净再来）：")
            for s in stale:
                print("   " + s[:120])
            return 2
        print("  ✓ 无残留进程")
        if port_busy(gripper_port):
            print(f"❌ {gripper_port} 被占用 —— 多半就是残留的规划器，先收干净")
            return 2
        print(f"  ✓ {gripper_port} 空闲")

        st, why = gripper_state(site.gripper_port if site else None)
        if st is None:
            print(f"❌ 读不到夹爪状态（{why}）—— 串口没通就别急着起栈")
            return 2
        if st != 1:
            name = {0: "未初始化", 2: "初始化进行中"}.get(st, f"未知值 {st}")
            print(f"❌ AG-95 {name}（0x0200={st}）—— 先跑：")
            print("     python3 scripts/ag95_init_close.py --go")
            print("   （写完停在**全开**——规划器起栈时自己会 open()，这里不需要闭合。")
            print("    要**闭合**态是另一回事，那是碰点标定：`--close`，见待执行命令.md）")
            print("   未初始化时起栈，规划器会在启动序列上静默卡住 ~15 分钟。")
            return 2
        print("  ✓ AG-95 已初始化（0x0200=1）")

        d = dashboard(a.robot_ip)
        if d is None:
            print(f"❌ 连不上控制柜 {a.robot_ip}:29999")
            return 2
        print(f"  · {d['robotmode']} | {d['programState']} | {d['safetystatus']}")
        # ⚠️ 这里**不**要求 PLAYING。次序是「起栈 → 按播放 → 拉回控制器」：
        #    URCap 的 External Control 要先有对端，先按播放再起栈，示教器上
        #    会报「连接被拒绝」、程序停在 STOPPED。等播放的逻辑在起栈之后。
        print("  · 此刻不要求 PLAYING（次序：起栈 → 按播放 → 拉回控制器）")
        print()

    # ---- 只判分模式 ----
    if a.no_launch:
        try:
            lines = open(a.log, errors="replace").read().splitlines()
        except OSError as exc:
            print(f"❌ 读不了日志 {a.log}：{exc}")
            return 2
        return verdict(lines, a.slots)

    # ---- 起栈 ----
    cmd = build_cmd(a)
    print("—— 起栈（carry_objects:=true，**臂会动**）——")
    print("  " + " ".join(cmd[:6]) + " …")
    logf = open(a.log, "w")
    domain = str(site.ros_domain_id) if site else os.environ.get("ROS_DOMAIN_ID", "99")
    env = dict(os.environ, ROS_DOMAIN_ID=domain, PYTHONUNBUFFERED="1")
    proc = subprocess.Popen(cmd, cwd=REPO, stdout=logf, stderr=subprocess.STDOUT,
                            env=env, start_new_session=True)
    try:
        print(f"  日志 -> {a.log}")
        print("\n" + "=" * 62)
        print("👉 现在去示教器按**播放**（URCap 的 External Control 程序）。")
        print("   本脚本**不发** play / power / stop —— 那是人的动作（红线 6）。")
        print("=" * 62)
        ok, d = wait_for_playing(a.robot_ip, a.play_timeout)
        if not ok:
            got = d["programState"].strip() if d else "连不上控制柜"
            print(f"❌ {a.play_timeout:.0f}s 内示教器没进入 PLAYING（现在是 {got}）"
                  f" —— 收起栈，什么都没发出去")
            return 2
        print(f"  ✓ 示教器 PLAYING（{d['programState'].strip()}）\n")
        lines, finished = wait_for_finish(a.log, time.time() + a.timeout)
        if not finished:
            print(f"\n❌ {a.timeout:.0f}s 内没等到「收工：」—— 被超时中断")
            return verdict(lines, a.slots, timed_out=True)
        return verdict(lines, a.slots)
    finally:
        logf.close()
        teardown(proc)


def verdict(lines, slots, timed_out=False):
    v = grade(lines, slots)
    print("\n—— 判定 ——")
    print(explain(v, slots))
    print()
    if timed_out:
        print("结果：❌ 超时")
        return 1
    if v["ok"]:
        print("结果：✅ A1 通过（已放置满 + 槽位用尽收工）")
        return 0
    print("结果：❌ A1 未通过")
    return 1


# `teardown` 也搬进 `board_lib.py` 了（见上方 re-export）—— 搬过去还顺手修掉了
# 它依赖的 `leftovers()` 里那处「按文件名排除调用方」的隐疾。


if __name__ == "__main__":
    sys.exit(main())

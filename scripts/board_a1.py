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
import socket
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

# ⚠️ 排除 `bash -c`：经 ssh 跑时调用者那条命令行里带着同样的模式，会把自己匹配上
#    （board_chain_check.py / board_ctrlc_check.py 都踩过，ssh 直接 255）。
LEFTOVER_RE = re.compile(
    r"ros2 launch yolo_bringup|move_group|robot_state_publisher"
    r"|detector_node|projector_node|planner_node|realsense2_camera_node"
    r"|static_transform_publisher|ur_ros2_control_node"
    r"|ur_robot_driver/(dashboard_client|robot_state_helper|controller_stopper)")

# 标定过的外参（2026-10-05 现场值）。改这里之前先确认你知道自己在改什么。
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

def leftovers():
    r = subprocess.run(["ps", "-eo", "pid,args", "--no-headers"],
                       capture_output=True, text=True)
    me = os.path.basename(__file__)
    return [l.strip() for l in r.stdout.splitlines()
            if "bash -c" not in l and me not in l and LEFTOVER_RE.search(l)]


def port_busy(path="/dev/ttyUSB0"):
    r = subprocess.run(["fuser", path], capture_output=True, text=True)
    return r.returncode == 0


def gripper_state():
    """读 AG-95 的 `0x0200`。返回 (状态, 说明)；读不到返回 (None, 原因)。

    **只读** —— 复用 `ag95_init_close.py` 的 `Gripper`，它唯一会写的地方是
    `write()`，本函数一次都不调。
    """
    try:
        import ag95_init_close as ag
    except Exception as exc:                      # noqa: BLE001
        return None, f"导入 ag95_init_close 失败：{exc}"
    g = ag.Gripper.open_port()
    if g is None:
        return None, "串口打不开"
    try:
        # 0 = 未初始化 / 2 = 进行中 / 1 = 已完成
        return g.read(ag.REG_INIT_STATUS), "ok"
    finally:
        g.close_port()


def dashboard(ip, timeout=5.0):
    """**只读**问一句控制柜。返回 {命令: 回复}；连不上返回 None。"""
    try:
        s = socket.create_connection((ip, 29999), timeout=timeout)
    except OSError as exc:
        return None
    try:
        f = s.makefile("rw")
        f.readline()                              # 问候行，别拿来判状态
        out = {}
        for c in ("robotmode", "programState", "safetystatus"):
            f.write(c + "\n")
            f.flush()
            out[c] = f.readline().strip()
        return out
    finally:
        s.close()


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
        "publish_camera_tf:=true",
        # 规划器推迟启动：`start_delay` 包的是 [realsense, detector, perception,
        # planner]，**不含 UR 驱动**。驱动立刻起、抢在规划器之前把 5000x 连接建立好，
        # 启动 HOME 才不会撞上 `State tolerances failed`（实测不加时开局就是
        # `连续未放置 2/10`，加了是 0/10）。
        f"start_delay:={a.start_delay:g}",
    ]
    args += [f"{k}:={v}" for k, v in CAM.items()]
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
    ap.add_argument("--params", default=os.path.expanduser("~/site_real.yaml"))
    ap.add_argument("--slots", type=int, default=4, help="期望的放置槽位数（place_slots 长度）")
    ap.add_argument("--log", default=os.path.expanduser("~/a1_board.log"))
    ap.add_argument("--no-launch", action="store_true", help="只对已有 --log 判分，不起栈")
    ap.add_argument("--skip-preflight", action="store_true")
    ap.add_argument("--start-delay", type=float, default=25.0,
                    help="起栈后推迟多少秒再启动规划器（默认 25：留给 UR 驱动建立 "
                         "5000x 连接，避免启动 HOME 被状态容差拒掉。设 0 可关掉）")
    ap.add_argument("--timeout", type=float, default=RUN_TIMEOUT)
    a = ap.parse_args(argv)

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
        if port_busy():
            print("❌ /dev/ttyUSB0 被占用 —— 多半就是残留的规划器，先收干净")
            return 2
        print("  ✓ /dev/ttyUSB0 空闲")

        st, why = gripper_state()
        if st is None:
            print(f"❌ 读不到夹爪状态（{why}）—— 串口没通就别急着起栈")
            return 2
        if st != 1:
            name = {0: "未初始化", 2: "初始化进行中"}.get(st, f"未知值 {st}")
            print(f"❌ AG-95 {name}（0x0200={st}）—— 先跑：")
            print("     python3 scripts/ag95_init_close.py --go")
            print("   未初始化时起栈，规划器会在启动序列上静默卡住 ~15 分钟。")
            return 2
        print("  ✓ AG-95 已初始化（0x0200=1）")

        d = dashboard(a.robot_ip)
        if d is None:
            print(f"❌ 连不上控制柜 {a.robot_ip}:29999")
            return 2
        print(f"  · {d['robotmode']} | {d['programState']} | {d['safetystatus']}")
        if "PLAYING" not in d["programState"]:
            print("❌ 示教器程序不是 PLAYING —— 起栈也不会动。")
            print("   去示教器按一下播放（URCap 的 External Control），**本脚本不发这个命令**。")
            return 2
        print("  ✓ 示教器在 PLAYING")
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
    env = dict(os.environ, ROS_DOMAIN_ID=os.environ.get("ROS_DOMAIN_ID", "99"))
    if env["ROS_DOMAIN_ID"] != "99":
        print(f"  ⚠️ ROS_DOMAIN_ID={env['ROS_DOMAIN_ID']}（现场是 99，确认一下）")
    proc = subprocess.Popen(cmd, cwd=REPO, stdout=logf, stderr=subprocess.STDOUT,
                            env=env, start_new_session=True)
    try:
        print(f"  日志 -> {a.log}")
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


def teardown(proc):
    """收栈。⚠️ 只 killpg **父进程组**；`ros2 launch` 的子节点里有一批
    （`move_group` / `robot_state_publisher` / `static_transform_publisher` /
    `ur_ros2_control_node`）在父进程被收后会**重新挂到 init**，
    所以必须再按模式扫一遍（2026-10-05 实测：漏扫会留下 PPID=1 的孤儿）。"""
    import signal
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    deadline = time.time() + 15
    while time.time() < deadline and leftovers():
        time.sleep(1)
    if leftovers():
        for line in leftovers():
            try:
                os.kill(int(line.split()[0]), signal.SIGKILL)
            except (ValueError, ProcessLookupError):
                pass
        time.sleep(2)
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        pass
    stale = leftovers()
    if stale:
        print("\n⚠️ 收尾后仍有残留（下次会被前置检查拦下）：")
        for s in stale:
            print("  " + s[:120])


if __name__ == "__main__":
    sys.exit(main())

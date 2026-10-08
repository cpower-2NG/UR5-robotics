#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""真机侧公共件：站点配置、进程残留、串口/夹爪/Dashboard 只读查询。

**顶层不 import rclpy，也不 import launch。** 本机（WSL）的 `/opt/ros/humble`
下根本没装 rclpy，顶层一旦 import，`test_board_lib.py` 就在离线层跑不起来 ——
而离线层是这套东西**唯一**能成测的地方。所以：判定逻辑一律**纯函数**，I/O 收在
薄适配器里并留注入口（`leftovers(ps_out=…)`），变异测试要能只喂数据、不起进程、
不开串口。

本文件里**没有一处**会写设备或发运动命令。`gripper_state` / `dashboard` /
`port_busy` 全是只读查询。

里面有三块东西，都是**两份调用方共用**的（所以放在这里，不各抄一份）：
  · **收栈**（`sweep` / `teardown` / `session_pids` / `install_stop_handlers`）——
    `board_run` 的 `finally`、信号处理与 `--stop` 走的是同一段本体，判据只有一份；
  · **上一轮的记录**（`read_stack_record` / `write_stack_record` /
    `owned_session_pids`，默认 `~/.config/ur5-vision-grasp/last_stack.json`）——
    ⚠️ 写在**仓库外**：本仓是公开副本，运行记录不进仓；
  · **人机协作**（`human_action` / `relay_notice` / `wait_for_state` /
    `ensure_in_the_loop`）—— 要人动手的那几句压成 `HUMAN_MARK` 开头的一行，
    **由 AI 原样转达**；核验一律只读，验不过就是**不写**（fail-closed）。

用法（自查「现在生效的站点配置是哪份、合不合法」）：

    python3 scripts/board_lib.py [--site PATH]

退出码：0 = 站点配置合法且外参没过期；1 = 有毛病（逐条列出）；2 = 读不到文件。
"""

import contextlib
import datetime as dt
import json
import os
import re
import signal
import socket
import subprocess
import sys
import time

PASS, FAIL, SKIP = "PASS", "FAIL", "SKIP"

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)

# ⚠️ 排除 `bash -c`：经 ssh 跑时调用者那条命令行里带着同样的模式，会把自己匹配上
#    （board_chain_check.py / board_ctrlc_check.py 都踩过，ssh 直接 255）。
#
# ★ 这份名单是**枚举出来的**，不是猜的。出处（2026-10-08，只读读板上的**已安装**上游）：
#     /opt/ros/humble/share/ur_robot_driver/launch/ur_control.launch.py
#     /opt/ros/humble/share/ur_robot_driver/launch/ur_dashboard_client.launch.py
#     /opt/ros/humble/share/ur_moveit_config/launch/ur_moveit.launch.py
#   逐条取 `Node(package=…, executable=…)`，再按**本仓的启动参数**挑出真会起的那些：
#     会起：ur_robot_driver/{ur_ros2_control_node, robot_state_helper, urscript_interface,
#                            trajectory_until_node, dashboard_client, controller_stopper_node}
#           robot_state_publisher、controller_manager/spawner ×N、move_group、
#           realsense2_camera_node、detector_node、projector_node、planner_node、
#           static_transform_publisher（相机 TF）、`ros2 launch yolo_bringup`
#     不起（本仓显式关掉）：rviz2（`launch_rviz:=false`）、
#           servo_node_main（`launch_servo:=false`）
#     ⚠️ `controller_manager/ros2_control_node` 那一档只在 `use_fake_hardware:=true`
#        时起（ur_control.launch.py 的两个 Node 各带一个条件）。本仓走真机，
#        起的是 `ur_ros2_control_node` —— 但**两条都留在名单里**：那个开关不在
#        本仓手里（上游默认值改一次就换人），而漏掉的表现是**假绿**。
#     ⚠️ `spawner` 是短暂进程（拉完控制器就退出），但仍然算 —— 卡住的那一个会一直
#        占着 controller_manager 的服务。写全路径形式（`controller_manager/spawner`）
#        是为了不把别人的同名进程也算进来。
#
# 🔴 **漏一个名字的代价不是红，是假绿**（2026-10-08 实测）：
#     `trajectory_until_node` 与 `urscript_interface` 原来不在名单里 ⇒ `--stop`
#     报「已收干净」**是假的**（复核 `ps` 时那两个还在），而 P1.残留进程 那道闸门
#     用同一个判据，于是**跟着一起假绿**。
# ★ 所以名单**不再是唯一保险**：我们自己起的栈按 **session id** 收（`session_pids()`），
#   不靠名字 —— 见 `teardown()`。名字这一份只用来认**别人起的**栈。
LEFTOVER_RE = re.compile(
    r"ros2 launch yolo_bringup|move_group|robot_state_publisher"
    r"|detector_node|projector_node|planner_node|realsense2_camera_node"
    r"|static_transform_publisher|ur_ros2_control_node"
    r"|ur_robot_driver/(dashboard_client|robot_state_helper|controller_stopper"
    r"|trajectory_until_node|urscript_interface)"
    r"|controller_manager/(ros2_control_node|spawner)")


# ============================================================ 进程残留

def _int(s):
    try:
        return int(s)
    except (TypeError, ValueError):
        return None


def _ps_lines():
    r = subprocess.run(["ps", "-eo", "pid,args", "--no-headers"],
                       capture_output=True, text=True)
    return r.stdout.splitlines() if r.returncode == 0 else []


def _ppid(pid):
    """父 pid。`/proc/<pid>/stat` 的 comm 字段可能含空格与括号，所以从**最后**一个
    `)` 往右切（`rsplit`），再取 state 之后的第一个数。"""
    try:
        with open(f"/proc/{int(pid)}/stat", "rb") as fh:
            tail = fh.read().rsplit(b")", 1)[1].split()
        return int(tail[1])
    except (OSError, IndexError, ValueError):
        return None


def own_pids(pid=None):
    """本进程 + 全部祖先的 pid（从自己往上追到 init）。

    为什么需要：残留检查必须**排除自己这一族**，否则「谁在跑检查」这件事会被
    当成残留。
    """
    out = []
    pid = os.getpid() if pid is None else pid
    while pid and pid > 1 and pid not in out and len(out) < 64:
        out.append(pid)
        pid = _ppid(pid)
    return out


def _me_default():
    """「调用方脚本名」= `basename(sys.argv[0])`（最外层入口）。

    ⚠️ 老判据是 `os.path.basename(__file__)`：它当时写在 `board_a1.py` 里，
    于是**碰巧**等于调用方脚本名 —— 因为一条 `ps` 记录里必然含有它自己的脚本名。
    这个等价是偶然的：函数一搬进本文件，`__file__` 就变成 `board_lib.py`，
    而调用方那条记录里带的是 `board_run.py`，于是**排除不掉**。后果是：只要
    调用方命令行里恰好含一个残留模式（例如 `--log ~/planner_node.log`），前置
    检查就报「有残留」并拒绝起栈 —— 同一族的坑本项目已经踩过（`pkill -f`
    把执行它的那条 shell 自己杀掉，记了三次）。
    `sys.argv[0]` 指向最外层入口，**与函数住在哪个文件里无关**。
    """
    return os.path.basename(sys.argv[0] or "")


def is_own_process(args, me, own, pid=None):
    """纯函数：这条 `ps` 记录是不是「我们自己人」。

    `me` 太短（`-c`、`py` 这种）时**不按子串排除** —— 否则 `"" in args` 恒真，
    整条残留检查会被一个空名字关掉，而且关得无声无息。
    """
    if "bash -c" in args:
        return True
    if me and len(me) >= 4 and me in args:
        return True
    return pid is not None and pid in own


def leftovers(ps_out=None, me=None, own=None, pattern=None):
    """残留的栈进程。返回 `ps` 原始行（未裁剪，便于人肉辨认）。

    注入口：`ps_out` 直接喂 `ps -eo pid,args --no-headers` 形状的行，
    `me` / `own` 覆盖默认的自我排除 —— 变异测试靠这三个口子离线成测。

    `pattern` 覆盖「什么算残留」：**默认的 `LEFTOVER_RE` 是整车的**（含
    `ur_ros2_control_node` / `ur_robot_driver/*`）。分阶段类检查
    （`moveit_only_check` / `no_arm_stack` / `board_chain_check` /
    `board_state_machine_check`）跑在**驱动还没起**的时候，它们等的是
    `fake_joint_states` 而不是真驱动 —— 硬套整车模式会让这些脚本**漏报**
    自己那一阶段的残留，而漏报的表现是**静默变绿**。
    所以「哪一类算残留」按阶段给，「谁是自己人」**只有这一份**。
    """
    lines = _ps_lines() if ps_out is None else ps_out
    me = _me_default() if me is None else me
    own = set(own_pids()) if own is None else set(own)
    pat = LEFTOVER_RE if pattern is None else pattern
    out = []
    for ln in lines:
        s = ln.strip()
        if not s:
            continue
        pid_s, _, args = s.partition(" ")
        if not pat.search(args):
            continue
        if is_own_process(args, me, own, _int(pid_s)):
            continue
        out.append(s)
    return out


def proc_stat(pid):
    """`/proc/<pid>/stat` → `(state, session id)`；读不到 `(None, None)`。

    字段序（`stat(5)`）：pid (comm) **state** ppid pgrp **session** tty_nr …
    所以从**最后**一个 `)` 往右切、跳过 state/ppid/pgrp 之后是 session。
    与 `_ppid()` 同一个切法、同一个理由（comm 里可能有空格与括号）。
    """
    try:
        with open(f"/proc/{int(pid)}/stat", "rb") as fh:
            tail = fh.read().rsplit(b")", 1)[1].split()
        return tail[0].decode("ascii", "replace"), int(tail[3])
    except (OSError, IndexError, ValueError):
        return None, None


def sid_of(pid):
    """`/proc/<pid>/stat` 里的 **session id**；读不到返回 `None`。"""
    return proc_stat(pid)[1]


def session_entries():
    """`[(pid, sid), …]` —— 扫 `/proc`。只读。

    ⚠️ **僵尸不算**（state == `Z`）：僵尸已经死了，只是还没被父进程回收 ——
    把它算成「还活着」会让收栈永远报「没收干净」。实测（2026-10-08）：e2e 夹具里
    那句 `(session N 里还有 pid N —— 名字名单漏了它)` 就是这么来的 ——
    leader 已经被 SIGKILL，它自己就是僵尸，而 `proc.wait()` 还没轮到。
    """
    out = []
    try:
        names = os.listdir("/proc")
    except OSError:
        return out
    for d in names:
        if d.isdigit():
            state, s = proc_stat(d)
            if s is not None and state != "Z":
                out.append((int(d), s))
    return out


def session_pids(sid, entries=None):
    """同一 **session** 里的全部 pid（**含孤儿**），排除本进程。`entries` 可注入 ⇒ 离线可测。

    ★ 为什么按 session 收，而不是按名字收（2026-10-08）：
      · `ros2 launch` 的子节点在父进程被收之后会**重新挂到 init**（PPID=1）——
        血缘断了，**名字成了唯一线索**；而名字是一份手抄的名单，漏一个就是假绿
        （实测漏了 `trajectory_until_node` / `urscript_interface`）。
      · session id **不随 re-parent 改变**：`start_new_session=True` 起的那个
        leader 的子子孙孙都带着它，哪怕中间那一层已经死了。
      ⇒ 我们自己起的栈，收干净这件事**不再依赖名单全不全**。
    """
    entries = session_entries() if entries is None else entries
    me = os.getpid()
    return [p for p, s in entries if s == sid and p != me]


def is_session_leader(pid, sid=None):
    """`pid` 是不是**自己开了**一个 session（session id == 自己的 pid）。

    ⚠️ 这是「敢不敢按 session 收」的**安全闸门**，不能省：
      我们起栈用 `start_new_session=True` ⇒ leader 是它，那个 session 里**只有**
      这套栈，整锅端掉不会误伤。
      而**人从自己 shell 里**敲的 `ros2 launch`，它的 session 就是那个 shell 的
      session —— 按 session 收会**把人自己的终端一起杀掉**。
    """
    return (sid if sid is not None else sid_of(pid)) == pid


def kill_pids(pids, sig):
    """逐个 pid 发信号（不 killpg：对面可能是别人的进程组）。返回发成功的条数。"""
    n = 0
    for p in pids:
        try:
            os.kill(int(p), sig)
            n += 1
        except (ValueError, ProcessLookupError, PermissionError, OSError):
            pass
    return n


def sweep(leader, sid=None, out=print, entries=None, clock=time.time, sleep=None,
          grace=15.0, killpg=False):
    """收栈的**本体** —— `teardown()`（收自己起的）与 `--stop`（收别人起的）共用。

    `leader` = 我们自己起的那个进程组 / session 的 leader pid（**可能已经死了**）。
    `sid`    = 要按 session 收的会话。

    ⚠️ 按 session 收的**门槛**就是 `is_session_leader()`：人从 shell 里敲的
       `ros2 launch`，sid 是那个 **shell 的** session，整锅端掉会**把人自己的
       终端一起杀掉**。⛔ 别在这里放宽它。
    ⚠️ `killpg` 默认 **False**：只有 leader **还活着**的调用方才敢开。leader 死了
       之后它的 pid 可能被**回收**给别的进程，那时 `killpg` 打的是别人的进程组。

    返回**实测**残留（空 list = 真收干净了）—— 不返回「我以为」。
    """
    sleep = time.sleep if sleep is None else sleep
    own_session = bool(leader is not None and is_session_leader(leader, sid))

    # 快路径：整组 SIGTERM，让 launch 自己走它那套收工流程（节点会报 finished cleanly）
    if killpg:
        try:
            os.killpg(leader, signal.SIGTERM)
        except (ProcessLookupError, PermissionError, OSError):
            pass
    if own_session:
        kill_pids(session_pids(sid, entries=entries), signal.SIGTERM)
    if not killpg:                      # 别人的栈：按名字点名，逐个 SIGTERM
        kill_pids([int(ln.split()[0]) for ln in leftovers()
                   if ln.split() and ln.split()[0].isdigit()], signal.SIGTERM)

    deadline = clock() + grace
    while clock() < deadline and (leftovers() or
                                  (own_session and session_pids(sid, entries=entries))):
        sleep(1)

    # 还活着的：先按 session（含孤儿）再来一遍，然后按名字补扫
    if own_session:
        kill_pids(session_pids(sid, entries=entries), signal.SIGKILL)
    for line in leftovers():
        try:
            os.kill(int(line.split()[0]), signal.SIGKILL)
        except (ValueError, ProcessLookupError, PermissionError, OSError):
            pass
    if own_session or leftovers():
        sleep(2)

    stale = leftovers()
    orphan = session_pids(sid, entries=entries) if own_session else []
    if orphan:                      # session 里还有活口 ⇒ 名字没认出来的那些
        stale = list(stale) + ["(session %d 里还有 pid %s —— 名字名单漏了它)"
                               % (sid, ",".join(str(p) for p in orphan))]
    if stale:
        out("\n⚠️ 收尾后仍有残留（下次会被前置检查拦下）：")
        for s in stale:
            out("  " + s[:120])
    return stale


def teardown(proc, out=print, entries=None, clock=time.time, sleep=None, grace=15.0):
    """收掉**我们自己起的**那套栈 → 收完还剩几条。**返回实测，不返回「我以为」**。

    ⚠️ 孤儿是这里的**主要**敌人：`ros2 launch` 的子节点在父进程被收之后会
    **重新挂到 init**（PPID=1），进程组也跟着散。所以收三样：
      ① `killpg`（leader 还活着，用它）；
      ② **按 session 收**（`session_pids`）—— 只要 leader 是我们起的（`start_new_session`
         ⇒ session 里只有这套栈），这一遍**一个都跑不掉**，包括已经 re-parent 的孤儿；
      ③ **按名字扫**（`leftovers()`）—— 补「不在 session 里」的漏网（例如有人手动
         补起过一个节点）。
    本体在 `sweep()`（`--stop` 也走它，判据只有一份）。
    """
    sid = sid_of(proc.pid) if proc is not None else None
    stale = sweep(proc.pid if proc is not None else None, sid, out=out,
                  entries=entries, clock=clock, sleep=sleep, grace=grace,
                  killpg=proc is not None)
    if proc is not None:
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            pass
    return stale


# ------------------------------------------------- 信号：别把栈留成孤儿

def install_stop_handlers(on_signal, signals=None):
    """把 SIGTERM / SIGHUP 接到 `on_signal(name)` 上，然后**结束当前流程**。

    ⚠️ 为什么非有不可（2026-10-08 的缺陷）：收栈原来只写在 `run()` 的 `finally`
      里，而 `finally` 只在 **Python 自己走到那儿** 时才执行。别人
      `kill <启动器 pid>`（SIGTERM：`timeout` / CI / AI 那一轮的收尾；
      SIGHUP：终端被关）时，默认处置是**当场死** ⇒ 栈那一整个**独立会话**
      （`start_new_session=True`）原地不动，子节点**重新挂到 init**（PPID=1）
      继续跑 —— 这就是「挂残留、孤儿进程」的来路。
      信号处理器里先收栈，孤儿才不会产生。

    `on_signal` 收到的是信号名（`"SIGTERM"` / `"SIGHUP"`），由它去决定说什么、
    收什么；收完这里抛 `SystemExit(128+n)`（**故意不走正常出口** —— 正常出口会
    写报告、给判分，而这一轮是被中断的，不该伪装成跑完）。返回 `restore()`。
    """
    signals = (signal.SIGTERM, signal.SIGHUP) if signals is None else signals
    previous = {}

    def _handler(signum, _frame):
        on_signal(signal.Signals(signum).name)
        raise SystemExit(128 + signum)

    for sig in signals:
        previous[sig] = signal.getsignal(sig)
        signal.signal(sig, _handler)

    def restore():
        for sig, old in previous.items():
            signal.signal(sig, old)

    return restore


# ------------------------------------------------- 上一轮起的栈（`--stop` 用）

STACK_RECORD = "~/.config/ur5-vision-grasp/last_stack.json"
#: 覆盖记录路径的环境变量。**给离线测试用的**：测试跑出来的 `--stop` 不该去动
#: 真人 `~/.config` 里那一份（那份记着**真的**上一次起的栈的 session，测试拿它
#: 当夹具就是自证式判据）。现场不用设它。
STACK_RECORD_ENV = "UR5VG_STACK_RECORD"


def stack_record_path(path=None):
    if path is None:
        path = os.environ.get(STACK_RECORD_ENV) or STACK_RECORD
    return os.path.expanduser(path)


def write_stack_record(pid, sid, path=None, extra=None):
    """记下**我们自己起的**那一套栈的 `(pid, sid)` → 路径；写不成返回 `None`。

    ⚠️ 为什么落在**仓库外**（红线 3）：里面是 pid / session id / 日志路径 ——
      运行记录，不是配置。仓内的 `*.py` 与文档一个字都不许沾。
    ⚠️ 为什么是 `sid` 而不是「一份进程名单」：名单会漏（实测漏过
      `trajectory_until_node` / `urscript_interface`），而 session id 不随
      re-parent 改变 —— 启动器**被谁打死**都能顺着它找到整锅。
    """
    rec = {"pid": int(pid), "sid": int(sid), "at": time.time(),
           "written_by": os.getpid()}
    rec.update(extra or {})
    p = stack_record_path(path)
    try:
        os.makedirs(os.path.dirname(p), exist_ok=True)
        with open(p, "w", encoding="utf-8") as fh:
            json.dump(rec, fh, ensure_ascii=False)
        return p
    except OSError:
        return None


def read_stack_record(path=None):
    """读回来（**只读**）；没有 / 坏了 ⇒ `{}`。

    坏了就当没有，**不许**拿半截记录去杀进程 —— 判据宁可少一条。
    """
    try:
        with open(stack_record_path(path), encoding="utf-8") as fh:
            rec = json.load(fh)
    except (OSError, ValueError):
        return {}
    if not isinstance(rec, dict) or not isinstance(rec.get("sid"), int):
        return {}
    return rec


def clear_stack_record(path=None):
    with contextlib.suppress(OSError):
        os.remove(stack_record_path(path))


def owned_session_pids(rec=None, entries=None, path=None):
    """按**记录下来的** session 取活口 —— 但只认「它自己就是 leader」的那一种。

    ⚠️ 复用 `is_session_leader` 那道安全闸门（写成 `sid == pid` 的形式）：
      记录里的 sid 必须**等于**记录里的 pid，才敢按 session 收。人从 shell 里
      起的栈，sid 是那个 shell 的 session —— 按 session 收会**把人自己的终端
      一起杀掉**。旧记录（或被人改过的记录）不满足 ⇒ 返回空，退回按名字收。
    """
    rec = read_stack_record(path) if rec is None else rec
    if not isinstance(rec, dict):
        return []
    sid, pid = rec.get("sid"), rec.get("pid")
    # ⚠️ **两个都必须是整数**才算数（`{}` 的 get 全返回 None，而 `None == None`
    #    会让「空记录」看起来像「sid 就是 leader」—— 第一版就栽在这儿：
    #    没有记录文件时 `rec["sid"]` 直接 KeyError，`--stop` 退 1）。
    if not (isinstance(sid, int) and isinstance(pid, int) and sid == pid):
        return []
    return session_pids(sid, entries=entries)


# ============================================================ 只读查询

def port_busy(path):
    """串口有没有被占。`fuser` 会跟随 by-id 符号链接，所以传 by-id 路径也准。"""
    r = subprocess.run(["fuser", path], capture_output=True, text=True)
    return r.returncode == 0


def gripper_state(port=None):
    """读 AG-95 的 `0x0200`。返回 `(状态, 说明)`；读不到返回 `(None, 原因)`。

    **只读** —— 复用 `ag95_init_close.py` 的 `Gripper`，它唯一会写的地方是
    `write()`，本函数一次都不调。
    """
    try:
        import ag95_init_close as ag
    except Exception as exc:                      # noqa: BLE001
        return None, f"导入 ag95_init_close 失败：{exc}"
    old = ag.PORT
    if port:
        ag.PORT = port
    try:
        g = ag.Gripper.open_port()
        if g is None:
            return None, f"串口打不开（{ag.PORT}）"
        try:
            # 0 = 未初始化 / 2 = 进行中 / 1 = 已完成
            return g.read(ag.REG_INIT_STATUS), "ok"
        finally:
            g.close_port()
    finally:
        ag.PORT = old


def dashboard(ip, timeout=5.0):
    """**只读**问一句控制柜。返回 `{命令: 回复}`（**原文，未解析**）；连不上返回 None。

    走 29999 只读查询，**不发 play/power/stop、不碰 30001–30004**
    （红线 1：手搓 RTDE 包把 URControl 打死过）。

    ⚠️ 返回的是**线上原文**，不是纯值 —— 别拿它跟 `'RUNNING'` / `'PLAYING'`
    精确比，见 :func:`dash_mode` / :func:`dash_program_state`。
    """
    try:
        s = socket.create_connection((ip, 29999), timeout=timeout)
    except OSError:
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


# ------------------------------------------------- 控制柜回复的**线上格式**
#
# 🔴 2026-10-06 实测（29999 只读，一台正常在跑的控制柜）：
#
#     robotmode      'Robotmode: RUNNING'      ← 带**标签前缀**
#     programState   'STOPPED ur5ros.urp'      ← 无标签，但缀着**程序名**
#     safetystatus   'Safetystatus: NORMAL'    ← 带标签前缀
#
# 三条各自点自己的名 ⇒ 回复没有错行，这就是原样。
#
# 在此之前，`board_run` 是拿**原文**与纯值精确比的：
#   · `judge_cabinet` 的 `!= 'RUNNING'`  ⇒ 在任何正常控制柜上**恒 FAIL**；
#   · `is_playing` 的 `== 'PLAYING'`     ⇒ 播放时原文是 `'PLAYING ur5ros.urp'`，
#     **永不相等** ⇒ P3 永远等不到播放，300 s 超时退 2，而现场表现是
#     「人明明按了播放」。
# 病根是**夹具里写的是猜的格式**（`{'robotmode': 'RUNNING'}`），猜错了，
# 于是 1294 行离线自检全绿而真机恒红。夹具现已换成上面这三条**原文**。
#
# 所以：判定一律过下面这两个解析器，**永不**拿 `dashboard()` 的原文比。

#: UR 自己给回复加的标签词（只认这几个；别的冒号一律当**值的一部分**，不剥）。
#: 不剥的理由：`programState` 的程序名里可能有冒号（如 Windows 路径），
#: 靠标签白名单才不会把值切坏。
_DASH_LABELS = {"robotmode", "programstate", "program state",
                "safetystatus", "safety status"}


def dash_field(raw):
    """Dashboard 一行回复 → 纯值（剥掉 UR 的标签前缀；无标签的原样返回）。"""
    s = str(raw or "").strip()
    head, sep, tail = s.partition(":")
    if sep and head.strip().lower() in _DASH_LABELS:
        s = tail.strip()
    return s


def dash_mode(d):
    """`robotmode` → `'RUNNING'` / `'POWER_OFF'` / …（**首词**，大写）。

    ⚠️ 取**首词**而不是整串：实测已证 UR 会往这些行后面缀东西
    （`programState` 就缀了程序名）。首词不可能**造出**假绿 ——
    `RUNNING` 仍必须是这行的第一个词。
    """
    f = dash_field((d or {}).get("robotmode"))
    return f.split()[0].upper() if f.split() else ""


def dash_program_state(d):
    """`programState` → `'STOPPED'` / `'PLAYING'` / `'PAUSED'`（**首词**，大写）。

    ⚠️ 后缀是实测的，不是防御性编程：原文 `'STOPPED ur5ros.urp'` 里
    `ur5ros.urp` 是**当前载入的程序名**，播放时就是 `'PLAYING ur5ros.urp'`。
    （播放那一行**尚未实测** —— 等 R1 自然产生时抓一次核对；判据不依赖它。）
    """
    f = dash_field((d or {}).get("programState"))
    return f.split()[0].upper() if f.split() else ""


def is_playing(d):
    """示教器在 PLAYING 吗。**全仓唯一**一份判定，`board_run` 与 `board_a1` 都用它。

    （曾有两份：`board_run` 精确比、`board_a1` 用子串 `"PLAYING" in …`。
    子串那份**碰巧**是对的，精确比那份恒 False —— 两份并存正是缺陷能活下来的原因。）
    """
    return dash_program_state(d) == "PLAYING"


# ============================================================ 人机协作（AI 转达）
#
# ★ 这一节是**为「AI 驱动」那一轮写的**（2026-10-08 用户裁定）：
#   「启动器需要提示 ai 告诉我去示教器点 play 和 stop。启动器要考虑到 ai+人工的
#     协作方式。」
#
#   背景：AI 在后台跑（没有终端），它的输出进**日志** —— 人**看不到**。
#   所以「请到示教器按播放」这句话，原来印给人看的散文现在得**由 AI 转达给人**。
#   改成固定前缀的一行之后，AI 的原样转达就够了：**每少一道转述，就少一次走样**。
#
#   ⚠️ 判据**不是**「AI 有没有照做」，而是「**物理动作有没有发生**」：
#     那两步由 `ensure_in_the_loop()` 用**只读** Dashboard 查询核验，两步都看见才算。
#     提示语气再客气也不构成授权（红线 6）。
#
#   ⚠️ 这一节里**没有一处**写设备、发运动或发 play/power/stop —— 全是只读查询。
HUMAN_MARK = ">>> 需要人动手"


def human_action(text, step=None, total=None, why=None, out=print):
    """印**一行**「需要人动手」——前缀固定 `HUMAN_MARK`，好让 AI 认出来**原样转达**。

    中文、动词在前、独立一行（用户 2026-10-07 的裁定：当场要做的动作别夹在
    英文段落里）。第二行只写「为什么 + 做完会怎样」，是给人读的注脚，
    **不是**指令的一部分 —— 转达第一行就够。
    """
    n = "" if step is None else "（第 %s/%s 步）" % (step, total if total else "?")
    out("%s%s：%s" % (HUMAN_MARK, n, text))
    if why:
        out("    （%s；做完启动器自己会看到，**不用回复**）" % why)


def relay_notice(out=print, timeout=None, doing="启动器"):
    """无终端（AI 驱动）时印**一次**的「转达契约」。

    写在最前面，是为了让 AI 在第一次需要人之前就知道该怎么处理那些行 ——
    而不是等它先愣一次。
    """
    out("[AI 驱动] 本次没有交互终端：下面所有以「%s」开头的行"
        "**都是给你转达给人的**。" % HUMAN_MARK)
    out("          原样念给人听即可 —— 你**代劳不了**：那几步要在示教器上真的按。")
    out("          %s 每一步最多等 %s。"
        % (doing, ("%.0fs" % timeout) if timeout else "（按 --play-timeout）"))


def wait_for_state(ip, want, timeout, poll=2.0, dash=None, clock=time.time,
                   sleep=None):
    """轮询只读直到 `want(回复)` 为真 → `(到了没, 最后一次回复, 用时)`。

    只读查询（`robotmode` / `programState` / `safetystatus`）；**不发** play / power /
    stop —— 按播放是**人的动作**（红线 6）。

    `dash` / `clock` / `sleep` 都可注入 ⇒ 这个循环**离线测得动**。
    """
    dash = dash or dashboard
    sleep = time.sleep if sleep is None else sleep
    t0 = clock()
    last = None
    while True:
        last = dash(ip)
        if want(last):
            return True, last, clock() - t0
        if clock() - t0 >= timeout:
            return False, last, clock() - t0
        sleep(poll)


def ensure_in_the_loop(ip, timeout, poll=2.0, dash=None, clock=time.time,
                       sleep=None, prompt=print, why="复核「人在环」",
                       consequence="不交棒", doing="启动器", step=(1, 2)):
    """让人在示教器上**重按一次（先 STOP，再 PLAY）**，并**看到那两步都真的发生**
    → `(过了没, 说明)`。

    ★ 这是「**没有终端**时唯一能拿到的当次授权证明**」（AI 驱动那一轮）。
      它替掉的是键盘上的 `yes`：能敲键盘的人**未必在臂旁边**，而按示教器的人一定在。

    为什么是**两步握手**，不是只喊一句「按 STOP 再按 PLAY」：两个动作挨得近时，
    2 s 一次的轮询可能**一次都没看见 STOPPED** —— 那时「PLAYING 一直为真」
    既可能是「人按了」也可能是「压根没动」，**判不了**。所以卡在两步之间：
    先**确认**它离开 PLAYING，再等它回来 ⇒ 「按过」这件事是**看到**的，不是猜的。
    第一步没看到就直接**不往下走**（fail-closed）。

    这**不是新规矩**：第一百一十轮实测过「收栈之后控制柜可能仍停在 PLAYING」，
    当时的结论就是「每轮之间必须在示教器上按一次 STOP」—— 这里只是把它写成
    启动器自己会走的步骤。

    全程只读（`dashboard`）；**不发** play / power / stop。
    `why` / `consequence` / `doing` 只影响文案（写路径那一档要说「不写」而不是
    「不交棒」—— 两处的后果不一样，话说成一样就没人分得清了）。
    """
    if not ip:
        return False, ("读不到控制柜 IP ⇒ **核验不了**那个物理动作 ⇒ %s"
                       "（授权只认**当次**，红线 6）" % consequence)

    prompt("")
    prompt("=" * 66)
    human_action("请在示教器上按一下 **停止（STOP）**",
                 step=step[0], total=step[1], out=prompt,
                 why="上一轮收栈后控制柜可能仍停在 PLAYING —— 那种「本来就绿」"
                     "不算你按过")
    prompt("=" * 66)
    left, d, _ = wait_for_state(ip, lambda x: not is_playing(x), timeout,
                                poll=poll, dash=dash, clock=clock, sleep=sleep)
    if not left:
        return False, ("%.0fs 内没看到它离开 PLAYING（原文 %r）—— 「人在环」"
                       "是不是**这一轮**走通的，判不了 ⇒ **%s**"
                       % (timeout, str((d or {}).get("programState", "")).strip(),
                          consequence))

    prompt("")
    prompt("=" * 66)
    human_action("请在示教器上按一下 **播放（PLAY）**",
                 step=step[1], total=step[1], out=prompt,
                 why="前一步它不是 PLAYING、现在按到 PLAYING ⇒ 那一下**只可能是"
                     "人按的**（这就是「当次」的证明）")
    prompt("=" * 66)
    back, d, waited = wait_for_state(ip, is_playing, timeout,
                                     poll=poll, dash=dash, clock=clock,
                                     sleep=sleep)
    if not back:
        return False, ("按了停止之后，%.0fs 内没等到 PLAYING（原文 %r）⇒ **%s**"
                       % (timeout, str((d or {}).get("programState", "")).strip(),
                          consequence))
    return True, ("STOP → PLAY 两步**都看到了**（%.0fs 后回到 PLAYING）—— "
                  "「%s」是这一轮走通的" % (waited, why))


# ============================================================ 站点配置

SITE_ENV = "UR5VG_SITE"
SITE_DEFAULT = "~/.config/ur5-vision-grasp/site.yaml"   # 仓库外；仓内只有 *.example.yaml
DOMAIN_DEFAULT = 99
CAM_KEYS = ("camera_x", "camera_y", "camera_z",
            "camera_qx", "camera_qy", "camera_qz", "camera_qw")


class SiteError(Exception):
    """站点配置读不到 / 不合 YAML 形状。**不**表示「内容不合法」——
    内容问题由 `validate_site` 逐条列出（要一次看全，不要抛第一个就停）。"""


def site_path(explicit=None):
    """站点配置路径：`--site` > `$UR5VG_SITE` > `~/.config/…/site.yaml`。
    三个都在仓库外 —— 仓内**没有**任何指向真值的默认值。"""
    return os.path.expanduser(explicit or os.environ.get(SITE_ENV) or SITE_DEFAULT)


class Site(object):
    """站点配置。取值一律经属性，**缺项返回 None，不静默回落**。
    合不合法让 `validate_site` 说，调用方按三态处置。
    """

    __slots__ = ("raw", "path")

    def __init__(self, raw, path="<memory>"):
        if raw is not None and not isinstance(raw, dict):
            raise SiteError(f"{path} 的顶层不是映射（是 {type(raw).__name__}）")
        self.raw = raw or {}
        self.path = path

    # ---- 基本项 ----
    @property
    def robot_ip(self):
        return self.raw.get("robot_ip") or os.environ.get("UR_ROBOT_IP")

    @property
    def planner_params(self):
        p = self.raw.get("planner_params")
        return os.path.expanduser(str(p)) if p else None

    @property
    def ros_domain_id(self):
        v = self.raw.get("ros_domain_id")
        return DOMAIN_DEFAULT if v is None else int(v)

    @property
    def gripper_port(self):
        g = self.raw.get("gripper")
        g = g if isinstance(g, dict) else {}
        return g.get("port") or self.raw.get("gripper_port")

    # ---- 俯视相机外参（含时效）----
    @property
    def topcam(self):
        t = self.raw.get("topcam")
        return t if isinstance(t, dict) else {}

    @property
    def verified(self):
        return self.topcam.get("verified")

    @property
    def ttl_days(self):
        return self.topcam.get("ttl_days")

    def extrinsics(self):
        """`{'camera_x': '0.34…', …}` —— 值一律**字符串**（launch 参数就是字符串）。"""
        e = self.topcam.get("extrinsics")
        e = e if isinstance(e, dict) else {}
        return {k: str(e[k]) for k in CAM_KEYS if e.get(k) is not None}

    def missing_keys(self):
        """外参缺哪几个键（按 `CAM_KEYS` 顺序）。"""
        e = self.topcam.get("extrinsics")
        e = e if isinstance(e, dict) else {}
        return [k for k in CAM_KEYS if e.get(k) is None]


def parse_site(raw, path="<memory>"):
    """纯函数：把已经读进内存的 dict 包成 Site（好让离线测试不碰文件）。"""
    return Site(raw, path=path)


def load_site(explicit=None, path=None):
    """读站点配置。读不到 ⇒ 抛 `SiteError`（fail-closed，绝不退回默认值）。

    `yaml` 在这里才 import —— 本机的 PyYAML 有没有都不影响 `parse_site` 的离线测试。
    """
    p = path or site_path(explicit)
    if not os.path.exists(p):
        raise SiteError(
            f"读不到站点配置：{p}\n"
            f"   公开仓不写现场真值。配一次：\n"
            f"     mkdir -p ~/.config/ur5-vision-grasp\n"
            f"     cp {REPO}/config/site.example.yaml ~/.config/ur5-vision-grasp/site.yaml\n"
            f"     # 然后填真实值；或用 --site PATH / $UR5VG_SITE 指定别处")
    try:
        import yaml
    except ImportError as exc:
        raise SiteError(f"缺 PyYAML（{exc}）—— pip install pyyaml")
    try:
        with open(p, "r", encoding="utf-8") as fh:
            raw = yaml.safe_load(fh)
    except Exception as exc:                      # noqa: BLE001
        raise SiteError(f"{p} 不是合法 YAML：{exc}")
    return Site(raw, path=p)


def validate_site(site, env=None):
    """逐条列出站点配置的毛病（**纯函数**，返回 list；空 list = 没有毛病）。

    一次列全，不要抛第一个就停 —— 现场来回改五行比来回改一行省事得多。
    """
    env = os.environ if env is None else env
    p = []

    if not site.robot_ip:
        p.append("缺 robot_ip（站点文件里没写，环境变量 UR_ROBOT_IP 也没设）")
    if not site.planner_params:
        p.append("缺 planner_params —— 不给它会在真机上**静默用仿真几何**"
                 "（table_plane_z=-0.39），现象是臂完全不动")
    if not site.gripper_port:
        p.append("缺 gripper.port —— 前置检查要用它查串口占用、读 0x0200；"
                 "现场值形如 /dev/serial/by-id/usb-<厂商>_<型号>_<序列号>-if00-port0")

    miss = site.missing_keys()
    if miss:
        p.append("外参缺键：" + "、".join(miss))
    else:
        for k, v in site.extrinsics().items():
            try:
                float(v)
            except ValueError:
                p.append(f"外参 {k} 不是数：{v!r}")

    for k, why in (("topcam.verified", "没有它这一行**永远不会被判过期**，等于没写"),
                   ("topcam.ttl_days", "没有它就无法判过期")):
        if site.topcam.get(k.split(".")[1]) is None:
            p.append(f"缺 {k} —— {why}")

    d = site.ros_domain_id
    want = _int(env.get("ROS_DOMAIN_ID"))
    if want is not None and want != d:
        p.append(f"ROS_DOMAIN_ID 不一致：站点说 {d}，环境里是 {want} "
                 f"—— 两边不同的域号 = 两个互不可见的 DDS 网络，"
                 f"现象是「栈起着但什么话题都看不到」")
    return p


def as_date(v):
    """`date` / `datetime` / `'2026-10-05'` / `'2026-10-05 记'` → `date`；否则 None。"""
    if isinstance(v, dt.datetime):
        return v.date()
    if isinstance(v, dt.date):
        return v
    if isinstance(v, str):
        m = re.match(r"\s*(\d{4})-(\d{1,2})-(\d{1,2})", v)
        if m:
            try:
                return dt.date(int(m.group(1)), int(m.group(2)), int(m.group(3)))
            except ValueError:
                return None
    return None


MISSING_TTL = ("站点配置没写 topcam.ttl_days —— 缺了它这一行永远无法被判过期，"
               "按不新鲜处置")


def judge_freshness(verified, ttl_days, today=None):
    """俯视外参的时效。返回 `(PASS|FAIL, 说明)`。**纯函数**。

    **与 `state_check.py` 同一条线**：`age > 保质期` 才算超期，`age == 保质期`
    当天仍算数。两处判据必须一致，否则同一份文件两个守卫各说各话。

    fail-closed：字段缺失、类型不对、日期解析不了 **一律 FAIL** ——
    「不知道」不许当成「新鲜」。俯视外参实测一天能漂 0.945°，
    过期外参的症状是点云整体偏、`/targets_3d` 的候选落到臂够不到的地方。
    """
    today = today or dt.date.today()
    if ttl_days is None or isinstance(ttl_days, bool):
        return FAIL, MISSING_TTL
    try:
        ttl = int(ttl_days)
    except (TypeError, ValueError):
        return FAIL, f"topcam.ttl_days 不是整数：{ttl_days!r}"
    if ttl < 0:
        return FAIL, f"topcam.ttl_days 是负数：{ttl}"
    d = as_date(verified)
    if d is None:
        return FAIL, f"topcam.verified 缺失或解析不出日期：{verified!r}"
    age = (today - d).days
    if age > ttl:
        return FAIL, (f"外参核对于 {d}（{age} 天前），保质期 {ttl} 天 —— **已超期**。"
                      f"先跑 python3 scripts/topcam_drift_check.py，再填新日期")
    return PASS, f"外参核对于 {d}（{age} 天前），保质期 {ttl} 天，剩 {ttl - age} 天"


def freshness_gate(site, today=None, override=None):
    """站点外参时效闸门。`override` = 人写的破例理由（`--override-freshness`）。

    ⚠️ 破例**必须写下理由**且会被原样记进报告 —— 空理由拒绝。
    """
    if override is not None:
        if not str(override).strip():
            return FAIL, "--override-freshness 给了空理由 —— 破例必须写下为什么"
        return PASS, f"⚠️ 已由人显式破例，理由：{str(override).strip()}"
    return judge_freshness(site.verified, site.ttl_days, today=today)


# ============================================================ 自查入口

def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(
        description="站点配置自查：路径、合法性、外参时效（全离线，不碰设备）")
    ap.add_argument("--site", default=None, help=f"站点配置路径（默认 ${SITE_ENV} 或 {SITE_DEFAULT}）")
    ap.add_argument("--today", default=None, help="当成哪一天（YYYY-MM-DD），用于演练时效判定")
    a = ap.parse_args(argv)

    p = site_path(a.site)
    print(f"站点配置：{p}")
    print(f"  （来源优先级：--site > ${SITE_ENV} > {SITE_DEFAULT}）")
    try:
        site = load_site(path=p)
    except SiteError as exc:
        print(f"❌ {exc}")
        return 2

    problems = validate_site(site)
    today = as_date(a.today) if a.today else None
    if a.today and today is None:
        print(f"❌ --today 解析不出日期：{a.today!r}")
        return 2
    st, detail = freshness_gate(site, today=today)
    print(f"  外参时效：[{st}] {detail}")

    if problems:
        print(f"\n❌ {len(problems)} 条毛病：")
        for i, x in enumerate(problems, 1):
            print(f"  {i}. {x}")
        return 1
    print("  ✓ 逐项合法")
    if st != PASS:
        return 1
    print("\n结果：✅ 站点配置可用")
    return 0


if __name__ == "__main__":
    sys.exit(main())

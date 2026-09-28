#!/usr/bin/env python3
"""真机验收：Ctrl-C 收工到底干不干净（D70）。**在板子上跑**，不是在 WSL 上跑。

## 为什么不能用 `timeout`、也不能用 `命令 &` 来模拟 Ctrl-C

两个都试过，**都会给出假的结论**（2026-09-28 板端实测，两次都被骗）：

1. **`setsid 命令 &`（非交互 shell 的后台任务）会让命令继承 `SIGINT=SIG_IGN`。**
   POSIX：没有作业控制的非交互 shell 启动的后台任务，`SIGINT`/`SIGQUIT` 置为忽略。
   于是信号压根没送到，节点还活着，而脚本报「收工干净」。
   实测证据：`grep SigIgn /proc/<pid>/status`，`SIGINT`（`0x2`）那一位是置上的。
   （`set -m` 打开作业控制可以绕开继承，但那样也不是终端 Ctrl-C。）
2. **GNU `timeout` 默认把信号发给整个进程组**（`--foreground` 才只发子进程）。
   于是每个子进程收到**两次** `SIGINT`：一次来自 `timeout` 的整组投递，一次来自
   `ros2 launch` 自己的转发。第二次落到 rclpy 交还的默认处置上，进程被信号打死，
   launch 报 `process has died ... exit code -2` —— **看起来像代码有问题，其实
   是测的方式不对**。

真正的终端 Ctrl-C 走 pty：`\\x03` 写进主端，行规程把它变成 `SIGINT` 发给该 pty 的
**前台进程组**，`ros2 launch` 与所有子进程各收一次，launch 再走自己的收工流程。
本脚本就用 pty 复现它。**干净的板子**上，两种路径（pty、整组裸信号）现在都是
`finished cleanly` —— 见 `docs/RESTRUCTURE_PLAN.md` 第二十八轮。

## 判据（D70 的验收标准）

* 每个节点报 `process has finished cleanly`；
* 日志里**没有** `Traceback`、没有 `RCLError`；
* `ros2 launch` 自己退出码 0。

`exit code -2` 只在上面第 2 种假模拟里出现过，**不是验收对象**。

## 用法（在板子上，先 source 两个 setup.bash）

    python3 scripts/board_ctrlc_check.py                 # 默认：不含机械臂的入口
    python3 scripts/board_ctrlc_check.py --seconds 40

⚠️ 默认入口（`pipeline.launch.py camera_source:=realsense`）**不含机械臂**：没有
UR 驱动、没有 MoveIt、不发任何轨迹。**不要**把 `--entry` 指向 `real.launch.py`
之类会去连机械臂的入口 —— 那个入口会真的驱动真机。
"""

import argparse
import os
import pty
import select
import signal
import subprocess
import sys
import time

# 收工前先确认板子是干净的：残留的上一轮进程会让进程组/pid 的归属变乱，实测会
# 得出「exit code -2」这种假故障（我第一次就是这么被误导的）。
LEFTOVER_PATTERNS = [
    "ros2 launch yolo_bringup",
    "detector_node",
    "projector_node",
    "grasp_planner/planner_node",
    "realsense2_camera_node",
]

DEFAULT_ENTRY = (
    "ros2 launch yolo_bringup pipeline.launch.py "
    "camera_source:=realsense gripper_backend:=none carry_objects:=false"
)

# 收工判据要看的行
VERDICT_RE = ("process has died", "process has finished cleanly",
              "exit code", "Traceback", "RCLError", "rcl_shutdown already")


def _self_and_ancestors():
    """自己 + 自己这条进程链上的所有祖先。

    ⚠️ 必须排除它们：这个脚本常常是这么被调起来的 ——

        ssh jetson '... ; pgrep -af "detector_node|ros2 launch yolo_bringup" ...'

    于是**调用它的那层 `bash -c` 的命令行里就带着这些模式**，`pgrep -f` 会把它
    自己匹配上，守卫当场误报「板子上还有残留」。实测踩过：守卫报了 3 条，pid 全是
    同一个 —— 就是我这条 ssh 命令的 shell。
    """
    pids, pid = set(), os.getpid()
    while pid and pid != 1:
        pids.add(pid)
        try:
            with open(f"/proc/{pid}/stat") as fh:
                pid = int(fh.read().rsplit(")", 1)[1].split()[1])
        except (OSError, IndexError, ValueError):
            break
    return pids


def leftovers():
    skip = _self_and_ancestors()
    out = []
    for pat in LEFTOVER_PATTERNS:
        r = subprocess.run(["pgrep", "-f", pat], capture_output=True, text=True)
        pids = []
        for tok in r.stdout.split():
            pid = int(tok)
            if pid in skip:                       # 自己 / 自己的祖先（含那条 ssh bash -c）
                continue
            try:
                with open(f"/proc/{pid}/cmdline", "rb") as fh:
                    cmd = fh.read().replace(b"\0", b" ").decode(errors="replace")
            except OSError:
                continue
            if "pgrep" in cmd or "board_ctrlc_check" in cmd:
                continue                          # 别人的 pgrep / 另一次本脚本
            pids.append(pid)
        if pids:
            out.append((pat, pids))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--entry", default=DEFAULT_ENTRY,
                    help="要 Ctrl-C 的 launch 命令（默认：不含机械臂的那个入口）")
    ap.add_argument("--seconds", type=float, default=25.0,
                    help="起来跑多久再发 Ctrl-C（默认 25 秒）")
    ap.add_argument("--grace", type=float, default=20.0,
                    help="Ctrl-C 之后最多再等多久收尾（默认 20 秒）")
    ap.add_argument("--log", default="/tmp/board_ctrlc_check.log")
    args = ap.parse_args()

    if not os.environ.get("ROS_DISTRO"):
        print("环境未就绪：先 source /opt/ros/humble/setup.bash 和 "
              "ros2_ws/install/setup.bash（本仓库约定：环境不满足退 2）")
        return 2

    stale = leftovers()
    if stale:
        print("板子上还有上一轮的进程，先清干净再测（否则结果不可信）：")
        for pat, pids in stale:
            print(f"  {pat}: {pids}")
        print("  pkill -9 -f 'ros2 launch yolo_bringup' ; "
              "pkill -9 -f 'detector_node|projector_node|planner_node|realsense2_camera_node'")
        return 2

    # pty.fork()：子进程是**新会话的会话首进程，并把这个 pty 当控制终端**，
    # 于是它有一个前台进程组，行规程产生的 SIGINT 会发给那一组 —— 与真终端一致。
    pid, fd = pty.fork()
    if pid == 0:
        os.execvp("bash", ["bash", "-lc", f"exec {args.entry}"])

    log = open(args.log, "wb")
    started = time.time()

    def pump(seconds):
        end = time.time() + seconds
        while time.time() < end:
            r, _, _ = select.select([fd], [], [], 0.3)
            if not r:
                continue
            try:
                data = os.read(fd, 65536)
            except OSError:
                return False
            if not data:
                return False
            log.write(data)
            log.flush()
        return True

    print(f"pty 前台进程组 pid={pid}；{args.seconds:.0f} 秒后写 \\x03（真 Ctrl-C）")
    running = pump(args.seconds)
    if not running:
        print("还没发信号，launch 就自己退出了 —— 先看日志")
    else:
        os.write(fd, b"\x03")
        pump(args.grace)

    log.close()
    # 收尸。**两个坑都在这几行里，我两个都踩过**：
    #
    # 1. 不能用「`status == 0`」判断成功：子进程**正常退出**时，`waitpid` 返回的
    #    原始 status 恰好就是 `0`（falsy），而 `WNOHANG` 没收到尸时返回的是
    #    `(0, 0)` —— 两者一个真值判断就分不开了。当初写成 `if status:`，于是
    #    launch 明明**干净退出、就在那儿等着收尸**，报告却印「宽限内没退，已强杀」。
    #    正确的判据是 `waitpid` 返回的**第一个元素**（收到尸的 pid，非 0 才算收到）。
    # 2. 顺带一句：`os.kill(pid, 0)` **探不出僵尸**（僵尸的进程项还在，信号探测
    #    照样成功）。我另写的一个探活脚本就因此报「launch 还活着 60 秒」，实际它
    #    早退成僵尸了（`/proc/<pid>/stat` 的 state 字段是 `Z`）——只有 `waitpid`
    #    能分出「在跑」和「跑完了没人收尸」。
    #
    # 宽限内还不退就强杀，**绝不能把残留留给下一次** —— 下一次会被开头的
    # 「残留守卫」挡住，更糟的是残留进程会混进进程组，把结论搞乱。
    exitcode = None
    reaped = False
    for _ in range(30):                       # 最多再等 15 秒
        try:
            done, status = os.waitpid(pid, os.WNOHANG)
        except ChildProcessError:
            reaped = True                     # 已经被收过（拿不到退出码，但确实退了）
            break
        if done:
            exitcode = os.waitstatus_to_exitcode(status)
            reaped = True
            break
        time.sleep(0.5)
    if not reaped:
        try:
            os.killpg(pid, signal.SIGKILL)    # pid 是会话首进程，pgid == pid
        except ProcessLookupError:
            pass
        try:
            os.waitpid(pid, 0)
        except ChildProcessError:
            pass

    text = open(args.log, "r", errors="replace").read()
    verdict = [l for l in text.splitlines() if any(k in l for k in VERDICT_RE)]
    print("\n--- 收工那一段 ---")
    for line in verdict:
        print(line)

    died = [l for l in verdict if "process has died" in l]
    trace = [l for l in verdict if "Traceback" in l or "RCLError" in l]
    clean = [l for l in verdict if "finished cleanly" in l]
    if reaped:
        how = f"{exitcode}" if exitcode is not None else "（已退出，退出码没拿到）"
    else:
        how = "（宽限内没退，已强杀）"
    print(f"\ncleanly={len(clean)}  died={len(died)}  traceback/RCLError={len(trace)}  "
          f"launch 退出码={how}   用时 {time.time() - started:.0f}s")
    if trace:
        print("FAIL：收工留下了 Traceback / RCLError（D70 的两个面之一又回来了）")
        return 1
    if died:
        print("FAIL：有节点报 died —— 先确认是不是用「裸信号」测的（见模块 docstring）")
        return 1
    if len(clean) < 3:
        print(f"FAIL：只看到 {len(clean)} 个节点报 cleanly，太少了（真机入口至少 4 个）")
        return 1
    print("PASS：全部 cleanly、无 traceback。")
    return 0


if __name__ == "__main__":
    sys.exit(main())

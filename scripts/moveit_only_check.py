#!/usr/bin/env python3
"""无臂预检：机械臂还没接线时，把 MoveIt 链路**除驱动外**整条验一遍。

**在板子上跑**（W5 第 4 步「只读验证」的驱动那一半需要真臂，这一半不需要）。

它起什么
--------
  · `real.launch.py`，但 `start_ur_driver:=false` —— **没有驱动就没有
    controller_manager，物理上没有任何东西可动**（moveit_servo 也已被
    real.launch.py 显式关掉，D59）。
  · `robot_state_publisher`（用 ur_description 的**未标定** URDF 起一份 TF 树。
    真机上它由驱动顺带起；无臂预检时缺了它，/compute_ik 会报
    FRAME_TRANSFORM_FAILURE —— 实测踩过，2026-09-28 板端）。
  · 一个**假 /joint_states** 发布器（HOME 姿态定频）。缺了它
    move_group 的 CurrentStateMonitor 拿不到状态，/compute_ik **挂住不回话**
    （服务在、也能列出来，但永远不完成 —— 实测踩过，同日）。

然后跑 `elbow_guard_probe.py`（只调 /compute_ik、/compute_fk，绝不发轨迹），
拿到抓取区/放置区候选位姿的肘/大臂高度余量。

⚠️ 数值口径：这里用的是**未标定** URDF 的运动学（ur_description 自己的注释就
   警告过：与示教器真值有非零偏差）。结论是「链路通 + 余量级」的，**最终
   上电前的判读必须用 ur_calibration 校准后的 URDF 重跑一遍**。

⚠️ 机械臂动作属于高风险操作。本脚本**结构上不可能**让臂动：没有驱动、没有
   controller_manager、控制箱也没接。但它起 MoveIt，所以**不要**在任何
   「已有别的驱动实例在跑」的环境里用（会形成两个 TF/描述来源）。

用法
----
    python3 scripts/moveit_only_check.py
    python3 scripts/moveit_only_check.py --grid 6

退出码：0 = 探针通过（所有候选连杆在护栏之上）；1 = 探针报告有候选低于护栏；
2 = 环境不满足（残留进程 / 缺包 / 服务起不来）。
"""

import argparse
import os
import re
import shutil
import signal
import subprocess
import sys
import time

# 残留守卫的匹配面。⚠️ 要先排除 `bash -c` 的行：这个脚本经 ssh 跑的时候，
# 调用者那条命令的命令行里就带着同样的模式，会把**自己**匹配上（
# scripts/board_ctrlc_check.py 的 docstring 记了同款陷阱）。
LEFTOVER_RE = re.compile(
    r"ros2 launch yolo_bringup|move_group|robot_state_publisher"
    r"|detector_node|projector_node|planner_node|realsense2_camera_node"
    r"|fake_joint_states")


def leftovers():
    r = subprocess.run(
        ["ps", "-eo", "pid,args", "--no-headers"], capture_output=True, text=True)
    out = []
    for line in r.stdout.splitlines():
        if "bash -c" in line or "moveit_only_check" in line:
            continue
        if LEFTOVER_RE.search(line):
            out.append(line.strip())
    return out


def sh(cmd, **kw):
    return subprocess.run(cmd, shell=True, capture_output=True, text=True, **kw)


def main():
    ap = argparse.ArgumentParser(description="无臂 MoveIt 预检（零运动风险）")
    ap.add_argument("--grid", type=int, default=4,
                    help="肘探针的采样密度（传给 elbow_guard_probe.py）")
    ap.add_argument("--ur-type", default="ur5")
    ap.add_argument("--log", default="/tmp/moveit_only_check.log")
    args = ap.parse_args()

    if not os.environ.get("ROS_DISTRO"):
        print("环境未就绪：先 source /opt/ros/humble/setup.bash 和 "
              "ros2_ws/install/setup.bash（约定：环境不满足退 2）")
        return 2
    for pkg in ("ur_moveit_config", "ur_description", "robot_state_publisher"):
        if not shutil.which("ros2") or subprocess.run(
                ["ros2", "pkg", "prefix", pkg],
                capture_output=True).returncode != 0:
            print(f"缺 {pkg} —— ros-humble-moveit / ros-humble-ur-* 没装齐，退 2")
            return 2

    stale = leftovers()
    if stale:
        print("板上有残留进程，先清场再测（否则 TF/服务归属会乱）：")
        for s in stale:
            print("  " + s)
        return 2

    procs = []  # (名字, Popen)

    def spawn(name, cmd, log):
        lf = open(log, "wb")
        procs.append((name, subprocess.Popen(
            cmd, shell=True, stdout=lf, stderr=subprocess.STDOUT,
            start_new_session=True)))

    try:
        # 1) MoveIt-only 的真机入口（不含驱动 → 不含 controller_manager）
        spawn("launch",
              "ros2 launch yolo_bringup real.launch.py "
              "start_ur_driver:=false publish_camera_tf:=false "
              "tf_required:=false gripper_backend:=none carry_objects:=false",
              args.log)

        # 2) robot_state_publisher（未标定 URDF；⚠️ 只用于预检，见模块 docstring）
        xr = sh("xacro $(ros2 pkg prefix ur_description)/share/ur_description"
                f"/urdf/ur.urdf.xacro ur_type:={args.ur_type} name:=ur "
                "safety_limits:=true > /tmp/ur5_check.urdf 2>/tmp/ur5_check.err")
        if xr.returncode != 0:
            print("xacro 失败：", xr.stderr or open("/tmp/ur5_check.err").read())
            return 2
        spawn("rsp",
              'ros2 run robot_state_publisher robot_state_publisher --ros-args '
              '-p "robot_description:=$(cat /tmp/ur5_check.urdf)"',
              "/tmp/moveit_only_check_rsp.log")

        # 3) 假 /joint_states（HOME 定频；move_group 靠它认为状态有效）
        fake_js = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "fake_joint_states.py")
        spawn("fake_js", f"python3 {fake_js}", "/tmp/moveit_only_check_js.log")

        # 4) 等 /compute_ik
        ready = False
        for i in range(36):
            time.sleep(5)
            if sh("ros2 service list").stdout.find("/compute_ik") >= 0:
                ready = True
                print(f"/compute_ik 就绪（约 {(i + 1) * 5} 秒）")
                break
        if not ready:
            print("120 秒内 /compute_ik 没起来 —— 日志：", args.log)
            return 2

        # 5) 肘高探针（只读）
        probe = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "elbow_guard_probe.py")
        rc = subprocess.call(
            [sys.executable, probe, "--node", "/grasp_planner",
             "--grid", str(args.grid)])
        return rc
    finally:
        for name, p in reversed(procs):
            try:
                os.killpg(p.pid, signal.SIGTERM)   # start_new_session ⇒ pgid==pid
            except ProcessLookupError:
                pass
        # ⚠️ 别用固定短宽限：`ros2 launch` 收到 TERM 后要**按顺序**收掉每个子
        # 节点，3 秒时它的 launch 自己先被我 SIGKILL 了，projector_node 就成了
        # 孤儿（实测踩过）。判据用 leftovers() 本身：清干净了才算完。
        deadline = time.time() + 15
        while time.time() < deadline and leftovers():
            time.sleep(1)
        if leftovers():
            for name, p in reversed(procs):
                if p.poll() is None:
                    try:
                        os.killpg(p.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
            for line in leftovers():           # 还在的直接按 pid 补刀
                try:
                    os.kill(int(line.split()[0]), signal.SIGKILL)
                except (ValueError, ProcessLookupError):
                    pass
            time.sleep(2)
        for name, p in reversed(procs):        # 自己的孩子要收尸，别留僵尸
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass
        stale = leftovers()
        if stale:
            print("⚠️ 收尾后仍有残留（下次开跑会被守卫拦下）：")
            for s in stale:
                print("  " + s)


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""起一套「够用于 `/compute_ik` / `/compute_fk` 的只读栈」，**里面没有任何能动的东西**。

## 为什么需要它（这不是个方便脚本，是补一个缺口）

`real.launch.py` 的 `start_ur_driver:=false` 会**连同驱动一起省掉
`robot_state_publisher`** —— 因为 RSP 是 `ur_control.launch.py` 顺带起的，
而 `_driver()` 在 `start_ur_driver:=false` 时**直接 `return []`**。

后果是实测出来的、而且**看起来像几何问题**：

```
· IK FRAME_TRANSFORM_FAILURE: 16
  ↳ 请求帧的 TF 查不到：多半少了 robot_state_publisher（真机上由驱动顺带起）
    —— 不是几何问题。
```

`/compute_ik` 照常被发现、照常被调用、**每次都失败**，于是包线扫出来
「一个位姿都没解出来」—— 而那张表若不加判断就会被读成「桌面全都不可用」。
（包线脚本自己在末尾有一条兜底告警，上面那段就是它打出来的。）

第八十三轮的 `moveit_only_check.py` 之所以能跑，是因为它**自己起了 RSP**
（并且是同一个描述包 + 显式 `kinematics_params`）。本脚本把那段抽出来，
让「无臂栈」变成一个可以独立起停的东西。

## 它起什么

| # | 进程 | 为什么 |
|---|---|---|
| 1 | `real.launch.py start_ur_driver:=false …` | MoveIt（含 `/compute_ik` `/compute_fk`）+ 感知 + 规划器节点（包线要读它的参数） |
| 2 | `robot_state_publisher` | **上面缺的就是它**。描述包与 `real.launch.py` 同源，否则又是「同一张图两套几何」 |
| 3 | `fake_joint_states.py` | 定频 HOME 关节角。**假的是关节角，不是几何** —— move_group 要有状态才认为 IK 可解 |

⚠️ **它不发任何轨迹**：没有驱动、没有 `controller_manager`、`robot_ip` 无人使用。
但既然整条链是活的，跑完请 `--stop` 清场，别和真机那次搞混。

## 用法

    python3 scripts/no_arm_stack.py            # 起，等到 /compute_ik 就绪后**留在前台**
    python3 scripts/no_arm_stack.py --stop     # 清场（起之前没清干净也用它）
    python3 scripts/no_arm_stack.py --check    # 只看「现在能不能用」，不起任何东西

退出码：0 = 就绪；2 = 环境/前置不满足。
``--stop`` 与 ``--check`` 下 0 = 干净/可用。
"""

import argparse
import os
import re
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

# 复用已经验过的那两件：读描述包（**不抄值**）与「等 rclpy 图发现」——
# 后者刻意不走 `ros2` CLI，因为 CLI 会挂，而「挂住」与「真的没有」用 CLI 分不开。
from moveit_only_check import description_package, leftovers, sh   # noqa: E402

LOG_DIR = "/tmp"
LAUNCH_LOG = f"{LOG_DIR}/no_arm_stack_launch.log"
RSP_LOG = f"{LOG_DIR}/no_arm_stack_rsp.log"
JS_LOG = f"{LOG_DIR}/no_arm_stack_js.log"
URDF_PATH = f"{LOG_DIR}/no_arm_stack.urdf"

PROCS = []


def spawn(name, cmd, log):
    """起一个进程，日志进文件。**自成进程组** —— 收场时按组杀，
    否则 `ros2 launch` 的子节点会漏（它收到 TERM 后要按顺序收每个子进程）。"""
    lf = open(log, "wb")
    p = subprocess.Popen(cmd, shell=True, stdout=lf, stderr=subprocess.STDOUT,
                         start_new_session=True)
    PROCS.append((name, p, lf))
    return p


def launch_command(entry):
    """无驱动入口。**单拎出来是为了能被自检钉住** —— `start_ur_driver:=false`
    是这套东西「物理上不可能动」的全部依据（没有驱动 ⇒ 没有 controller_manager
    ⇒ 没有轨迹 action）。谁把它改成 true，一条断言就该红。"""
    return (f"ros2 launch yolo_bringup {entry} "
            "start_ur_driver:=false publish_camera_tf:=false "
            "tf_required:=false gripper_backend:=none carry_objects:=false")


def rsp_command(urdf_path):
    """`robot_state_publisher` 那条命令 —— **这就是 `start_ur_driver:=false`
    缺掉的那一块**。

    单拎出来同样是为了可测：少了它，`/compute_ik` 照常被发现、照常被调用、
    **每次都回 `FRAME_TRANSFORM_FAILURE`**，而那个症状看起来像几何问题
    （包线会扫出「一个位姿都没解出来」）。这不是假想：本脚本就是为它而写的。
    """
    return ("ros2 run robot_state_publisher robot_state_publisher --ros-args "
            f'-p "robot_description:=$(cat {urdf_path})"')


def wait_for_ik(timeout=180.0):
    """等 `/compute_ik` 与 `/compute_fk` **都**出现。

    ⚠️ 必须用 rclpy 图发现，不能用 `ros2 service list`：CLI 在板子上会挂住，
    而挂住和「真的没有这个服务」用 CLI 分不开 —— 会把一个环境问题读成一个
    几何结论。（这条是第八十三轮 `wait_for_services` 的同一个理由。）
    """
    import rclpy
    from rclpy.node import Node
    want = {"/compute_ik", "/compute_fk"}
    t0 = time.time()
    rclpy.init()
    node = Node("no_arm_stack_probe")
    try:
        while time.time() - t0 < timeout:
            have = {n for n, _ in node.get_service_names_and_types()}
            if want <= have:
                return True, time.time() - t0
            time.sleep(1.0)
        return False, time.time() - t0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def stop_all():
    """清场：先按进程组 TERM，再等，再 KILL。最后回报还剩什么。"""
    for name, p, lf in PROCS:
        if p.poll() is None:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                print(f"  已 TERM {name}（pgid {os.getpgid(p.pid)}）")
            except (ProcessLookupError, PermissionError):
                pass
    # `ros2 launch` 收子进程要时间，固定短宽限不够 —— 给足再 KILL
    for _ in range(30):
        if all(p.poll() is not None for _, p, _ in PROCS):
            break
        time.sleep(1.0)
    for name, p, lf in PROCS:
        if p.poll() is None:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                print(f"  已 KILL {name}")
            except (ProcessLookupError, PermissionError):
                pass
        try:
            lf.close()
        except OSError:
            pass


def main():
    ap = argparse.ArgumentParser(
        description="只读的「无臂栈」：MoveIt + RSP + 假关节角，够 /compute_ik 用")
    ap.add_argument("--stop", action="store_true", help="清场后退出")
    ap.add_argument("--check", action="store_true",
                    help="只看现在能不能用（不起任何东西）")
    ap.add_argument("--ur-type", default="ur5")
    ap.add_argument("--entry", default="real.launch.py")
    ap.add_argument("--timeout", type=float, default=180.0)
    args = ap.parse_args()

    if not os.environ.get("ROS_DISTRO"):
        print("环境未就绪：先 source /opt/ros/humble/setup.bash 与 "
              "ros2_ws/install/setup.bash（约定：环境不满足退 2）")
        return 2

    stale = leftovers()
    if args.check:
        ok, waited = wait_for_ik(timeout=10.0)
        print("可用：/compute_ik 与 /compute_fk 都在" if ok else
              "不可用：至少少一个服务 —— 先跑不带 --check 的本脚本把它起起来")
        if stale:
            # ⚠️ 措辞要准：`--check` 是**另一个进程**，它无从知道这些是
            # 「上一份栈还在跑」还是「没人管的残留」——两者的处置完全相反
            # （前者别动，后者要清）。写成「不是本脚本起的残留」就是在说一件
            # 它没查过的事。只报事实，让读的人自己判。
            print(f"另有 {len(stale)} 个进程在跑（本脚本无从判断是上一份栈还是残留；"
                  f"要清就 `--stop`）：")
            for s in stale[:5]:
                print("  " + s)
        return 0 if ok else 2

    if args.stop:
        print("清场：")
        stop_all()
        rest = leftovers()
        if rest:
            print(f"⚠️ 还有 {len(rest)} 个残留（**不是**本脚本起的，请人工看）：")
            for s in rest[:10]:
                print("  " + s)
            return 2
        print("  干净")
        return 0

    if stale:
        print("板上有残留进程，先清场再起（否则 TF/服务/话题归属会乱）：")
        for s in stale:
            print("  " + s)
        return 2

    desc_pkg = description_package()
    if not desc_pkg:
        print("读不到 real.launch.py 的 DESCRIPTION_PACKAGE —— 拒绝对一份不知道"
              "从哪来的几何起栈，退 2")
        return 2

    try:
        # 1) 无驱动的真机入口（没有驱动 ⇒ 没有 controller_manager ⇒ 物理上不可能动）
        print(f"入口：{launch_command(args.entry)}\n（日志 {LAUNCH_LOG}）", flush=True)
        spawn("launch", launch_command(args.entry), LAUNCH_LOG)

        # 2) RSP —— **这就是 start_ur_driver:=false 缺掉的那一块**。
        #    描述包与 real.launch.py 同源；`kinematics_params` 必须**显式传**，
        #    否则 xacro 那个参数默认指向 ur_description，悄悄退回标称，
        #    于是同一张图里 IK 走标定、TF 走标称（第八十二→八十三轮的真实教训）。
        prefix = sh(f"ros2 pkg prefix {desc_pkg}", timeout=15).stdout.strip()
        if not prefix:
            print(f"描述包 {desc_pkg} 找不到，退 2")
            return 2
        share = os.path.join(prefix, "share", desc_pkg)
        kin = os.path.join(share, "config", args.ur_type,
                           "default_kinematics.yaml")
        xr = sh(f"xacro {share}/urdf/ur.urdf.xacro ur_type:={args.ur_type} "
                f"name:=ur safety_limits:=true kinematics_params:={kin} "
                f"> {URDF_PATH}")
        if xr.returncode != 0:
            print(f"xacro 失败：{xr.stderr}")
            return 2
        with open(URDF_PATH, encoding="utf-8") as f:
            m = re.search(r"calib_\d+", f.read())
        print(f"描述包 = {desc_pkg}"
              + (f"，kinematics/hash = {m.group(0)}" if m else
                 "，⚠️ 生成物里没找到 calib_* hash"), flush=True)
        spawn("rsp", rsp_command(URDF_PATH), RSP_LOG)

        # 3) 假关节角：**假的是关节角，不是几何**。move_group 要有状态才解 IK。
        spawn("fake_js", f"python3 {os.path.join(HERE, 'fake_joint_states.py')}",
              JS_LOG)

        ok, waited = wait_for_ik(timeout=args.timeout)
        if not ok:
            print(f"{waited:.0f} 秒内 /compute_ik /compute_fk 没齐 —— 见 {LAUNCH_LOG}")
            stop_all()
            return 2
        print(f"就绪（约 {waited:.0f} 秒）：/compute_ik 与 /compute_fk 都在", flush=True)
        print("⇒ 现在可以跑 table_safety_envelope.py / moveit_only_check.py；"
              "跑完用 --stop 清场。", flush=True)

        # 留在前台：调用方按进程组起停它。Ctrl-C / TERM 都走同一条清场路径。
        stop = {"v": False}

        def _on_sig(_s, _f):
            stop["v"] = True

        signal.signal(signal.SIGINT, _on_sig)
        signal.signal(signal.SIGTERM, _on_sig)
        while not stop["v"]:
            time.sleep(1.0)
        print("\n收到停止信号，清场：")
        stop_all()
        return 0
    except Exception as e:                                     # noqa: BLE001
        print(f"起栈失败：{type(e).__name__}: {e}")
        stop_all()
        return 2


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""无臂预检：机械臂还没接线时，把 MoveIt 链路**除驱动外**整条验一遍。

**在板子上跑**（W5 第 4 步「只读验证」的驱动那一半需要真臂，这一半不需要）。

它起什么
--------
  · `real.launch.py`，但 `start_ur_driver:=false` —— **没有驱动就没有
    controller_manager，物理上没有任何东西可动**（moveit_servo 也已被
    real.launch.py 显式关掉，D59）。
  · `robot_state_publisher`（起一份 TF 树。真机上它由驱动顺带起；无臂预检时缺了它，
    /compute_ik 会报 FRAME_TRANSFORM_FAILURE —— 实测踩过，2026-09-28 板端）。
    它用的描述包**从 `real.launch.py` 里读**（`DESCRIPTION_PACKAGE`），
    免得同一张图里出现两套几何 —— 见下面那条 ⚠️。
  · 一个**假 /joint_states** 发布器（HOME 姿态定频）。缺了它
    move_group 的 CurrentStateMonitor 拿不到状态，/compute_ik **挂住不回话**
    （服务在、也能列出来，但永远不完成 —— 实测踩过，同日）。

然后跑 `elbow_guard_probe.py`（只调 /compute_ik、/compute_fk，绝不发轨迹），
拿到抓取区/放置区候选位姿的肘/大臂高度余量。

加了 `--envelope-z` 还会再跑 `table_safety_envelope.py`：**扫一整片桌面**，画出
「哪些 (x,y) 会把大臂/腕压到台面下、哪些够不到」的边界 —— 那是给现场**划区**用
的（肘探针看的是「已经框好的区安不安全」）。两者共用同一份几何与同一套判据，
所以包线的结论就是护栏会做的判断。

⚠️ 数值口径（**第八十三轮改**）：这里用的描述包与 `real.launch.py` **同源**，
   所以 `move_group`（IK/FK）与 `robot_state_publisher`（TF）是**同一套几何**。
   第八十二轮把 `real.launch.py` 的 `DESCRIPTION_PACKAGE` 换成了
   `yolo_ur_description`（本机出厂标定），而本脚本原先写死从 `ur_description`
   手搓 URDF ⇒ **同一张图里两套几何**（实测：`calib_209549117540498681` 对
   `calib_2353869974907916917`，`tool0` 位置最多差 3.59 mm）。
   现在两边都从 `DESCRIPTION_PACKAGE` 取，**不会再分叉**；开跑时会打印本次
   实际用的包与 `kinematics/hash`，判读时先看那一行。

   ⇒ 老口径（「这里用的是未标定 URDF，结论只是余量级，上电前必须用校准后的
   URDF 重跑」）**已由本次改动消解**：预检用的就是上电要用的那一份。

⚠️ 机械臂动作属于高风险操作。本脚本**结构上不可能**让臂动：没有驱动、没有
   controller_manager、控制箱也没接。但它起 MoveIt，所以**不要**在任何
   「已有别的驱动实例在跑」的环境里用（会形成两个 TF/描述来源）。

用法
----
    python3 scripts/moveit_only_check.py
    python3 scripts/moveit_only_check.py --grid 6
    python3 scripts/moveit_only_check.py --no-pipeline    # 见下
    # 划区用：真机台面在 base_frame ≈ 0，扫一遍画边界
    python3 scripts/moveit_only_check.py --envelope-z 0.0 --envelope-json /tmp/env.json
    # 只想量包线时不重跑探针也行（扫得慢，可以先 --grid 1 快速过一遍探针）

`--no-pipeline`：不 include `pipeline.launch.py`。**开发机上没有
`ros-humble-realsense2-camera` 时必需** —— `camera_source:=realsense` 会去拉那个包，
拉不到就**把整个 launch 带崩**（`move_group` 已经起来了也会被 SIGINT 掉，而报错行
只讲相机，看起来像 MoveIt 坏了）。板子上装了相机包，**默认不加这个开关**。

⚠️ 加了它就没有 `/grasp_planner` 了，`elbow_guard_probe.py` 会退回它自己的
   兜底默认值并**打警告**。所以这个开关只用于「验 MoveIt 链路本身通不通」，
   **不要**用它去主持护栏口径的判读（护栏要读运行中节点的参数才算数）。

退出码：0 = 探针通过（所有候选连杆在护栏之上）；1 = 探针报告有候选低于护栏；
2 = 环境不满足（残留进程 / 缺包 / 服务起不来 / 包线一个位姿都没量出来）。
⚠️ 包线**自己**的结论（图上有撞桌格）**不进**退出码 —— 扫的范围本来就比任何
   合理的区都大，那是结论不是失败。只有它「什么都没量到」才算失败。
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


# `real.launch.py` 顶部那个常量。**读它，不要抄一个值下来** —— 抄下来的那份
# 迟早和真身分叉，而分叉的后果是「同一张图两套几何」，见 description_package()。
DESCRIPTION_PACKAGE_RE = re.compile(
    r"""^DESCRIPTION_PACKAGE\s*=\s*["']([^"']+)["']""", re.M)


def sh(cmd, timeout=15, **kw):
    """跑一条命令，**带超时**。

    ⚠️ 这个超时不是装饰：`ros2` 的 CLI 在某些环境里（WSL2 实测）会**永远挂住**
    —— `ros2 service list` 既不返回也不报错。没有超时的话，本脚本会卡在
    `wait_for_services` 之外的那些调用上，连 `finally` 的收尾都跑不到。
    """
    try:
        return subprocess.run(cmd, shell=True, capture_output=True, text=True,
                              timeout=timeout, **kw)
    except subprocess.TimeoutExpired:
        print(f"  ! `{cmd}` {timeout}s 没返回（环境问题？当没就绪处理）")
        return subprocess.CompletedProcess(cmd, returncode=124, stdout="",
                                           stderr="timeout")


def description_package():
    """从 `real.launch.py` 读出它实际用的 `DESCRIPTION_PACKAGE`。

    读失败就返回 None（调用方退 2）——**不要**在这里兜底成 `ur_description`：
    静默退回标称正是这次要修掉的那个 bug。
    """
    prefix = sh("ros2 pkg prefix yolo_bringup", timeout=15).stdout.strip()
    if not prefix:
        return None
    launch = os.path.join(prefix, "share", "yolo_bringup", "launch",
                          "real.launch.py")
    try:
        with open(launch, encoding="utf-8") as f:
            m = DESCRIPTION_PACKAGE_RE.search(f.read())
    except OSError:
        return None
    return m.group(1) if m else None


def read_description_hashes(timeout=25.0):
    """读 `/robot_state_publisher` 与 `move_group` 各自的 `robot_description`，
    返回 `{节点名: calib_* 或 None}`。

    **只调 `get_parameters` 服务，不走 `ros2` CLI** —— 理由同
    `wait_for_services()`：CLI 会挂，而挂住和「真的没有」用 CLI 分不开。
    """
    import rclpy
    from rcl_interfaces.srv import GetParameters
    from rclpy.node import Node

    own = not rclpy.ok()
    if own:
        rclpy.init()
    node = Node("moveit_only_check_desc")
    out, targets = {}, []
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(node, timeout_sec=0.2)
            targets = [n for n in node.get_node_names()
                       if n in ("robot_state_publisher", "move_group",
                                "move_group_node")]
            if len(targets) >= 2:
                break
            time.sleep(0.3)
        for name in targets:
            cli = node.create_client(GetParameters, f"/{name}/get_parameters")
            if not cli.wait_for_service(timeout_sec=5.0):
                out[name] = None
                continue
            req = GetParameters.Request()
            req.names = ["robot_description"]
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
            try:
                m = re.search(r"calib_\d+", fut.result().values[0].string_value)
            except Exception:
                m = None
            out[name] = m.group(0) if m else None
    finally:
        node.destroy_node()
        if own and rclpy.ok():
            rclpy.shutdown()
    return out


def wait_for_services(needed, timeout=180.0):
    """等一组服务出现在 ROS 图里，返回 (是否齐了, 用时秒)。

    **不走 `ros2` CLI，直接问 rclpy 的图发现。** 原因是实测出来的：
    `ros2 service list` 在 WSL2 上**永远挂住**（既不返回也不报错）。原来第 4 步
    每轮 `sleep 5` 再跑一次 CLI、超时 15 s ⇒ **一轮 20 s**，跑满 36 轮要
    **12 分钟**才吐出一句「/compute_ik 没起来」—— 而那是**假结论**：同一时刻
    move_group 已经在打 `You can start planning now!`。挂住和「服务真的不在」
    是两件事，用 CLI 分不开，用 rclpy 就分得开，而且板子上行为一致。
    """
    import rclpy
    from rclpy.node import Node

    own = not rclpy.ok()
    if own:
        rclpy.init()
    node = Node("moveit_only_check_ready")
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(node, timeout_sec=0.2)
            names = {n for n, _ in node.get_service_names_and_types()}
            if all(s in names for s in needed):
                return True, time.monotonic() - t0
            time.sleep(0.3)
        return False, time.monotonic() - t0
    finally:
        node.destroy_node()
        if own and rclpy.ok():
            rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description="无臂 MoveIt 预检（零运动风险）")
    ap.add_argument("--grid", type=int, default=4,
                    help="肘探针的采样密度（传给 elbow_guard_probe.py）")
    ap.add_argument("--ur-type", default="ur5")
    ap.add_argument("--log", default="/tmp/moveit_only_check.log")
    ap.add_argument("--no-pipeline", action="store_true",
                    help="不 include pipeline.launch.py（开发机没装 "
                         "realsense2_camera 时必须；见模块 docstring）")
    ap.add_argument("--envelope-z", type=float, default=None,
                    help="**跑桌面安全包线**：给工作台台面在 base_frame 的 z"
                         "（真机 ≈ 0.0）。不给就不跑。"
                         "它扫一整片桌面、算出「哪些位置会把大臂压到台面下」的"
                         "边界，用来定 pick_zone / place_zone（见 "
                         "scripts/table_safety_envelope.py）。")
    ap.add_argument("--envelope-step", type=float, default=0.05,
                    help="包线的网格步长（m，默认 0.05）")
    ap.add_argument("--envelope-bounds", default="0.15,0.95,-0.50,0.50",
                    help="包线的 x_min,x_max,y_min,y_max（默认 0.15,0.95,-0.50,0.50）")
    ap.add_argument("--envelope-levels", default="grasp,place",
                    help="包线扫哪些高度（grasp / place / grasp,place）")
    ap.add_argument("--envelope-retry-seed", default=None,
                    help="包线的重试种子（home 或 6 个数）：把「够不到」与"
                         "「种子太偏」分开。不给就不重试。")
    ap.add_argument("--envelope-json", default=None,
                    help="把包线的网格转储到这个路径（留证用）")
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
              "tf_required:=false gripper_backend:=none carry_objects:=false"
              + (" start_pipeline:=false" if args.no_pipeline else ""),
              args.log)

        # 2) robot_state_publisher
        #    ⚠️ 描述包**必须与 real.launch.py 同源**（下面 move_group 就是它起的），
        #    否则同一张图里两套几何：IK/FK 走一套、TF 走另一套。见模块 docstring。
        desc_pkg = description_package()
        if not desc_pkg:
            print("读不到 real.launch.py 的 DESCRIPTION_PACKAGE（yolo_bringup "
                  "装了吗？）—— 拒绝对一份不知道从哪来的几何做判读，退 2")
            return 2
        prefix = sh(f"ros2 pkg prefix {desc_pkg}", timeout=15).stdout.strip()
        if not prefix:
            print(f"描述包 {desc_pkg} 找不到，退 2")
            return 2
        share = os.path.join(prefix, "share", desc_pkg)
        kin = os.path.join(share, "config", args.ur_type,
                           "default_kinematics.yaml")
        # `kinematics_params` 要**显式传**：`ur.urdf.xacro` 那个参数默认指向
        # `$(find ur_description)/config/…`，不传就悄悄退回标称 —— 而本包存在的
        # 全部意义就是把这一项换掉。
        xr = sh(f"xacro {share}/urdf/ur.urdf.xacro ur_type:={args.ur_type} "
                f"name:=ur safety_limits:=true kinematics_params:={kin} "
                "> /tmp/ur5_check.urdf 2>/tmp/ur5_check.err")
        if xr.returncode != 0:
            print("xacro 失败：", xr.stderr or open("/tmp/ur5_check.err").read())
            return 2
        try:
            with open("/tmp/ur5_check.urdf", encoding="utf-8") as f:
                used = re.search(r"calib_\d+", f.read())
        except OSError:
            used = None
        print(f"描述包 = {desc_pkg}"
              + (f"，kinematics/hash = {used.group(0)}" if used
                 else "，⚠️ 生成物里没找到 calib_* hash"))
        spawn("rsp",
              'ros2 run robot_state_publisher robot_state_publisher --ros-args '
              '-p "robot_description:=$(cat /tmp/ur5_check.urdf)"',
              "/tmp/moveit_only_check_rsp.log")

        # 3) 假 /joint_states（HOME 定频；move_group 靠它认为状态有效）
        fake_js = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "fake_joint_states.py")
        spawn("fake_js", f"python3 {fake_js}", "/tmp/moveit_only_check_js.log")

        # 4) 等 /compute_ik（rclpy 图发现，不走 CLI —— 见 wait_for_services）
        ready, waited = wait_for_services(["/compute_ik"], timeout=180.0)
        if not ready:
            print(f"{waited:.0f} 秒内 /compute_ik 没起来 —— 日志：", args.log)
            return 2
        print(f"/compute_ik 就绪（约 {waited:.0f} 秒）")

        # 4b) 复核「两套几何」真的没有了。这是本轮建立的不变量，也最容易悄悄回归：
        #     RSP 与 move_group 各自持有一份 robot_description，只要描述包一分叉，
        #     探针量到的姿态与 TF 树就不是同一个机器人（第八十二→八十三轮的真实教训）。
        #     读不全（参数服务没起来 / 版本没声明这个参数）**只 SKIP，不判失败** ——
        #     这一段的判据是下面的探针，这里是为它留证。
        hashes = read_description_hashes()
        print("描述一致性复核：")
        for name, h in hashes.items():
            print(f"  {name:22s} {h}")
        vals = [h for h in hashes.values() if h]
        if len(vals) >= 2 and len(set(vals)) == 1:
            print("  [PASS] 两边同一个 hash —— 同一套几何")
        elif len(vals) >= 2:
            print("  [FAIL] 两边 hash 不同 —— 图里有两套几何，下面的数不能用来判读")
            return 2
        else:
            print("  [SKIP] 没读全，不据此判失败")

        # 5) 肘高探针（只读）
        #    ⚠️ `--no-pipeline` 下**必须**显式给规划组名：没有 /grasp_planner 可读
        #    参数，探针会退回它自己的兜底默认 `ur5_manipulator`，而这条链路的
        #    move_group 是 ur_moveit_config 起的（组名 `ur_manipulator`）—— 用错
        #    的结果是 **44/44 INVALID_GROUP_NAME**，看起来像「全部不可达」（实测，
        #    2026-10-02）。不加开关时探针读运行中节点的参数，那是权威来源。
        here = os.path.dirname(os.path.abspath(__file__))
        probe_cmd = [sys.executable, os.path.join(here, "elbow_guard_probe.py"),
                     "--node", "/grasp_planner", "--grid", str(args.grid)]
        if args.no_pipeline:
            probe_cmd += ["--ik-group", "ur_manipulator"]
        rc = subprocess.call(probe_cmd)

        # 5b) 桌面安全包线（可选，只读）
        #     上一步量的是「我打算去的那些点安全吗」；这一步量的是「哪片桌面可以
        #     选作区」—— 它扫一整片，用**同一份几何**（同一个 move_group、
        #     同一套描述包，见上面的描述一致性复核）把撞桌边界画出来，现场照着
        #     划 pick_zone / place_zone，而不是先框一个再跑一遍看。
        #     ⚠️ 台面高度走**命令行**（真机 ≈ 0），不从 /grasp_planner 读 ——
        #     节点手上那份是仿真 yaml 的 -0.39（`table_plane_z` 也没有 launch 覆盖口子）。
        rc_env = 0
        if args.envelope_z is not None:
            b = [float(v) for v in args.envelope_bounds.split(",")]
            if len(b) != 4:
                print(f"!! --envelope-bounds 要 4 个数，收到 {args.envelope_bounds!r}")
                return 2
            env_cmd = [sys.executable,
                       os.path.join(here, "table_safety_envelope.py"),
                       "--node", "/grasp_planner",
                       "--table-z", str(args.envelope_z),
                       "--step", str(args.envelope_step),
                       "--levels", args.envelope_levels,
                       "--x-min", str(b[0]), "--x-max", str(b[1]),
                       "--y-min", str(b[2]), "--y-max", str(b[3])]
            if args.no_pipeline:
                env_cmd += ["--ik-group", "ur_manipulator"]
            if args.envelope_retry_seed:
                env_cmd += ["--retry-seed", args.envelope_retry_seed]
            if args.envelope_json:
                env_cmd += ["--json", args.envelope_json]
            print("\n>>> 5b) 桌面安全包线（台面 z = %+.4f）" % args.envelope_z)
            rc_env = subprocess.call(env_cmd)
            # 包线自己的退出码**只有 0/2**：图上出现撞桌格是结论不是失败（扫的范围
            # 本来就比任何合理的区大），只有「一个位姿都没解出来」才是环境问题 ——
            # 而那种「没有撞桌格」是假绿，跟探针的 return 2 同一个理由，所以并进来。
            if rc_env == 2:
                print("⚠️ 包线没能给出结论（环境问题）—— 预检按失败计。")

        # 6) 整段护栏接到真 FK 上验一遍（B）
        #    上一步量的是**路点**，这一步量的是**路点之间**：在真 UR5 的 URDF 上
        #    找一条「两端都在台面上、中段却俯冲下去」的真实路径，再看护栏拦不拦。
        #    只调 /compute_fk，**不发轨迹**；而且本脚本没有驱动、没有
        #    controller_manager，结构上不可能动。
        rc2 = subprocess.call(
            [sys.executable, os.path.join(here, "test_guard_path.py"), "--only", "D"])
        return rc or rc2 or rc_env
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

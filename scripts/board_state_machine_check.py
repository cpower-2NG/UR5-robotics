#!/usr/bin/env python3
"""状态机预检：**没有机械臂、没有 Gazebo**，把 `planner_node` 的整条状态机跑一遍。

**在板子上跑**（开发机也可以，只要装了 `ur_description` / `ur_moveit_config`）。

为什么需要它（它补的是哪一段）
------------------------------
`moveit_only_check.py` 验到「链路通 + IK 解得出 + 肘/大臂有余量」就停了 ——
它**故意不发任何轨迹**。于是第三十轮把「运动」记为未覆盖，理由是
「没有驱动就没有控制器，规划器的状态机走不动，那一步要么真臂要么 Gazebo」。

其实还有第三条路：规划器要的**不是 Gazebo，也不是真控制器**，它只是要
（甲）一个会应答的 `FollowJointTrajectory` action，和（乙）一份会动的
`/joint_states`。这两样都用桩造出来，状态机就能真跑 —— 于是
**决策链、IK、肘高护栏、槽位记账、终止与收工路径、遥测**这一段，
在没有任何硬件的情况下也能验。

⚠️ **零运动保证**：本脚本起的 `real.launch.py` 是 `start_ur_driver:=false`
   —— 没有驱动就没有 `controller_manager`，**没有任何东西能执行轨迹**；
   轨迹的「应答」是本进程里的一个桩。不打开任何串口、不发 RTDE、不碰急停。
   再强调一次：那个 action server 是**假的**，它只是把 goal 回掉。

它起什么
--------
  1. `real.launch.py start_ur_driver:=false publish_camera_tf:=false
     tf_required:=false` —— 真机入口，去掉驱动（命令与
     `moveit_only_check.py` 一致，复用同一个入口而不是另起一份）
  2. **本进程内的假臂**：`FollowJointTrajectory` action server（一律回
     `SUCCEEDED` + `error_code=0`）+ 会跟着轨迹走的 `/joint_states`
     + 按 10 Hz 注入一个落在 `pick_zone` 里的**合成目标**
  3. `robot_state_publisher` —— 用 `ur_description` 的 URDF 撑起 TF 树
  4. `planner_node` —— **由 `real.launch.py` 自己起**（别再自己 `ros2 run` 一个：
     那样它拿不到 pipeline 对 `ik_group` 的覆盖，会退回硬编码的
     `ur5_manipulator`，而真机 move_group 的组名是 `ur_manipulator`，
     于是每次 IK 都返回 −15。本脚本第一版就踩了这个，见 `main()` 里的注释）
  5. 订阅 `/grasp_planner/telemetry`，把状态时间线打出来并**判它走完了一整轮**

两个**顺序/名字**上的硬要求（各踩过一次，都写进 `main()` 的注释了）
--------------------------------------------------------------
  · **action 名必须对**：`real.launch.py` 走 `camera_source:=realsense`，
    `pipeline.launch.py:92` 于是把 `trajectory_action` 覆盖成
    **`/scaled_joint_trajectory_controller/follow_joint_trajectory`**（不是仿真的
    `/joint_trajectory_controller/...`）。听错名字的症状是规划器连报
    「轨迹 action 服务不可用（控制器起了吗？）」—— **每条轨迹都失败**，
    看起来像控制器坏了。可用 `--action` 覆盖。
  · **假臂要先起**：规划器 INIT 阶段会「张开夹爪 → 回 HOME」，那里只等这个
    action 约 5 秒。等 `/compute_ik` 就绪（≈5 s）再起假臂**已经晚了** ——
    所以在 `main()` 里假臂排在 launch 之后、`robot_state_publisher` 之前。

判据（退出码 0 的必要条件）
--------------------------
  · 状态序列里出现 APPROACH → DESCEND → LIFT → HOME_WITH_OBJECT
    → PLACE_APPROACH → PLACE_DESCEND → PLACE_RETREAT
  · 假臂至少应答了 7 条轨迹
  · 遥测里 `slots_used` 至少到 1（放置记账真的发生了）
  · 等到 `FINISHED` 且遥测级别为 OK（收工路径也走到了）

用法
----
    python3 scripts/board_state_machine_check.py
    python3 scripts/board_state_machine_check.py --timeout 240

退出码：0 = 状态机走完一整轮；1 = 没走完（含超时）；2 = 环境不满足。
"""

import argparse
import os
import re
import signal
import subprocess
import sys
import threading
import time

# 残留守卫：⚠️ 必须排除 `bash -c` 与我自己的行 —— 经 ssh 跑的时候，调用者那条
# 命令行里就带着同样的模式，会把**自己**匹配上（同款陷阱见
# scripts/moveit_only_check.py 与 scripts/board_ctrlc_check.py）。
LEFTOVER_RE = re.compile(
    r"ros2 launch yolo_bringup|move_group|robot_state_publisher"
    r"|detector_node|projector_node|planner_node|realsense2_camera_node"
    r"|fake_joint_states")

# 走完一整轮抓→放应有的状态次序（遥测里的 state 字段）
CYCLE = ["APPROACH", "DESCEND", "LIFT", "HOME_WITH_OBJECT",
         "PLACE_APPROACH", "PLACE_DESCEND", "PLACE_RETREAT"]

ARM = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
HOME = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
# 落在 grasp_planner.yaml 默认 pick_zone（x[0.30,0.90] y[-0.18,0.16] z[-0.45,-0.30]）里
TARGET_XYZ = (0.50, 0.00, -0.35)
# 与 pipeline.launch.py:92 的 REAL_TRAJECTORY_ACTION 同值。本脚本跑的是
# real.launch.py（camera_source:=realsense），所以是 scaled_ 那个。跑完会拿
# 规划器**在线的那个参数**回头核对一次，不靠这条常量自证。
REAL_ACTION = "/scaled_joint_trajectory_controller/follow_joint_trajectory"


def leftovers():
    r = subprocess.run(["ps", "-eo", "pid,args", "--no-headers"],
                       capture_output=True, text=True)
    out = []
    for line in r.stdout.splitlines():
        if "bash -c" in line or "board_state_machine_check" in line:
            continue
        if LEFTOVER_RE.search(line):
            out.append(line.strip())
    return out


def sh(cmd):
    return subprocess.run(cmd, shell=True, capture_output=True, text=True)


# ----------------------------------------------------------------------
# 假臂：action server + /joint_states + 合成目标注入
# ----------------------------------------------------------------------
def make_mock_arm(rclpy):
    from control_msgs.action import FollowJointTrajectory
    from diagnostic_msgs.msg import DiagnosticArray
    from rclpy.action import ActionServer
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from vision_msgs.msg import (Detection3D, Detection3DArray,
                                 ObjectHypothesisWithPose)

    class MockArm(Node):
        """把规划器要的两件事凑出来，并记录遥测状态时间线。"""

        def __init__(self, action_name):
            super().__init__("board_state_machine_mock_arm")
            self._q = list(HOME)
            self._lock = threading.Lock()
            self.n_goals = 0
            self.states = []          # [(state, {kv})]
            self.finished = False
            self.finish_ok = None

            self._pub_js = self.create_publisher(JointState, "/joint_states", 10)
            self._pub_target = self.create_publisher(
                Detection3DArray, "/targets_3d", 10)
            self.create_timer(0.05, self._publish_js)
            self.create_timer(0.1, self._publish_target)
            self.create_subscription(DiagnosticArray, "/grasp_planner/telemetry",
                                     self._on_telem, 10)
            self._srv = ActionServer(self, FollowJointTrajectory, action_name,
                                     self._execute,
                                     callback_group=ReentrantCallbackGroup())

        def _publish_js(self):
            m = JointState()
            m.header.stamp = self.get_clock().now().to_msg()
            with self._lock:
                m.name = list(ARM)
                m.position = [float(v) for v in self._q]
            self._pub_js.publish(m)

        def _publish_target(self):
            msg = Detection3DArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            d = Detection3D()
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = "block"
            hyp.hypothesis.score = 0.9
            # ⚠️ 规划器读的是 **results[0].pose.pose.position**（见
            #    _select_and_approach），**不是** bbox.center —— 只填 bbox 的话
            #    它看到的是 (0,0,0)，会被 pick_zone 滤掉，症状是「检测明明进来了
            #    但永远没有候选」。两处都填，与 projector_node 的输出一致。
            hyp.pose.pose.position.x = TARGET_XYZ[0]
            hyp.pose.pose.position.y = TARGET_XYZ[1]
            hyp.pose.pose.position.z = TARGET_XYZ[2]
            hyp.pose.pose.orientation.w = 1.0
            d.results.append(hyp)
            d.bbox.center.position.x = TARGET_XYZ[0]
            d.bbox.center.position.y = TARGET_XYZ[1]
            d.bbox.center.position.z = TARGET_XYZ[2]
            d.bbox.center.orientation.w = 1.0
            d.bbox.size.x = d.bbox.size.y = d.bbox.size.z = 0.04
            msg.detections.append(d)
            self._pub_target.publish(msg)

        def _on_telem(self, arr):
            for st in arr.status:
                kv = {k.key: k.value for k in st.values}
                state = kv.get("state", "?")
                if not self.states or self.states[-1][0] != state:
                    self.states.append((state, kv))
                    # ⚠️ rclpy 把 DiagnosticStatus.level（ROS 的 byte）给成
                    #    **bytes**，不是 int —— 直接拿去格式化会 TypeError，
                    #    而它是在执行器线程里抛的，会**静默打死整个节点**
                    #    （本脚本第一版就栽在这里：目标注入随之停摆，症状是
                    #    「检测一个都没到」）。
                    raw = st.level
                    lvl = {0: "OK", 1: "WARN", 2: "ERROR"}.get(
                        raw if isinstance(raw, int) else raw[0], raw)
                    print(f"    [遥测] {state:<18} 级别={lvl} "
                          f"used={kv.get('slots_used')} skipped={kv.get('slots_skipped')}"
                          f" place_index={kv.get('place_index')}"
                          + (f"  原因={kv.get('finish_reason')}"
                             if kv.get("finish_reason") else ""), flush=True)
                    if state == "FINISHED":
                        self.finished = True
                        self.finish_ok = lvl == "OK"

        def _execute(self, goal_handle):
            self.n_goals += 1
            traj = goal_handle.request.trajectory
            if traj.points:
                last = traj.points[-1]
                with self._lock:
                    for n, p in zip(traj.joint_names, last.positions):
                        if n in ARM:
                            self._q[ARM.index(n)] = float(p)
            time.sleep(0.05)          # 桩：不真等一个轨迹时长
            res = FollowJointTrajectory.Result()
            res.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            goal_handle.succeed()
            print(f"    [假臂] 第 {self.n_goals} 条轨迹到位", flush=True)
            return res

    return MockArm


def main():
    ap = argparse.ArgumentParser(description="状态机预检（零运动风险）")
    ap.add_argument("--timeout", type=float, default=180.0,
                    help="等状态机走完的秒数（默认 180）")
    ap.add_argument("--action", default=REAL_ACTION,
                    help=f"假臂应答的 FollowJointTrajectory action 名（默认 {REAL_ACTION}）")
    ap.add_argument("--finish-timeout", type=float, default=60.0,
                    help="走完一轮后等 FINISHED 的秒数（默认 60；规划器的空闲"
                         "看门狗 idle_finish_timeout 默认 20 s）")
    ap.add_argument("--log", default="/tmp/board_state_machine_check.log")
    ap.add_argument("--keep", action="store_true", help="跑完不清场（排障用）")
    args = ap.parse_args()

    if not os.environ.get("ROS_DISTRO"):
        print("环境未就绪：先 source /opt/ros/humble/setup.bash 和 "
              "ros2_ws/install/setup.bash（约定：环境不满足退 2）")
        return 2

    stale = leftovers()
    if stale:
        print("板上有残留进程，先清场再测（否则 TF/服务/话题归属会乱）：")
        for s in stale:
            print("  " + s)
        return 2

    procs = []

    def spawn(name, cmd, log):
        procs.append((name, subprocess.Popen(
            cmd, shell=True, stdout=open(log, "wb"),
            stderr=subprocess.STDOUT, start_new_session=True)))

    ok = False
    mock = None
    ex = None
    rc = 2                     # 默认「环境不满足」；走到判据那步再改
    try:
        # 1) 真机入口，去掉驱动（与 moveit_only_check.py 同一条命令）
        spawn("launch",
              "ros2 launch yolo_bringup real.launch.py "
              "start_ur_driver:=false publish_camera_tf:=false "
              "tf_required:=false gripper_backend:=none carry_objects:=false",
              args.log)

        # 2) 本进程内的假臂 —— ⚠️ **必须排在 planner 的 INIT 前面**。
        #    规划器启动后很快就要「回 HOME」，那里只等 action 服务约 5 秒；
        #    若放到「等 /compute_ik 就绪」之后（≈5 s）再起，就已经错过，
        #    规划器会一路失败到 FINISHED。第一版就是按「先等 IK 再起假臂」
        #    排的，见 docs/RESTRUCTURE_PLAN.md 第三十三轮。
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        rclpy.init()
        mock = make_mock_arm(rclpy)(args.action)
        ex = MultiThreadedExecutor()
        ex.add_node(mock)
        threading.Thread(target=ex.spin, daemon=True).start()
        print(f"假臂已起：action={args.action}", flush=True)

        # 3) robot_state_publisher（ur_description 的 URDF 撑 TF 树）
        xr = sh("xacro $(ros2 pkg prefix ur_description)/share/ur_description"
                "/urdf/ur.urdf.xacro ur_type:=ur5 name:=ur safety_limits:=false"
                " > /tmp/bsmc_ur5.urdf")
        if xr.returncode != 0:
            print("xacro 失败 —— ur_description 没装齐？退 2\n", xr.stderr)
            return 2
        spawn("rsp",
              'ros2 run robot_state_publisher robot_state_publisher --ros-args '
              '-p "robot_description:=$(cat /tmp/bsmc_ur5.urdf)"',
              "/tmp/bsmc_rsp.log")

        # 4) 等 move_group 的 /compute_ik
        ready = False
        for i in range(36):
            time.sleep(5)
            if "/compute_ik" in sh("ros2 service list").stdout:
                print(f"/compute_ik 就绪（约 {(i + 1) * 5} 秒）", flush=True)
                ready = True
                break
        if not ready:
            print("180 秒内 /compute_ik 没起来 —— move_group 日志：", args.log)
            return 2

        # 5) ⚠️ **不要**自己再起一个 planner_node —— `real.launch.py` 已经起了
        #    一个（走 pipeline.launch.py，`ik_group` 被正确覆盖成真机的
        #    `ur_manipulator`）。另外用 `ros2 run` 起一个的话，它**拿不到那份
        #    覆盖**，会退回代码里的硬编码默认值 `ur5_manipulator` —— 而真机
        #    move_group 的 SRDF 组名是 `ur_manipulator`，于是**每一次 IK 都
        #    返回 -15（NO_IK_SOLUTION）**，状态机卡在「接近位姿 IK 失败」上空转
        #    10 次然后判「卡住」。本脚本第一版就是这么错的（实测症状见
        #    docs/RESTRUCTURE_PLAN.md 第三十三轮）。
        #    等 launch 自己那个规划器就绪即可 —— 它的启动日志有一行「已就绪」。
        print("等 real.launch 里的 planner_node 就绪……", flush=True)
        p_ready = False
        for _ in range(60):
            time.sleep(2)
            try:
                if "已就绪" in open(args.log, errors="ignore").read():
                    p_ready = True
                    break
            except OSError:
                pass
        if not p_ready:
            print("规划器没就绪 —— 日志：", args.log)
            return 2
        print("规划器已就绪", flush=True)

        # 5b) 拿**在线参数**回头核对 action 名（不靠 REAL_ACTION 自证）。此刻
        #     规划器已过 INIT，对不上就说明假臂听错了名字 —— 那正是第一版的
        #     症状（连报「轨迹 action 服务不可用」），所以这里要报出来。
        live = sh("ros2 param get /grasp_planner trajectory_action").stdout
        live = live.split("is:", 1)[1].strip() if "is:" in live else ""
        print(f"规划器在线的 trajectory_action = {live or '（取不到）'}")
        if live and live != args.action:
            print(f"  ✗ 假臂听的是 {args.action}，两边不一致 —— 轨迹必然失败")
            return 2

        # 6) 等一整轮
        deadline = time.time() + args.timeout
        cycle_done = False
        while time.time() < deadline:
            time.sleep(1)
            seen = [s for s, _ in mock.states]
            if all(c in seen for c in CYCLE) and mock.n_goals >= len(CYCLE):
                cycle_done = True
                break
        if cycle_done:
            print(f"抓→放一整轮走完（{mock.n_goals} 条轨迹），等它自己收工……",
                  flush=True)

        # 6b) 等收工（FINISHED）。规划器走的是**空闲看门狗**（`idle_finish_timeout`
        #     默认 20 s）：放好的方块因 pick/place 分区而不再是候选，于是不再发起
        #     尝试 ⇒ 到点收工。⚠️ 合成目标仍在 10 Hz 注入**不影响**它 —— 看门狗的
        #     计时起点是「上一次**发起尝试**」，不是「上一次**收到检测**」
        #     （planner_node 的 `_idle_since` 注释专门写了这一条）。
        fin_deadline = time.time() + args.finish_timeout
        while time.time() < fin_deadline and not mock.finished:
            time.sleep(1)

        seen = [s for s, _ in mock.states]
        last = mock.states[-1][1] if mock.states else {}
        print("\n===== 结果 =====")
        print(f"  状态序列：{' → '.join(seen) if seen else '（一个都没收到）'}")
        print(f"  假臂应答轨迹：{mock.n_goals} 条")
        print(f"  槽位记账：used={last.get('slots_used')} "
              f"skipped={last.get('slots_skipped')} "
              f"place_index={last.get('place_index')}")
        missing = [c for c in CYCLE if c not in seen]
        ok = cycle_done
        if missing:
            print(f"  ✗ 没走到的状态：{missing}")
        if ok and str(last.get("slots_used", "0")) == "0":
            print("  ✗ 状态走完了但 slots_used 还是 0 —— 放置记账没发生")
            ok = False
        # 收工路径也算判据的一部分：它覆盖的是「不再接受目标 + 停机」那一段。
        if not mock.finished:
            print("  ✗ 没等到 FINISHED（收工路径没走到）—— 看门狗没到点或卡住了")
            ok = False
        elif mock.finish_ok is not True:
            print("  ✗ 收工了，但遥测级别是 ERROR（不是正常收工）")
            ok = False
        if ok:
            print("  ✓ 状态机走完了一整轮抓→放，并按看门狗正常收工")
            print("    （零运动：轨迹全部由桩应答）")
        else:
            print(f"  **失败** —— 规划器日志：{args.log}")

        ex.shutdown()
        mock.destroy_node()
        rclpy.shutdown()
        ex = mock = None
        rc = 0 if ok else 1
    finally:
        # ⚠️ 这里**不要** `return` —— finally 里的 return 会吞掉异常，
        #    排障时最需要的那条 traceback 就没了。结果用 rc 带出去。
        # 上面的早退路径（`return 2`）会跳过那段 shutdown，所以在这里兜一次。
        if mock is not None or ex is not None:
            try:
                if ex is not None:
                    ex.shutdown()
                if mock is not None:
                    mock.destroy_node()
                import rclpy as _rclpy
                if _rclpy.ok():
                    _rclpy.shutdown()
            except Exception:
                pass
        if args.keep:
            print("（--keep：不清场）")
            return rc
        for name, p in reversed(procs):
            try:
                os.killpg(p.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        # ⚠️ 别用固定短宽限：`ros2 launch` 收到 TERM 后要**按顺序**收掉每个子
        #    节点，宽限太短会把它自己 SIGKILL 掉、留下孤儿（moveit_only_check.py
        #    记过同款）。判据用 leftovers() 本身。
        deadline = time.time() + 15
        while time.time() < deadline and leftovers():
            time.sleep(1)
        for line in leftovers():
            try:
                os.kill(int(line.split()[0]), signal.SIGKILL)
            except (ValueError, ProcessLookupError):
                pass
        for name, p in reversed(procs):
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass
        stale = leftovers()
        if stale:
            print("⚠️ 收尾后仍有残留（下次开跑会被守卫拦下）：")
            for s in stale:
                print("  " + s)
    return rc


if __name__ == "__main__":
    sys.exit(main())

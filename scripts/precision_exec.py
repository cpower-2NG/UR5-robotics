#!/usr/bin/env python3
"""执行计划 JSON v2：**每条轨迹前查闸门**，夹爪动作与臂动作分开记日志。

与 ``run_grasp.py`` 的关系
-------------------------
闸门、两点式 JTC、结果同时查 status 与 error_code —— 全部沿用，一字没改。
新增的只有两件事：

1. **夹爪动作**（``before`` / ``after``）穿插在航点之间，用
   ``grasp_planner.gripper.Ag95Gripper``（公开接口 open/close/current_position/
   is_empty）。写寄存器前它会自己查 ``0x0200 == 1``（初始化完成），没初始化就拒绝。
2. **JSONL 日志**：每个航点、每次夹爪动作、每次闸门都落一行，验收报告直接读它。
   日志与计划放同一个目录（仓库外），不污染仓库。

默认**预演**：不加 --go 一条轨迹都不发。

闸门（任一不过就原地停，不发下一条）
------------------------------------
  · robot_mode == 7 (RUNNING)、safety_mode == 1 (NORMAL)
  · 示教器程序 programState 以 PLAYING 开头
    —— STOPPED 时 external control 的 URScript 不读 servoJ，JTC 照收目标但臂不动
  · scaled_joint_trajectory_controller == active
    —— controller_stopper 在程序停止时会把它停掉；action server 存在 ≠ 已激活
  · 当前关节角与**上一条航点**最大差 < 0.05 rad（别人动过臂 ⇒ 拒绝）
  · 结果 status == SUCCEEDED **且** error_code == 0

用法：
  python3 precision_exec.py --plan pick.json --list                 # 只打印
  python3 precision_exec.py --plan pick.json --check                # 只查闸门
  python3 precision_exec.py --plan pick.json --go --upto lift       # 真动
"""
import argparse
import json
import math
import os
import sys
import time

sys.dont_write_bytecode = True

ARM = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
TRAJ_ACTION = "/scaled_joint_trajectory_controller/follow_joint_trajectory"
ROBOT_MODE_RUNNING = 7
SAFETY_MODE_NORMAL = 1
JOINT_DRIFT_RAD = 0.05
AG95_EMPTY_TOL_MM = 5.0


def log_line(path, **rec):
    rec["t"] = round(time.time(), 3)
    with open(path, "a") as f:
        f.write(json.dumps(rec, ensure_ascii=False) + "\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--plan", required=True)
    ap.add_argument("--go", action="store_true", help="**真的发轨迹**")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--check", action="store_true", help="只查闸门，不发轨迹")
    ap.add_argument("--only", default=None, help="只跑这一条航点")
    ap.add_argument("--upto", default=None, help="跑到这一条为止（含）")
    ap.add_argument("--settle", type=float, default=1.5, help="每条轨迹完成后再等几秒")
    ap.add_argument("--log", default=None,
                    help="JSONL 日志路径（默认：计划同目录、同名的 .jsonl）")
    ap.add_argument("--gripper-port",
                    default="/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AB0P6UEX-if00-port0")
    ap.add_argument("--gripper-slave", type=int, default=1)
    ap.add_argument("--gripper-force", type=int, default=50)
    ap.add_argument("--dry-run-gripper", action="store_true",
                    help="夹爪只读（一个寄存器都不写）——链路自检用")
    ap.add_argument("--grasp-planner-src", default=None,
                    help="grasp_planner 源码目录（没 source ROS 环境时才需要）")
    args = ap.parse_args()

    plan = json.load(open(args.plan))
    names = [w["name"] for w in plan["waypoints"]]
    if args.only:
        if args.only not in names:
            print("!! 计划里没有航点 %r（有 %s）" % (args.only, names))
            return 2
        order = [args.only]
    elif args.upto:
        if args.upto not in names:
            print("!! 计划里没有航点 %r（有 %s）" % (args.upto, names))
            return 2
        order = names[:names.index(args.upto) + 1]
    else:
        order = names
    wr = {w["name"]: w for w in plan["waypoints"]}
    meta = plan.get("meta", {})
    log_path = args.log or os.path.splitext(args.plan)[0] + ".jsonl"

    print("=" * 78)
    kind = meta.get("kind", "?")
    extra = ""
    if kind == "pick":
        extra = (" 块 #%s %s  抓点 (%.1f, %.1f) mm  槽 %s"
                 % (meta.get("block_idx"), meta.get("class"),
                    *[v * 1000 for v in meta.get("grasp_point", [0, 0])],
                    meta.get("slot")))
    print("计划 %s  kind=%s%s" % (args.plan, kind, extra))
    for n in names:
        w = wr[n]
        act = []
        if w.get("before"):
            act.append("前:" + w["before"]["gripper"])
        if w.get("after"):
            act.append("后:" + w["after"]["gripper"])
        print("  %-15s q(deg) [%s]  %4.1fs  %s"
              % (n, " ".join("%6.1f" % math.degrees(v) for v in w["q"]),
                 w["duration"], " ".join(act)))
    print("本次要跑：%s" % order)
    print("=" * 78)
    if args.list or (not args.go and not args.check):
        print("⇒ 预演：一条轨迹都没发。真动加 --go。")
        return 0

    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool
    from ur_dashboard_msgs.msg import RobotMode, SafetyMode
    from ur_dashboard_msgs.srv import RawRequest
    from controller_manager_msgs.srv import ListControllers
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    from action_msgs.msg import GoalStatus

    rclpy.init()
    node = Node("precision_exec")
    got = {}
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    # 三个状态话题是 TRANSIENT_LOCAL：只在变化时发，靠 latched 补发给后加入者。
    qos_l = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(JointState, "/joint_states",
                             lambda m: got.__setitem__("js", m), qos)
    node.create_subscription(RobotMode, "/io_and_status_controller/robot_mode",
                             lambda m: got.__setitem__("rm", m), qos_l)
    node.create_subscription(SafetyMode, "/io_and_status_controller/safety_mode",
                             lambda m: got.__setitem__("sm", m), qos_l)
    node.create_subscription(Bool, "/io_and_status_controller/robot_program_running",
                             lambda m: got.__setitem__("pr", m), qos_l)
    t0 = time.time()
    # robot_program_running 只在**状态变化时**发，latch 缓存可能是空的 —— 别为它空等
    # 满 20 s（run_grasp.py 就是这么等的，白等）。核心三件到齐就往下走。
    while any(k not in got for k in ("js", "rm", "sm")) and time.time() - t0 < 20:
        rclpy.spin_once(node, timeout_sec=0.2)
    for _ in range(5):                      # 再给 pr 半秒机会，仅作附加信息
        rclpy.spin_once(node, timeout_sec=0.1)
    miss = [k for k in ("js", "rm", "sm") if k not in got]
    if miss:
        print("!! 20 s 没收齐状态 %s —— 驱动起了吗？" % miss)
        return 2

    def joints():
        d = dict(zip(got["js"].name, got["js"].position))
        return [float(d[j]) for j in ARM]

    ctl = node.create_client(ListControllers, "/controller_manager/list_controllers")
    raw = node.create_client(RawRequest, "/dashboard_client/raw_request")

    def controller_state(name):
        if not ctl.wait_for_service(timeout_sec=5.0):
            return None
        f = ctl.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(node, f, timeout_sec=8.0)
        r = f.result()
        if r is None:
            return None
        return next((c.state for c in r.controller if c.name == name), None)

    def program_state():
        if not raw.wait_for_service(timeout_sec=5.0):
            return None
        req = RawRequest.Request()
        req.query = "programState"
        f = raw.call_async(req)
        rclpy.spin_until_future_complete(node, f, timeout_sec=8.0)
        r = f.result()
        return None if r is None else r.answer

    i0 = names.index(order[0])
    expected = list(plan["q_start"] if i0 == 0 else wr[names[i0 - 1]]["q"])
    print("  闸门基准 = %s" % ("计划起点 q_start" if i0 == 0
                              else "上一条航点 %s" % names[i0 - 1]))

    def gate(where):
        cur = joints()
        rm, sm = int(got["rm"].mode), int(got["sm"].mode)
        drift = max(abs(a - b) for a, b in zip(cur, expected))
        cstate = controller_state("scaled_joint_trajectory_controller")
        pstate = program_state()
        playing = bool(pstate) and pstate.strip().upper().startswith("PLAYING")
        ok = (rm == ROBOT_MODE_RUNNING and sm == SAFETY_MODE_NORMAL
              and drift < JOINT_DRIFT_RAD and cstate == "active" and playing)
        print("  [%s] robot_mode=%d safety_mode=%d 程序=%r JTC=%s 最大差 %.2f° → %s"
              % (where, rm, sm, pstate, cstate, math.degrees(drift),
                 "OK" if ok else "**拒绝**"))
        if not playing:
            print("    ↑ 示教器上程序没在跑：external control 的 URScript 不读 servoJ，"
                  "发轨迹也不会动")
        if cstate != "active":
            print("    ↑ JTC 没激活：controller_stopper 在程序停止时把它停掉了")
        log_line(log_path, event="gate", where=where, robot_mode=rm, safety_mode=sm,
                 program=pstate, jtc=cstate, drift_deg=round(math.degrees(drift), 3),
                 ok=bool(ok))
        return ok, cur

    ok, cur = gate("发轨迹前")
    if not ok:
        print("!! 闸门不过，一条轨迹都不发")
        return 2

    ac = ActionClient(node, FollowJointTrajectory, TRAJ_ACTION)
    if args.check:
        print("  JTC action 在" if ac.wait_for_server(timeout_sec=5.0)
              else "  !! JTC action 不在")
        node.destroy_node()
        rclpy.shutdown()
        return 0
    if not ac.wait_for_server(timeout_sec=10.0):
        print("!! %s 不在" % TRAJ_ACTION)
        return 2

    # ---------------------------------------------------------------- 夹爪
    grip = {"obj": None}

    def gripper():
        if grip["obj"] is not None:
            return grip["obj"]
        if args.grasp_planner_src:
            sys.path.insert(0, os.path.abspath(args.grasp_planner_src))
        try:
            from grasp_planner.gripper import Ag95Gripper
        except ImportError as e:
            print("!! 导不进 grasp_planner.gripper（%s）—— source 过 ROS 环境吗？"
                  " 也可以给 --grasp-planner-src" % e)
            raise
        g = Ag95Gripper(node, port=args.gripper_port, baudrate=115200,
                        slave_id=args.gripper_slave, force=args.gripper_force,
                        stroke_mm=95.0, open_mm=95.0, close_mm=0.0,
                        dry_run=bool(args.dry_run_gripper), open_is_high=True)
        print("  夹爪：%s" % g.describe())
        log_line(log_path, event="gripper_init", desc=g.describe(),
                 dry_run=bool(args.dry_run_gripper))
        grip["obj"] = g
        return g

    def wait_stable(g, timeout=8.0, stable=0.6):
        """轮询回读，直到连续 stable 秒不变（或超时）。返回最后读数（毫米）。"""
        t0 = time.time()
        last, last_change = None, time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(node, timeout_sec=0.0)
            p = g.current_position()
            now = time.time()
            if p is not None and (last is None or abs(p - last) > 0.5):
                last_change = now
            if p is not None:
                last = p
            if last is not None and now - last_change >= stable:
                return last
            time.sleep(0.1)
        return last

    def do_gripper(kind, where):
        g = gripper()
        t0 = time.time()
        if kind == "open":
            g.open()
        elif kind == "close":
            g.close()
        elif kind != "readback":
            print("!! 不认识的夹爪动作 %r" % kind)
            return None
        pos = wait_stable(g)
        dt = time.time() - t0
        print("  [%s] 夹爪 %s → 回读 %s mm（%.1f s）%s"
              % (where, kind, "None" if pos is None else "%.1f" % pos, dt,
                 "  ⚠️ 读不到" if pos is None else ""))
        log_line(log_path, event="gripper", where=where, action=kind,
                 position_mm=None if pos is None else round(float(pos), 2),
                 duration_s=round(dt, 2))
        return pos

    # ---------------------------------------------------------------- 轨迹
    def send(target, dur, what):
        goal = FollowJointTrajectory.Goal()
        tj = goal.trajectory
        tj.joint_names = list(ARM)
        now = joints()
        for p, t in ((now, 0.0), (target, dur)):
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in p]
            pt.velocities = [0.0] * len(ARM)
            pt.time_from_start = Duration(sec=int(t),
                                          nanosec=int((t - int(t)) * 1e9))
            tj.points.append(pt)
        print("  发 %s（%.1fs，起点=当前实测）…" % (what, dur))
        fut = ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=10)
        gh = fut.result()
        if gh is None or not gh.accepted:
            print("   !! 目标被拒")
            log_line(log_path, event="traj", waypoint=what, ok=False,
                     error="goal rejected")
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(node, rf, timeout_sec=dur + 30)
        res = rf.result()
        if res is None:
            print("   !! 没等到结果")
            log_line(log_path, event="traj", waypoint=what, ok=False,
                     error="no result")
            return False
        st, ec = int(res.status), int(res.result.error_code)
        ok = (st == GoalStatus.STATUS_SUCCEEDED and ec == 0)
        print("   %s status=%d error_code=%d" % ("OK" if ok else "**失败**", st, ec))
        log_line(log_path, event="traj", waypoint=what, ok=bool(ok),
                 status=st, error_code=ec)
        if not ok:
            return False
        time.sleep(args.settle)
        rclpy.spin_once(node, timeout_sec=0.3)
        fin = joints()
        err = max(abs(a - b) for a, b in zip(fin, target))
        print("   到位误差（实测 vs 目标）最大 %.2f°" % math.degrees(err))
        log_line(log_path, event="arrive", waypoint=what,
                 err_deg=round(math.degrees(err), 3))
        return True

    ok_all = True
    closed_at = None       # 闭合后的回读，抬升后拿它比
    lifted_at = None
    for name in order:
        w = wr[name]
        ok, _ = gate("'%s' 之前" % name)
        if not ok:
            print("!! 闸门不过，停在这里")
            return 2
        if w.get("before"):
            do_gripper(w["before"]["gripper"], name + ".before")
        if w.get("q") is not None:
            if not send(w["q"], w["duration"], name):
                print("!! %s 失败，后续一条都不发" % name)
                return 1
            expected[:] = w["q"]
        if w.get("after"):
            pos = do_gripper(w["after"]["gripper"], name + ".after")
            # 手指被物体挡住 ⇒ 回读停在中间；全闭 ⇒ 空夹。
            if w["after"]["gripper"] == "close":
                closed_at = pos
                if pos is None:
                    print("   ⚠️ 闭合后读不到位置 —— 判不了夹没夹住")
                    ok_all = False
                elif pos <= AG95_EMPTY_TOL_MM:
                    print("   ⚠️ 闭合后回读 %.1f mm ≈ 全闭 —— **空夹**" % pos)
                    ok_all = False
            elif name == "lift":
                lifted_at = pos

    # 抬升后复读：夹住的块如果滑了，回读会往闭合位缩。
    if closed_at is not None and lifted_at is not None:
        d = lifted_at - closed_at
        print("  抬升后回读 %.1f mm（闭合时 %.1f，差 %+.1f mm）→ %s"
              % (lifted_at, closed_at, d,
                 "没滑" if abs(d) <= 1.0 else "**滑了**"))
        log_line(log_path, event="grasp_check", closed_mm=round(closed_at, 2),
                 lifted_mm=round(lifted_at, 2), delta_mm=round(d, 2),
                 slipped=bool(abs(d) > 1.0))
        if abs(d) > 1.0:
            ok_all = False

    node.destroy_node()
    rclpy.shutdown()
    print("全部完成。" if ok_all else "完成，但有**判红**项（见上）。")
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())

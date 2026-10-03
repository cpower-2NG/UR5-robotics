#!/usr/bin/env python3
"""**真机第一次运动**：小幅度、可预期、能中途停。

为什么单独写一个而不是直接跑规划器：第一次动臂要的是「**幅度已知、目标由当前
位姿算出来、失败了看得见**」，而不是「跑一套状态机看看会不会出事」。所以：

  · 目标**从实时的 `/joint_states` 算**（相对量），不用任何写死的位姿 ——
    写死的位姿在「机械臂不在我以为的地方」时会变成一次大摆动；
  · 只动**一个关节**（`shoulder_pan`），幅度默认 **5°**，而且**动完再回原位**；
  · 走 `scaled_joint_trajectory_controller` ⇒ **示教器的速度滑条压着它**
    （这是当初切到 `scaled_` 的唯一理由）；
  · 结果**同时查 `status` 与 `error_code`** —— 只查一个会被「Result 是默认空壳、
    error_code 恰为 0」那条老路骗过（D52）。

夹爪那半走 `0x0101`(力) → `0x0103`(位置)，写前查 `0x0200`（未初始化/进行中一律拒）。

**默认预演，一个寄存器都不写、一条轨迹都不发。** 真动要 `--go`。

    python3 scripts/real_first_motion.py                      # 预演
    python3 scripts/real_first_motion.py --go --arm-deg 5 --gripper-cycle

⚠️ 动手前确认：**急停在手边**、工作区（含臂的整个扫掠范围）无人无物。
"""

import argparse
import os
import sys
import time

sys.dont_write_bytecode = True

ARM_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
              "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
TRAJ_ACTION = "/scaled_joint_trajectory_controller/follow_joint_trajectory"


def plan_arm(cur, joint_index, delta_rad, duration):
    """``(去程点数, 回程点数)`` —— 两点式轨迹，起点的**时间从 0 开始**。

    ⚠️ 起点必须**原样带上当前关节角**并给 ``t=0``：JTC 要求轨迹起点与当前状态
    一致，否则一上来就是一次跳变。这是「相对量」在这里的全部意义。
    """
    fwd = [list(cur), list(cur)]
    fwd[1][joint_index] += delta_rad
    return fwd, duration


def main():
    ap = argparse.ArgumentParser(description="真机第一次运动（默认预演）")
    ap.add_argument("--go", action="store_true", help="**真的动**")
    ap.add_argument("--arm-deg", type=float, default=5.0,
                    help="shoulder_pan 转多少度（默认 5，动完回原位）")
    ap.add_argument("--arm-joint", type=int, default=0,
                    help="动第几个关节（0=shoulder_pan，默认）")
    ap.add_argument("--duration", type=float, default=3.0, help="去程秒数")
    ap.add_argument("--gripper-cycle", action="store_true",
                    help="夹爪做一次「合拢 → 张开」（写寄存器，会动手指）")
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from sensor_msgs.msg import JointState
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    from action_msgs.msg import GoalStatus

    rclpy.init()
    n = Node("real_first_motion")
    got = {}
    n.create_subscription(JointState, "/joint_states",
                          lambda m: got.setdefault("js", m), 10)
    t0 = time.time()
    while "js" not in got and time.time() - t0 < 15:
        rclpy.spin_once(n, timeout_sec=0.2)
    if "js" not in got:
        print("!! 15 s 没收到 /joint_states —— 驱动起了吗？")
        return 2
    d = dict(zip(got["js"].name, got["js"].position))
    missing = [j for j in ARM_JOINTS if j not in d]
    if missing:
        print(f"!! /joint_states 里缺关节：{missing}")
        return 2
    cur = [float(d[j]) for j in ARM_JOINTS]
    idx = args.arm_joint
    delta = args.arm_deg * 3.141592653589793 / 180.0

    print("=" * 72)
    print(f"当前 {ARM_JOINTS[idx]} = {cur[idx]:+.4f} rad（{cur[idx] * 57.2958:+.2f}°）")
    print(f"目标 = {cur[idx] + delta:+.4f} rad（{(cur[idx] + delta) * 57.2958:+.2f}°）"
          f"  ⇒ 变化 {args.arm_deg:+.2f}°，{args.duration:.1f} s，**动完回原位**")
    print(f"轨迹发到 {TRAJ_ACTION}（受示教器速度滑条缩放）")
    print("=" * 72)
    if not args.go:
        print("⇒ 预演：**一条轨迹都没发、一个寄存器都没写**。真动加 `--go`。")
        return 0

    # ---- 夹爪：合拢 → 张开（直接写寄存器，走独立链路）----
    if args.gripper_cycle:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import ag95_init_close as G
        g = G.Gripper.open_port()
        if g is None:
            print("!! 夹爪串口打不开"); return 2
        try:
            st = g.read(G.REG_INIT_STATUS)
            print(f"夹爪 0x0200 = {st}", "（已初始化 ✓）" if st == G.INIT_READY
                  else "—— **没初始化，拒绝动作**")
            if st != G.INIT_READY:
                return 2
            print(f"  写 0x0101 = {G.FORCE_N}（力）")
            g.write(G.REG_FORCE, G.FORCE_N)
            print("  写 0x0103 = 0 → **合拢**")
            if not g.write(G.REG_TARGET, 0):
                print("!! 写失败"); return 2
            pos, dt = g.settle()
            print(f"     {dt:.1f}s 后 0x0202 = {pos}（0 = 全闭）")
            time.sleep(0.6)
            print("  写 0x0103 = 1000 → **张开**")
            if not g.write(G.REG_TARGET, 1000):
                print("!! 写失败"); return 2
            pos, dt = g.settle()
            print(f"     {dt:.1f}s 后 0x0202 = {pos}（1000 = 全开）")
        finally:
            g.close() if hasattr(g, "close") else None

    # ---- 机械臂：去程 + 回程 ----
    ac = ActionClient(n, FollowJointTrajectory, TRAJ_ACTION)
    if not ac.wait_for_server(timeout_sec=10.0):
        print(f"!! {TRAJ_ACTION} 不在 —— 控制器起了吗？")
        return 2

    def send(points, dur, what):
        goal = FollowJointTrajectory.Goal()
        tj = goal.trajectory
        tj.joint_names = list(ARM_JOINTS)
        for p in points:
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in p]
            pt.velocities = [0.0] * len(ARM_JOINTS)
            pt.time_from_start = Duration(sec=int(dur), nanosec=0)
            tj.points.append(pt)
        tj.points[0].time_from_start = Duration(sec=0, nanosec=0)
        print(f"  发{what}…")
        fut = ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(n, fut, timeout_sec=10)
        gh = fut.result()
        if gh is None or not gh.accepted:
            print("   !! 目标被拒"); return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(n, rf, timeout_sec=dur + 30)
        res = rf.result()
        if res is None:
            print("   !! 没等到结果"); return False
        st, ec = res.status, res.result.error_code
        ok = (st == GoalStatus.STATUS_SUCCEEDED and ec == 0)
        print(f"   {'✅' if ok else '❌'} status={st} error_code={ec}"
              f"（两个都要对 —— 只查一个会被空壳 Result 骗过）")
        return ok

    pts, dur = plan_arm(cur, idx, delta, args.duration)
    if not send(pts, dur, f"去程 +{args.arm_deg:g}°"):
        return 1
    time.sleep(1.0)
    back = [list(cur), list(cur)]
    if not send(back, dur, "回程"):
        return 1

    # 回读：确认真的回到了原处（不是听它自报）
    rclpy.spin_once(n, timeout_sec=0.5)
    d2 = dict(zip(got["js"].name, got["js"].position))
    now = float(d2[ARM_JOINTS[idx]])
    print(f"回读 {ARM_JOINTS[idx]} = {now:+.4f} rad；与原值差 "
          f"{abs(now - cur[idx]) * 1000:.2f} mrad")
    n.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""**把机械臂慢慢收回 HOME** —— 发之前先逐点验路径。

为什么不能直接发：从任意姿势回 HOME 在关节空间里是**一条直线**，而各关节
**同时插值** —— 两端都在台面之上，中间照样可能俯冲下去。规划器里那条
`_path_ok` 就是干这个的；这里**自己再做一遍**，因为本脚本绕过规划器直接
对控制器发轨迹。

判据与规划器同口径：把 `forearm_link` / `wrist_1_link` 的原点在整条线段上
采样，**每一点都必须高于 `table_plane_z + guard_margin`**。任一点不过 ⇒
**不发**，并报出是第几个采样点、低了多少。

默认**只看不发**；`--go` 才发。轨迹时长由 `--duration` 控制（默认 8 s，
配合示教器速度滑条）。

    python3 scripts/real_go_home.py            # 只验路径，不发
    python3 scripts/real_go_home.py --go       # 验过之后真发
"""

import argparse
import os
import sys
import time

sys.dont_write_bytecode = True

ARM = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def main():
    ap = argparse.ArgumentParser(description="慢慢收回 HOME（默认只验不发）")
    ap.add_argument("--go", action="store_true")
    ap.add_argument("--duration", type=float, default=8.0, help="轨迹时长（秒）")
    ap.add_argument("--samples", type=int, default=40, help="路径采样点数")
    ap.add_argument("--params", default=os.path.expanduser("~/site_real.yaml"))
    args = ap.parse_args()

    import yaml
    par = yaml.safe_load(open(args.params, encoding="utf-8"))[
        "grasp_planner"]["ros__parameters"]
    home = [float(v) for v in par["home_joints"]]
    floor = float(par["table_plane_z"]) + float(par["guard_margin"])
    links = list(par.get("guard_links", ["forearm_link", "wrist_1_link"]))

    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from sensor_msgs.msg import JointState
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    from moveit_msgs.srv import GetPositionFK
    from action_msgs.msg import GoalStatus

    rclpy.init()
    n = Node("real_go_home")
    got = {}
    # ⚠️ 必须是**赋值**不是 `setdefault` —— `setdefault` 只存第一条消息，
    #    「回读」就会读到最初那一帧，于是「轨迹成功但臂没动」这种假象
    #    （第一次就是这么被骗的）。每次回调都更新。
    n.create_subscription(JointState, "/joint_states",
                          lambda m: got.__setitem__("js", m), 10)
    t0 = time.time()
    while "js" not in got and time.time() - t0 < 15:
        rclpy.spin_once(n, timeout_sec=0.2)
    if "js" not in got:
        print("!! 没收到 /joint_states"); return 2
    d = dict(zip(got["js"].name, got["js"].position))
    cur = [float(d[k]) for k in ARM]
    print(f"当前 (deg): {[round(v*57.2958,1) for v in cur]}")
    print(f"HOME (deg): {[round(v*57.2958,1) for v in home]}")
    print(f"判据：{links} 的原点必须高于 {floor:+.4f} m（= 台面 {par['table_plane_z']:+.3f} + 余量 {par['guard_margin']}）")

    # ---- 逐点验路径（用 /compute_fk，只读）----
    cli = n.create_client(GetPositionFK, "/compute_fk")
    if not cli.wait_for_service(timeout_sec=10.0):
        print("!! /compute_fk 不在（move_group 起了吗？）—— 拿不到路径判据，**拒绝发**")
        return 2

    def fk_z(q):
        req = GetPositionFK.Request()
        req.header.frame_id = "base_link"
        req.fk_link_names = links
        js = JointState(); js.name = ARM; js.position = [float(v) for v in q]
        req.robot_state.joint_state = js
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(n, fut, timeout_sec=10)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            return None
        return [p.pose.position.z for p in r.pose_stamped]

    worst = (1e9, -1)
    for i in range(args.samples + 1):
        t = i / args.samples
        q = [c + (h - c) * t for c, h in zip(cur, home)]
        zs = fk_z(q)
        if zs is None:
            print(f"!! 第 {i}/{args.samples} 个采样点 FK 失败 —— **拒绝发**")
            return 2
        m = min(zs)
        if m < worst[0]:
            worst = (m, i)
    print(f"路径最低点：第 {worst[1]}/{args.samples} 个采样点，最低连杆 {worst[0]:+.4f} m")
    if worst[0] < floor:
        print(f"❌ **拦下**：最低 {worst[0]:+.4f} < 下限 {floor:+.4f}"
              f"（低 {floor-worst[0]:.4f} m）—— 这条路径不发。")
        return 1
    print(f"✅ 路径全程高于下限（余量 {worst[0]-floor:.4f} m）")
    if not args.go:
        print("⇒ 只验不发（加 `--go` 才发）")
        return 0

    # ---- 发轨迹 ----
    ac = ActionClient(n, FollowJointTrajectory,
                      "/scaled_joint_trajectory_controller/follow_joint_trajectory")
    if not ac.wait_for_server(timeout_sec=10.0):
        print("!! 轨迹 action 不在"); return 2
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(ARM)
    pt = JointTrajectoryPoint()
    pt.positions = home
    pt.velocities = [0.0] * len(ARM)
    pt.time_from_start = Duration(sec=int(args.duration),
                                  nanosec=int((args.duration % 1) * 1e9))
    goal.trajectory.points = [pt]
    fut = ac.send_goal_async(goal)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=10)
    gh = fut.result()
    if gh is None or not gh.accepted:
        print("!! 目标被拒"); return 1
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(n, rf, timeout_sec=args.duration + 40)
    res = rf.result()
    if res is None:
        print("!! 没等到结果"); return 1
    ok = (res.status == GoalStatus.STATUS_SUCCEEDED and res.result.error_code == 0)
    print(f"{'✅' if ok else '❌'} status={res.status} error_code={res.result.error_code}")
    time.sleep(1.0)
    rclpy.spin_once(n, timeout_sec=0.5)
    d2 = dict(zip(got["js"].name, got["js"].position))
    now = [float(d2[k]) for k in ARM]
    print(f"回读 (deg): {[round(v*57.2958,1) for v in now]}")
    print(f"与 HOME 最大差 {max(abs(a-b) for a,b in zip(now,home))*57.2958:.2f}°")
    n.destroy_node(); rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

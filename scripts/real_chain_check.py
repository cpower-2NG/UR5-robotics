#!/usr/bin/env python3
"""第 0 步（只读）真机链路体检：**不发任何轨迹**。

它在**已经有东西在跑**的环境里工作 —— 它不起任何节点，只**看**：
  ① 服务在不在：/compute_ik、/compute_fk
  ② /joint_states 有没有、几个关节、值是多少
  ③ TF 树：base_link 到 tool0 的每一跳是否可达（逐帧打印）
  ④ 拿一个候选位姿真的调一次 /compute_ik，再用 /compute_fk 把解出来的
     关节角打回来看每个连杆的高度

⚠️ 为什么不用 `ros2 service list` / `ros2 node list`：本仓库实测过这些 CLI 在
   WSL2 上会**永远挂住**（既不返回也不报错）。这里一律走 rclpy 的图发现。
   （板子上 CLI 是好的，但两边用同一套代码更省心。）

⚠️ 本脚本**结构上不可能让臂动**：它只调 /compute_ik 与 /compute_fk（纯计算），
   不建 action client、不发 FollowJointTrajectory。有驱动在跑时也不动。

退出码：0 = 全通；2 = 环境不满足（服务/话题没起来，或 /compute_ik 一次都没解出来）。
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node

ARM_JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]
# base_link → tool0 的每一跳（UR5；⚠️ UR5 **没有** elbow_link —— 肘关节的
# 子树根是 forearm_link，这一条是实测确认过的，别照搬别的型号的链）
TF_CHAIN = [
    ("base_link", "shoulder_link"),
    ("shoulder_link", "upper_arm_link"),
    ("upper_arm_link", "forearm_link"),
    ("forearm_link", "wrist_1_link"),
    ("wrist_1_link", "wrist_2_link"),
    ("wrist_2_link", "wrist_3_link"),
    ("wrist_3_link", "tool0"),
]


def wait_service(node, name, timeout=60.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        if name in {n for n, _ in node.get_service_names_and_types()}:
            return time.monotonic() - t0
        time.sleep(0.2)
    return None


def main():
    rclpy.init()
    node = Node("real_chain_check")
    rc = 0
    try:
        print("=" * 78)
        print("第 0 步（只读）真机链路体检 —— 本脚本不发任何轨迹")
        print("=" * 78)

        # ---- ① 服务 ----
        print("\n① 服务")
        for s in ("/compute_ik", "/compute_fk"):
            dt = wait_service(node, s, timeout=60.0)
            if dt is None:
                print(f"  ✗ {s} 60 秒内没出现")
                rc = 2
            else:
                print(f"  ✓ {s} 在（等了 {dt:.0f}s）")
        if rc == 2:
            return rc

        # ---- ② /joint_states ----
        print("\n② /joint_states")
        from sensor_msgs.msg import JointState
        msgs = []
        node.create_subscription(JointState, "/joint_states",
                                 lambda m: msgs.append(m), 10)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 15.0 and not msgs:
            rclpy.spin_once(node, timeout_sec=0.2)
        if not msgs:
            print("  ✗ 15 秒内一条 /joint_states 都没有 —— 没有驱动、也没有假发布器？")
            return 2
        m = msgs[-1]
        print(f"  ✓ 收到（{len(m.name)} 个关节）：")
        for n, p in zip(m.name, m.position):
            mark = "" if n in ARM_JOINTS else "   ← 非臂关节"
            print(f"      {n:<26} {p:+.4f} rad{mark}")
        missing = [j for j in ARM_JOINTS if j not in m.name]
        if missing:
            print(f"  ✗ /joint_states 里缺臂关节：{missing}")
            rc = 2

        # ---- ③ TF 树 ----
        print("\n③ TF 树（base_link → tool0 逐跳）")
        from tf2_ros import Buffer, TransformListener
        buf = Buffer()
        # ⚠️ 监听器必须挂在一个会被 spin 的节点上；用本节点即可
        TransformListener(buf, node)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 10.0:
            rclpy.spin_once(node, timeout_sec=0.2)
            try:
                buf.lookup_transform("base_link", "tool0",
                                     rclpy.time.Time())
                break
            except Exception:
                pass
        ok_all = True
        for parent, child in TF_CHAIN:
            try:
                tr = buf.lookup_transform(parent, child, rclpy.time.Time())
                t = tr.transform.translation
                print(f"  ✓ {parent:<16}→ {child:<16} "
                      f"t=({t.x:+.4f},{t.y:+.4f},{t.z:+.4f})")
            except Exception as e:
                ok_all = False
                print(f"  ✗ {parent:<16}→ {child:<16} {type(e).__name__}: "
                      f"{str(e)[:70]}")
        # 相机帧（真机上由驱动提供；本无臂预检里**不该**有）
        for cam in ("camera_link", "camera_color_optical_frame"):
            try:
                buf.lookup_transform("base_link", cam, rclpy.time.Time())
                print(f"  · base_link → {cam} 可查（有相机 TF）")
            except Exception:
                print(f"  · base_link → {cam} 查不到（本预检里正常：相机没起）")
        if not ok_all:
            rc = 2

        # ---- ④ 真调一次 IK + FK ----
        print("\n④ /compute_ik + /compute_fk（真调一次，只看计算）")
        from moveit_msgs.srv import GetPositionIK, GetPositionFK
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs.msg import JointState as JS

        ik_cli = node.create_client(GetPositionIK, "/compute_ik")
        fk_cli = node.create_client(GetPositionFK, "/compute_fk")

        # 候选取自 grasp_planner 的抓取区中心附近（x 0.45 / y 0 / 顶面上方）
        target = (0.45, 0.0, 0.05)
        req = GetPositionIK.Request()
        req.ik_request.group_name = "ur_manipulator"
        req.ik_request.ik_link_name = "tool0"
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.sec = 2
        req.ik_request.pose_stamped = PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = "base_link"
        req.ik_request.pose_stamped.pose.position.x = target[0]
        req.ik_request.pose_stamped.pose.position.y = target[1]
        req.ik_request.pose_stamped.pose.position.z = target[2]
        # 工具朝下（绕 y 转 180°）：wrist_3 的 z 朝 → base 的 -z
        req.ik_request.pose_stamped.pose.orientation.y = 1.0
        req.ik_request.pose_stamped.pose.orientation.w = 0.0
        # ⚠️ 只给**6 个臂关节**。绝不注入夹爪关节 —— 真机模型里没有它，
        #    MoveIt 会抛未捕获异常把 move_group **整个打 SIGABRT**
        #    （留档：第六十二轮 (3)）。
        req.ik_request.robot_state.joint_state = JS(
            name=list(ARM_JOINTS), position=[0.0] * 6)

        fut = ik_cli.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 10.0:
            rclpy.spin_once(node, timeout_sec=0.2)
        if not fut.done():
            print("  ✗ /compute_ik 10 秒没回话")
            return 2
        res = fut.result()
        code = res.error_code.val
        print(f"  IK error_code = {code} "
              f"({'SUCCESS' if code == 1 else 'FAILED'})")
        if code != 1:
            print("  ✗ 一个位姿都没解出来 —— 环境不满足，结论无从谈起")
            return 2
        sol = {n: p for n, p in zip(res.solution.joint_state.name,
                                    res.solution.joint_state.position)
               if n in ARM_JOINTS}
        print("  解：" + "  ".join(f"{n.split('_')[0]}={sol[n]:+.3f}"
                                  for n in ARM_JOINTS))

        freqs = GetPositionFK.Request()
        freqs.header.frame_id = "base_link"
        freqs.fk_link_names = ["forearm_link", "wrist_1_link", "tool0"]
        freqs.robot_state.joint_state = JS(
            name=list(ARM_JOINTS), position=[sol[n] for n in ARM_JOINTS])
        f2 = fk_cli.call_async(freqs)
        t0 = time.monotonic()
        while not f2.done() and time.monotonic() - t0 < 10.0:
            rclpy.spin_once(node, timeout_sec=0.2)
        if not f2.done():
            print("  ✗ /compute_fk 10 秒没回话")
            return 2
        r = f2.result()
        print(f"  FK error_code = {r.error_code.val}")
        for name, ps in zip(r.fk_link_names, r.pose_stamped):
            p = ps.pose.position
            print(f"      {name:<16} ({p.x:+.4f}, {p.y:+.4f}, {p.z:+.4f})")

        print("\n" + "=" * 78)
        print("结论：链路通（服务齐、关节状态有、TF 到 tool0 通、IK/FK 都能算）"
              if rc == 0 else "结论：有问题，见上面 ✗")
        print("⚠️ 本脚本**只读**；它不替代「单步低速 + 急停就绪的第一次真机动作」。")
        print("=" * 78)
        return rc
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())

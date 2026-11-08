#!/usr/bin/env python3
"""E8 探针：MoveIt 场景 / attach / 规划（M1、M2）。

**只规划，不发轨迹** —— 只调 ``/apply_planning_scene``、``/compute_ik``、
``/compute_fk``、``/plan_kinematic_path``、``/get_planning_scene``。不发
``/move_action``、不碰控制器，臂不会动。

三项判据（契约 §3 E8）：

- **M1 无假碰撞**：几何上明显空着的悬停位，MoveIt 不得拒绝。做法：同一目标位姿
  先用 ``avoid_collisions=False`` 解一次（拿到候选解），再开碰撞检查解一次。
  开了就失败、且**独立核验器**认定候选解离所有块/桌面都 ≥5 mm ⇒ 记一次假碰撞。
  两次都解不出 ⇒ 单列 ``unreachable``（可能是限位/够不着），**不算** M1 通过。
- **M2 20/20 无碰撞**：每块取 2 个抓取朝向 × 2 个接近高度规划整条轨迹，
  再逐点用 ``/compute_fk`` + ``stage4_scene_model`` 的 OBB 间隙核验
  （接触场景余量 3 mm）。**不用 MoveIt 自己的碰撞结论判 MoveIt**。
- **attach/detach 正确**：attach 后物体从 world 消失、进 attached；detach 后回到
  world 且位姿不变（≤1 mm）。

夹爪在接近阶段是**全开**的（比闭合更占地方），所以规划起点和核验都按全开
（``stroke_half``）算 —— 这是保守方向。

用法（仿真栈已起）::

    python3 scripts/stage4_moveit_probe.py --truth <scene_truth.json> \
        --report runs/.../e8-sim.json
"""
import argparse
import json
import math
import os
import sys
import time

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np                                            # noqa: E402
import rclpy                                                  # noqa: E402
import stage4_scene_model as sm                               # noqa: E402
from geometry_msgs.msg import Pose, PoseStamped               # noqa: E402
from moveit_msgs.msg import (AttachedCollisionObject,        # noqa: E402
                             BoundingVolume, CollisionObject, Constraints,
                             OrientationConstraint, PlanningScene,
                             PlanningSceneComponents, PositionConstraint)
from moveit_msgs.srv import (ApplyPlanningScene, GetMotionPlan,  # noqa: E402
                             GetPlanningScene, GetPositionFK,
                             GetPositionIK, GetStateValidity)
from builtin_interfaces.msg import Duration                     # noqa: E402
from rclpy.node import Node                                   # noqa: E402
from sensor_msgs.msg import JointState                        # noqa: E402
from shape_msgs.msg import SolidPrimitive                     # noqa: E402

GROUP = "ur5_manipulator"
IK_LINK = "tool0"
ATTACH_LINK = "ag95_base_link"
BASE = "base_link"
FINGER_JOINTS = ("ag95_left_finger_joint", "ag95_right_finger_joint")
ARM_JOINTS = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
              "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")
TOUCH_LINKS = ["ag95_left_finger_link", "ag95_right_finger_link",
               "tool0", "ag95_base_link"]
# M1a 判「臂杆原点够高」用的连杆（tool0 由 gap 单独覆盖，不重复）
CHECK_LINKS = ["wrist_3_link", "wrist_2_link", "wrist_1_link", "forearm_link",
               "upper_arm_link", "shoulder_link", "base_link_inertia"]

TABLE_TOP = 0.0            # 仿真台面顶面 = base z 0（块心 z=0.01475 可反推）
OPEN_Q = sm.GRIPPER["stroke_half"]   # 接近阶段夹爪全开
SAFE_MARGIN = 0.005        # M1 判「明显空着」的余量
CONTACT_MARGIN = 0.003     # M2 接触场景余量（契约：2–3 mm）


def log(*a):
    print(*a, flush=True)


def call(node, client, req, timeout=10.0):
    fut = client.call_async(req)
    t0 = time.time()
    while not fut.done() and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.02)
    return fut.result() if fut.done() else None


def wait_service(node, client, name, timeout=60.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if client.service_is_ready():
            return True
        rclpy.spin_once(node, timeout_sec=0.2)
    log("!! 服务没就绪：%s" % name)
    return False


# --------------------------------------------------------------- 关节状态
def read_joint_state(node, timeout=15.0):
    got = {}

    def cb(msg):
        got["js"] = msg

    sub = node.create_subscription(JointState, "/joint_states", cb, 10)
    t0 = time.time()
    while "js" not in got and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    if "js" not in got:
        return None
    src = got["js"]
    js = JointState()
    js.name = list(src.name)
    js.position = [float(p) for p in src.position]
    return js


def joints_map(js):
    return dict(zip(js.name, js.position))


def with_joints(js, updates):
    """以 js 为底，覆盖/追加若干关节值，返回新的 JointState。"""
    m = joints_map(js)
    m.update({k: float(v) for k, v in updates.items()})
    out = JointState()
    out.name = list(js.name) + [k for k in updates if k not in js.name]
    out.position = [m[k] for k in out.name]
    return out


def open_fingers(js):
    """把两指设成全开（右指是 mimic，但显式给出也无害）。"""
    return with_joints(js, {FINGER_JOINTS[0]: OPEN_Q,
                            FINGER_JOINTS[1]: -OPEN_Q})


def close_fingers(js):
    """把两指设成闭合（q=0，两指内侧正好贴在块上）。M1b 的种子。"""
    return with_joints(js, {FINGER_JOINTS[0]: 0.0,
                            FINGER_JOINTS[1]: 0.0})


# --------------------------------------------------------------- 场景
def make_block_object(b, frame=BASE):
    yaw = math.radians(b.get("yaw", 0.0))
    s = float(b.get("size") or (b["top"] - TABLE_TOP))
    obj = CollisionObject()
    obj.header.frame_id = frame
    obj.id = "block-%s" % b["id"]
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [s, s, s]
    pose = Pose()
    pose.position.x = float(b["x"])
    pose.position.y = float(b["y"])
    pose.position.z = float(b.get("z", TABLE_TOP + s / 2.0))
    pose.orientation.z = math.sin(yaw / 2.0)
    pose.orientation.w = math.cos(yaw / 2.0)
    obj.primitives = [box]
    obj.primitive_poses = [pose]
    obj.operation = CollisionObject.ADD
    return obj


def make_table_object(frame=BASE):
    obj = CollisionObject()
    obj.header.frame_id = frame
    obj.id = "table"
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [2.0, 2.0, 0.05]
    pose = Pose()
    pose.position.z = TABLE_TOP - 0.025
    pose.orientation.w = 1.0
    obj.primitives = [box]
    obj.primitive_poses = [pose]
    obj.operation = CollisionObject.ADD
    return obj


def send_scene(node, client, world_objs=(), attached=()):
    req = ApplyPlanningScene.Request()
    req.scene = PlanningScene()
    req.scene.is_diff = True
    req.scene.world.collision_objects = list(world_objs)
    if attached:
        req.scene.robot_state.attached_collision_objects = list(attached)
        req.scene.robot_state.is_diff = True
    resp = call(node, client, req)
    return bool(resp and resp.success)


def read_scene(node, client):
    req = GetPlanningScene.Request()
    req.components.components = (
        PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
    resp = call(node, client, req, timeout=20.0)
    if resp is None:
        return None
    return {
        "world": [(o.id, o) for o in resp.scene.world.collision_objects],
        "attached": [(a.object.id, a) for a in
                     resp.scene.robot_state.attached_collision_objects],
    }


def scene_ids(sc):
    if sc is None:
        return [], []
    return [i for i, _ in sc["world"]], [i for i, _ in sc["attached"]]


# --------------------------------------------------------------- FK / 核验
def fk_tool(node, fk_client, js, links=(IK_LINK,)):
    """一次 FK 拿若干连杆的 (xyz, quat)。失败返回 None。"""
    req = GetPositionFK.Request()
    req.header.frame_id = BASE
    req.fk_link_names = list(links)
    req.robot_state.joint_state = js
    resp = call(node, fk_client, req, timeout=10.0)
    if resp is None or resp.error_code.val != 1:
        return None
    out = {}
    for name, ps in zip(links, resp.pose_stamped):
        p = ps.pose
        out[name] = (np.array([p.position.x, p.position.y, p.position.z]),
                     (p.orientation.x, p.orientation.y,
                      p.orientation.z, p.orientation.w))
    return out


def gripper_boxes(tool_pose, q=OPEN_Q):
    """tool0 位姿 → 三个 OBB（本体 + 左右指），按全开口径。"""
    return sm.gripper_obbs(tool_pose, q)


# --------------------------------------------------------------- 位姿
def pose_stamped(x, y, z, quat, frame=BASE):
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.position.z = float(z)
    (ps.pose.orientation.x, ps.pose.orientation.y,
     ps.pose.orientation.z, ps.pose.orientation.w) = quat
    return ps


def make_ik_req(js, ps, avoid, timeout=0.5):
    req = GetPositionIK.Request()
    ik = req.ik_request
    ik.group_name = GROUP
    ik.ik_link_name = IK_LINK
    ik.pose_stamped = ps
    ik.robot_state.joint_state = js
    ik.avoid_collisions = bool(avoid)
    ik.timeout = Duration(sec=int(timeout),
                          nanosec=int((timeout % 1) * 1e9))
    return req


def solution_state(base_js, resp):
    sol = resp.solution.joint_state
    return with_joints(base_js, dict(zip(sol.name, sol.position)))


# --------------------------------------------------------------- M1
#
# 独立性的边界（写清，别夸大）：臂杆的碰撞几何在 ur_description 里是 **mesh**，
# 没法用 OBB 独立复算；**夹爪**是本体+两指三个 box，尺寸逐字来自我们自己的
# URDF，可以独立算。所以 M1 的独立判据只对夹爪成立，臂杆那部分借用 MoveIt 自己
# 的接触报告来归因（下面 M1a 的判法），并在结论里写明这一点。
#
#   M1a 空着不得被拒：取每块上方两个接近高度 × 两个朝向的位姿（夹爪全开），
#       先免碰撞 IK 拿候选解 → 用我们自己的 OBB 算夹爪间隙 → 再问
#       check_state_validity 这个解本身有效无效 → 最后开碰撞 IK。
#       判为「假碰撞」只有两条路：(i) MoveIt 自己说这个解有效、我们算夹爪也够远，
#       但开碰撞 IK 仍拒绝；(ii) MoveIt 报的接触里有夹爪连杆，而我们算的间隙
#       ≥5 mm。都不成立就按「真被挡住」（臂杆撞桌面等）记账，不算通过也不算失败。
#   M1b 撞着不得放过：把两指合拢插进块里（几何上必然重叠），开碰撞 IK 必须拒绝。
#       若被接受，说明场景根本没进 MoveIt（块没加进去），是最危险的假绿灯。
ARM_MIN_Z = 0.15          # 臂杆原点高度的保守界：块只有 29.5 mm 高、桌面在 z=0
GRIPPER_CLEAR = 0.005     # M1a 判「夹爪离障碍足够远」的余量（5 mm）
M1A_GOOD_ENOUGH = 8       # M1a 至少要有这么多「双方都说空」的位姿，才有说服力
GRIPPER_LINKS = {"tool0", "ag95_base_link", "ag95_left_finger_link",
                 "ag95_right_finger_link"}


def gripper_gap(node, fk_client, js, blocks, table):
    """独立判据：全开夹爪（本体+两指）离所有块和桌面的最小间隙（米）。

    FK 失败返回 None。
    """
    fk = fk_tool(node, fk_client, js)
    if fk is None:
        return None
    gap = None
    for lk in gripper_boxes(fk[IK_LINK]):
        for sc in blocks + [table]:
            g = sm.gap(lk, sc)
            gap = g if gap is None else min(gap, g)
    return gap


def approach_poses(truth):
    """每块 × 两个朝向 × 两个接近高度（指尖离块顶 10 / 50 mm）。"""
    poses = []
    for b in truth["blocks"]:
        top = float(b.get("top") or (b["z"] + b["size"] / 2.0))
        byaw = b.get("yaw", 0.0)
        for yaw_off in (0.0, 90.0):
            for tip_clear in (0.010, 0.050):
                z = top + tip_clear + sm.GRIPPER["tip_z"]
                poses.append({
                    "block": b["id"], "yaw_deg": byaw + yaw_off,
                    "tip_clear_mm": tip_clear * 1000.0,
                    "ps": pose_stamped(float(b["x"]), float(b["y"]), z,
                                       sm.down_quat(math.radians(byaw
                                                                 + yaw_off))),
                })
    return poses


def is_obstacle(name):
    """规划场景里的障碍物（块 / 桌面），不是机器人连杆。"""
    return name == "table" or name.startswith("block-")


def contact_kinds(pairs):
    """把接触对分成三类：夹爪×障碍、臂杆×障碍、机器人自碰。"""
    kinds = set()
    for a, b in pairs:
        oa, ob = is_obstacle(a), is_obstacle(b)
        if oa or ob:
            other = b if oa else a
            kinds.add("gripper_obstacle" if other in GRIPPER_LINKS
                      else "arm_obstacle")
        else:
            kinds.add("self")
    return kinds


def classify_m1a(pairs, gap):
    """状态被 MoveIt 判无效时归因 —— 只有夹爪×障碍才算假碰撞的候选。"""
    kinds = contact_kinds(pairs)
    if "gripper_obstacle" in kinds:
        if gap is not None and gap >= GRIPPER_CLEAR:
            return "false_collision", kinds   # MoveIt 说夹爪撞了，我们算够远
        return "gripper_collide", kinds       # 我们自己的几何也认账
    if "arm_obstacle" in kinds:
        return "blocked_arm", kinds           # 臂杆 mesh 撞障碍：真被挡住
    if kinds == {"self"}:
        return "blocked_self", kinds          # 腕部折进前臂这类自碰
    return "blocked_unknown", kinds


def same_joints(a, b, tol=1e-3):
    """两个 JointState 在公差内相同（只看共同的关节名）。"""
    ma, mb = joints_map(a), joints_map(b)
    common = set(ma) & set(mb)
    return bool(common) and all(abs(ma[k] - mb[k]) <= tol for k in common)


def solve_ik(node, ik_client, ps, seeds, avoid, timeout=1.0, cap=3):
    """按种子顺序试 IK，返回 (解, 用到的种子下标)；全失败返回 (None, None)。

    解是**合并回种子**得到的完整 JointState —— 种子里的指头开合被保留。
    """
    for i, sd in enumerate(seeds[:cap]):
        r = call(node, ik_client, make_ik_req(sd, ps, avoid=avoid,
                                              timeout=timeout),
                 timeout=timeout + 15.0)
        if r is not None and r.error_code.val == 1:
            return solution_state(sd, r), i
    return None, None


def check_m1a(node, ik_client, fk_client, validity_client, seed_js, truth,
              blocks, table):
    """M1a：可以证明空着的位姿，MoveIt 必须收。

    每一步都留证据：免碰撞 IK 出解 → 我们用夹爪 OBB 独立算间隙 → MoveIt 的
    check_state_validity 说这个解有效无效 → **以这个解本身为种子**问碰撞感知
    IK 收不收。最后一步是关键：种子就是解，所以「收不到」只能是它拒了，不是搜不到。
    """
    rows = []
    # 种子池：停泊位 + 已经**被 MoveIt 判为有效**的那些解。不拿无效解当种子 ——
    # 那会把后续位姿拖进「腕部折进前臂」的坏支路，整批都变成 blocked。
    good = [seed_js]
    for ap in approach_poses(truth):
        ps = ap["ps"]
        row = {k: v for k, v in ap.items() if k != "ps"}
        sol, seed_i = solve_ik(node, ik_client, ps, good, avoid=False)
        if sol is None:
            row.update(verdict="unreachable", gap_mm=None, valid=None,
                       ik_avoid=None, contacts=[])
            rows.append(row)
            log("  M1a block-%-2s yaw=%3.0f clear=%4.0fmm 免碰撞 IK 解不出 → "
                "unreachable" % (ap["block"], ap["yaw_deg"],
                                 ap["tip_clear_mm"]))
            continue

        gap = gripper_gap(node, fk_client, sol, blocks, table)
        vreq = GetStateValidity.Request()
        vreq.robot_state.joint_state = sol
        vreq.group_name = GROUP
        vr = call(node, validity_client, vreq)
        valid = None if vr is None else bool(vr.valid)
        pairs = [(c.contact_body_1, c.contact_body_2) for c in vr.contacts] \
            if vr is not None else []

        if valid:
            # 同一个位姿，改以「已知有效的解」为种子再问一次碰撞感知 IK
            r_safe = call(node, ik_client,
                          make_ik_req(sol, ps, avoid=True, timeout=1.0),
                          timeout=16.0)
            ik_avoid = bool(r_safe and r_safe.error_code.val == 1)
            verdict = "ok" if ik_avoid else "false_collision"
            kinds = set()
            if not any(same_joints(g, sol) for g in good):
                good.insert(0, sol)
                del good[3:]
        else:
            ik_avoid = False
            verdict, kinds = classify_m1a(pairs, gap)

        row.update(verdict=verdict, seed=seed_i,
                   gap_mm=None if gap is None else round(gap * 1000.0, 1),
                   valid=valid, ik_avoid=ik_avoid,
                   kinds=sorted(kinds),
                   contacts=["%s×%s" % p for p in pairs][:4])
        rows.append(row)
        log("  M1a block-%-2s yaw=%3.0f clear=%4.0fmm 夹爪间隙=%5.1fmm "
            "状态有效=%s 开碰撞IK=%s %s → %s"
            % (ap["block"], ap["yaw_deg"], ap["tip_clear_mm"],
               0.0 if gap is None else gap * 1000.0, valid, ik_avoid,
               "" if not kinds else "/".join(sorted(kinds)), verdict))
    return rows


def check_m1b(node, ik_client, seed_js, truth):
    """M1b：几何上必然重叠的位姿，MoveIt 必须拒绝。

    位姿 = 块心上方 (body_z + 指长/2) = 132.1 mm 的 tool0（两指合拢时正好把块夹在
    中间）。两指闭合态沿 x 只跨 ±22 mm，块就卡在中间 —— 真重叠，独立判据 gap=0。

    种子先取「同一块上方 50 mm 的悬停解」，再下探 —— 从停泊位直接解这么低的位姿
    常常解不出（上一版 6 块里 1 块 unreachable 就是这个原因），那属于 IK 搜索失败，
    不是「MoveIt 收下了碰撞」。
    """
    rows = []
    for b in truth["blocks"]:
        top = float(b.get("top") or (b["z"] + b["size"] / 2.0))
        yaw = math.radians(b.get("yaw", 0.0))
        z = float(b.get("z", TABLE_TOP + b["size"] / 2.0)) + \
            sm.GRIPPER["body_z"] + sm.GRIPPER["finger"][2] / 2.0
        ps = pose_stamped(float(b["x"]), float(b["y"]), z, sm.down_quat(yaw))
        row = {"block": b["id"], "pose": [float(b["x"]), float(b["y"]), z]}

        hover = pose_stamped(float(b["x"]), float(b["y"]),
                             top + 0.050 + sm.GRIPPER["tip_z"],
                             sm.down_quat(yaw))
        sol_h, _ = solve_ik(node, ik_client, hover, [seed_js], avoid=False)

        seeds = [s for s in (sol_h, seed_js) if s is not None]
        sol, _ = solve_ik(node, ik_client, ps, seeds, avoid=False)
        if sol is None:
            row.update(ik_no_check=False, ik_avoid=False,
                       verdict="unreachable")
            rows.append(row)
            log("  M1b block-%-2s 免碰撞 IK 解不出 → unreachable" % b["id"])
            continue
        r_safe = call(node, ik_client,
                      make_ik_req(sol, ps, avoid=True, timeout=1.0),
                      timeout=16.0)
        safe_ok = bool(r_safe and r_safe.error_code.val == 1)
        row["ik_no_check"] = True
        row["ik_avoid"] = safe_ok
        row["verdict"] = "false_negative" if safe_ok else "ok"
        rows.append(row)
        log("  M1b block-%-2s 免检查=1 开检查=%d → %s"
            % (b["id"], safe_ok, row["verdict"]))
    return rows


# --------------------------------------------------------------- M2
def goal_constraints(ps):
    c = Constraints()
    pc = PositionConstraint()
    pc.header = ps.header
    pc.link_name = IK_LINK
    shp = SolidPrimitive()
    shp.type = SolidPrimitive.SPHERE
    shp.dimensions = [0.005]   # 5 mm 球：1 mm 时 OMPL 的目标采样经常超时
    bv = BoundingVolume()
    bv.primitives = [shp]
    bv.primitive_poses = [ps.pose]
    pc.constraint_region = bv
    oc = OrientationConstraint()
    oc.header = ps.header
    oc.link_name = IK_LINK
    oc.orientation = ps.pose.orientation
    oc.absolute_x_axis_tolerance = 0.01
    oc.absolute_y_axis_tolerance = 0.01
    oc.absolute_z_axis_tolerance = 0.01
    oc.weight = 1.0
    c.position_constraints = [pc]
    c.orientation_constraints = [oc]
    return [c]


def plan_to(node, plan_client, start_js, ps, timeout=8.0):
    req = GetMotionPlan.Request()
    mpr = req.motion_plan_request
    mpr.group_name = GROUP
    mpr.start_state.joint_state = start_js
    mpr.start_state.is_diff = False
    mpr.goal_constraints = goal_constraints(ps)
    mpr.num_planning_attempts = 3
    mpr.allowed_planning_time = max(1.0, timeout)
    resp = call(node, plan_client, req, timeout=timeout + 20.0)
    if resp is None:
        return None, "no_response"
    code = resp.motion_plan_response.error_code.val
    if code != 1:
        return None, "error_%d" % code
    return resp.motion_plan_response.trajectory.joint_trajectory, "ok"


def check_path(node, fk_client, base_js, traj, blocks, table):
    """轨迹逐点 FK + 独立 OBB 核验。返回 (碰撞点数, 最小间隙米, 采样数)。"""
    names = list(traj.joint_names)
    pts = [list(p.positions) for p in traj.points]
    worst = None
    hits = 0
    samples = sm.interp_positions(pts, step=0.05) if pts else []
    for pos in samples:
        js = with_joints(base_js, dict(zip(names, pos)))
        fk = fk_tool(node, fk_client, js)
        if fk is None:
            continue
        for lk in gripper_boxes(fk[IK_LINK]):
            for sc in blocks + [table]:
                g = sm.gap(lk, sc)
                worst = g if worst is None else min(worst, g)
                if g <= CONTACT_MARGIN:
                    hits += 1
    return hits, worst, len(samples)


def check_m2(node, plan_client, fk_client, ik_client, start_js, truth, blocks,
             table):
    rows = []
    for b in truth["blocks"]:
        top = float(b.get("top") or (b["z"] + b["size"] / 2.0))
        byaw = b.get("yaw", 0.0)
        for yaw_off in (0.0, 90.0):
            for tip_clear in (0.010, 0.050):
                x, y = float(b["x"]), float(b["y"])
                z = top + tip_clear + sm.GRIPPER["tip_z"]
                ps = pose_stamped(x, y, z,
                                  sm.down_quat(math.radians(byaw + yaw_off)))
                traj, err = plan_to(node, plan_client, start_js, ps)
                row = {"block": b["id"], "yaw_deg": byaw + yaw_off,
                       "tip_clear_mm": tip_clear * 1000.0,
                       "plan": err, "n_points": 0, "n_samples": 0,
                       "collision_samples": 0, "worst_gap_mm": None}
                if traj is None or len(traj.points) == 0:
                    row["plan"] = err if traj is None else "empty_trajectory"
                    # 规划失败要分两种：这个位姿本来就没有免碰撞解（不算规划器的
                    # 错），还是有解但规划器没找到（记 plan_failed）
                    chk, _ = solve_ik(node, ik_client, ps, [start_js],
                                      avoid=True, timeout=2.0)
                    row["verdict"] = ("pose_blocked" if chk is None
                                      else "plan_failed")
                    rows.append(row)
                    log("  M2 block-%-2s yaw=%3.0f clear=%4.0fmm → %s（%s）"
                        % (b["id"], byaw + yaw_off, tip_clear * 1000,
                           "该位姿无免碰撞解" if chk is None
                           else "规划器没找到路径", row["plan"]))
                    continue
                hits, worst, ns = check_path(node, fk_client, start_js, traj,
                                             blocks, table)
                row.update(n_points=len(traj.points), n_samples=ns,
                           collision_samples=hits,
                           worst_gap_mm=None if worst is None
                           else round(worst * 1000.0, 2))
                row["verdict"] = "ok" if hits == 0 else "collision"
                rows.append(row)
                log("  M2 block-%-2s yaw=%3.0f clear=%4.0fmm → 航点=%d 采样=%d "
                    "碰撞=%d 最小间隙=%.1fmm"
                    % (b["id"], byaw + yaw_off, tip_clear * 1000,
                       len(traj.points), ns, hits,
                       0.0 if worst is None else worst * 1000.0))
    return rows


# --------------------------------------------------------------- attach
def obj_pose(o):
    """CollisionObject 的整体位姿 = ``o.pose`` ⊗ ``primitive_poses[0]``。

    实测（2026-11-08，S1:0）：MoveIt 读回的 ``primitive_poses`` 永远是单位位姿，
    真正的位姿只在 ``o.pose`` 里。只读 primitive_poses 会把「位姿对不对」错判成
    628.72 mm 误差（那次错误结论已撤回）。
    """
    if hasattr(o, "object"):      # AttachedCollisionObject 包了一层
        o = o.object
    p, q = o.pose.position, o.pose.orientation
    P = np.array([p.x, p.y, p.z])
    Q = (q.x, q.y, q.z, q.w)
    if o.primitive_poses:
        pp, pq = o.primitive_poses[0], o.primitive_poses[0].orientation
        R = sm.quat_to_mat(Q)
        P = P + R @ np.array([pp.position.x, pp.position.y, pp.position.z])
        Q = quat_mul(Q, (pq.x, pq.y, pq.z, pq.w))
    return P, Q


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_angle(a, b):
    """两个四元数之间的转角（弧度）。"""
    d = abs(sum(x * y for x, y in zip(a, b)))
    d = min(1.0, max(-1.0, d))
    return 2.0 * math.acos(d)


def find_obj(scene, oid, which):
    for i, o in (scene or {}).get(which, []):
        if i == oid:
            return o
    return None


def attach_detach(node, scene_client, get_scene, fk_client, start_js, truth):
    """attach → 核验 → detach → 核验，判据三条：

    1. attach 后：从 world 消失、出现在 attached（按 id）；
    2. attach 后：attached 里的位姿 == ``link⁻¹ ⊗ world``（说明是按世界系理解的）；
    3. detach 后：回到 world、attached 清空，且世界位姿与 attach 前一致（≤1 mm）。
    """
    b = truth["blocks"][0]
    oid = "block-%s" % b["id"]
    s = float(b.get("size") or (b["top"] - TABLE_TOP))
    yaw = math.radians(b.get("yaw", 0.0))
    W = np.array([float(b["x"]), float(b["y"]),
                  float(b.get("z", TABLE_TOP + s / 2.0))])
    wq = (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

    out = {"object": oid, "world_pose": [float(v) for v in W]}
    before = read_scene(node, get_scene)
    w0, a0 = scene_ids(before)
    out["before"] = {"world": w0, "attached": a0}

    fk = fk_tool(node, fk_client, start_js, (ATTACH_LINK,))
    if fk is None:
        out.update(pass_=False, note="FK 拿不到 link 位姿")
        return out
    link_p, link_q = fk[ATTACH_LINK]
    R = sm.quat_to_mat(link_q)
    local_expect = R.T @ (W - link_p)

    aco = AttachedCollisionObject()
    aco.link_name = ATTACH_LINK
    aco.touch_links = TOUCH_LINKS
    aco.object.id = oid
    aco.object.header.frame_id = BASE
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [s, s, s]
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = [float(v) for v in W]
    (pose.orientation.x, pose.orientation.y,
     pose.orientation.z, pose.orientation.w) = wq
    aco.object.primitives = [box]
    aco.object.primitive_poses = [pose]
    aco.object.operation = CollisionObject.ADD

    ok_add = send_scene(node, scene_client, attached=[aco])
    time.sleep(0.5)
    mid = read_scene(node, get_scene)
    w1, a1 = scene_ids(mid)
    att = find_obj(mid, oid, "attached")
    local_err = None
    if att is not None:
        lp, _ = obj_pose(att)
        local_err = float(np.linalg.norm(lp - local_expect))
    out["after_attach"] = {"ok": ok_add, "world": w1, "attached": a1,
                           "local_pose_mm": None if local_err is None
                           else round(local_err * 1000, 2)}

    aco.object.operation = CollisionObject.REMOVE
    ok_rm = send_scene(node, scene_client, attached=[aco])
    time.sleep(0.5)
    after = read_scene(node, get_scene)
    w2, a2 = scene_ids(after)
    back = find_obj(after, oid, "world")
    pose_err = None
    ang_err = None
    if back is not None:
        wp, wqb = obj_pose(back)
        pose_err = float(np.linalg.norm(wp - W))
        ang_err = quat_angle(wqb, wq)
    out["after_detach"] = {
        "ok": ok_rm, "world": w2, "attached": a2,
        "pose_err_mm": None if pose_err is None
        else round(pose_err * 1000, 2),
        "angle_err_deg": None if ang_err is None
        else round(math.degrees(ang_err), 3)}

    out["pass"] = bool(ok_add and ok_rm
                       and oid in a1 and oid not in w1
                       and oid in w2 and oid not in a2
                       and local_err is not None and local_err <= 0.002
                       and pose_err is not None and pose_err <= 0.001
                       and ang_err is not None and ang_err <= math.radians(0.2))
    log("  attach: %s→attached（link 系位姿误差=%s mm）；detach→world "
        "（位姿误差=%s mm，%s°）→ %s"
        % (oid, "n/a" if local_err is None else "%.2f" % (local_err * 1000),
           "n/a" if pose_err is None else "%.2f" % (pose_err * 1000),
           "n/a" if ang_err is None else "%.2f" % math.degrees(ang_err),
           "PASS" if out["pass"] else "FAIL"))
    return out


# --------------------------------------------------------------- main
def main(argv=None):
    ap = argparse.ArgumentParser(description="E8 MoveIt 探针（只规划，不动臂）")
    ap.add_argument("--truth", required=True, help="scene_truth.json")
    ap.add_argument("--report", default=None, help="结果 JSON 输出路径")
    ap.add_argument("--plans", type=int, default=20, help="M2 要求干净的条数")
    ap.add_argument("--skip-m1", action="store_true")
    ap.add_argument("--skip-m2", action="store_true")
    ap.add_argument("--skip-attach", action="store_true")
    args = ap.parse_args(argv)

    rclpy.init()
    node = Node("stage4_moveit_probe")

    scene_client = node.create_client(ApplyPlanningScene,
                                      "/apply_planning_scene")
    get_scene = node.create_client(GetPlanningScene, "/get_planning_scene")
    ik_client = node.create_client(GetPositionIK, "/compute_ik")
    fk_client = node.create_client(GetPositionFK, "/compute_fk")
    plan_client = node.create_client(GetMotionPlan, "/plan_kinematic_path")
    validity_client = node.create_client(GetStateValidity,
                                         "/check_state_validity")

    need = [("scene", scene_client, "/apply_planning_scene"),
            ("getscene", get_scene, "/get_planning_scene"),
            ("ik", ik_client, "/compute_ik"),
            ("fk", fk_client, "/compute_fk")]
    if not args.skip_m1:
        need.append(("validity", validity_client, "/check_state_validity"))
    if not args.skip_m2:
        need.append(("plan", plan_client, "/plan_kinematic_path"))
    if not all([wait_service(node, c, n) for _, c, n in need]):
        node.destroy_node()
        rclpy.shutdown()
        return 2

    with open(args.truth, encoding="utf-8") as f:
        truth = json.load(f)
    start_js = read_joint_state(node)
    if start_js is None:
        log("!! 拿不到 /joint_states")
        node.destroy_node()
        rclpy.shutdown()
        return 2
    # 规划与核验都按「夹爪全开」算（保守方向）
    plan_js = open_fingers(start_js)

    report = {"truth": args.truth, "batch": truth.get("batch"),
              "index": truth.get("index"), "table_top": TABLE_TOP,
              "open_q_mm": OPEN_Q * 1000.0,
              "joints0": joints_map(start_js),
              "joints_plan": joints_map(plan_js)}

    # 1) 场景
    objs = [make_table_object()] + [make_block_object(b)
                                    for b in truth["blocks"]]
    applied = send_scene(node, scene_client, world_objs=objs)
    time.sleep(0.5)
    sc = read_scene(node, get_scene)
    world, attached = scene_ids(sc)
    report["scene"] = {"applied": applied, "world": world,
                       "attached": attached,
                       "want": [o.id for o in objs],
                       "pass": bool(applied
                                    and all(o.id in world for o in objs))}
    log("场景：applied=%s world=%s attached=%s → %s"
        % (applied, world, attached,
           "PASS" if report["scene"]["pass"] else "FAIL"))

    blocks = [sm.block_obb(b, TABLE_TOP) for b in truth["blocks"]]
    table = sm.table_obb(TABLE_TOP)

    # 2) M1
    if not args.skip_m1:
        log("\n== M1 无假碰撞 ==")
        seed_closed = close_fingers(start_js)
        m1a = check_m1a(node, ik_client, fk_client, validity_client, plan_js,
                        truth, blocks, table)
        log("  --- M1b：必然重叠的位姿必须被拒 ---")
        m1b = check_m1b(node, ik_client, seed_closed, truth)
        a_false = [r for r in m1a if r["verdict"] == "false_collision"]
        a_unreach = [r for r in m1a if r["verdict"] == "unreachable"]
        a_grip = [r for r in m1a if r["verdict"] == "gripper_collide"]
        a_ok = [r for r in m1a if r["verdict"] == "ok"]
        b_bad = [r for r in m1b if r["verdict"] != "ok"]
        report["m1a"] = m1a
        report["m1b"] = m1b
        report["m1_summary"] = {
            "m1a_n": len(m1a), "m1a_ok": len(a_ok),
            "m1a_false_collision": len(a_false),
            "m1a_unreachable": len(a_unreach),
            "m1a_gripper_collide": len(a_grip),
            "m1a_min_ok": M1A_GOOD_ENOUGH,
            "m1b_n": len(m1b), "m1b_ok": len(m1b) - len(b_bad),
            "m1b_bad": [r.get("verdict") for r in b_bad],
            "pass": bool(m1a and m1b and not a_false and not a_unreach
                         and not a_grip and len(a_ok) >= M1A_GOOD_ENOUGH
                         and not b_bad)}
        log("M1 汇总：%s" % json.dumps(report["m1_summary"],
                                       ensure_ascii=False))

    # 3) M2
    if not args.skip_m2:
        log("\n== M2 规划 + 独立核验 ==")
        m2 = check_m2(node, plan_client, fk_client, ik_client, plan_js, truth,
                      blocks, table)
        n_ok = len([r for r in m2 if r["verdict"] == "ok"])
        n_col = len([r for r in m2 if r["verdict"] == "collision"])
        n_pf = len([r for r in m2 if r["verdict"] == "plan_failed"])
        n_pb = len([r for r in m2 if r["verdict"] == "pose_blocked"])
        report["m2"] = m2
        report["m2_summary"] = {
            "n": len(m2), "ok": n_ok, "collision": n_col,
            "plan_failed": n_pf, "pose_blocked": n_pb, "want": args.plans,
            # 契约要的是「20 条轨迹、条条无碰撞」：规划出来的路径必须 100% 干净，
            # 且干净条数不少于 20。规划器没找到路径单列（不是碰撞，但会吃掉配额）。
            "pass": n_ok >= args.plans and n_col == 0}
        log("M2 汇总：%s" % json.dumps(report["m2_summary"],
                                       ensure_ascii=False))

    # 4) attach / detach
    if not args.skip_attach:
        log("\n== attach / detach ==")
        report["attach"] = attach_detach(node, scene_client, get_scene,
                                            fk_client, start_js, truth)

    node.destroy_node()
    rclpy.shutdown()

    passed = report["scene"]["pass"]
    if "m1_summary" in report:
        passed = passed and report["m1_summary"]["pass"]
    if "m2_summary" in report:
        passed = passed and report["m2_summary"]["pass"]
    if "attach" in report:
        passed = passed and report["attach"]["pass"]
    report["pass"] = bool(passed)

    log("\n== 总判：%s ==" % ("PASS" if passed else "FAIL"))
    if args.report:
        d = os.path.dirname(os.path.abspath(args.report))
        os.makedirs(d, exist_ok=True)
        with open(args.report, "w", encoding="utf-8") as f:
            json.dump(report, f, ensure_ascii=False, indent=1)
        log("→ %s" % args.report)
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())

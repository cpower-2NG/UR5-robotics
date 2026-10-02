#!/usr/bin/env python3
"""
肘高预检（W1b / D49 的现场闸门）—— 纯计算，**不发任何轨迹**
=============================================================
回答一个问题：规划器打算去的那些位姿，解出来的肘/大臂到底在桌面**之上**吗？

为什么需要这个脚本
------------------
`planner_node.py` 里有一道肘高护栏（`elbow_guard_enabled` / `guard_links` /
`table_plane_z` / `guard_margin`）。但护栏是**运行期**拦的：它只在规划器真的
走向那个目标时才开火，而那时臂已经在动了。现场第一次上电时，应当先**离线**
把整片抓取区/放置区的候选点位全部解一遍 IK、用 FK 打出台面侧各连杆的高度，
人工确认「没有一条会插进桌子」，再让臂动。

它守的是 D49 那个只在真机上成立的风险：仿真里 UR5 底座坐在 0.80 m 立柱上、
桌面 0.41 m，所以 **肘朝下的 IK 分支**落在桌面上方，一辈子碰不到；真机把臂
直接装在桌面上，同一个分支会落到**桌面以下约 21 cm**。而 `avoid_collisions=True`
当时防的是一个**空场景**（桌子不在 MoveIt 规划场景里），拦不住它。

本脚本用**运行中规划器自己的参数**（不是硬编码），所以它量的就是规划器将要
走的那批位姿：

  抓取段：pick_zone 内按网格采样 (x,y)，z 取 **place_z**（= 方块可见顶面高度，
          也正是检测报出来的那个 wz —— 即规划器实际下发的抓取高度），
          下降点与接近点各解一次。`--stress` 会额外压一遍区域的 z 上下界。
  放置段：place_slots 每个槽位的下降点与接近点

采样点解不出 IK 时会逐条列出。**它们不代表护栏有问题**：`pick_zone` 是个
矩形**过滤器**，比可达工作空间大是正常的 —— 检测报什么它就收什么，解不出来
的点规划器会跳过并告警。但若失败集中在区域**边界**上，说明区框得比臂能到的
范围还大，可以顺手把区收窄。

用法:
    # 先起 bringup（仿真或真机），再：
    python3 scripts/elbow_guard_probe.py
    python3 scripts/elbow_guard_probe.py --node /grasp_planner --grid 4

退出码：0 = 全部连杆都在护栏之上（可以动）；1 = 有连杆低于护栏（**不要动**）；
2 = **一个位姿都没解出来**（环境不满足，结论无从谈起 —— 先查 ik_group / /compute_ik）。
只调 /compute_ik 与 /compute_fk，**绝不发轨迹**。
"""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents

# 与 planner_node.grasp_planner 默认值一致的兜底（节点读不到参数时才用）
FALLBACK = {
    "ik_group": "ur5_manipulator", "ik_link": "tool0", "ik_timeout": 2.0,
    "base_frame": "base_link",
    "arm_joint_names": ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"],
    "grasp_rpy": [math.pi, 0.0, 0.0],
    "home_joints": [0.0, -1.57, 1.57, -1.57, -1.57, 0.0],
    "ik_seed": [-0.551, -0.429, 2.045, 3.094, -1.571, -2.122],
    "tool0_to_fingertip": 0.13, "approach_clearance": 0.05,
    "pick_zone": [0.30, 0.90, -0.18, 0.16, -0.45, -0.30],
    "place_slots": [0.44, 0.24, 0.60, 0.24, 0.76, 0.24,
                    0.44, -0.24, 0.60, -0.24, 0.76, -0.24],
    "place_z": -0.35, "place_release_lift": 0.01,
    "elbow_guard_enabled": True,
    "guard_links": ["forearm_link", "wrist_1_link"],
    "table_plane_z": -0.39, "guard_margin": 0.02,
    "gripper_backend": "sim", "sim_gripper_joint": "robotiq_85_left_knuckle_joint",
    "sim_gripper_open": 0.0, "sim_gripper_close": 0.8,
}
PARAM_NAMES = list(FALLBACK)


# MoveItErrorCodes 的数字 → 名字。运行期从 moveit_msgs 取（与板上装的版本一致），
# 取不到（老版本没有那些负数常量）就退回这份 Humble 实测的表。
# ⚠️ 这个映射有实际用途：-21（FRAME_TRANSFORM_FAILURE）和「服务不回话」是
#    **环境问题**（少 robot_state_publisher / 少 /joint_states），不是解不出来；
#    只有 -31（NO_IK_SOLUTION）才是「这个点位真不可达」。今天三者曾被一锅炖成
#    「IK 失败 44/44」，把无臂预检环境的问题误报成了 44 个不可达点位。
def _error_names():
    try:
        from moveit_msgs.msg import MoveItErrorCodes as E
        m = {v: k for k, v in vars(E).items()
             if k.isupper() and isinstance(v, int) and k != "SLOT_TYPES"}
        return m or _FALLBACK_ERROR_NAMES
    except Exception:
        return _FALLBACK_ERROR_NAMES


_FALLBACK_ERROR_NAMES = {
    1: "SUCCESS", -15: "INVALID_GROUP_NAME", -17: "INVALID_ROBOT_STATE",
    -18: "INVALID_LINK_NAME", -21: "FRAME_TRANSFORM_FAILURE",
    -23: "ROBOT_STATE_STALE", -31: "NO_IK_SOLUTION", 99999: "FAILURE",
}

ERROR_NAMES = _error_names()


def rpy_to_quat(rpy):
    """与 planner_node.rpy_to_quat 逐字一致 —— 位姿必须和规划器发的一模一样。"""
    cr, sr = math.cos(rpy[0] / 2), math.sin(rpy[0] / 2)
    cp, sp = math.cos(rpy[1] / 2), math.sin(rpy[1] / 2)
    cy, sy = math.cos(rpy[2] / 2), math.sin(rpy[2] / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def val(rs, i, default):
    """从 rcl_interfaces/ParameterValue 里取值。

    ⚠️ 必须按 `type` 分支读对应的 `*_value` 字段：直接读 `double_value` 对
    INTEGER 参数永远返回 0.0，而**不报错** —— 一个静默的假数据源。
    """
    t = rs.values[i].type
    if t == 1:      # BOOL
        return rs.values[i].bool_value
    if t == 2:      # INTEGER
        return int(rs.values[i].integer_value)
    if t == 3:      # DOUBLE
        return float(rs.values[i].double_value)
    if t == 4:      # STRING
        return rs.values[i].string_value
    if t == 5:      # BYTE_ARRAY
        return list(rs.values[i].byte_array_value)
    if t == 6:      # BOOL_ARRAY
        return list(rs.values[i].bool_array_value)
    if t == 7:      # INTEGER_ARRAY
        return [int(v) for v in rs.values[i].integer_array_value]
    if t == 8:      # DOUBLE_ARRAY
        return [float(v) for v in rs.values[i].double_array_value]
    if t == 9:      # STRING_ARRAY
        return list(rs.values[i].string_array_value)
    return default


class Probe(Node):
    def __init__(self, target_node, ik_group=None):
        super().__init__("elbow_guard_probe")
        self.target = target_node
        # 覆盖规划组名。**没有 /grasp_planner 可读参数时必须给**：兜底默认是仿真
        # 的 ur5_manipulator，而真机链路的组名是 ur_manipulator（ur_moveit_config
        # 硬编码 name:=ur），用错会得到 44/44 的 INVALID_GROUP_NAME（实测，2026-10-02）。
        self.ik_group_override = ik_group
        self.pget = self.create_client(GetParameters, f"{target_node}/get_parameters")
        self.ik = self.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.create_client(GetPositionFK, "/compute_fk")
        self.scene = self.create_client(GetPlanningScene, "/get_planning_scene")
        self._model_joints = None      # 只查一次；None = 还没查，set() = 查不到
        self._warned_gripper = False   # 「模型里没这个关节」只提示一次

    # ---- 模型里到底有哪些关节 ----
    def model_joint_names(self):
        """问 move_group 要**它自己模型里**的关节名集合（失败返回空集）。

        ⚠️ 这不是锦上添花，是一条**血换来的**护栏。IK 请求里的 robot_state 若
        带了模型不认识的关节名，move_group **不是**忽略它，而是抛
        `moveit::Exception`：「Variable 'robotiq_85_left_knuckle_joint' is not
        known to model 'ur'」，没人接 ⇒ `std::terminate` ⇒ **SIGABRT 整个进程
        死掉**（实测，2026-10-02）。之后所有 IK 调用都变成「服务未应答」，
        而本脚本原来会把「一个位姿都没解出来」报成**退出码 0 = 通过**。

        所以：**宁可不给夹爪值，也不给错的名字**。查不到就返回空集，
        调用方据此**不注入**夹爪关节。
        """
        if self._model_joints is not None:
            return self._model_joints
        self._model_joints = set()
        if not self.scene.wait_for_service(timeout_sec=5.0):
            print("⚠️ /get_planning_scene 不在 —— 无法确认模型关节名，"
                  "本次**不给 IK 请求注入夹爪关节**（注入错名字会让 move_group 崩）。")
            return self._model_joints
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ROBOT_STATE
        fut = self.scene.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        if not fut.done() or fut.result() is None:
            print("⚠️ /get_planning_scene 没回话 —— 同上，不注入夹爪关节。")
            return self._model_joints
        self._model_joints = set(fut.result().scene.robot_state.joint_state.name)
        return self._model_joints

    # ---- 读运行中节点的参数 ----
    def read_params(self):
        if not self.pget.wait_for_service(timeout_sec=5.0):
            print(f"⚠️ {self.target} 的 get_parameters 不可用 —— 用兜底默认值。\n"
                  "   （节点没起？本脚本量的就不是它将要走的位姿了。）")
            return self._apply_override(dict(FALLBACK))
        req = GetParameters.Request()
        req.names = PARAM_NAMES
        fut = self.pget.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        if not fut.done() or fut.result() is None:
            print("⚠️ 读参数超时 —— 用兜底默认值。")
            return self._apply_override(dict(FALLBACK))
        rs = fut.result()
        p = {}
        for i, name in enumerate(PARAM_NAMES):
            v = val(rs, i, FALLBACK[name])
            # 未声明的参数回的是默认值（空数组/0），此时用兜底
            if v in ([], 0, 0.0, "") and FALLBACK[name] not in ([], 0, 0.0, ""):
                v = FALLBACK[name]
            p[name] = v
        return self._apply_override(p)

    def _apply_override(self, p):
        """把命令行覆盖叠到参数上（目前只有规划组名）。

        两条取参数的路径（真的读到 / 用兜底）**都要**过这里，否则
        「读了参数但组名读错」这条路会绕过覆盖。
        """
        if self.ik_group_override:
            p["ik_group"] = self.ik_group_override
        return p

    # ---- IK / FK ----
    def solve_ik(self, p, x, y, z, seed, grip):
        """返回 ``(解或 None, 失败原因或 None)``。

        ⚠️ 「服务没回话」和「回了但解不出来」必须分开报：前者是
        move_group 的环境问题（见 main 里的提示），后者才是几何问题。
        """
        r = GetPositionIK.Request().ik_request
        r.group_name = p["ik_group"]
        r.ik_link_name = p["ik_link"]
        r.pose_stamped.header.frame_id = p["base_frame"]
        r.pose_stamped.pose.position.x = float(x)
        r.pose_stamped.pose.position.y = float(y)
        r.pose_stamped.pose.position.z = float(z)
        (r.pose_stamped.pose.orientation.x, r.pose_stamped.pose.orientation.y,
         r.pose_stamped.pose.orientation.z,
         r.pose_stamped.pose.orientation.w) = rpy_to_quat(p["grasp_rpy"])
        r.avoid_collisions = True
        r.timeout.sec = int(max(1.0, p["ik_timeout"]))
        # ⚠️ planner_node 只放 6 个臂关节，夹爪由 MoveIt 从**当前状态**补齐。
        #    本探针改为**显式**给夹爪值 —— 这样「张开」与「闭合」两种情形都能测，
        #    而且结果与臂的当前状态无关（可重复）。`place_release_lift` 那一整坑
        #    正是「只有闭合时（手里夹着东西）才发作」。
        #
        # ⚠️ 先在**普通 list** 上拼好再一次性赋值：ROS 2 的 `double[]` 字段在
        #    Python 侧是 `array.array('d')`，赋值后再 `+ [x]` 会抛
        #    「can only append array (not "list") to array」。
        names = list(p["arm_joint_names"])
        pos = [float(v) for v in seed]
        # ⚠️ 只在**确认模型认识这个关节**之后才追加它。否则 move_group 会抛
        # `moveit::Exception` 并**整个进程 SIGABRT**（见 model_joint_names 的
        # docstring）—— 一个探测工具把被测服务打死，还报成「通过」。
        if p["gripper_backend"] == "sim" and grip is not None:
            if p["sim_gripper_joint"] in self.model_joint_names():
                names.append(p["sim_gripper_joint"])
                pos.append(float(grip))
            else:
                if not self._warned_gripper:
                    print(f"ℹ️ 模型里没有 {p['sim_gripper_joint']}"
                          f"（这是**仿真**的夹爪关节，真机 URDF 里不存在）——"
                          f"本次按「只给 6 个臂关节」解，和 planner_node 的做法一致。")
                    self._warned_gripper = True
        js = JointState()
        js.name = names
        js.position = pos
        r.robot_state.joint_state = js
        fut = self.ik.call_async(GetPositionIK.Request(ik_request=r))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        if not fut.done() or fut.result() is None:
            return None, "服务未应答(15s)"
        resp = fut.result()
        if resp.error_code.val != 1:
            return None, ERROR_NAMES.get(resp.error_code.val,
                                         f"error_code={resp.error_code.val}")
        sol = dict(zip(resp.solution.joint_state.name,
                       resp.solution.joint_state.position))
        return [sol[n] for n in p["arm_joint_names"]], None

    def fk_heights(self, p, angles, links):
        req = GetPositionFK.Request()
        req.header.frame_id = p["base_frame"]
        req.fk_link_names = list(links)
        js = JointState()
        js.name = list(p["arm_joint_names"])
        js.position = [float(v) for v in angles]
        req.robot_state.joint_state = js
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        if not fut.done() or fut.result() is None:
            return {}
        resp = fut.result()
        return {n: ps.pose.position.z
                for n, ps in zip(resp.fk_link_names, resp.pose_stamped)}


def build_cases(p, grid, stress=False):
    """按**规划器自己的参数**生成候选位姿。

    抓取段的下降 z 取 **`place_z`** —— 这不是偷懒：`place_z` 的语义就是
    「方块静止时**可见顶面**的高度」（见 grasp_planner.yaml 的那一大段推导），
    而检测报出来的 wz 也是顶面高度（俯视深度相机看到的正是顶面）。所以
    `place_z` 就是**规划器实际会下发的那个抓取 z**，用它采样才量得到真位姿。

    ⚠️ 别拿 `pick_zone` 的 zmin/zmax 当默认取样点：那是**矩形边界**，不是
    可达位姿。仿真里 zmin=-0.45 → tool0=-0.32，实测 12/28 全解不出来 ——
    而方块根本不在那个高度。那会把「区域框得比可达范围大」误报成护栏问题。
    要压这个边界用 ``--stress``。

    放置段严格照 planner_node 的算法：
        下降 tool0 z = place_z + tool0_to_fingertip + place_release_lift
        接近 tool0 z = 上式 + approach_clearance
    """
    ttf, clear = p["tool0_to_fingertip"], p["approach_clearance"]
    xmin, xmax, ymin, ymax, zmin, zmax = p["pick_zone"]
    zs = [(p["place_z"], "顶面")]
    if stress:
        zs += [(zmin, "低界"), (zmax, "高界")]
    cases = []
    for i in range(grid):
        for j in range(grid):
            x = xmin + (xmax - xmin) * (i / max(1, grid - 1))
            y = ymin + (ymax - ymin) * (j / max(1, grid - 1))
            for z, tag in zs:
                cases.append((f"抓取下降 {tag} ({x:.2f},{y:+.2f})", x, y,
                              z + ttf, p["sim_gripper_open"]))
                cases.append((f"抓取接近 {tag} ({x:.2f},{y:+.2f})", x, y,
                              z + ttf + clear, p["sim_gripper_open"]))
    slots = p["place_slots"]
    for k in range(0, len(slots) - 1, 2):
        sx, sy = slots[k], slots[k + 1]
        z = p["place_z"] + ttf + p["place_release_lift"]
        cases.append((f"放置下降 #{k//2} ({sx:.2f},{sy:+.2f})", sx, sy, z,
                      p["sim_gripper_close"]))
        cases.append((f"放置接近 #{k//2} ({sx:.2f},{sy:+.2f})", sx, sy,
                      z + clear, p["sim_gripper_close"]))
    return cases


def main():
    ap = argparse.ArgumentParser(description="肘高预检：只读，不发轨迹")
    ap.add_argument("--node", default="/grasp_planner", help="规划器节点名")
    ap.add_argument("--grid", type=int, default=4,
                    help="pick_zone 内每个轴的采样数（默认 4×4）")
    ap.add_argument("--stress", action="store_true",
                    help="额外采样 pick_zone 的 z 上下界（区域边界，不是真位姿；"
                         "会解不出来一批，属预期）")
    ap.add_argument("--ik-group", default=None,
                    help="覆盖规划组名。**没有 /grasp_planner 可读参数时必须给** —— "
                         "兜底默认是仿真的 ur5_manipulator，而真机链路（"
                         "ur_moveit_config 硬编码 name:=ur）的组名是 ur_manipulator，"
                         "用错会得到 44/44 的 INVALID_GROUP_NAME（实测）。")
    args = ap.parse_args()

    rclpy.init()
    pr = Probe(args.node, ik_group=args.ik_group)
    try:
        if not pr.ik.wait_for_service(timeout_sec=10.0):
            print("!! /compute_ik 不可用 —— move_group 起了吗？")
            return 2
        if not pr.fk.wait_for_service(timeout_sec=10.0):
            print("!! /compute_fk 不可用")
            return 2

        p = pr.read_params()
        links = list(p["guard_links"])
        floor = p["table_plane_z"] + p["guard_margin"]

        print("=" * 78)
        print(f"肘高预检（只读）  目标节点 {args.node}")
        print(f"  规划组 {p['ik_group']} / 末端 {p['ik_link']} / 坐标系 {p['base_frame']}")
        print(f"  护栏 {'开' if p['elbow_guard_enabled'] else '**关**'}"
              f" | 连杆 {links} | 台面 {p['table_plane_z']:+.3f}"
              f" + 余量 {p['guard_margin']:.3f} ⇒ 下限 {floor:+.3f}")
        print("=" * 78)

        cases = build_cases(p, args.grid, stress=args.stress)
        print(f"{'候选位姿':<28}{'tool0 z':>9}{'夹爪':>6}   各连杆 z")
        worst, unsolved, rejected, reasons = {}, 0, [], {}
        for label, x, y, z, grip in cases:
            arm, why = pr.solve_ik(p, x, y, z, p["ik_seed"], grip)
            if arm is None:
                print(f"{label:<28}{z:>+9.3f}{grip:>6.2f}   ⛔ IK 失败"
                      f"（{why}）")
                unsolved += 1
                reasons[why] = reasons.get(why, 0) + 1
                continue
            zs = pr.fk_heights(p, arm, links)
            bad = [k for k, v in zs.items() if v < floor]
            if bad:
                rejected.append((label, {k: zs[k] for k in bad}))
            print(f"{label:<28}{z:>+9.3f}{grip:>6.2f}   " +
                  "  ".join(f"{k.replace('_link',''):>10}={v:+7.3f}" for k, v in zs.items())
                  + ("   🔴 低于护栏" if bad else ""))
            for k, v in zs.items():
                worst[k] = min(worst.get(k, 1e9), v)

        home = pr.fk_heights(p, p["home_joints"], links)
        print("\nHOME 位：" + "  ".join(f"{k.replace('_link','')}={v:+.3f}"
                                       for k, v in home.items()))

        print("\n" + "=" * 78)
        print("各连杆在以上所有候选位姿里的**最低**高度（这就是护栏的实际余量）：")
        for k, v in sorted(worst.items(), key=lambda kv: kv[1]):
            print(f"  {k:<16}{v:+.3f}   离护栏下限 {floor:+.3f} 还有 {v - floor:+.3f} m")
        lo = min(worst.values()) if worst else float("nan")
        home_lo = min(home.values()) if home else float("nan")

        print(f"\nIK 解不出来的候选：{unsolved} / {len(cases)}"
              + ("（这些点位规划器会跳过并告警，不会硬发）" if unsolved else ""))
        for why, cnt in sorted(reasons.items(), key=lambda kv: -kv[1]):
            print(f"     · {why}: {cnt} 个")
            if "未应答" in why:
                print("       ↳ 服务在但不回话：move_group 没有 /joint_states 时"
                      " CurrentStateMonitor 拿不到状态，/compute_ik 会**挂住**。"
                      "真机由 UR 驱动发 /joint_states；无臂预检要自己补"
                      "（假 /joint_states，见 scripts/moveit_only_check.py）。")
            elif why == "FRAME_TRANSFORM_FAILURE":
                print("       ↳ 请求帧（base_frame）的 TF 查不到：无臂预检时"
                      "**少 robot_state_publisher**（真机上由驱动顺带起）—— "
                      "不是解不出来。")
            elif why == "NO_IK_SOLUTION":
                pass  # 不可达；边界上的属预期，下面的区提示已经覆盖
            else:
                print("       ↳ 含义见 moveit_msgs/MoveItErrorCodes；"
                      "若集中在所有点位上，多半也是环境而不是几何。")
        if unsolved:
            # 区分「护栏问题」与「区框太大」：这是两种完全不同的处置。
            xs = [float(c[1]) for c in cases]
            ys = [float(c[2]) for c in cases]
            print(f"     ↳ 提示：若失败集中在 x={min(xs):.2f} / x={max(xs):.2f}、"
                  f"y={min(ys):+.2f} / y={max(ys):+.2f} 这类**区域边界**上，")
            print("       那是 pick_zone 框得比可达范围大（它只是个过滤器），"
                  "不是护栏问题 —— 把区收窄即可。")
        if rejected:
            print(f"🔴 有 {len(rejected)} 个候选解出的连杆**低于护栏**：")
            for label, bad in rejected[:8]:
                print(f"     {label}: " +
                      "  ".join(f"{k}={v:+.3f}" for k, v in bad.items()))
            if len(rejected) > 8:
                print(f"     …另有 {len(rejected)-8} 个")

        print("\n结论：")
        if not p["elbow_guard_enabled"]:
            print("  ⚠️ 护栏是**关**的！下面的结论请按「无护栏」理解 —— 现场上电前建议打开。")
        if rejected:
            print(f"  ⛔ **不要直接上电跑**：有 {len(rejected)} 个候选会低于护栏。")
            print("     先把 table_plane_z 按现场实测的**桌面高度**填对，再看一遍；")
            print("     仍然低就说明抓取区/放置区伸得太远或太靠边，要缩区。")
        elif math.isnan(lo):
            print("  ⚠️ 一个位姿都没解出来 —— 先查 ik_group 与 /compute_ik。")
        elif lo < floor:
            print(f"  ⛔ 最低连杆 {lo:+.3f} 低于下限 {floor:+.3f}，不要上电。")
        else:
            print(f"  ✅ 所有候选的连杆最低 {lo:+.3f}，都在护栏下限 {floor:+.3f} 之上"
                  f"（余量 {lo - floor:+.3f} m）。")
        if not math.isnan(home_lo) and home_lo < floor:
            print(f"  ⛔ 但 **HOME 位**的连杆只有 {home_lo:+.3f} < {floor:+.3f} —— "
                  "护栏会在初始化时就拦死自己，把 table_plane_z 填高了。")

        print("\n提醒：本脚本只证明「肘/大臂解在桌面之上」，它**不**替代")
        print("      · 桌面进 MoveIt 规划场景（W1b 第 2 项）")
        print("      · 单步低速 + 急停就绪的第一次真机动作")

        # 退出码。⚠️ 原来这里是 `return 1 if rejected else 0`，于是
        # **一个位姿都没解出来**（lo 是 NaN、rejected 空）时报的是 **0 = 通过** ——
        # 「什么都没量到」被报成「全部都在护栏之上」。本脚本是 W5 第 4b 步的闸门，
        # 而 moveit_only_check.py 又按退出码决定整个预检的成败，所以这个假绿会把
        # 4b 直接放过去（实测踩到：--no-pipeline 下 44/44 INVALID_GROUP_NAME + EXIT=0）。
        if rejected:
            return 1
        if math.isnan(lo):
            return 2
        # HOME 位低于护栏同样是一条「不要上电」的结论（护栏会在初始化拦死自己），
        # 原来只打印 ⛔、不影响退出码。
        if not math.isnan(home_lo) and home_lo < floor:
            return 1
        return 0
    finally:
        pr.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())

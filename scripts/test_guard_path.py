#!/usr/bin/env python3
"""
整段护栏自检（B）—— **不需要仿真器、不需要 MoveIt、不需要真机**
=================================================================
`planner_node` 的肘高护栏原来只查**路点本身**（`_posture_ok`）：解出来的那个
位姿过不过。但臂是从当前位姿**连续**走到目标的 —— 两端都在台面之上，中间照样
可以俯冲下去再抬起来。B 把检查扩到**整段**（`_path_ok`）：把「当前位置→目标」
的关节空间线段采样若干点，逐点查一遍。

这个自检要回答的核心问题只有一个：
**「整段检查」真的是拦下动作的那道闸门吗，还是别的什么东西碰巧也在拦？**

所以 A 段的关键是 **A1 与 A4 这一对**：
  A1  中段俯冲、两端都在台面上  → **拦下**
  A4  同一个位姿，只把 guard_path_samples 设成 0（退回「只查终点」）→ **放行**
两者只差那一个参数，结论相反 —— 这就把功劳**唯一地**归给了整段检查。
（少了 A4，A1 通过可能只是因为「有点不对劲就报错」，而不是采样真的在量中段。）

为什么不用真 FK
---------------
用真 /compute_fk 就得起 move_group，这个自检就再也不能在没有仿真的机器上跑。
所以 A 段**把 `_call_fk` 换成一个合成的非线性函数**：让某个「连杆」的高度是
关节角的二次函数。沿关节空间的直线插值时，高度走一条抛物线 —— 两端高、
中间低，正是真实机械臂会发生的俯冲的形状（真实 FK 也是非线性的）。
它不假装自己是 UR5，它只提供「已知答案的几何」来检采样逻辑。

⚠️ 这个自检起的是**一个真的 `GraspPlanner` 实例**，而它自带一个 2 秒的
「先开爪、再回 HOME」定时器 —— 也就是说**控制器在的时候它会指挥真机回 HOME**。
板上实测过（无臂栈里，日志见 `main()` 里那段注释）。所以 `main()` 在起完节点后
**立刻 cancel 那个定时器**，并把 `targets_topic` 指到一个没人发的话题；
停不掉就**中止**，不发任何一条轨迹。**要把本自检对着真机跑之前，先看那段注释。**

用法
----
    source /opt/ros/humble/setup.bash
    source ros2_ws/install/setup.bash
    python3 scripts/test_guard_path.py
退出码 0 = 全通过。
"""

import ast
import math
import os
import sys
import threading
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
NODE_SRC = (PROJECT_ROOT / "ros2_ws" / "src" / "grasp_planner" / "grasp_planner"
            / "planner_node.py")
YAML_SRC = (PROJECT_ROOT / "ros2_ws" / "src" / "grasp_planner" / "config"
            / "grasp_planner.yaml")

_failures = []


def check(cond, label, detail=""):
    if cond:
        print(f"  ✓ {label}")
    else:
        print(f"  ✗ {label}" + (f"\n      {detail}" if detail else ""))
        _failures.append(label)
    return cond


# ----------------------------------------------------------------------
# A. 采样数学：合成 FK，已知答案
# ----------------------------------------------------------------------
ELBOW_IDX = 2           # UR5 的固定顺序：pan, lift, elbow, wrist_1..3
E_MID = 1.0             # 合成抛物线的顶点，就在线段正中间
FLOOR = 0.20            # 台面+余量：端点 0.25 在它之上，中点 0.00 在它之下


# A 段为了造合成几何会改这些**共享的**节点属性。跑完必须逐个还原 —— 否则后面
# 的段会在被污染的状态上量，而且**是静默的**。
#
# 实测（2026-10-02，板端）：全跑 A/B/C/D 时，D 段拿到的是 A 留下的
# `guard_links = ["forearm_link"]`（单连杆），于是找不到「两端高、中间低」的
# 路径，打印「~ 跳过」然后**照样打「✓ 全部通过」**；而 `--only D` 能找到那条
# 路径并真的拦下它（两边的数字与开发机逐位一致）。
# ⇒ 同一个自检，**D 段的强度取决于它是被全跑还是被单跑**，而全跑那个绿是假的。
# 这正是本轮在查的那类病：**「没测到」和「测到了、是好的」长得一模一样。**
_MUTATED_BY_A = (
    "_call_fk", "guard_links", "table_plane_z", "guard_margin",
    "elbow_guard_enabled", "arm_joints", "_current_joints",
    "guard_path_samples",
)


def part_a(node):
    print("\n===== A. 整段采样：中段俯冲能不能被拦下 =====")
    before = len(_failures)

    saved_attrs = {a: getattr(node, a) for a in _MUTATED_BY_A}
    saved_warn = node.get_logger().warn
    try:
        _part_a_body(node)
    finally:
        # 中途抛异常也照样还原 —— 否则一次失败会顺着状态污染把后面的段带歪，
        # 把真正的失败信息埋掉。
        for a, v in saved_attrs.items():
            setattr(node, a, v)
        node.get_logger().warn = saved_warn
    return len(_failures) == before


def _part_a_body(node):
    if True:                                     # 只为保持原来的缩进层次
        # 合成几何：z(elbow) = (elbow - E_MID)² —— 非线性，所以沿关节空间
        # 直线插值时高度是抛物线。取线段端点 elbow = 0.5 / 1.5。
        calls = {"n": 0}

        def fake_fk(link, joints):
            calls["n"] += 1
            return (0.0, 0.0, (joints[ELBOW_IDX] - E_MID) ** 2)

        node._call_fk = fake_fk                    # D 段会把它换回真的
        node.guard_links = ["forearm_link"]
        node.table_plane_z = FLOOR
        node.guard_margin = 0.0
        node.elbow_guard_enabled = True
        node.arm_joints = ["shoulder_pan_joint", "shoulder_lift_joint",
                           "elbow_joint", "wrist_1_joint", "wrist_2_joint",
                           "wrist_3_joint"]
        start = [0.0] * 6
        start[ELBOW_IDX] = 0.5
        target = [0.0] * 6
        target[ELBOW_IDX] = 1.5
        node._current_joints = dict(zip(node.arm_joints, start))

        # 端点本身确实是过的 —— 先把这个前提钉死，否则 A1 拦下的可能是端点，
        # 整段检查根本没参与。
        check(node._posture_ok(target),
              "前提：终点单独查是通过的（所以下面拦下的一定是中段）",
              f"z(终点)={fake_fk('', target)[2]:.3f} floor={FLOOR:.3f}")

        # A1 整段检查开着 → 中段俯冲被拦
        node.guard_path_samples = 8
        calls["n"] = 0
        check(not node._path_ok(target),
              "A1 中段俯冲 → 拦下",
              "两端都过，中段却放行了")
        # 采样循环**命中即返回**，所以在 t=1/8 处就停了 —— 次数少是正常的，
        # 关键是它**不止查了端点**（端点 1 次 + 中段至少 1 次）。
        check(calls["n"] >= 2,
              f"A1 越过了端点、真的走进线段（调了 {calls['n']} 次 FK）",
              "只查了端点，整段检查没参与")

        # A2 一条全程在台面上的线段 → 放行（防止「护栏只会说不」），
        #    而且这里没有中途返回，所以能数出**整段确实走完了**。
        flat = [0.0] * 6
        flat[ELBOW_IDX] = E_MID + 1.0        # z = 1.0，从头到尾都在台面上
        node._current_joints = dict(zip(node.arm_joints, flat))
        calls["n"] = 0
        check(node._path_ok(list(flat)),
              "A2 全程都在台面之上 → 放行")
        check(calls["n"] == node.guard_path_samples + 1,
              f"A2 采样点数是 guard_path_samples+1=({node.guard_path_samples}+1)"
              f"（实际 {calls['n']} 次 FK）",
              "采样数不对，参数没生效")

        # A3 终点本身就在台面之下 → 也要拦（且不该靠中段来发现）。
        #    z(e) = (e−1)² 关于 e=1 对称，所以「低」是靠近 1 而不是远离 1：
        #    e=0.9 → z=0.01 < floor 0.20，而起点 e=0.5 → z=0.25 是过的。
        node._current_joints = dict(zip(node.arm_joints, start))
        low_end = [0.0] * 6
        low_end[ELBOW_IDX] = 0.9
        check(not node._path_ok(low_end),
              "A3 终点就在台面之下 → 拦下",
              f"z(终点)={fake_fk('', low_end)[2]:.3f} 应低于 floor={FLOOR:.3f}")

        # A4 ★ 同一个位姿，只关掉整段检查 → 放行。
        #    这一条证明「拦下」是整段采样的功劳，不是别的什么碰巧也在拦。
        node._current_joints = dict(zip(node.arm_joints, start))
        node.guard_path_samples = 0
        check(node._path_ok(target),
              "★ A4 同一个中段俯冲，guard_path_samples=0 → 放行"
              "（证明闸门就是整段采样）",
              "退回「只查终点」后仍被拦，说明拦它的另有其人")

        # A5 护栏整个关掉 → 什么都不拦
        node.guard_path_samples = 8
        node.elbow_guard_enabled = False
        check(node._path_ok(target), "A5 elbow_guard_enabled=False → 放行")
        node.elbow_guard_enabled = True

        # A6 还拿不到 /joint_states（起点未知）→ 退回端点检查：终点是过的，
        #    所以放行，但**必须出声**（不静默降级）。
        logs = []
        node.get_logger().warn = lambda msg, **kw: logs.append(str(msg))
        node._current_joints = {}
        check(node._path_ok(target),
              "A6 拿不到当前关节角 → 退回端点检查并放行")
        check(any("/joint_states" in m for m in logs),
              "A6 且**打了警告**（降级不静默）",
              f"实际日志 {logs}")

        # A7 FK 查不通 → fail-closed 拒绝（与 _posture_ok 同一条纪律）
        node._current_joints = dict(zip(node.arm_joints, start))
        node._call_fk = lambda link, joints: None
        check(not node._path_ok(target),
              "A7 FK 查不通 → 拒绝（fail-closed）")

        # A8 关节角要「展开到最近等价角」再采样：目标若是 0.5+2π 而当前是 0.5，
        #    两条路（原地不动 vs 转一整圈）在关节空间里是完全不同的线段。
        #    展开之后应当判成「原地不动」，全程都在台面上 → 放行。
        node._call_fk = fake_fk
        node._current_joints = dict(zip(node.arm_joints, flat))
        wrap = list(flat)
        wrap[ELBOW_IDX] = flat[ELBOW_IDX] + 2 * math.pi
        check(node._path_ok(wrap),
              "A8 目标写成 +2π 的等价角 → 按原地不动判（不做一圈大回转）")


# ----------------------------------------------------------------------
# D. 接真 FK：在真 UR5 模型上，整段检查真的会开火吗
# ----------------------------------------------------------------------
# 「中段俯冲」这件事不是合成出来骗自己的 —— 真机械臂沿关节空间直线运动时，
# 连杆高度是关节角的非线性函数，两端高、中间低是**会真发生**的。D 段就在真
# UR5 的 URDF 上把这种路径**找出来**，再看护栏拦不拦。
#
# 判据刻意做成「先测量、再设台面」而不是「先设台面、再看结果」：
#   1. 沿路径密采样，量出各点的最低连杆高度；
#   2. 要求**最低点严格在路径内部**（不在两端）—— 否则就不是「整段」的功劳；
#   3. 把台面设在「比两端低、比最低点高」的那条缝里；
#   4. 此时两个**端点单独查都必须通过**，而 `_path_ok` 必须拒绝。
# 这样结论就唯一地归给整段检查，而不是「台面设歪了」。
#
# 需要 /compute_fk 在。它不在就**跳过**（不算失败）—— 这个自检在没起 MoveIt
# 的机器上照样要能跑。
def part_d(node, fk, pristine_guard_links):
    print("\n===== D. 真 UR5 模型上的整段检查（要 /compute_fk，没有就跳过）=====")
    before = len(_failures)

    if not node._fk_client.wait_for_service(timeout_sec=5.0):
        print("  ~ 跳过：/compute_fk 不在。先 `python3 scripts/moveit_only_check.py`"
              " 把只读链路起起来，再跑本段。")
        return True

    # ⚠️ A 段把 _call_fk 打了桩，这里必须换回来 —— 否则本段量的是合成几何，
    # 却顶着「真 UR5 模型」的标题，是一个**看起来很硬、其实什么都没验**的绿。
    node._call_fk = fk

    # ★ 先钉死「量的是哪几个连杆」。A 段会改 `guard_links`，若它没还原干净，
    #   本段就会在**单连杆**几何上找不到俯冲路径 → 「跳过」→ 「全部通过」。
    #   这是一个**沉默的**假绿，所以这里宁可直接判失败，也不去打那个勾。
    #   （正常路径下 part_a 的 finally 已经还原，这一条是兜底断言。）
    if sorted(node.guard_links) != sorted(pristine_guard_links):
        check(False, "D 前置：护栏量的是原装的 guard_links",
              f"拿到 {node.guard_links}，原装 {pristine_guard_links} ——"
              f" 状态被前面的段污染了，本段不测（否则会静默跳过并报绿）")
        return False

    links = list(node.guard_links)
    arm = list(node.arm_joints)

    def min_z(q):
        """这条位姿里，护栏连杆的最低高度；查不到返回 None。"""
        zs = []
        for link in links:
            p = fk(link, q)
            if p is None:
                return None
            zs.append(p[2])
        return min(zs)

    # 起点：规划器的 HOME。候选终点：从 HOME 出发把几个关节分别掰到别处 ——
    # 全是**随手挑的姿态**，不是「找一条会撞的路径」，而是「看这些普通动作里
    # 有没有本来就会俯冲的」。找到了就说明这个风险是日常的，不是构造出来的。
    start = list(node.home_joints)
    cands = []
    for idx in range(len(arm)):
        for delta in (1.2, 1.8, -1.2, -1.8, 2.4):
            q = list(start)
            q[idx] = q[idx] + delta
            cands.append((f"{arm[idx]}{delta:+.1f}", q))

    N = 40              # 稠密采样（只为找最低点，不代表护栏的采样密度）
    found = None
    for label, end in cands:
        z0, z1 = min_z(start), min_z(end)
        if z0 is None or z1 is None:
            continue
        prof = [min_z([s + (e - s) * (i / N) for s, e in zip(start, end)])
                for i in range(N + 1)]
        if any(v is None for v in prof):
            continue
        zmin = min(prof)
        argmin = prof.index(zmin)
        # 最低点必须在**内部**，且比两端都低出一截（否则谈不上「俯冲」）
        if 0 < argmin < N and zmin < min(z0, z1) - 0.01:
            found = (label, end, z0, z1, zmin, argmin / N)
            break

    if found is None:
        print(f"  ~ 跳过：{len(cands)} 条候选里没找到「两端高、中间低」的路径 ——"
              " 不是失败，只是这一组姿态没触发。")
        return True

    label, end, z0, z1, zmin, t_min = found
    print(f"  找到一条真的会俯冲的路径：{label}")
    print(f"    起点最低连杆 z={z0:.3f}  终点 z={z1:.3f}  "
          f"中段最低 z={zmin:.3f}（在 {t_min * 100:.0f}% 处）")

    # 台面设进那条缝：比两端低 5 mm，比最低点高 5 mm
    floor = zmin + 0.005
    node.table_plane_z = floor
    node.guard_margin = 0.0
    node.elbow_guard_enabled = True
    node.arm_joints = arm
    node._current_joints = dict(zip(arm, start))
    tgt = [float(v) for v in end]

    # D0：把「护栏自己算出来的高度」与上面独立量出来的对一遍。
    # 这一条专门防一种最难看的假绿 —— A 段打的桩没撤干净，于是本段顶着
    # 「真 UR5 模型」的标题，量的却是合成几何。
    check(abs(min_z(start) - z0) < 1e-9 and abs(min_z(tgt) - z1) < 1e-9,
          "D0 护栏走的是真 FK（与独立量出的高度逐位一致，没留着 A 段的桩）",
          f"复测 z0={min_z(start):.6f} z1={min_z(tgt):.6f}")

    check(node._posture_ok(start), "D1 起点单独查是通过的（台面设在缝里）",
          f"z={z0:.3f} floor={floor:.3f}")
    check(node._posture_ok(tgt), "D2 终点单独查是通过的",
          f"z={z1:.3f} floor={floor:.3f}")

    node.guard_path_samples = 40        # 密到足以看见那个缝
    t0 = time.monotonic()
    blocked = not node._path_ok(tgt)
    dt = time.monotonic() - t0
    # 代价是要看的：护栏现在是每个路点都要跑一遍的，`/compute_fk` 每次
    # 都是往 move_group 的一次同步往返。这里量出来的是 41 点 × 2 连杆，
    # 按默认的 8 点算是它的 1/5 左右。
    n_pts, n_links = 41, len(links)
    per_pt = dt / n_pts
    print(f"  整段检查的代价：{n_pts} 点 × {n_links} 连杆 = {n_pts * n_links} 次 "
          f"/compute_fk 用了 {dt:.2f}s（≈{per_pt * 1000:.0f} ms/点）"
          f"；默认 8 点（9 个采样）约 {per_pt * 9:.2f}s/次机械臂动作")
    check(blocked,
          "★ D3 整段检查拦下了这条真路径（端点都过、中段不过）",
          "真 UR5 上没拦住 —— B 没起作用")

    node.guard_path_samples = 0
    check(node._path_ok(tgt),
          "★ D4 同一条路径，guard_path_samples=0 → 放行"
          "（再一次把功劳唯一归给整段采样）",
          "退回端点检查后仍被拦，说明拦它的是别的")

    return len(_failures) == before


# ----------------------------------------------------------------------
# B. 接线：护栏挂在「机械臂的唯一出口」上，且不碰夹爪那条路
# ----------------------------------------------------------------------
def part_b():
    print("\n===== B. 接线：护栏在哪一层，夹爪会不会被误伤 =====")
    before = len(_failures)

    if not NODE_SRC.is_file():
        check(False, "planner_node.py 存在", str(NODE_SRC))
        return False

    src = NODE_SRC.read_text(encoding="utf-8")
    tree = ast.parse(src)
    funcs = {n.name: n for n in ast.walk(tree)
             if isinstance(n, ast.FunctionDef)}

    def calls_in(fname):
        """函数体里出现过的 `self.xxx(...)` 方法名集合。"""
        node = funcs.get(fname)
        if node is None:
            return set()
        out = set()
        for sub in ast.walk(node):
            if (isinstance(sub, ast.Call)
                    and isinstance(sub.func, ast.Attribute)
                    and isinstance(sub.func.value, ast.Name)
                    and sub.func.value.id == "self"):
                out.add(sub.func.attr)
        return out

    arm = calls_in("_send_arm_trajectory")
    low = calls_in("_send_trajectory")

    check("_path_ok" in arm,
          "_send_arm_trajectory 调了 _path_ok（护栏在机械臂的唯一出口上）",
          f"实际调用 {sorted(arm)}")
    check("_path_ok" not in low,
          "‼ _send_trajectory **没有**调 _path_ok —— 它被仿真夹爪后端复用，"
          "护栏下移会让夹爪一动不动",
          f"实际调用 {sorted(low)}")

    # 所有机械臂动作都要经过 _send_arm_trajectory，不能有绕过它的地方
    bypass = []
    for name, node in funcs.items():
        if name in ("_send_arm_trajectory", "_send_trajectory"):
            continue
        if "_send_trajectory" in calls_in(name):
            bypass.append(name)
    check(not bypass,
          "没有别的函数直接调 _send_trajectory（否则就是绕过护栏的洞）",
          f"绕过者：{bypass}")

    return len(_failures) == before


# ----------------------------------------------------------------------
# C. 参数镜像：节点默认值与 yaml 不能各说各话
# ----------------------------------------------------------------------
def part_c():
    print("\n===== C. 参数：guard_path_samples 两边都有 =====")
    before = len(_failures)

    for path, label in ((NODE_SRC, "planner_node.py"), (YAML_SRC, "yaml")):
        text = path.read_text(encoding="utf-8") if path.is_file() else ""
        check("guard_path_samples" in text, f"{label} 里有 guard_path_samples")

    return len(_failures) == before


def main():
    import argparse

    ap = argparse.ArgumentParser(description="整段护栏自检（B）")
    ap.add_argument("--only", choices=["A", "B", "C", "D"], default=None,
                    help="只跑一段（D 需要 /compute_fk 在）")
    args = ap.parse_args()

    print(__doc__.strip().splitlines()[1])
    ok = True
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from grasp_planner.planner_node import GraspPlanner

    # 起一个**真的** GraspPlanner（`gripper_backend:=none`，不碰硬件）。
    # A/D 两段共用这一个实例 —— rclpy 在一个进程里 init 两次不可靠。
    #
    # `targets_topic` 指到一个**没人发**的话题：这个实例订阅的是真话题的话，
    # 场上有感知在跑时它会真的去规划并执行抓取。
    rclpy.init(args=["--ros-args",
                     "-p", "gripper_backend:=none",
                     "-p", "carry_objects:=false",
                     "-p", "targets_topic:=/precheck_guard_path_no_targets"])
    node = GraspPlanner()
    real_fk = node._call_fk          # A 段会打桩，D 段要用回真的
    # D 段拿它当「原装值」，用来断言前面的段没把状态漏过来（见 part_d 的说明）
    pristine_guard_links = list(node.guard_links)

    # ⚠️⚠️ **这个实例必须一条轨迹都发不出去。**
    #
    # `GraspPlanner.__init__` 里有一个 2 秒的定时器，到点就走「先张开夹爪、
    # 再回 HOME」——而 `_send_arm_trajectory` 只要求轨迹 action 服务在
    # （`wait_for_server(5s)`），**不问它后面是不是仿真**。板上实测
    # （2026-10-03，在无臂栈里跑本自检）：
    #
    #     [INFO]  初始化：先张开夹爪，再回 HOME ...
    #     [INFO]  夹爪后端 none（未配置夹爪硬件）：跳过「张开」——不发任何夹爪指令
    #     [ERROR] 轨迹 action 服务不可用（控制器起了吗？）
    #     [ERROR] 初始 HOME失败：goal_status=<未知 None> error_code=None
    #
    # 少了控制器只是「发失败」；**控制器在的时候它就会真的动** —— 也就是说
    # 一个开头就写明「不需要仿真器、不需要 MoveIt、不需要真机」的几何预检，
    # 会指挥真机回 HOME。这不是假想：上面那几行就是它的日志。
    #
    # 执行器还没转（spin 线程在下面才起），所以这里 cancel 是确定性的。
    # 拿不到就**直接中止** —— 宁可不跑预检，也不发一条真轨迹。
    home_timer = getattr(node, "_home_timer", None)
    if home_timer is None:
        print("\n✗ 停不掉「初始回 HOME」定时器（planner_node 的内部结构变了？）\n"
              "  —— 本次预检可能发出真实的 HOME 轨迹，已中止。\n"
              "  请先确认控制器状态，再决定要不要继续。")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 1
    home_timer.cancel()

    # ⚠️ 必须**转**这个节点。`_call_fk` 是「投出去 → 用 threading.Event().wait()
    # 阻塞等 future」的写法，它自己**不转执行器**，靠的是**别处**在 spin
    # （真节点是 rclpy.spin）。本自检不转的话，响应永远没人处理，症状是
    # 每条 FK 都「请求超时（5s）」—— 看起来像 MoveIt 坏了，其实是测试台架少了
    # 一个线程。A/B/C 三段不需要它（A 用的是打桩的 FK），D 段需要。
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    spin_thread = threading.Thread(target=ex.spin, daemon=True)
    spin_thread.start()
    try:
        if args.only in (None, "A"):
            ok = part_a(node) and ok
        if args.only in (None, "B"):
            ok = part_b() and ok
        if args.only in (None, "C"):
            ok = part_c() and ok
        if args.only in (None, "D"):
            ok = part_d(node, real_fk, pristine_guard_links) and ok
    finally:
        # rclpy 的 SIGINT/SIGTERM 处理器会自己调 shutdown，所以这里必须挡一道
        # （与仓内其它节点同一个写法）：否则信号一到，收尾反而抛 RCLError，
        # 把真正的失败信息盖掉。
        # 顺序要紧：先停执行器、**等那根线程真的退出**，再销毁节点。
        # 少了 join 就会在解释器收尾时 `terminate called without an active
        # exception`（SIGABRT，退出码 134）—— 判定明明全过，退出码却是失败，
        # 而上层 `moveit_only_check.py` 正是看退出码的。
        ex.shutdown()
        spin_thread.join(timeout=5.0)
        ex.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print()
    if _failures:
        print(f"✗ {len(_failures)} 项未通过：")
        for f in _failures:
            print(f"    - {f}")
        return 1
    print("✓ 全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())

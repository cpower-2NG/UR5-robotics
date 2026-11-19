#!/usr/bin/env python3
"""E9 自主移遮挡：**规划级只读探针**（只算 / 只规划，不动臂）。

契约（阶段 4 入口 E9）：自主移遮挡 —— S4 全桌合法终态率 ≥80%。本探针只做
**仿真半段**：不调 ``/move_action``、不写夹爪寄存器、不发轨迹；臂不会动。

一次尝试（一个布局 × 一个指定目标块）分四步，**全部是规划/几何**：

1. **目标可抓否**：用 E7 的 B 档口径（竖直向下，4 个候选位姿）评测目标块。
2. **清障**：有两类「遮挡」，处理顺序固定 —— **先「压在目标上的块」，再「挡住
   夹爪的块」**。前者不是夹爪碰不到，而是一动目标它就会掉（终态必然「悬空」），
   所以必须**先**把它挪走；后者按「挡住的候选位姿数」从多到少挑。对每个要挪的块
   **先确保它自己可抓**（递归：它也可能被别的块挡住 / 压着），再找一个自由区落点
   （桌上、拾取区与垫之外、离现有块 ≥5 mm），用 IK + 碰撞核验「能把它放到那儿」。
   挪走后再重测目标，直到「可抓 ∧ 顶面无块」或无法推进（深度/步数上限 → 本次失败）。
3. **放置位姿**：把目标块放到指定宫格格心（垫面上），核验机器人能在那个位姿
   接近/放下它（同一套竖直抓取几何 —— 放下的运动是接近的逆过程）。
4. **终态合法**：契约 §4.3 的「所有块在合法格 / 自由区，不重叠、不落地、不越界」，
   由纯几何的 ``stage4_clear_model.terminal_report`` 判（**接触合法、穿插非法**）。

成功 = 存在清障方案 ∧ 目标可抓 ∧ 放置位姿合法 ∧ 终态合法。

**独立性边界**（照抄 E7/E8）：夹爪是三个 box，几何自己算；UR5 臂杆是 mesh，
「臂被挡」只能借 MoveIt 的接触报告；**不验接近/退出路径**（E8/M2 的事），
本探针只验「抓取位姿 / 放置位姿本身」。

**不在这里发明的几何**：宫格、垫、拾取区、桌面尺寸全部取自
``stage4_clear_model``（它又反查 ``gen_scene.py`` 的源码），自检见
``test_stage4_clear_model.py``。

用法（仿真栈已起，只为拿 move_group）::

    python3 scripts/stage4_clear_probe.py \
        --truth /tmp/grasp_scene_S4_0/scene_truth.json \
        --truth /tmp/grasp_scene_S4_1/scene_truth.json ... \
        --report runs/2026-11-08-stage4-entry/e9/e9-sim.json

同一套逻辑在真机上也能跑（只调服务、不动臂），但**真机半段**的「不丢、不掉、不撞」
要实际执行，不在本探针里。
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

import rclpy                                                  # noqa: E402
import stage4_clear_model as cm                               # noqa: E402
import stage4_clear_run as cr                                 # noqa: E402
import stage4_moveit_joints as mj                             # noqa: E402
import stage4_moveit_probe as mp                              # noqa: E402
import stage4_scene_model as sm                               # noqa: E402
import stage4_workspace_probe as wp                           # noqa: E402
from geometry_msgs.msg import Pose                           # noqa: E402
from moveit_msgs.msg import CollisionObject                   # noqa: E402
from moveit_msgs.srv import (ApplyPlanningScene,              # noqa: E402
                             GetPlanningScene, GetPositionFK,
                             GetPositionIK, GetStateValidity)
from rclpy.node import Node                                   # noqa: E402
from shape_msgs.msg import SolidPrimitive                     # noqa: E402

GATE = 0.80          # 契约：S4 全桌合法终态率 ≥80%
MAX_DEPTH = 3        # 递归清障深度上限（S4 最多叠 2 层，3 足够且能兜住环）
MAX_CLEAR = 6        # 每层最多「挪走一块 → 重测」的次数
DEST_SCAN = 6        # 每个遮挡块最多试几个几何上可行的落点（IK 失败就换下一个）
#: 探测「指 vs 邻块」间隙用的夹爪半开量（米）= **执行接近开度**。
#: 2026-11-19 S2:2 tan 对实证：探针按全开 47.5 mm 算 partner 还剩 2.7 mm，
#: 执行却按 pick_open 50 mm 接近（半开 25 mm），指尖已伸进 partner 约 10 mm
#: ——下降即撞块。探针必须和执行同一开度。
APPROACH_OPEN_HALF_M = cr.PICK_OPEN_MM / 2000.0
PAD_ID = "pad"


def log(*a):
    print(*a, flush=True)


# ---------------------------------------------------------------- 场景物件
def make_pad_object(frame=mp.BASE):
    """放置垫的碰撞代理（外框 × 实测垫厚，按 gen_scene 的垫位姿）。

    仿真世界里有这块垫，但 E7 的 ``make_table_object`` 只是个 2×2 的桌面代理，
    规划场景里**没有垫** —— 放置位姿要把垫算进去，否则「贴垫放置」会和现实分家。
    """
    obj = CollisionObject()
    obj.header.frame_id = frame
    obj.id = PAD_ID
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    # ⚠️ 每个值都过 float()：垫参数未实测时 cm 里是 `_UnmeasuredPad` 哨兵，
    # 直接赋给 ROS msg 字段**不触发任何数值运算** ⇒ nan 静默进消息
    # （2026-11-14 审计：真正的拦截当时只靠后面 pad_obb 的除法与调用顺序）。
    # float() 走哨兵的 __float__ ⇒ 当场抛 PadNotMeasured，结构上防呆。
    box.dimensions = [float(cm.PAD_FOOTPRINT[0]), float(cm.PAD_FOOTPRINT[1]),
                      float(cm.PAD_THICK)]
    pose = Pose()
    pose.position.x, pose.position.y = (float(cm.PAD_CENTER[0]),
                                        float(cm.PAD_CENTER[1]))
    pose.position.z = float(cm.PAD_THICK) / 2.0
    pose.orientation.z = math.sin(float(cm.PAD_YAW_RAD) / 2.0)
    pose.orientation.w = math.cos(float(cm.PAD_YAW_RAD) / 2.0)
    obj.primitives = [box]
    obj.primitive_poses = [pose]
    obj.operation = CollisionObject.ADD
    return obj


def pad_obb():
    return sm.Obb.from_pose(
        [cm.PAD_CENTER[0], cm.PAD_CENTER[1], cm.PAD_THICK / 2.0],
        [cm.PAD_FOOTPRINT[0], cm.PAD_FOOTPRINT[1], cm.PAD_THICK],
        (0.0, 0.0, math.sin(cm.PAD_YAW_RAD / 2.0),
         math.cos(cm.PAD_YAW_RAD / 2.0)),
        name=PAD_ID)


def apply_layout(node, clients, blocks):
    """把规划场景换成「桌面 + 垫 + 这批块」。返回是否全部到位。"""
    want = ["table", PAD_ID] + ["block-%s" % b["id"] for b in blocks]
    stale = [i for i in clients["ids"] if i not in want]
    if stale:
        wp.remove_objects(node, clients["scene"], stale)
    objs = [mp.make_table_object(), make_pad_object()] + \
        [mp.make_block_object(b) for b in blocks]
    ok = mp.send_scene(node, clients["scene"], world_objs=objs)
    time.sleep(0.2)
    world, _ = mp.scene_ids(mp.read_scene(node, clients["getscene"]))
    clients["ids"] = want
    return bool(ok and all(i in world for i in want))


# ---------------------------------------------------------------- 单块评测
def _find_block(blocks, key):
    for b in blocks:
        if str(b["id"]) == str(key):
            return b
    return None


def eval_block(node, clients, block, blocks, pool, prefer_closing=None):
    """B 档可抓否（E7 口径）+ 每块挡了多少个候选位姿（``per_other``）。"""
    others, names = [], []
    for b in blocks:
        if str(b["id"]) == str(block["id"]):
            continue
        others.append(sm.block_obb(b, mp.TABLE_TOP))
        names.append(str(b["id"]))
    others.append(pad_obb())
    names.append(PAD_ID)
    target = sm.block_obb(block, mp.TABLE_TOP)
    table = sm.table_obb(mp.TABLE_TOP)
    return wp.evaluate_block(node, clients["ik"], clients["fk"],
                             clients["validity"], block, target, others, table,
                             pool, vertical_only=True, other_names=names,
                             prefer_closing=prefer_closing,
                             open_half_m=APPROACH_OPEN_HALF_M)


def inside_table(block, margin=cm.FREE_CLEAR):
    size = float(block["size"])
    half = size / 2.0
    ca = math.cos(math.radians(block.get("yaw", 0.0)))
    sa = math.sin(math.radians(block.get("yaw", 0.0)))
    for sx in (-1, 1):
        for sy in (-1, 1):
            x = block["x"] + sx * half * ca - sy * half * sa
            y = block["y"] + sx * half * sa + sy * half * ca
            if not (cm.TABLE_XMIN + margin <= x <= cm.TABLE_XMAX - margin and
                    cm.TABLE_YMIN + margin <= y <= cm.TABLE_YMAX - margin):
                return False
    return True


def find_destination(node, clients, blocks, bid, pool):
    """给 ``bid`` 找自由区落点：几何先筛（离现有块 ≥5 mm、在桌内），再 IK 核验。

    返回 ``(dest, why)``；``dest`` 为 None 时 ``why`` 说明卡在哪。
    """
    blk = _find_block(blocks, bid)
    if blk is None:
        return None, "block_missing"
    rest = [b for b in blocks if str(b["id"]) != str(bid)]
    rest_obbs = [sm.block_obb(b, mp.TABLE_TOP) for b in rest] + [pad_obb()]
    tried = 0
    for x, y in cm.free_destinations():
        g = cm.ghost(blk, x, y)
        gob = sm.block_obb(g, mp.TABLE_TOP)
        if any(sm.separation(gob, o) < cm.FREE_CLEAR for o in rest_obbs):
            continue
        if not inside_table(g):
            continue
        layout = rest + [g]
        if not apply_layout(node, clients, layout):
            return None, "scene_apply_failed"
        res = eval_block(node, clients, g, layout, pool)
        if res["b_ok"]:
            return {"x": float(x), "y": float(y), "block": g,
                    "row": res["hit"]}, None
        tried += 1
        if tried >= DEST_SCAN:
            return None, "no_ik_destination"
    return None, "no_geometric_destination"


# ---------------------------------------------------------------- 递归清障
def ensure_graspable(node, clients, blocks, bid, depth, pool, active,
                     prefer_closing=None):
    """保证 ``bid`` 在当前（规划后的）布局里**可以动**：可抓 ∧ 顶面无块。

    「可抓」不够 —— 若还有块压在 ``bid`` 上，抓走它上面那块就会掉，终态必然
    不合法（``terminal_report`` 判悬空）。所以先递归挪走**压在它上面的块**，
    再按 ``blocker_scores`` 挪走**挡住夹爪的块**，直到两个条件同时满足。

    ``prefer_closing`` 只作用于 ``depth == 0`` 那一块（= 本次的目标；清障块的
    朝向要按它们自己的几何定，不跟着目标转）。

    返回 ``{"blocks": 规划后的块位, "steps": 挪动记录, "final": 评测结果}``，
    或 None（深度/步数用尽、找不到可放的遮挡块）。
    ``active`` 是当前递归路径上的块，防止 A 挡 B、B 又挡 A 的环。
    """
    if depth > MAX_DEPTH:
        return None
    active = set(active) | {str(bid)}
    cur = [dict(b) for b in blocks]
    steps = []
    first = None
    for it in range(MAX_CLEAR):
        if not apply_layout(node, clients, cur):
            return None
        blk = _find_block(cur, bid)
        if blk is None:
            return None
        res = eval_block(node, clients, blk, cur, pool,
                         prefer_closing=prefer_closing if depth == 0 else None)
        if it == 0:
            first = res
        deps = [d for d in cm.dependents(cur, bid) if str(d) not in active]
        if res["b_ok"] and not deps:
            return {"blocks": cur, "steps": steps, "final": res,
                    "initial": first}

        # 优先级：压在上面的块（不动它目标动不了）> 挡住夹爪的块。
        # 有依赖块时只试依赖块 —— 挪开挡夹爪的块并不会让上面的块消失。
        if deps:
            order = list(deps)
        else:
            order = list(cm.blocker_scores(res["rows"]))
        progressed = False
        for other in order:
            if str(other) == PAD_ID or str(other) in active:
                continue
            sub = ensure_graspable(node, clients, cur, other, depth + 1, pool,
                                   active)
            if sub is None:
                continue
            dest, _why = find_destination(node, clients, sub["blocks"], other,
                                          pool)
            if dest is None:
                continue
            src = _find_block(sub["blocks"], other)
            new = [dest["block"] if str(b["id"]) == str(other) else dict(b)
                   for b in sub["blocks"]]
            steps = sub["steps"] + [{
                "move": other,
                "from": [src["x"], src["y"], src["z"]],
                "to": [dest["x"], dest["y"], dest["block"]["z"]],
                "grasp": dest["row"],
            }]
            cur = new
            progressed = True
            break
        if not progressed:
            return None
    return None


def check_placement(node, clients, blocks, target_id, cell, pool,
                    prefer_closing=None):
    """把目标块放到宫格 ``cell``，核验「能在那个位姿接近/放下它」。"""
    tb = _find_block(blocks, target_id)
    if tb is None:
        return None
    final = [cm.place_in_cell(tb, cell) if str(b["id"]) == str(target_id)
             else dict(b) for b in blocks]
    if not apply_layout(node, clients, final):
        return None
    res = eval_block(node, clients, _find_block(final, target_id), final, pool,
                     prefer_closing=prefer_closing)
    return {"ok": res["b_ok"], "final": final, "row": res["hit"],
            "reasons": res["reasons"]}


# ---------------------------------------------------------------- 单次尝试
def run_attempt(node, clients, truth, target_id, cell, pool,
                prefer_closing=None):
    """一个指定目标块的完整尝试（每个尝试都从原始布局重来）。"""
    out = {"id": target_id}
    blocks = [dict(b) for b in truth["blocks"]]
    if not apply_layout(node, clients, blocks):
        out.update(ok=False, reason="scene_apply_failed")
        return out

    res = ensure_graspable(node, clients, blocks, target_id, 0, pool,
                           frozenset(), prefer_closing=prefer_closing)
    if res is None:
        out.update(ok=False, reason="no_clearing_plan")
        return out
    out["clearing_steps"] = res["steps"]
    out["grasp_hit"] = res["final"]["hit"]
    out["grasp_reasons"] = res["final"]["reasons"]
    first = res.get("initial") or {}
    out["initial_b_ok"] = bool(first.get("b_ok"))
    out["initial_reasons"] = first.get("reasons")
    # 初始评测里失败的候选位姿（带 per_other），用来解释「为什么需要清障」。
    out["initial_failed_rows"] = [
        {k: r.get(k) for k in ("axis", "sign", "closing", "variant", "verdict",
                               "gap_target_mm", "gap_others_mm",
                               "gap_table_mm", "per_other")}
        for r in (first.get("rows") or []) if r.get("verdict") != "ok"]

    plac = check_placement(node, clients, res["blocks"], target_id, cell, pool,
                           prefer_closing=prefer_closing)
    if plac is None or not plac["ok"]:
        out.update(ok=False, reason="place_pose_illegal",
                   place_reasons=(plac or {}).get("reasons"))
        return out
    out["place_hit"] = plac["row"]

    # 贴合对终态穿插容差 2 mm（PAIR_PEN_TOL）：手摆贴合对的真值有拆分边界
    # 内偏（S1:1 实测 3.84 mm「穿插」）与拆分片 yaw 噪声（s1-4 残余 −1.45 mm，
    # 台账第三百三十九轮），物理上它们本就贴着、终态保持接触合法。
    # 打分/报告的终态检查**不传**这个参数，口径不变。
    rep = cm.terminal_report(plac["final"], pair_pen_tol=cm.PAIR_PEN_TOL)
    out["terminal"] = rep
    trow = _find_block(rep["blocks"], target_id)
    ok = bool(rep["ok"] and trow and trow["zone"] == "slot" and trow["legal"])
    out["final_blocks"] = [{k: b.get(k) for k in
                            ("id", "x", "y", "z", "yaw", "size")}
                           for b in plac["final"]]
    out.update(ok=ok, reason=None if ok else "terminal_illegal")
    return out


# ---------------------------------------------------------------- main
def main(argv=None):
    ap = argparse.ArgumentParser(
        description="E9 自主移遮挡探针（规划级只读，不动臂）")
    ap.add_argument("--truth", action="append", required=True,
                    help="scene_truth.json（可重复，一次起栈连测多场）")
    ap.add_argument("--report", default=None)
    ap.add_argument("--cell", type=int, default=4,
                    help="目标块放进哪个宫格（0–8，默认 4 = 垫心）；"
                         "给了 --cells 时这是缺省回退值")
    ap.add_argument("--cells", default=None, metavar="8,6,4,2,0",
                    help="逐目标格位：按 **truth 块 id 升序**依次配（见 "
                         "stage4_clear_model.assign_cells）。正式轮用它一次把 "
                         "5 个目标都探完；每个 attempt 记自己的 cell。")
    ap.add_argument("--gate", type=float, default=GATE)
    ap.add_argument("--max-targets", type=int, default=0,
                    help="每场只测前 N 个目标块（冒烟用）")
    ap.add_argument("--prefer-closing", type=int, choices=(0, 1), default=None,
                    help="把目标块的哪个合拢轴排到最前（0=块 x 轴 / 1=块 y 轴，"
                         "tool0 绕 z 差 90°）。E4b 低悬停要求两指侧对俯视视线："
                         "2026-11-10 s11 实测 closing=0 时两指前后一条线、近侧指影"
                         "压在块顶。**只改顺序不改判据** —— 偏好候选照样过 SAT+IK，"
                         "过不了就回落，``hit`` 里如实记实际选中的 closing。")
    ap.add_argument("--group", default="ur5_manipulator",
                    help="MoveIt 规划组：仿真 ur5_manipulator / 真机 ur_manipulator")
    ap.add_argument("--table-top", type=float, default=0.0,
                    help="桌面高度（base 系，m）；真机 0.0")
    ap.add_argument("--keep-scene", action="store_true",
                    help="跑完不撤场景（默认撤，免得幽灵块跟着 MoveIt 活下去）")
    ap.add_argument("--grid", default=None, metavar="GRID.json",
                    help="本场**实测**垫位姿（stage4_pad_from_burst.py 解出）。探针把目标块"
                         "放进宫格时要用它定位；缺省 = 与 --truth 同目录的 "
                         "grid-pre.json / grid.json，再没有就停（fail-closed）。"
                         "模型常量与印刷垫差 87.5 mm/88.7°，真机不许用")
    args = ap.parse_args(argv)

    # 垫位姿必须在建场景**之前**定下来：初态布局里有垫，终态布局里目标块要
    # 落到宫格中心，两处都读 ``cm.PAD_CENTER``。
    if args.grid:
        cm.load_grid(args.grid)
        log("垫位姿（实测）：%s" % args.grid)
    else:
        hint = os.path.dirname(os.path.abspath(args.truth[0]))
        found = cm.find_grid(hint)
        if not found:
            print("!! 找不到本场实测垫位姿（%s 下没有 grid-pre.json / grid.json）。\n"
                  "   先跑 stage4_pad_from_burst.py <pad 机位 burst> --out <run>/grid-pre.json 解垫，再回来；"
                  "不要用模型常量（差 87.5 mm/88.7°）。" % hint,
                  file=sys.stderr)
            return 2
        cm.load_grid(found)
        log("垫位姿（实测）：%s" % found)
    log("       垫心 (%.1f, %.1f) mm  yaw %.3f°"
        % (cm.PAD_CENTER[0] * 1000.0, cm.PAD_CENTER[1] * 1000.0,
           math.degrees(cm.PAD_YAW_RAD)))

    mp.GROUP = args.group
    mp.TABLE_TOP = float(args.table_top)

    rclpy.init()
    node = Node("stage4_clear_probe")
    clients = {
        "scene": node.create_client(ApplyPlanningScene,
                                    "/apply_planning_scene"),
        "getscene": node.create_client(GetPlanningScene,
                                       "/get_planning_scene"),
        "ik": node.create_client(GetPositionIK, "/compute_ik"),
        "fk": node.create_client(GetPositionFK, "/compute_fk"),
        "validity": node.create_client(GetStateValidity,
                                       "/check_state_validity"),
        "ids": [],
    }
    for name in ("scene", "getscene", "ik", "fk", "validity"):
        cli = clients[name]
        if not mp.wait_service(node, cli, cli.srv_name):
            log("!! 服务没起来：%s" % cli.srv_name)
            node.destroy_node()
            rclpy.shutdown()
            return 2

    # 模型关节名从 /get_planning_scene 现取 —— 真机模型没有夹爪关节，未过滤的
    # /joint_states 发进 /compute_ik 会 SIGABRT 打死 move_group（2026-11-08 实测）。
    mp.MODEL_JOINTS = mp.fetch_model_joints(node, clients["getscene"])
    if mp.MODEL_JOINTS is None:
        mp.MODEL_JOINTS = list(mj.ARM_JOINTS)
        log("!! 取不到模型关节名，退回 6 个臂关节（真机安全，仿真会被丢夹爪）")
    else:
        log("模型关节 %d 个；规划组 %s；桌面 z=%.3f m"
            % (len(mp.MODEL_JOINTS), mp.GROUP, mp.TABLE_TOP))

    start_js = mp.read_joint_state(node)
    if start_js is None:
        log("!! 拿不到 /joint_states")
        node.destroy_node()
        rclpy.shutdown()
        return 2
    pool = [mp.open_fingers(start_js)]

    scenes = []
    cells_report = {}
    for path in args.truth:
        with open(path, encoding="utf-8") as f:
            truth = json.load(f)
        ids = [b["id"] for b in truth["blocks"]]
        if args.max_targets:
            ids = ids[:args.max_targets]
        cell_map = (cm.assign_cells(ids, cm.parse_cells(args.cells))
                    if args.cells else None)
        log("\n== %s（%s:%s，%d 块 / 测 %d 个目标）=="
            % (path, truth.get("batch"), truth.get("index"),
               len(truth["blocks"]), len(ids)))
        if cell_map:
            log("   格位：%s"
                % "  ".join("%s→%d" % kv for kv in sorted(cell_map.items())))
        attempts = []
        for tid in ids:
            cell = (cell_map or {}).get(str(tid), args.cell)
            t0 = time.time()
            a = run_attempt(node, clients, truth, tid, cell, pool,
                            prefer_closing=args.prefer_closing)
            a["cell"] = cell
            a["seconds"] = round(time.time() - t0, 2)
            attempts.append(a)
            steps = len(a.get("clearing_steps") or [])
            if a["ok"]:
                log("  target-%-2s OK   格 %d  清障 %d 步  %.1f s  终态合法"
                    % (tid, cell, steps, a["seconds"]))
            else:
                log("  target-%-2s FAIL %s（格 %d，清障 %d 步，%.1f s）"
                    % (tid, a.get("reason"), cell, steps, a["seconds"]))
        n_ok = len([a for a in attempts if a["ok"]])
        log("  本场：%d/%d = %.1f%%" % (n_ok, len(attempts),
                                        100.0 * n_ok / max(1, len(attempts))))
        if cell_map:
            cells_report.update(cell_map)
        scenes.append({"truth": path, "batch": truth.get("batch"),
                       "index": truth.get("index"), "cells": cell_map,
                       "attempts": attempts})

    # 清理：把探针放进 MoveIt 的块/垫/桌面撤掉（规划场景残留会跟着栈活下去）。
    cleanup = None
    if not args.keep_scene and clients["ids"]:
        gone = wp.remove_objects(node, clients["scene"], clients["ids"])
        time.sleep(0.2)
        world, attached = mp.scene_ids(mp.read_scene(node, clients["getscene"]))
        cleanup = {"ok": gone, "world": world, "attached": attached}
        log("\n清理场景：ok=%s 剩余 world=%s attached=%s"
            % (gone, world, attached))

    node.destroy_node()
    rclpy.shutdown()

    n = sum(len(s["attempts"]) for s in scenes)
    n_ok = sum(len([a for a in s["attempts"] if a["ok"]]) for s in scenes)
    ratio = n_ok / n if n else 0.0
    per_batch = {}
    for s in scenes:
        b = per_batch.setdefault(s["batch"], {"n": 0, "ok": 0})
        b["n"] += len(s["attempts"])
        b["ok"] += len([a for a in s["attempts"] if a["ok"]])
    for b in per_batch.values():
        b["ratio"] = b["ok"] / b["n"] if b["n"] else 0.0
    reason_hist = {}
    for s in scenes:
        for a in s["attempts"]:
            if not a["ok"]:
                reason_hist[a["reason"]] = reason_hist.get(a["reason"], 0) + 1

    passed = bool(n and ratio >= args.gate)
    report = {"truths": args.truth, "cell": args.cell,
              "cells": cells_report or None,
              "prefer_closing": args.prefer_closing,
              "group": mp.GROUP, "table_top": mp.TABLE_TOP,
              "model_joints": mp.MODEL_JOINTS,
              "scenes": scenes, "per_batch": per_batch,
              "pooled": {"n": n, "ok": n_ok, "ratio": ratio},
              "fail_reasons": reason_hist,
              "gate": args.gate, "pass": passed,
              "scene_cleanup": cleanup}

    log("\n== 汇总 ==")
    for batch, b in sorted(per_batch.items()):
        log("  %s：%d/%d = %.1f%%" % (batch, b["ok"], b["n"],
                                       100.0 * b["ratio"]))
    log("  合计：%d/%d = %.1f%%（门 %.0f%%）"
        % (n_ok, n, 100.0 * ratio, 100.0 * args.gate))
    if reason_hist:
        log("  失败原因：%s"
            % "  ".join("%s×%d" % (k, v)
                        for k, v in sorted(reason_hist.items())))
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

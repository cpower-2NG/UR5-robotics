#!/usr/bin/env python3
"""E7 全姿态工作空间探针（**只算，不动臂**）。

契约（阶段 4 入口 E7）：B 档 = 只允许竖直向下接近（顶面朝上的抓取），
C 档 = 允许倾斜/侧向接近；在 S1–S3 上分别测「可抓比例」。

「一个块可抓」的口径 = **存在一个候选抓取位姿，同时满足**
  1) 免碰撞 IK 能解出关节解（种子取「已知有效的解」池 + 当前关节）；
  2) 用**我们自己的 OBB**（``stage4_scene_model``）在 FK 得到的位姿上算间隙，
     夹爪（本体 + 两指，全开）与场景里任何东西都不重叠；
  3) MoveIt 的 ``/check_state_validity`` 说这个解有效（臂杆部分是 mesh，
     独立核验做不到，只能借 MoveIt 的报告 —— 见 E8 实验 §10.1 的独立性边界）。

**不做的事**：不验接近路径（那是规划的事，E8/M2 管）；不发 ``/move_action``；
不写夹爪寄存器。所以本探针跑在真机上也不产生任何运动 —— 但它需要 ``move_group``。

候选几何（12 个摆位 × 2 个腕部朝向 = 24 个位姿）由
``stage4_scene_model.grasp_candidates`` 给，离线自检见 ``test_stage4_scene_model.py``。

用法（同一次启动可以连测多场）：
  python3 scripts/stage4_workspace_probe.py \
      --truth /tmp/grasp_scene_S1_0/scene_truth.json \
      --truth /tmp/grasp_scene_S2_0/scene_truth.json \
      --report out.json [--solo-diagnose] [--max-blocks N]

``--solo-diagnose``：凡是「整个块一个位姿都不成」的，把场景换成「只剩这一块 +
桌面」再测一遍，用来区分**被别的块挡住**和**工作空间真的够不着**（前者属 E9 的
清理/拆堆，不属 E7）。逐块记录 ``solo_b_ok`` / ``solo_c_ok``。
"""
import argparse
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import (ApplyPlanningScene, GetPositionIK,
                             GetPositionFK, GetPlanningScene,
                             GetStateValidity)
from moveit_msgs.msg import CollisionObject, PlanningScene

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import stage4_scene_model as sm                                # noqa: E402
import stage4_moveit_probe as mp                               # noqa: E402

B_GATE = 0.90        # 契约：B 档 S1–S3 ≥90% 可抓
C_GATE = 0.70        # 契约：C 档 ≥70%
IK_TIMEOUT = 0.5     # 每个种子的裸 IK 时限（短超时只会把「能抓」误判成「不能」，
                     # 保守方向；拿不到才换种子）
IK_CAP = 3           # 种子池上限
GOOD_SEED_CAP = 3    # 池里保留的「已验证有效解」个数


def log(*a):
    print(*a, flush=True)


# ------------------------------------------------------------------ 场景
def remove_objects(node, client, ids):
    """从规划场景里删掉这些 id（diff）。"""
    req = ApplyPlanningScene.Request()
    req.scene = PlanningScene()
    req.scene.is_diff = True
    objs = []
    for oid in ids:
        o = CollisionObject()
        o.id = oid
        o.operation = CollisionObject.REMOVE
        objs.append(o)
    req.scene.world.collision_objects = objs
    resp = mp.call(node, client, req)
    return bool(resp and resp.success)


def apply_scene(node, scene_client, get_scene, truth, prev_ids):
    """把场景换成 truth 这一场，返回 (blocks_obb, table_obb, ids)。"""
    blocks = [sm.block_obb(b, mp.TABLE_TOP) for b in truth["blocks"]]
    table = sm.table_obb(mp.TABLE_TOP)
    want = ["block-%s" % b["id"] for b in truth["blocks"]] + ["table"]
    stale = [i for i in prev_ids if i not in want]
    if stale:
        remove_objects(node, scene_client, stale)
    objs = [mp.make_table_object()] + [mp.make_block_object(b)
                                       for b in truth["blocks"]]
    ok = mp.send_scene(node, scene_client, world_objs=objs)
    time.sleep(0.2)
    world, _ = mp.scene_ids(mp.read_scene(node, get_scene))
    return blocks, table, want, ok and all(i in world for i in want)


# ------------------------------------------------------------------ 单块评测
def gripper_gaps(node, fk_client, js, target, others, table, q, other_names=None):
    """FK → 夹爪三个 OBB → 与目标块 / 其他块 / 桌面的最小间隙（米）。

    ``other_names``（可选，与 ``others`` 同序）：给了就额外返回
    ``per_other``（名字 → 该障碍物的最小间隙，米）。E9 靠它把「整块不可抓」
    归因到**具体是哪一块挡住了**；不给时行为与之前逐位相同。
    """
    fk = mp.fk_tool(node, fk_client, js)
    if fk is None:
        return None
    obbs = sm.gripper_obbs(fk[mp.IK_LINK], q)
    out = {"target": None, "others": None, "table": None}
    per = ({n: None for n in other_names}
           if other_names is not None else None)
    for lk in obbs:
        g = sm.gap(lk, target)
        out["target"] = g if out["target"] is None else min(out["target"], g)
        g = sm.gap(lk, table)
        out["table"] = g if out["table"] is None else min(out["table"], g)
        for i, o in enumerate(others):
            g = sm.gap(lk, o)
            out["others"] = g if out["others"] is None else min(out["others"], g)
            if per is not None:
                nm = other_names[i]
                per[nm] = g if per[nm] is None else min(per[nm], g)
    if out["others"] is None:
        out["others"] = float("inf")
    out["per_other"] = per
    out["min"] = min(out["target"], out["others"], out["table"])
    return out


def evaluate_candidate(node, ik_client, fk_client, validity_client, cand,
                       target, others, table, pool, other_names=None):
    """一个候选位姿的判定。返回 (verdict, detail)；成功时 detail 里带解。"""
    ps = mp.pose_stamped(*cand["pos"], cand["quat"])
    sol, seed_i = mp.solve_ik(node, ik_client, ps, pool, avoid=False,
                              timeout=IK_TIMEOUT, cap=IK_CAP)
    if sol is None:
        return "unreachable", {"seed": None}

    gaps = gripper_gaps(node, fk_client, sol, target, others, table, mp.OPEN_Q,
                        other_names=other_names)
    if gaps is None:
        return "fk_failed", {"seed": seed_i}
    detail = {"seed": seed_i,
              "gap_target_mm": round(gaps["target"] * 1000.0, 1),
              "gap_others_mm": (None if gaps["others"] == float("inf")
                                else round(gaps["others"] * 1000.0, 1)),
              "gap_table_mm": round(gaps["table"] * 1000.0, 1)}
    if gaps["per_other"] is not None:
        detail["per_other"] = gaps["per_other"]

    vreq = GetStateValidity.Request()
    vreq.robot_state.joint_state = sol
    vreq.group_name = mp.GROUP
    vr = mp.call(node, validity_client, vreq)
    valid = None if vr is None else bool(vr.valid)
    pairs = [(c.contact_body_1, c.contact_body_2) for c in vr.contacts] \
        if vr is not None else []
    detail["valid"] = valid
    detail["contacts"] = ["%s×%s" % p for p in pairs][:4]

    if gaps["min"] <= 0.0:
        # 我们自己的几何就不答应（MoveIt 若说有效，那就是漏判 —— 记下来）
        return "gripper_collide", detail
    if valid:
        detail["sol"] = sol
        return "ok", detail

    kinds = mp.contact_kinds(pairs)
    if "gripper_obstacle" in kinds:
        # 我们自己算的间隙 ≥ 0：接触若真在夹爪上，是 MoveIt 的假碰撞
        return "false_collision", detail
    if "arm_obstacle" in kinds:
        return "blocked_arm", detail
    if kinds == {"self"}:
        return "blocked_self", detail
    return "blocked_unknown", detail


def evaluate_block(node, ik_client, fk_client, validity_client, block_truth,
                   target, others, table, pool, full_scan=False,
                   vertical_only=False, other_names=None,
                   prefer_closing=None):
    """一个块：先试 B 档（竖直向下）4 个位姿，再试其余 20 个；命中即停。

    「命中即停」只影响统计粒度（有多少个位姿可行），不影响两个布尔量：
    B/C 都是存在性判据。``full_scan=True`` 则 24 个全测 —— 用来回答
    「为什么 C 没比 B 多」，代价是慢。

    ``vertical_only=True`` 只测 B 档的 4 个位姿（E9 的口径：E7 已证明 C 档
    在这套几何下没有额外信息量，清障规划不把它当前提）。
    ``other_names`` 透传给 :func:`evaluate_candidate`，给 E9 做遮挡归因。

    ``prefer_closing``（0/1）：把该合拢轴的候选排到最前。E9 用它让目标块按
    **不挡俯视视线**的朝向被抓（2026-11-10）：块是 29.5 mm 立方，两个合拢轴在
    夹持几何上等价，但夹爪绕 z 差 90° ⇒ 俯视看到的指位完全不同。**这只改排序
    不改判据**：偏好的候选照样要过 SAT + IK + 状态有效性。

    **选择口径（2026-11-17）**：``vertical_only``（E9）下 4 个竖直候选全测，
    再按离其他块的最小间隙取最大（``sm.pick_best_hit``）；贴合对里「手指贴着
    partner」的候选不再因为排序靠前而胜出（s1-4 空夹/别住实证）。平手才回落到
    上面的排序；E7 的 24 候选仍是命中即停。
    """
    cands = sm.grasp_candidates(block_truth, mp.TABLE_TOP)
    if vertical_only:
        cands = [c for c in cands if c["vertical"]]
    want = None if prefer_closing is None else int(prefer_closing)
    cands.sort(key=lambda c: (int(want is not None and c["closing"] != want),
                              not c["vertical"], c["axis"], -c["sign"],
                              c["closing"], c["variant"]))
    reasons = {}
    rows = []
    ok_rows = []
    b_ok = c_ok = False
    n_eval = 0
    for cand in cands:
        # E9 竖直 4 候选**全测**再选（贴合对要按离邻块的间隙挑，不能命中即停；
        # 2026-11-17 s1-4 空夹/别住：closing 0 的 partner 间隙 1.6 mm 先过就
        # 胜出，物理手指贴住邻块）。E7 的 24 候选仍保持命中即停。
        if c_ok and not full_scan and not vertical_only:
            break
        n_eval += 1
        verdict, detail = evaluate_candidate(
            node, ik_client, fk_client, validity_client, cand,
            target, others, table, pool, other_names=other_names)
        reasons[verdict] = reasons.get(verdict, 0) + 1
        row = {"axis": cand["axis"], "sign": cand["sign"],
               "closing": cand["closing"], "variant": cand["variant"],
               "vertical": cand["vertical"], "verdict": verdict}
        row.update({k: v for k, v in detail.items() if k != "sol"})
        rows.append(row)
        if verdict == "ok":
            c_ok = True
            if cand["vertical"]:
                b_ok = True
            ok_rows.append(row)
            sol = detail["sol"]
            if not any(mp.same_joints(g, sol) for g in pool):
                pool.insert(0, sol)
                del pool[GOOD_SEED_CAP:]
    hit = sm.pick_best_hit(ok_rows, vertical_only=vertical_only)
    return {"b_ok": b_ok, "c_ok": c_ok, "n_eval": n_eval,
            "n_total": len(cands), "hit": hit, "reasons": reasons,
            "rows": rows}


# ------------------------------------------------------------------ 单场
def run_scene(node, clients, truth, use_solo, full_scan=False):
    scene_client, get_scene = clients["scene"], clients["getscene"]
    ik_client, fk_client = clients["ik"], clients["fk"]
    validity_client = clients["validity"]

    prev_ids = clients.setdefault("ids", [])
    blocks, table, want, ok = apply_scene(node, scene_client, get_scene,
                                          truth, prev_ids)
    clients["ids"] = want
    if not ok:
        return {"batch": truth.get("batch"), "index": truth.get("index"),
                "scene_pass": False, "error": "规划场景应用失败"}

    by_name = {"block-%s" % b["id"]: ob for b, ob in zip(truth["blocks"], blocks)}
    start_js = mp.read_joint_state(node)
    if start_js is None:
        return {"batch": truth.get("batch"), "error": "拿不到 /joint_states"}
    pool = [mp.open_fingers(start_js)]

    rows = []
    for b, ob in zip(truth["blocks"], blocks):
        others = [o for n, o in by_name.items() if n != "block-%s" % b["id"]]
        r = evaluate_block(node, ik_client, fk_client, validity_client, b,
                           ob, others, table, pool, full_scan)
        r.update({"id": b["id"], "layer": b.get("layer", 0),
                  "x": b["x"], "y": b["y"], "z": b.get("z"),
                  "yaw": b.get("yaw"), "color": b.get("color")})
        rows.append(r)
        log("  block-%-2s layer=%d B=%-5s C=%-5s 位姿=%2d/%d %s"
            % (b["id"], b.get("layer", 0), r["b_ok"], r["c_ok"], r["n_eval"],
               r["n_total"], json.dumps(r["reasons"], ensure_ascii=False)))

    solo_rows = []
    if use_solo:
        for r in rows:
            if r["c_ok"]:
                continue
            b = [x for x in truth["blocks"] if x["id"] == r["id"]][0]
            ob = by_name["block-%s" % b["id"]]
            keep = ["block-%s" % b["id"], "table"]
            remove_objects(node, scene_client,
                           [i for i in want if i not in keep])
            mp.send_scene(node, scene_client,
                          world_objs=[mp.make_table_object(),
                                      mp.make_block_object(b)])
            time.sleep(0.2)
            solo = evaluate_block(node, ik_client, fk_client, validity_client,
                                  b, ob, [], table, pool, full_scan)
            solo["id"] = b["id"]
            solo_rows.append(solo)
            log("  [solo] block-%-2s B=%-5s C=%-5s → %s"
                % (b["id"], solo["b_ok"], solo["c_ok"],
                   "被别的块挡住" if solo["c_ok"] else "工作空间内也不可抓"))
            # 还原整场
            mp.send_scene(node, scene_client,
                          world_objs=[mp.make_table_object()]
                          + [mp.make_block_object(x) for x in truth["blocks"]])
            time.sleep(0.2)
            world, _ = mp.scene_ids(mp.read_scene(node, get_scene))
            if not all(n in world for n in want):
                log("  !! 场景还原不完整：world=%s" % world)

    n = len(rows)
    b_ok = len([r for r in rows if r["b_ok"]])
    c_ok = len([r for r in rows if r["c_ok"]])
    return {"batch": truth.get("batch"), "index": truth.get("index"),
            "scene_pass": True, "truth": None,
            "blocks": rows, "solo": solo_rows,
            "summary": {"n": n, "b_ok": b_ok, "c_ok": c_ok,
                        "b_ratio": (b_ok / n if n else 0.0),
                        "c_ratio": (c_ok / n if n else 0.0)}}


# ------------------------------------------------------------------ 汇总
def approach_summary(scenes):
    """按接近方向（块轴 × 正负）统计位姿的成败 —— 只有 ``--full-scan`` 才完整。

    这是回答「C 档为什么没比 B 多」的证据：横向/朝下的接近方向是到底解不出、
    被臂挡住、还是被夹爪自己的几何挡在桌面外。
    """
    agg = {}
    for s in scenes:
        if not s.get("scene_pass"):
            continue
        for b in s["blocks"]:
            for r in b.get("rows", []):
                key = "axis%d%s" % (r["axis"], "+" if r["sign"] > 0 else "−")
                a = agg.setdefault(key, {"n": 0, "ok": 0})
                a["n"] += 1
                if r["verdict"] == "ok":
                    a["ok"] += 1
    return agg


def pool_summary(scenes):
    out = {}
    for batch in sorted({s["batch"] for s in scenes if s.get("scene_pass")}):
        rows = [b for s in scenes if s.get("scene_pass") and s["batch"] == batch
                for b in s["blocks"]]
        n = len(rows)
        out[batch] = {"n": n,
                      "b_ok": len([b for b in rows if b["b_ok"]]),
                      "c_ok": len([b for b in rows if b["c_ok"]])}
        out[batch]["b_ratio"] = out[batch]["b_ok"] / n if n else 0.0
        out[batch]["c_ratio"] = out[batch]["c_ok"] / n if n else 0.0

    rows = [b for s in scenes if s.get("scene_pass") for b in s["blocks"]]
    n = len(rows)
    allsum = {"n": n, "b_ok": len([b for b in rows if b["b_ok"]]),
              "c_ok": len([b for b in rows if b["c_ok"]])}
    allsum["b_ratio"] = allsum["b_ok"] / n if n else 0.0
    allsum["c_ratio"] = allsum["c_ok"] / n if n else 0.0

    # solo 诊断：只统计「整块不可抓」的那些块
    solo = [b for s in scenes if s.get("scene_pass") for b in s.get("solo", [])]
    if solo:
        allsum["solo_n"] = len(solo)
        allsum["solo_b_ok"] = len([b for b in solo if b["b_ok"]])
        allsum["solo_c_ok"] = len([b for b in solo if b["c_ok"]])
        allsum["solo_c_ok_blocks"] = [b["id"] for b in solo if b["c_ok"]]
    return out, allsum


def main(argv=None):
    ap = argparse.ArgumentParser(description="E7 工作空间探针（只算，不动臂）")
    ap.add_argument("--truth", action="append", required=True,
                    help="scene_truth.json（可重复，一次起栈连测多场）")
    ap.add_argument("--report", default=None)
    ap.add_argument("--full-scan", action="store_true",
                    help="每块 24 个位姿全测（慢），回答「C 为什么没比 B 多」")
    ap.add_argument("--solo-diagnose", action="store_true",
                    help="整块不可抓的，再在「只剩它+桌面」的场景里测一遍")
    ap.add_argument("--max-blocks", type=int, default=0,
                    help="每场只测前 N 块（冒烟用）")
    ap.add_argument("--b-gate", type=float, default=B_GATE)
    ap.add_argument("--c-gate", type=float, default=C_GATE)
    args = ap.parse_args(argv)

    rclpy.init()
    node = Node("stage4_workspace_probe")
    clients = {
        "scene": node.create_client(ApplyPlanningScene, "/apply_planning_scene"),
        "getscene": node.create_client(GetPlanningScene, "/get_planning_scene"),
        "ik": node.create_client(GetPositionIK, "/compute_ik"),
        "fk": node.create_client(GetPositionFK, "/compute_fk"),
        "validity": node.create_client(GetStateValidity, "/check_state_validity"),
    }
    for name, cli in clients.items():
        if name == "ids":
            continue
        if not mp.wait_service(node, cli, cli.srv_name):
            log("!! 服务没起来：%s" % cli.srv_name)
            node.destroy_node()
            rclpy.shutdown()
            return 2
    clients["ids"] = []

    scenes = []
    for path in args.truth:
        with open(path, encoding="utf-8") as f:
            truth = json.load(f)
        if args.max_blocks:
            truth["blocks"] = truth["blocks"][:args.max_blocks]
        log("\n== %s（%s:%s，%d 块）=="
            % (path, truth.get("batch"), truth.get("index"),
               len(truth["blocks"])))
        rep = run_scene(node, clients, truth, args.solo_diagnose,
                        args.full_scan)
        rep["truth"] = path
        s = rep.get("summary")
        if s:
            log("  本场：B %d/%d = %.1f%%   C %d/%d = %.1f%%"
                % (s["b_ok"], s["n"], 100 * s["b_ratio"],
                   s["c_ok"], s["n"], 100 * s["c_ratio"]))
        scenes.append(rep)

    node.destroy_node()
    rclpy.shutdown()

    per_batch, allsum = pool_summary(scenes)
    approaches = approach_summary(scenes)
    passed = (allsum["n"] > 0
              and allsum["b_ratio"] >= args.b_gate
              and allsum["c_ratio"] >= args.c_gate)
    report = {"truths": args.truth, "scenes": scenes,
              "per_batch": per_batch, "pooled": allsum,
              "approaches": approaches,
              "gates": {"b": args.b_gate, "c": args.c_gate},
              "pass": bool(passed)}

    log("\n== 汇总 ==")
    for batch, s in per_batch.items():
        log("  %s：B %d/%d = %.1f%%  C %d/%d = %.1f%%"
            % (batch, s["b_ok"], s["n"], 100 * s["b_ratio"],
               s["c_ok"], s["n"], 100 * s["c_ratio"]))
    log("  合计：B %d/%d = %.1f%%（门 %.0f%%）C %d/%d = %.1f%%（门 %.0f%%）"
        % (allsum["b_ok"], allsum["n"], 100 * allsum["b_ratio"],
           100 * args.b_gate, allsum["c_ok"], allsum["n"],
           100 * allsum["c_ratio"], 100 * args.c_gate))
    if "solo_n" in allsum:
        log("  solo 诊断（整块不可抓的 %d 块）：空场里 B %d / C %d 可抓"
            % (allsum["solo_n"], allsum["solo_b_ok"], allsum["solo_c_ok"]))
    if args.full_scan:
        log("  按接近方向：%s"
            % "  ".join("%s %d/%d" % (k, v["ok"], v["n"])
                        for k, v in sorted(approaches.items())))
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

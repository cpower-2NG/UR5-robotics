#!/usr/bin/env python3
"""V2 仿真：**全姿态值不值** —— A 自上而下 / B ≤60° / C 任意（契约 §11.3）。

三档怎么定义（先定，不看数）
============================
候选的「接近角」= tool0 z 轴（块 → 夹爪的方向，见
``stage4_scene_model.grasp_candidates``）与竖直向上 +z 的夹角。

- **A 自上而下**：只有 a=(0,0,−1) 的 4 个位姿（现有候选里的 ``vertical``）。
- **B ≤60°**：A + 新生成的斜向候选（接近角 30/45/60°，绕块 4 个方位）。
- **C 任意**：B + 现有候选里的水平/向上位姿（接近角 90°/180°）。

**为什么必须新生成斜向候选**：生产候选族只出 0° 和 90° 两档（3 轴 × 2 方向），
没有 30–60° 的中间角。不补这一族，B 档 ≡ A 档 —— 那不是实验结论，是候选族
的空洞。斜向候选的几何与生产候选同一套约定：a 为接近方向，c 为合拢轴，
指尖外沿越过被接近表面 ``grasp_depth``；对立方体，表面点取支撑点
``center − a·(half·Σ|a·axis|)``。

口径
====
「一个候选有效」= 免碰撞 IK 有解 ∧ 自有 OBB 间隙 >0 ∧ MoveIt
``/check_state_validity`` 有效（与 E7 逐字相同，调的是同一个
``stage4_workspace_probe.evaluate_candidate``）。

- **有效候选率** = 该档 ok 候选数 / 该档候选总数。
- **可抓率（存在性）** = 至少一个 ok 候选的块数 / 块数。
- 这是**规划级**终点，不是端到端成功率；报告里分开写。

用法（与 V1 相同的起栈方式）
============================
    python3 scripts/stage4_v2_tiers.py --scene S1:0 --scene S3:0 \\
        --out ~/robotics/notes/.../v2 [--max-blocks N]
退出码：0 = 全部场景跑完；1 = 有场景失败；2 = 环境缺失。
"""
import argparse
import json
import os
import sys
import time

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np                                                # noqa: E402

import stage4_scene_model as sm                                   # noqa: E402

ANGLES = (30.0, 45.0, 60.0)
AZIMUTHS = 4
#: 每个候选最多取几次 IK 解。**必须 >1**：KDL 每次返回的关节解不同，
#: 第一解撞臂不代表该位姿不可抓（2026-11-11 定点诊断：同一候选同一场景，
#: 6 次重复判出 ok / blocked_arm / blocked_self 三种结果；S2:0 的 b0 在
#: 两遍实验里 0/28 与 21/28 交替，就是「首解没搜到好解、种子池不生长」的雪崩）。
#: 重采样 5 次后同一批候选 24/24 稳定通过；avoid=True 的碰撞感知 IK 会在
#: 0.5 s 服务时限下超时，故不用。
IK_SAMPLES = 5
#: 夹持深度与指尖外沿，取自生产常量（不自立一套）。
GRASP_DEPTH = sm.GRASP_DEPTH
TIP_Z = sm.GRIPPER["tip_z"]


def _unit(v):
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


def angled_candidates(block, table_top_z, angles=ANGLES, azimuths=AZIMUTHS):
    """斜向接近候选族（接近角 ``angles``，绕块 ``azimuths`` 个方位）。

    与生产候选的字段同名同义；``axis=3`` 是与生产候选（0/1/2）区分的标记，
    ``tilt_deg`` 记录接近角，``vertical`` 只在 0° 时为 True。
    """
    size = block.get("size")
    if size is None:
        size = block["top"] - table_top_z
    size = float(size)
    z = block.get("z")
    if z is None:
        z = table_top_z + size / 2.0
    center = np.array([float(block["x"]), float(block["y"]), float(z)])
    axes = sm.block_axes(block)
    out = []
    for theta in angles:
        th = np.radians(theta)
        for k in range(azimuths):
            phi = 2.0 * np.pi * k / azimuths
            a = _unit([np.sin(th) * np.cos(phi), np.sin(th) * np.sin(phi),
                       -np.cos(th)])
            # 立方体沿 −a 的支撑点（表面点）
            r = size / 2.0 * sum(abs(float(a @ ax)) for ax in axes)
            face = center - a * r
            pos = face - a * (TIP_Z - GRASP_DEPTH)
            # 合拢轴取水平且垂直于 a。**只有两个唯一腕部朝向**：
            # (c, y) 与 (−c, −y)。原先「两个合拢符号 × 两个 variant」的写法
            # 生成 4 个，其中 sign=−1 的两项与 sign=+1 的两项逐一相同
            # （2026-11-11 自检发现：B 档 48 个斜向候选只有 24 个唯一）。
            c0 = _unit(np.cross([0.0, 0.0, 1.0], a))
            yv = np.cross(a, c0)
            for variant, (cc, yy) in enumerate(((c0, yv), (-c0, -yv))):
                rot = np.column_stack([cc, yy, a])
                out.append({
                    "block": block.get("id"), "axis": 3, "sign": 1,
                    "closing": 0, "variant": variant,
                    "vertical": False, "tilt_deg": float(theta),
                    "azimuth_deg": float(np.degrees(phi)),
                    "approach": a.tolist(), "pos": pos.tolist(),
                    "quat": sm.mat_to_quat(rot),
                    "pose": (pos.tolist(), sm.mat_to_quat(rot)),
                })
    return out


def tier_candidates(block, table_top_z):
    """返回 {'A': [...], 'B': [...], 'C': [...]}；三档是包含关系。"""
    prod = sm.grasp_candidates(block, table_top_z)
    prod = [dict(c, tilt_deg=(0.0 if c["vertical"] else
                              (180.0 if c["approach"][2] > 0.5 else 90.0)))
            for c in prod]
    vertical = [c for c in prod if c["vertical"]]
    tilted = angled_candidates(block, table_top_z)
    return {"A": vertical,
            "B": vertical + tilted,
            "C": prod + tilted}


def evaluate_robust(node, clients, cand, target, others, table, pool,
                    evaluate=None):
    """对候选重采样 IK：任一解通过判定即算 ok。

    判定本体仍是 ``stage4_workspace_probe.evaluate_candidate``（E7 逐字同一套），
    这里只改「给它几次取解机会」。``evaluate`` 可注入，便于单测。
    """
    if evaluate is None:
        import stage4_workspace_probe as wp
        evaluate = wp.evaluate_candidate
    ev = evaluate
    verdict, detail = "unreachable", {"seed": None}
    for k in range(IK_SAMPLES):
        verdict, detail = ev(node, clients["ik"], clients["fk"],
                             clients["validity"], cand, target, others,
                             table, pool)
        if verdict == "ok":
            detail = dict(detail, samples=k + 1)
            return verdict, detail
    return verdict, detail


def run_scene(node, clients, truth, max_blocks=0):
    import stage4_moveit_probe as mp
    import stage4_workspace_probe as wp
    scene_client, get_scene = clients["scene"], clients["getscene"]
    prev_ids = clients.setdefault("ids", [])
    blocks, table, want, ok = wp.apply_scene(node, scene_client, get_scene,
                                             truth, prev_ids)
    if not ok:
        # move_group 刚起来时 /apply_planning_scene 偶尔拒收（实测 S3:0 一次）。
        # 服务在并不等于内部已就绪：重等一次再试，仍失败才算本场失败。
        import time as _t
        _t.sleep(5.0)
        for name, cli in clients.items():
            if name != "ids" and not mp.wait_service(node, cli, cli.srv_name):
                break
        blocks, table, want, ok = wp.apply_scene(node, scene_client,
                                                 get_scene, truth, prev_ids)
    if not ok:
        return {"batch": truth.get("batch"), "index": truth.get("index"),
                "scene_pass": False, "error": "规划场景应用失败"}
    clients["ids"] = want
    by_name = {"block-%s" % b["id"]: ob
               for b, ob in zip(truth["blocks"], blocks)}
    start_js = mp.read_joint_state(node)
    pool = [mp.open_fingers(start_js)] if start_js is not None else []
    truths = truth["blocks"][:max_blocks] if max_blocks else truth["blocks"]
    rows = []
    for b in truths:
        others = [o for n, o in by_name.items()
                  if n != "block-%s" % b["id"]]
        cands = tier_candidates(b, mp.TABLE_TOP)
        verdicts = {}
        # 并集只评一次（三档是包含关系），按候选身份去重
        uniq = {}
        for tier, lst in cands.items():
            for c in lst:
                key = (c["axis"], c["sign"], c["closing"], c["variant"],
                       round(c.get("tilt_deg", 0.0), 1),
                       round(c.get("azimuth_deg", -1.0), 1))
                uniq.setdefault(key, c)
        for key, c in uniq.items():
            verdict, detail = evaluate_robust(
                node, clients, c, by_name["block-%s" % b["id"]],
                others, table, pool)
            verdicts[key] = verdict
            if verdict == "ok":
                sol = detail["sol"]
                if not any(mp.same_joints(g, sol) for g in pool):
                    pool.insert(0, sol)
                    del pool[wp.GOOD_SEED_CAP:]
        stat = {}
        for tier, lst in cands.items():
            n = len(lst)
            keys = [(c["axis"], c["sign"], c["closing"], c["variant"],
                     round(c.get("tilt_deg", 0.0), 1),
                     round(c.get("azimuth_deg", -1.0), 1)) for c in lst]
            n_ok = sum(1 for k in keys if verdicts.get(k) == "ok")
            stat[tier] = {"n": n, "ok": n_ok,
                          "rate": (n_ok / n) if n else 0.0}
        rows.append({"id": b["id"], "layer": b.get("layer", 0),
                     "x": b["x"], "y": b["y"], "z": b.get("z"),
                     "color": b.get("color"), "tiers": stat,
                     "tilt_ok": sorted({v for k, v in verdicts.items()
                                        if v == "ok"}),
                     "n_eval": len(uniq)})
        print("  block-%-2s A %d/%d  B %d/%d  C %d/%d"
              % (b["id"], stat["A"]["ok"], stat["A"]["n"],
                 stat["B"]["ok"], stat["B"]["n"],
                 stat["C"]["ok"], stat["C"]["n"]), flush=True)
    return {"batch": truth.get("batch"), "index": truth.get("index"),
            "scene_pass": True, "rows": rows}


def pool(rows_by_scene):
    out = {"n": 0}
    for tier in ("A", "B", "C"):
        out[tier] = {"cands": 0, "ok": 0, "blocks": 0, "blocks_ok": 0}
    for sc in rows_by_scene:
        for r in sc.get("rows", []):
            out["n"] += 1
            for tier in ("A", "B", "C"):
                s = r["tiers"][tier]
                o = out[tier]
                o["cands"] += s["n"]
                o["ok"] += s["ok"]
                o["blocks"] += 1
                o["blocks_ok"] += int(s["ok"] > 0)
    for tier in ("A", "B", "C"):
        o = out[tier]
        o["valid_rate"] = (o["ok"] / o["cands"]) if o["cands"] else 0.0
        o["exist_rate"] = ((o["blocks_ok"] / o["blocks"])
                           if o["blocks"] else 0.0)
    return out


def render(p):
    out = ["| 档 | 候选数 | 有效候选 | 有效候选率 | 可抓块 | 可抓率 |",
           "|---|---|---|---|---|---|"]
    for tier, name in (("A", "A 自上而下"), ("B", "B ≤60°"), ("C", "C 任意")):
        o = p[tier]
        out.append("| %s | %d | %d | %.1f%% | %d/%d | %.1f%% |" % (
            name, o["cands"], o["ok"], 100 * o["valid_rate"],
            o["blocks_ok"], o["blocks"], 100 * o["exist_rate"]))
    return "\n".join(out)


def main(argv=None):
    ap = argparse.ArgumentParser(description="V2 仿真：A/B/C 三档姿态")
    ap.add_argument("--scene", action="append", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--max-blocks", type=int, default=0)
    ap.add_argument("--attach", action="store_true",
                    help="栈已起（同一场景），不自己 launch")
    args = ap.parse_args(argv)
    os.makedirs(args.out, exist_ok=True)

    import rclpy
    from moveit_msgs.srv import (ApplyPlanningScene, GetPositionFK,
                                 GetPositionIK, GetPlanningScene,
                                 GetStateValidity)
    from rclpy.node import Node

    import stage4_moveit_probe as mp
    import stage4_v1_sim as v1

    rclpy.init()
    node = Node("stage4_v2_tiers")
    clients = {
        "scene": node.create_client(ApplyPlanningScene,
                                    "/apply_planning_scene"),
        "getscene": node.create_client(GetPlanningScene,
                                       "/get_planning_scene"),
        "ik": node.create_client(GetPositionIK, "/compute_ik"),
        "fk": node.create_client(GetPositionFK, "/compute_fk"),
        "validity": node.create_client(GetStateValidity,
                                       "/check_state_validity"),
    }
    clients["ids"] = []
    # move_group 的服务只在仿真栈起来之后才存在：等待必须放在每场的
    # launch（--attach 时由调用方保证）之后，不能在这里等。

    def wait_services():
        for name, cli in clients.items():
            if name == "ids":
                continue
            if not mp.wait_service(node, cli, cli.srv_name):
                raise RuntimeError("服务没起来：%s" % cli.srv_name)

    scenes = []
    try:
        for scene in args.scene:
            print("\n== %s ==" % scene, flush=True)
            proc = log = None
            try:
                if not args.attach:
                    proc, log = v1.launch_sim(
                        scene, os.path.join(args.out,
                                            scene.replace(":", "_")
                                            + ".launch.log"))
                tp = v1.wait_truth(scene)
                if tp is not None:
                    # 每场都重等：上一场的栈已停，服务要重新发现。
                    wait_services()
                if tp is None:
                    rep = {"batch": scene.split(":")[0], "scene_pass": False,
                           "error": "等不到 scene_truth.json"}
                else:
                    with open(tp, encoding="utf-8") as f:
                        truth = json.load(f)
                    if args.max_blocks:
                        truth["blocks"] = truth["blocks"][:args.max_blocks]
                    rep = run_scene(node, clients, truth, args.max_blocks)
                    rep["truth"] = tp
            except Exception as e:
                rep = {"batch": scene.split(":")[0], "scene_pass": False,
                       "error": "%s: %s" % (type(e).__name__, e)}
            finally:
                if proc is not None:
                    v1.stop_sim(proc)
                    log.close()
                    time.sleep(4.0)
            rep["scene"] = scene
            scenes.append(rep)
            print("  %s" % (rep.get("error") or "ok"), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    good = [s for s in scenes if s.get("scene_pass")]
    p = pool(good)
    print("\n## 汇总\n")
    print(render(p))
    with open(os.path.join(args.out, "v2.json"), "w",
              encoding="utf-8") as f:
        json.dump({"kind": "stage4-v2", "angles": ANGLES,
                   "azimuths": AZIMUTHS, "scenes": scenes, "pooled": p,
                   "note": "规划级（存在性/有效候选率），不是端到端成功率"},
                  f, ensure_ascii=False, indent=1)
    print("\n→ %s" % os.path.join(args.out, "v2.json"))
    return 0 if good and len(good) == len(scenes) else 1


if __name__ == "__main__":
    sys.exit(main())

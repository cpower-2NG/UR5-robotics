#!/usr/bin/env python3
"""E9 真机半段：把探针的「清障方案」变成一份**按顺序可执行**的抓放计划序列。

本模块**纯离线**：只读探针报告 + 场景真值，产出计划 JSON 与执行脚本；
不 import ROS、不发轨迹、不写夹爪寄存器。真执行是板上
``precision_exec.py --go`` 的事（每条动作都要当次运动令牌）。

分工（为什么这样切）
--------------------
``stage4_clear_probe`` 回答「**存不存在**一个合法方案」：目标可抓 ∧ 清障 ∧
放置位姿合法 ∧ 终态合法，夹爪间隙用自有 OBB+SAT 逐个候审判。它不产生轨迹。
本模块回答「**怎么把那个方案做出来**」：把每一步清障与最终目标翻译成
``precision_plan.build_pick`` 的 A3 抓放几何（竖直抓、18.5 mm 夹持深度、
整链选支、高度护栏），``q_start`` 逐段衔接。

一个必须说清的例外
------------------
``build_pick`` 有一条粗判据「被抓块离最近邻块中心距 ≥60 mm」。E9 的目标
恰恰是**贴面/邻块**场景：探针已经用 OBB 判过「这个候选位姿夹爪不碰」，
它比中心距细（贴面块只要在合拢轴方向让开就能抓）。所以这里显式给
``min_block_gap=0.0``，并在 manifest 里写 ``gap_check: probe_verified``。
**没有探针判决的调用方不许这样放宽。**

指令顺序（每条都在 manifest 里可见）
------------------------------------
1. 先按探针给的顺序执行清障步骤（``clearing_steps``：谁 → 从哪 → 到哪）；
2. 最后执行目标块 → 指定宫格；
3. 每步内部固定：approach → close → lift → to_slot → place_approach →
   place(open) → retreat → survey（``build_pick`` 的老航点序列，不另立）。

用法（开发机，链式 q_start）
---------------------------
    python3 scripts/stage4_clear_run.py \\
        --probe-report runs/<run>/e9/<scene>-probe.json \\
        --urdf ~/robotics/notes/ur5-vision-grasp/runs/2026-10-25-precision/robot.urdf \\
        --from-frame <joint_states.json 所在目录> \\
        --out-dir ~/robotics/notes/ur5-vision-grasp/runs/<run>/e9-plan/<scene>

退出码：0 = 全部计划写出且预检通过；1 = 某一步规划失败（不写半份序列）；
2 = 输入缺失/不可用。
"""
import argparse
import json
import os
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import precision_plan as pp                                       # noqa: E402
import stage4_clear_model as cm                                   # noqa: E402
import urkin                                                      # noqa: E402

#: 收尾巡检位（base 系，m）。用阶段 4 工作区上方的空位，不用阶段 2 的
#: ``SURVEY_TOOL0["place"]``（那在 y≈−0.39 m，每步白跑一趟）。
#: tool0 z=0.40 ⇒ 指尖 0.196 m，高过任何 29.5 mm 块；x/y 在拾取区上方。
E9_SURVEY_TOOL0 = (0.520, 0.300, 0.400)


def log(*a):
    print(*a, flush=True)


# ---------------------------------------------------------------- 输入解析
def load_scene(report, scene_index, truth_override=None):
    scenes = report.get("scenes") or []
    if not scenes:
        raise ValueError("探针报告里没有 scenes")
    if not (0 <= scene_index < len(scenes)):
        raise ValueError("--scene-index 越界：%d（共 %d 场）"
                         % (scene_index, len(scenes)))
    sc = scenes[scene_index]
    truth_path = truth_override or sc["truth"]
    if truth_override:
        sc = dict(sc, truth=truth_override)
    with open(truth_path, encoding="utf-8") as f:
        truth = json.load(f)
    return sc, truth


def pick_attempt(scene, target=None):
    attempts = scene.get("attempts") or []
    ok = [a for a in attempts if a.get("ok")]
    if target is not None:
        for a in ok:
            if str(a.get("id")) == str(target):
                return a
        raise ValueError("目标 %r 在报告里没有成功的尝试" % (target,))
    if not ok:
        raise ValueError("这一场没有成功的尝试（ok=True）")
    return ok[0]


# ---------------------------------------------------------------- 几何
def grasp_yaw_deg(hit, block_yaw):
    """探针命中的抓取候选 → ``build_pick`` 的 tool0 绕 z 角。

    ``grasp_candidates`` 的约定：tool0 z = 接近方向 a，tool0 x = 合拢轴 c。
    B 档竖直抓只可能是 a=(0,0,−1)（axis=2, sign=−1）。``closing`` 是块局部轴：
    0 → c = 块 x 轴 ⇒ ψ = 块 yaw；1 → c = 块 y 轴 ⇒ ψ = 块 yaw + 90°。
    ``variant`` 是绕 a 转 180°，对方块在几何上等价（``fold_yaw_180`` 会折掉）。
    """
    if not hit:
        raise ValueError("没有抓取命中记录")
    if not hit.get("vertical") or int(hit.get("axis", -1)) != 2 \
            or int(hit.get("sign", 0)) != -1:
        raise ValueError("E9 只执行 B 档竖直抓，收到 axis=%r sign=%r vertical=%r"
                         % (hit.get("axis"), hit.get("sign"), hit.get("vertical")))
    closing = int(hit.get("closing", 0))
    if closing not in (0, 1):
        raise ValueError("closing 只认 0/1，收到 %r" % (closing,))
    return float(block_yaw) + (0.0 if closing == 0 else 90.0)


def find_block(blocks, bid):
    for b in blocks:
        if str(b.get("id")) == str(bid):
            return b
    return None


def move_in_layout(blocks, bid, x, y, z_bottom):
    """就地更新布局里的一块（不改原 truth）。``top`` 是块顶离台面高度。"""
    b = find_block(blocks, bid)
    b["x"], b["y"] = float(x), float(y)
    b["z"] = float(z_bottom) + float(b["size"]) / 2.0
    b["top"] = float(z_bottom) + float(b["size"])
    return b


def surface_of(to_z, size, where):
    """落点块心 z → 支撑面高度；只认台面与垫面两种（多认就是把错当真）。"""
    z = float(to_z) - float(size) / 2.0
    for cand in (0.0, float(cm.PAD_THICK)):
        if abs(z - cand) <= 1e-4:
            return cand
    raise ValueError("%s 的落点块心 z=%.4f 推不出支撑面（得到 %.4f m，"
                     "既不是台面也不是垫面）" % (where, float(to_z), z))


def actions_from_attempt(truth, attempt, pad_thick):
    """探针的一次成功尝试 → 有序动作表（清障在前，目标在最后）。

    返回的 ``blocks`` 是**原始布局**（不预先搬运）：搬运由
    ``plan_sequence`` 在**算完那一步之后**才施加 —— 否则清障那一步会按
    「块已经躺在落点」去算，臂会跑去错的地方。
    起点用探针记录的 ``step["from"]``（那一刻的实测位姿），不用布局推算。
    """
    blocks = [dict(b) for b in truth["blocks"]]
    actions = []
    for step in attempt.get("clearing_steps") or []:
        bid = step["move"]
        blk = find_block(blocks, bid)
        if blk is None:
            raise ValueError("清障步骤引用了不存在的块 %r" % (bid,))
        src, to = step["from"], step["to"]
        surf = surface_of(to[2], blk["size"], "清障块 %r" % (bid,))
        actions.append({
            "kind": "clear",
            "block": str(bid),
            "from_xy": [float(src[0]), float(src[1])],
            "from_top": float(src[2]) + float(blk["size"]) / 2.0,
            "slot": [float(to[0]), float(to[1])],
            "surface_z": surf,
            "grasp_yaw_deg": grasp_yaw_deg(step.get("grasp"),
                                           blk.get("yaw", 0.0)),
            "grasp_hit": {k: step["grasp"].get(k) for k in
                          ("axis", "sign", "closing", "variant", "verdict")}
            if step.get("grasp") else None,
        })

    tid = str(attempt["id"])
    tgt = find_block(blocks, tid)
    if tgt is None:
        raise ValueError("真值里没有目标块 %r" % (tid,))
    final = find_block(attempt.get("final_blocks") or [], tid)
    if final is None:
        raise ValueError("尝试记录里没有目标块 %r 的终态位姿" % (tid,))
    surf = surface_of(final["z"], tgt["size"], "目标块 %r" % (tid,))
    if abs(surf - float(pad_thick)) > 1e-4:
        raise ValueError("目标块 %r 的落点支撑面 %.4f m 不是垫面 %.4f m —— "
                         "探针与执行器对放置面的理解不一致"
                         % (tid, surf, float(pad_thick)))
    actions.append({
        "kind": "target",
        "block": tid,
        "from_xy": [float(tgt["x"]), float(tgt["y"])],
        "from_top": float(tgt["top"]),
        "slot": [float(final["x"]), float(final["y"])],
        "surface_z": surf,                  # 目标放到垫上
        "grasp_yaw_deg": grasp_yaw_deg(attempt.get("grasp_hit"),
                                       tgt.get("yaw", 0.0)),
        "grasp_hit": {k: attempt["grasp_hit"].get(k) for k in
                      ("axis", "sign", "closing", "variant", "verdict")},
    })
    return actions, blocks


# ---------------------------------------------------------------- 计划
def to_pp_block(b):
    """真值块（m）→ precision_plan 的块字典（cx/cy/top 单位 mm）。"""
    size_mm = float(b["size"]) * 1000.0
    top_mm = float(b["top"]) * 1000.0
    return {"idx": int(b["id"]),
            "class": b.get("color") or b.get("class") or "block",
            "cx": float(b["x"]) * 1000.0,
            "cy": float(b["y"]) * 1000.0,
            "top": top_mm,
            "h": top_mm,
            "size": size_mm,
            "yaw": float(b.get("yaw", 0.0))}


def plan_sequence(chain, q_start, actions, blocks, out_dir,
                  survey_tool0=E9_SURVEY_TOOL0, place_clearance=None):
    """逐步生成计划；任何一步预检失败就抛 ``pp.PlanError``，不写半份序列。"""
    layout = [dict(b) for b in blocks]
    q = [float(v) for v in q_start]
    plans = []
    for i, act in enumerate(actions, start=1):
        blk = find_block(layout, act["block"])
        if blk is None:
            raise ValueError("布局里没有块 %r" % (act["block"],))
        pp_blocks = [to_pp_block(b) for b in layout]
        # ``build_pick`` 只从 cand 里取两样真东西：抓点 xy（这里不用 ——
        # ``grasp_at="block_center"``）与「候选离块心 ≤12 mm」的检查。
        # 其余字段（score/width/angle）是阶段 2 的 GraspNet 元数据，E9 没有，
        # 填的是几何真值而不是编的分数：width = 块边长，angle = 合拢轴角。
        cand = {"xyz_base": [float(blk["x"]), float(blk["y"]),
                             float(blk.get("top", blk["size"]))],
                "score": 0.0,
                "width_mm": float(blk["size"]) * 1000.0,
                "dist_mm": 0.0,
                "angle_deg": float(act["grasp_yaw_deg"]),
                "seed": None}
        wps, checks, meta = pp.build_pick(
            chain, q, cand, to_pp_block(blk), pp_blocks, act["slot"],
            yaw_grasp_deg=act["grasp_yaw_deg"], grasp_at="block_center",
            surface_z=act["surface_z"], survey_tool0=tuple(survey_tool0),
            place_clearance=place_clearance,
            # 压在目标上的块：`h`（块顶离台面）≠ 物理块高，必须显式给
            # 物理块高，否则抓取高度按两个块高算（= 2026-11-02 拆塔事故）。
            block_h_m=float(blk["size"]),
            # 见模块头：探针已用 OBB 判过间隙，这里的 60 mm 粗判据必须让位。
            min_block_gap=0.0)
        plan = {"version": 2,
                "meta": dict(meta,
                             e9={"kind": act["kind"], "block": act["block"],
                                 "slot": list(act["slot"]),
                                 "from_xy": list(act["from_xy"]),
                                 "grasp_yaw_deg": round(
                                     float(act["grasp_yaw_deg"]), 3),
                                 "gap_check": "probe_verified",
                                 "checks": checks}),
                "q_start": list(q),
                "waypoints": wps}
        name = "%02d-%s-block-%s.json" % (i, act["kind"], act["block"])
        path = os.path.join(out_dir, name)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(plan, f, ensure_ascii=False, indent=1)
        plans.append({"index": i, "name": name, "path": path,
                      "kind": act["kind"], "block": act["block"],
                      "slot": act["slot"], "from_xy": act["from_xy"],
                      "grasp_yaw_deg": act["grasp_yaw_deg"],
                      "n_waypoints": len(wps)})
        q = [float(v) for v in wps[-1]["q"]]
        move_in_layout(layout, act["block"], act["slot"][0], act["slot"][1],
                       act["surface_z"])
    return plans, layout, q


def run_script_text(plans, board_dir, log_dir):
    lines = ["#!/bin/bash",
             "# E9 自主移遮挡：按顺序执行本场计划。任何一步 rc≠0 立即停",
             "# （fail-closed，不跳过、不重试）。令牌 = 当次 STOP→PLAY 握手产物。",
             "set -u",
             'TOKEN="$1"',
             '[ -n "${TOKEN:-}" ] || { echo "用法: $0 <token>"; exit 2; }',
             'fail() { echo "!!!!! STOP: $*"; exit 9; }']
    for p in plans:
        lines.append('echo "===== %s : exec"' % p["name"])
        lines.append(
            '~/precision/run.sh ~/precision/precision_exec.py '
            '--plan %s/%s --go --require-auth "$TOKEN" '
            '--log %s/%s.jsonl' % (board_dir, p["name"], log_dir, p["name"]))
        lines.append('rc=$?')
        lines.append('[ $rc -eq 0 ] || fail "%s rc=$rc"' % p["name"])
        lines.append('sleep 1')
    lines.append('echo "ALL DONE"')
    return "\n".join(lines) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="E9 真机半段：探针方案 → 可执行计划序列（离线）")
    ap.add_argument("--probe-report", required=True)
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--scene-index", type=int, default=0)
    ap.add_argument("--truth", default=None,
                    help="覆盖报告里记的 truth 路径（报告在板上生成时是板上绝对路径）")
    ap.add_argument("--target", default=None,
                    help="指定目标块 id（默认取这一场第一个成功尝试）")
    ap.add_argument("--from-frame", default=None,
                    help="从帧目录的 joint_states.json 取 q_start")
    ap.add_argument("--q-start", default=None,
                    help="逗号分隔的 6 个关节角（弧度）")
    ap.add_argument("--pad-thick", type=float, default=None,
                    help="垫厚（m），目标块放垫上；缺省 = stage4_clear_model.PAD_THICK")
    ap.add_argument("--place-clearance", type=float, default=None,
                    help="放置时块底离表面间隙（m）；缺省 = precision_plan 的 5 mm")
    ap.add_argument("--survey", default=None,
                    help="收尾巡检位 tool0 x,y,z（m）；缺省 %s"
                         % (E9_SURVEY_TOOL0,))
    ap.add_argument("--board-dir", default="$HOME/precision/plans/e9",
                    help="板上计划目录（写进 run.sh）")
    ap.add_argument("--board-log-dir", default="$HOME/precision/logs/e9",
                    help="板上执行日志目录（写进 run.sh）")
    args = ap.parse_args(argv)

    with open(args.probe_report, encoding="utf-8") as f:
        report = json.load(f)
    scene, truth = load_scene(report, args.scene_index, args.truth)
    attempt = pick_attempt(scene, args.target)
    pad_thick = cm.PAD_THICK if args.pad_thick is None else float(args.pad_thick)
    actions, blocks = actions_from_attempt(truth, attempt, pad_thick)

    if args.from_frame:
        js = json.load(open(os.path.join(args.from_frame,
                                         "joint_states.json")))
        chain = urkin.Chain(args.urdf)
        q_start = [float(js["position"][js["name"].index(n)])
                   for n in chain.move_joints]
    elif args.q_start:
        chain = urkin.Chain(args.urdf)
        q_start = [float(v) for v in args.q_start.split(",")]
    else:
        ap.error("要给 --from-frame 或 --q-start")

    survey = (E9_SURVEY_TOOL0 if args.survey is None else
              tuple(float(v) for v in args.survey.split(",")))
    os.makedirs(args.out_dir, exist_ok=True)

    log("场景 %s:%s，目标块 %s，%d 个动作（清障 %d + 目标 1）"
        % (truth.get("batch"), truth.get("index"), attempt["id"],
           len(actions), len(actions) - 1))
    try:
        plans, final_layout, q_end = plan_sequence(
            chain, q_start, actions, blocks, args.out_dir,
            survey_tool0=survey, place_clearance=args.place_clearance)
    except pp.PlanError as e:
        log("!! 预检不过：%s" % e)
        log("   计划**没有**写出完整序列 —— 不要执行。")
        return 1

    manifest = {
        "probe_report": args.probe_report,
        "truth": scene["truth"],
        "batch": truth.get("batch"), "index": truth.get("index"),
        "target": attempt["id"],
        "scene_index": args.scene_index,
        "pad_thick": pad_thick,
        "survey_tool0": list(survey),
        "gap_check": "probe_verified",
        "actions": actions,
        "plans": plans,
        "q_start": q_start,
        "q_end": q_end,
        "final_layout": [{k: b.get(k) for k in
                          ("id", "x", "y", "z", "yaw", "size")}
                         for b in final_layout],
        "terminal": attempt.get("terminal"),
    }
    mpath = os.path.join(args.out_dir, "manifest.json")
    with open(mpath, "w", encoding="utf-8") as f:
        json.dump(manifest, f, ensure_ascii=False, indent=1)

    sh = run_script_text(plans, args.board_dir, args.board_log_dir)
    spath = os.path.join(args.out_dir, "run.sh")
    with open(spath, "w", encoding="utf-8") as f:
        f.write(sh)
    os.chmod(spath, 0o755)

    for p in plans:
        log("  %-28s %-6s 块 %-3s (%.3f, %.3f) → (%.3f, %.3f)  ψ=%6.1f°  %d 航点"
            % (p["name"], p["kind"], p["block"],
               p["from_xy"][0], p["from_xy"][1], p["slot"][0], p["slot"][1],
               p["grasp_yaw_deg"], p["n_waypoints"]))
    log("→ %s" % mpath)
    log("→ %s" % spath)
    return 0


if __name__ == "__main__":
    sys.exit(main())

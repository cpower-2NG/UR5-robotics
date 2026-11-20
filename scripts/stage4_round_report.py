#!/usr/bin/env python3
"""阶段 4「一轮」的归档件 ``round.json`` —— **脚本写、永不手写**（契约 §4.5 / v0.6 §20.3）。

一轮 = 一个场景布局里对**全部指定块**的块级尝试。本脚本只做**装配**：
把机器证据（执行日志 / 跑后三机位真值 / 彩色量 / pad solve）与人提供的
**观察事实**（安全计数 / 干预 / 人眼复核 / 指定说法）拼成固定 schema 的
``round.json``，然后交给 ``stage4_score.py`` 判。

固定 schema（字段名不要改，判分器按这些键读）::

    round_id, scene{batch,layout,seed}, n_targets,
    terminal{blocks,overlaps,off_table,out_of_bounds,source},
    safety{collisions,drops,unexpected_motion,estop},
    interventions{count}, human_review{required,approved},
    pad{source,measured,thickness_m,footprint_m,surface_z_m},
    attempts[]{
      target{block_id}, spec,
      selected{instance_id,auto_top1},
      truth{instance_id,truth_source},
      grasp{lifted,dropped,wrong_block,exec_rc,readback{gripped},slip,
            postrun{target_left_origin,arrived}},
      place{in_slot,over_line,out_of_bounds},
      failure{class}, void{reason},
    }

**缺失字段不填默认值**（§20.3）：拿不到的证据就不写这个键，同时把
``complete=false`` 与 ``missing=[...]`` 写进文件、进程返回 **2**。判分器看到
``complete=false`` 直接判数据缺失（退出码 2），不会把「没证据」算成「系统差」。

证据各从哪来（一个事实一个家）：

| 字段 | 来源 |
|---|---|
| 执行 rc / 闭合回读 / 滑移 | 板端 ``precision_exec.py`` 的 ``<plan>.jsonl``（``event=gripper/grasp_check/gate/traj``） |
| 目标块离位 / 到位 | 跑后**腕部三机位**真值（``stage4_truth_from_frame.py`` 产出）+ 垫面彩色量 |
| 放置格 / 压线 / 越界 | 垫面彩色顶面质心（``measure_target_colour`` 的输出）+ 实测 pad 位姿 |
| 全桌终态 | 三机位真值合并后 ``stage4_clear_model.terminal_report`` |
| 安全 / 干预 / 人眼复核 / 指定说法 / 真值来源 | ``--facts`` 文件（**人的观察**，不是判决） |

``--facts`` 只放人看得到、机器量不出的事实，**不放结论**::

    {
      "round_id": "S1-L2-r1",
      "scene": {"batch": "S1", "layout": "L2", "seed": 11},
      "truth_source": "postrun-wrist-3view",
      "specs": {"2": "橙色、靠北那块"},
      "auto_top1": {"2": true},
      "failure_class": {"2": "execution"},
      "voids": {"3": "相机掉线"},
      "safety": {"collisions": 0, "drops": 0, "unexpected_motion": 0, "estop": 0},
      "interventions": {"count": 0},
      "human_review": {"required": false, "approved": true}
    }

用法::

    python3 scripts/stage4_round_report.py \\
        --manifest runs/.../plan/manifest.json \\
        --exec-log runs/.../board-log \\
        --view-truth pick=runs/.../truth-pick.json \\
        --view-truth free=runs/.../truth-free.json \\
        --view-truth pad=runs/.../truth-pad.json \\
        --colour runs/.../verify.json \\
        --pad runs/.../grid-pre.json \\
        --pad-spec ~/.config/ur5-vision-grasp/pad-spec.json \\
        --facts runs/.../round-facts.json \\
        --out runs/.../round.json

退出码：0 = 字段齐全；2 = 有证据缺失（文件照写，``complete=false``）。
"""
import argparse
import json
import math
import os
import sys

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import stage4_clear_model as cm                              # noqa: E402
import stage4_clear_run as cr                                # noqa: E402

#: AG-95 空夹阈值：闭合回读 ≤ 5 mm = 手指合到头 = 没夹住（与 precision_exec 同源）。
EMPTY_TOL_MM = 5.0

#: 目标块「离开原位 / 到达放置格」的位移容差（米）。与 ``compare_layout`` 同口径。
MOVE_TOL_M = 0.040

#: 合并三机位真值时的去重口径（米）。同 ``merge_truth_blocks`` 缺省。
DEDUP_M = 0.040

#: 「同一块」的最大层高差（米）。塔的上下层只差一个块边（29.5 mm），
#: 只按 3-D 40 mm 近邻会把邻层当成原块（动塔顶、塔底留在原地被误判成
#: 「目标已离开」）。与 ``merge_truth_blocks`` 的 layer_m 同源。
LAYER_M = 0.015

SAFETY_KEYS = ("collisions", "drops", "unexpected_motion", "estop")


def log(*a):
    print(*a, flush=True)


# ---------------------------------------------------------------- 读文件
def _load_json(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def _looks_like_json(s):
    s = s.strip()
    return s.startswith("{") or s.startswith("[")


def read_events(path):
    """``precision_exec.py`` 的 jsonl → 事件 dict 列表（轨迹采样行跳过）。"""
    out = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or not _looks_like_json(line):
                continue
            try:
                rec = json.loads(line)
            except ValueError:
                continue
            if isinstance(rec, dict) and rec.get("event"):
                out.append(rec)
    return out


def find_exec_log(log_dir, plan_name):
    """板端日志名 = 计划名 + ``.jsonl``（计划名本身已带 ``.json`` ⇒ 双后缀）。"""
    if not log_dir:
        return None
    for name in (plan_name + ".jsonl", plan_name,
                 os.path.splitext(plan_name)[0] + ".jsonl"):
        p = os.path.join(log_dir, name)
        if os.path.exists(p):
            return p
    return None


# ---------------------------------------------------------------- 执行证据
def exec_evidence(events):
    """一串执行事件 → 尝试级执行证据。

    返回的键只在**有证据**时出现；没有的键留给上层判「数据缺失」：

    * ``exec_rc``：gate/traj 任一步 ok=false ⇒ 2/1；否则 0。这是从日志
      复原的**执行结论**，不是文件里抄来的；日志没归档 ⇒ 不出现。
    * ``closed_mm``：``where=grasp.after`` 的 close 回读（空夹判据的原料）。
    * ``slipped``：``grasp_check.slipped``（抬升后 |lift−closed| > 1 mm）。
    * ``lifted``：出现 ``grasp_check`` 才说明抬升航点真跑了、且两次回读都在。
    """
    out = {"events": len(events)}
    rc = 0
    for ev in events:
        kind = ev.get("event")
        if kind == "gate" and ev.get("ok") is False:
            rc = max(rc, 2)
        elif kind == "aborted_token":
            # 令牌过期 ⇒ 本轮 void（契约 §4.6）：不算抓取失败、不进分母。
            # 2026-11-14 起这条有结构化日志（precision_exec 写 event），
            # 不再只活在 stdout 字符串里。
            out["aborted_token"] = True
        elif kind == "stop_requested":
            # 人为软停（watch_round --stop）⇒ 同为 void，原因分开记。
            out["stop_requested"] = True
        elif kind in ("traj", "gripper_fail", "gripper_selftest"):
            if ev.get("event") == "gripper_fail":
                rc = max(rc, 1)
            elif ev.get("ok") is False:
                rc = max(rc, 1)
        elif kind == "grasp_check":
            out["lifted"] = True
            out["slipped"] = bool(ev.get("slipped"))
            out["closed_mm"] = ev.get("closed_mm")
            out["lifted_mm"] = ev.get("lifted_mm")
    for ev in events:
        if ev.get("event") == "gripper" and ev.get("action") == "close" \
                and str(ev.get("where", "")).endswith(".after"):
            pos = ev.get("position_mm")
            if pos is not None:
                out["closed_mm"] = float(pos)
    out["exec_rc"] = rc
    if out.get("closed_mm") is None:
        out.pop("closed_mm", None)
    return out


def _target_plan(manifest):
    """manifest 里**本目标**的那份 ``kind=target`` 计划。

    S3 的 manifest 可能还带着「被吸收的清障」动作 —— 它是**别的块**的目标
    动作（只是由本次尝试触发），不能当成本次的 target 计划。按
    ``plan.block == manifest.target`` 选；旧 manifest 没有 ``block`` 字段时
    退回第一份 target 计划（S0–S2 行为不变）。
    """
    target = str(manifest.get("target"))
    fallback = None
    for p in manifest.get("plans") or []:
        if p.get("kind") != "target":
            continue
        if p.get("block") is None:
            if fallback is None:
                fallback = p
            continue
        if str(p.get("block")) == target:
            return p
    return fallback


def _plan_meta(plan):
    try:
        return _load_json(plan["path"]).get("meta") or {}
    except (OSError, ValueError, KeyError):
        return {}


# ---------------------------------------------------------------- 真值 / 终态
def merge_views(view_truths):
    """``[(机位名, 真值文件), ...]`` → (合并块, 溯源)。"""
    truths, prov = [], {"views": []}
    for name, path in view_truths:
        doc = _load_json(path)
        blocks = doc.get("blocks") or []
        truths.append((name, blocks))
        prov["views"].append({"view": name, "path": path, "n_blocks": len(blocks)})
    merged, fusion = cr.merge_truth_blocks(truths, dedup_m=DEDUP_M)
    prov.update(fusion)
    return merged, prov


def terminal_from_blocks(blocks, truth_source):
    rep = cm.terminal_report(blocks)
    return {
        "blocks": [{"id": b.get("id"), "zone": b.get("zone"),
                   "in_bounds": b.get("in_bounds"), "legal": b.get("legal"),
                   "cell": b.get("cell")} for b in rep.get("blocks") or []],
        "overlaps": len(rep.get("overlaps") or []),
        "off_table": rep.get("off_table"),
        "out_of_bounds": rep.get("out_of_bounds"),
        "source": truth_source,
    }


def _near(a, b):
    """同一位置（水平 40 mm 且**层高差 ≤15 mm**；缺 z 时退回旧 3-D 口径）。

    S3 的两层塔上下只差 29.5 mm：只看 3-D 距离会把邻层当成原块。
    """
    d = math.sqrt((a["x"] - b["x"]) ** 2 + (a["y"] - b["y"]) ** 2
                  + (a["z"] - b["z"]) ** 2)
    if d > MOVE_TOL_M:
        return False
    if "z" in a and "z" in b:
        return abs(float(a["z"]) - float(b["z"])) <= LAYER_M
    return True


def _find_at(blocks, pos):
    best = None
    for b in blocks:
        d = math.sqrt((b["x"] - pos["x"]) ** 2 + (b["y"] - pos["y"]) ** 2
                      + (b["z"] - pos["z"]) ** 2)
        if d > MOVE_TOL_M:
            continue
        if "z" in b and "z" in pos and \
                abs(float(b["z"]) - float(pos["z"])) > LAYER_M:
            continue
        if best is None or d < best[0]:
            best = (d, b)
    return best[1] if best else None


# ---------------------------------------------------------------- 垫参数
def load_pad(grid_path, spec_path, thickness_override=None):
    """``grid-pre.json``（垫面 z）+ 垫足迹（+可选垫厚）→ ``obs.pad``。

    垫厚**不要求卡尺实测**（2026-11-11 用户决定：垫是两张 A4 叠放，厚度远低于
    相机与夹爪的机械误差）。没给 ``thickness_m`` 就用 pad solve 的垫面 z ——
    base 原点在台面上，垫面 z 就是垫厚；这仍来自当轮实测（grid-pre），
    不是模型常量。足迹仍必须给：``--pad-spec`` 的 ``footprint_m``（卷尺或
    A4 规格值都行）。

    缺必需项都返回 ``(None, [缺失说明])``：判分器不接受模型常量（§20.5）。
    """
    missing, pad = [], {}
    if not grid_path:
        missing.append("--pad（grid-pre.json）未给：垫面 z 无实测值")
    else:
        try:
            g = _load_json(grid_path)
            center = [float(v) for v in g["grid_center_base_mm"]]
            yaw = float(g["grid_yaw_base_deg"])
            pad["surface_z_m"] = center[2] / 1000.0
            pad["source"] = os.path.basename(grid_path)
            # 放置格 / 压线判定都走 cm 的实测垫位姿（不是模型常量）
            cm.configure_pad(center[:2], yaw, source=pad["source"])
        except (OSError, ValueError, KeyError, IndexError) as e:
            missing.append("读不了 pad solve %s：%s" % (grid_path, e))
    thick = thickness_override
    if thick is not None:
        pad["thickness_source"] = "--pad-thick（显式覆盖）"
    fp = None
    if spec_path:
        try:
            s = _load_json(spec_path)
            if thick is None and s.get("thickness_m") is not None:
                thick = float(s["thickness_m"])
                pad["thickness_source"] = "pad-spec"
            fp = [float(v) for v in s["footprint_m"]]
        except (OSError, ValueError, KeyError, TypeError) as e:
            missing.append("读不了 --pad-spec %s：%s" % (spec_path, e))
    if thick is None and "surface_z_m" in pad:
        thick = pad["surface_z_m"]
        pad["thickness_source"] = ("surface_z（未卡尺实测：A4 叠放厚度低于"
                                   "相机/夹爪机械误差，用户 2026-11-11 决定）")
    if thick is not None:
        pad["thickness_m"] = float(thick)
    if fp is not None:
        pad["footprint_m"] = fp
    else:
        missing.append("垫足迹无实测值：--pad-spec 需含 footprint_m")
    if thick is None:
        missing.append("垫厚无来源：给 --pad-spec 或 --pad-thick")
    if missing:
        return None, missing
    pad["measured"] = True
    # 判分器的反模型常量闸门：来源里不许出现 model/gen_scene/constant
    if spec_path:
        pad["source"] += "+" + os.path.basename(spec_path)
    return pad, []


# ---------------------------------------------------------------- 放置判定
def place_verdict(colour, cell, size, pad, target_color=None,
                  grid_order=None, cell_color=None):
    """彩色顶面质心 → ``place{in_slot, over_line, out_of_bounds[, colour_cell]}``
    或 ``(None, 缺失)``。

    ``target_color`` + ``grid_order`` 给了就附加 ``colour_cell``（S3 颜色归格）：
    落格必须**是该颜色的格**；同色重复块的第二块允许落备用格（``spare``）。
    主判据 ``in_slot/over_line/out_of_bounds`` 口径不变。
    """
    if not isinstance(pad, dict):
        return None, ["垫参数缺失（--pad + --pad-spec）：放置格判不了"]
    if not isinstance(colour, dict):
        return None, ["缺垫面彩色量（--colour）：放置格判不了"]
    if colour.get("ok") is not True:
        return None, ["彩色量不够帧（%d 帧 / 缺 %d）—— 判不了放置"
                      % (colour.get("n_frames") or 0, colour.get("n_missed") or 0)]
    xy = colour.get("xy_mm")
    if not (isinstance(xy, (list, tuple)) and len(xy) == 2):
        return None, ["彩色量没有 xy_mm"]
    x, y = float(xy[0]) / 1000.0, float(xy[1]) / 1000.0
    got = cm.cell_index_of(x, y)
    inside, _ = cm.cell_contains(x, y, 0.0, size)
    hu, hv = pad["footprint_m"][0] / 2.0, pad["footprint_m"][1] / 2.0
    u, v = cm.to_pad_frame(x, y)
    half = size / 2.0
    out = (abs(u) + half > hu) or (abs(v) + half > hv)
    res = {"in_slot": (got is not None and cell is not None
                       and int(got) == int(cell)),
           "over_line": not inside,
           "out_of_bounds": bool(out)}
    if target_color and cell is not None and (grid_order or cell_color):
        try:
            want_cell = (str(grid_order[int(cell)]) if grid_order
                         else str(cell_color))
        except (IndexError, TypeError, ValueError):
            want_cell = None
        got_color = (colour.get("truth_color")
                     or colour.get("measured_color") or "")
        cc = {"expected_cell_color": want_cell,
              "target_color": str(target_color),
              "measured_color": (str(got_color) if got_color else None),
              "cell_valid_for_colour": (want_cell in (str(target_color),
                                                      "spare")
                                        if want_cell else None),
              "block_color_match": (str(got_color) == str(target_color)
                                    if got_color else None)}
        cc["ok"] = bool(cc["cell_valid_for_colour"]
                        and cc["block_color_match"])
        res["colour_cell"] = cc
    return res, []


def _failure_class(attempt, facts_cls):
    """失败分类（契约 §4.1 七类）。facts 给了就用人给的，否则按观测轴推。"""
    classes = ("perception", "candidate", "ik", "planning", "execution",
               "placement", "human")
    if facts_cls in classes:
        return facts_cls
    if attempt.get("void"):
        return None
    g = attempt.get("grasp") or {}
    p = attempt.get("place")
    sel = (attempt.get("selected") or {}).get("instance_id")
    tru = (attempt.get("truth") or {}).get("instance_id")
    if sel is not None and tru is not None and int(sel) != int(tru):
        return "perception"
    if g:
        rb = (g.get("readback") or {}).get("gripped")
        if g.get("lifted") is False or rb is False:
            return "candidate"
        if g.get("dropped") or g.get("wrong_block") or g.get("exec_rc") != 0:
            return "execution"
    if p is not None and p.get("in_slot") is False:
        return "placement"
    return None


# ---------------------------------------------------------------- 尝试装配
def build_attempt(manifest, facts, log_dir, post_blocks, colour, pad):
    """一条块级尝试 → (attempt, missing 列表)。只写有证据的键。"""
    miss = []
    target = str(manifest.get("target"))
    attempt = {"target": {"block_id": target}}
    if manifest.get("action_index") is not None:
        attempt["action_index"] = int(manifest["action_index"])

    spec = (facts.get("specs") or {}).get(target)
    if spec is not None:
        attempt["spec"] = spec

    # ---- 系统选了哪一块（当前 E9 链按指定 id 直接执行；同源问题见 docstring）
    plan = _target_plan(manifest)
    if plan is None:
        miss.append("manifest 里没有 kind=target 的计划 ⇒ selected 未知")
        meta = {}
    else:
        meta = _plan_meta(plan)
    if meta.get("block_idx") is not None:
        auto = (facts.get("auto_top1") or {}).get(target)
        sel = {"instance_id": int(meta["block_idx"])}
        if isinstance(auto, bool):
            sel["auto_top1"] = auto
        sel["source"] = "plan.meta.block_idx"
        attempt["selected"] = sel

    # ---- 独立真值（身份由试验布局清单给；来源必须由人声明并在白名单里）
    src = facts.get("truth_source")
    attempt["truth"] = {"instance_id": manifest.get("target"),
                        "truth_source": src}

    # ---- 执行证据
    ev = None
    if log_dir and plan is not None:
        lp = find_exec_log(log_dir, plan.get("name") or
                           os.path.basename(plan.get("path", "")))
        if lp is None:
            miss.append("执行日志缺 %s.jsonl" % (plan.get("name") or "?"))
        else:
            ev = exec_evidence(read_events(lp))
    elif not log_dir:
        miss.append("没给 --exec-log ⇒ 执行 rc / 回读 / 滑移无证据")
    else:
        miss.append("没找到 target 计划 ⇒ 执行证据无主")

    grasp = {}
    if ev is not None:
        grasp["exec_rc"] = ev["exec_rc"]
        if "lifted" in ev:
            grasp["lifted"] = bool(ev["lifted"])
        elif ev["exec_rc"] != 0:
            # 执行在抬升前就判红了（闸门不过 / traj 失败）：日志完整到失败点，
            # 「没抬起」是读得出来的结论，不是缺证据。
            grasp["lifted"] = False
        else:
            miss.append("执行日志里没有 grasp_check（抬升后复读）事件")
        if "slipped" in ev:
            grasp["slip"] = bool(ev["slipped"])
        if "closed_mm" in ev:
            grasp["readback"] = {"gripped": ev["closed_mm"] > EMPTY_TOL_MM,
                                 "closed_mm": ev["closed_mm"]}

    # ---- 目标块现在在哪（跑后三机位真值）
    truth_doc = manifest.get("truth")
    origin = None
    if truth_doc and os.path.exists(truth_doc):
        try:
            for b in _load_json(truth_doc).get("blocks") or []:
                if str(b.get("id")) == target:
                    origin = b
                    break
        except (OSError, ValueError):
            origin = None
    if origin is None:
        miss.append("布局真值 %s 里找不到目标块 #%s" % (truth_doc, target))
    post = None
    if post_blocks is not None and origin is not None:
        post = _find_at(post_blocks, origin)
        grasp["postrun"] = {"target_left_origin": post is None
                            or not _near(post, origin)}
        if post is None:
            grasp["postrun"]["note"] = "跑后三机位里原点附近没有目标块"

    sel_id = (attempt.get("selected") or {}).get("instance_id")
    if sel_id is not None:
        grasp["wrong_block"] = (int(sel_id) != int(manifest.get("target")))

    # ---- 放置（彩色量：垫面深度看不出 29.5 mm 台阶）
    size = None
    if origin is not None:
        size = float(origin.get("size") or cm.PAD_PITCH)
    if size is None:
        size = cm.PAD_PITCH
    cell = manifest.get("cell")
    pl, pl_miss = place_verdict(colour, cell, size, pad,
                                target_color=manifest.get("target_color"),
                                cell_color=manifest.get("cell_color"))
    arrived = None
    if pl is None:
        miss.extend(pl_miss)
    else:
        # 只增：垫上实测坐标 + 目标格心（扰动嫌疑检测要吃这两样，见
        # ``disturbance_suspects``；判分逻辑只读三个布尔，不受影响）
        if isinstance(colour, dict) and colour.get("xy_mm"):
            pl["xy_mm"] = [round(float(v), 1) for v in colour["xy_mm"]]
        if cell is not None:
            pl["cell"] = int(cell)
        attempt["place"] = pl
        arrived = pl["in_slot"] and not pl["over_line"] and not pl["out_of_bounds"]
        if grasp.get("postrun") is not None:
            grasp["postrun"]["arrived"] = arrived
        elif post is not None:
            grasp["postrun"] = {"target_left_origin": not _near(post, origin),
                                "arrived": arrived}

    # ---- 掉块：只在**有证据**时下结论（§20.3 不填默认值）。
    # 「抬起来了」+「既不在原点、也没到格」才算掉；放置判不了就不写这个键，
    # 让判分器按缺数据处理，而不是把「没测」算成「没掉」。
    if "lifted" in grasp:
        if not grasp["lifted"]:
            grasp["dropped"] = False
        elif arrived is not None and origin is not None and post_blocks is not None:
            still = post is not None and _near(post, origin)
            grasp["dropped"] = bool(not arrived and not still)

    if grasp:
        # 三机位确认不全（例如没有 readback 或 postrun）⇒ 判分器会进 person_review
        attempt["grasp"] = grasp

    # ---- 清障记录（S3）：谁被搬、搬到哪、被谁触发、执行证据
    clearing = []
    for c in manifest.get("clearing") or []:
        rec = {"block": str(c.get("block")), "kind": c.get("kind"),
               "cell": c.get("cell"),
               "absorbed": bool(c.get("absorbed")),
               "absorbed_by": c.get("absorbed_by"),
               "plan": c.get("plan")}
        ev_c = None
        if log_dir and rec["plan"]:
            lp = find_exec_log(log_dir, rec["plan"])
            if lp is not None:
                ev_c = exec_evidence(read_events(lp))
        if ev_c is not None:
            rec["exec"] = {"exec_rc": ev_c["exec_rc"],
                           "lifted": ev_c.get("lifted"),
                           "closed_mm": ev_c.get("closed_mm"),
                           "slipped": ev_c.get("slipped")}
        else:
            # 清障计划缺日志只记账，不把整轮判成不完整（硬证据仍看目标计划）
            rec["missing"] = True
        clearing.append(rec)
    if clearing:
        attempt["clearing"] = clearing

    void_reason = (facts.get("voids") or {}).get(target)
    if void_reason:
        attempt = {"target": {"block_id": target}, "spec": attempt.get("spec"),
                   "void": {"reason": void_reason}}
    else:
        attempt["failure"] = {"class": _failure_class(
            attempt, (facts.get("failure_class") or {}).get(target))}
    return attempt, miss


def disturbance_suspects(attempts, min_mm=5.0, extra_places=None):
    """邻块扰动嫌疑（台账第三百四十一轮判分盲区的补账；**只记录，不判红**）。

    判分只评每块自己的格位：s1-1-r1-attempt2 里黄绿被紫块的释放推离格心
    ~8.6 mm 仍 ``place_ok``。这里对**按放置次序**的每一对（先 i 后 j）：
    若 i 的落点相对自己格心的偏移在「i 格心 → j 格心」方向上的投影
    > ``min_mm``，记一条嫌疑（i 疑似被后放的 j 推开）。方向对得上才算——
    任意方向的自身放置误差不该被算成推挤。

    ``extra_places``：清障落格（S3）—— ``{action_index, block,
    desired_center_mm, cell}``，与目标一起按 ``action_index`` 全局排序。
    输入：``build_attempt`` 装的 attempts（place 里要有 ``xy_mm`` 与
    ``cell``）。返回嫌疑列表（空 = 无嫌疑 / 证据不足）。
    """
    rows = []
    for i, a in enumerate(attempts):
        order = a.get("action_index")
        rows.append((i if order is None else order,
                     (a.get("target") or {}).get("block_id"),
                     a.get("place") or {}))
    for j, e in enumerate(extra_places or []):
        order = e.get("action_index")
        rows.append((len(attempts) + j if order is None else order,
                     e.get("block"),
                     {"xy_mm": e.get("desired_center_mm"),
                      "cell": e.get("cell")}))
    rows.sort(key=lambda r: (r[0] if r[0] is not None else 10 ** 9,
                             str(r[1])))
    pts = []
    for i, (_order, target, p) in enumerate(rows):
        xy, cell = p.get("xy_mm"), p.get("cell")
        if not (isinstance(xy, (list, tuple)) and len(xy) == 2) \
                or cell is None:
            continue
        try:
            cx, cy = cm.cell_center(int(cell))
        except (KeyError, ValueError, TypeError):
            continue
        pts.append({"i": i, "target": target,
                    "x": float(xy[0]), "y": float(xy[1]),
                    "cx": float(cx) * 1000.0, "cy": float(cy) * 1000.0})
    out = []
    for a in pts:
        for b in pts:
            if b["i"] <= a["i"]:
                continue                     # 只看「后被放」的块
            dx, dy = a["x"] - a["cx"], a["y"] - a["cy"]
            ux, uy = b["cx"] - a["cx"], b["cy"] - a["cy"]
            n = math.hypot(ux, uy)
            if n < 1e-9:
                continue
            proj = (dx * ux + dy * uy) / n
            if proj > float(min_mm):
                out.append({"pushed": a["target"], "by": b["target"],
                            "offset_mm": round(proj, 1)})
    return out


def build_round(manifests, facts, log_dir, view_truths, colours, pad,
                pad_missing, round_id=None):
    """整轮装配 → (round.json dict, missing 列表)。"""
    miss = list(pad_missing or [])
    scene = facts.get("scene") or {}
    if not (round_id or facts.get("round_id")):
        miss.append("round_id 缺失（--round-id 或 facts.round_id）")

    post_blocks, prov = None, None
    if view_truths:
        post_blocks, prov = merge_views(view_truths)
        log("  跑后真值：%d 机位 → %d 块" % (len(view_truths), len(post_blocks)))
    else:
        miss.append("没给 --view-truth ⇒ 终态 / 目标块离位判不了")

    src = facts.get("truth_source")
    if src is None:
        miss.append("facts.truth_source 缺失（必须声明独立真值来源）")
    attempts = []
    for i, man in enumerate(manifests):
        colour = colours[i] if i < len(colours) else None
        att, amiss = build_attempt(man, facts, log_dir, post_blocks, colour, pad)
        attempts.append(att)
        miss.extend("块 %s：%s" % (man.get("target"), m) for m in amiss)

    # ---- 轮级人观察事实（缺了就是缺证据，不给默认 0）
    def _int(v):
        return isinstance(v, int) and not isinstance(v, bool)

    safety = facts.get("safety")
    if not isinstance(safety, dict) or any(
            not _int(safety.get(k)) for k in SAFETY_KEYS):
        miss.append("facts.safety 缺 %s 的整数计数" % "/".join(SAFETY_KEYS))
        safety = None
    iv = facts.get("interventions")
    if not isinstance(iv, dict) or not _int(iv.get("count")):
        miss.append("facts.interventions.count 缺整数")
        iv = None
    hr = facts.get("human_review")
    if not isinstance(hr, dict) or not isinstance(hr.get("required"), bool):
        miss.append("facts.human_review.required 缺布尔")
        hr = None

    terminal = None
    if post_blocks is not None:
        terminal = terminal_from_blocks(post_blocks, src)

    clearing_all = [c for m in manifests for c in (m.get("clearing") or [])]
    clearing_summary = {
        "n_moves": len(clearing_all),
        "n_absorbed": sum(1 for c in clearing_all if c.get("absorbed")),
        "n_free": sum(1 for c in clearing_all if not c.get("absorbed")),
        "missing": sum(1 for c in clearing_all if c.get("missing")),
    }
    extra_places = [{"action_index": c.get("index"),
                     "block": c.get("block"),
                     "desired_center_mm": c.get("desired_center_mm"),
                     "cell": c.get("cell")} for c in clearing_all]

    doc = {
        "version": 1, "kind": "stage4-round",
        "round_id": round_id or facts.get("round_id"),
        "scene": {"batch": scene.get("batch"), "layout": scene.get("layout"),
                  "seed": scene.get("seed")},
        "n_targets": len(manifests),
        "terminal": terminal,
        "safety": safety,
        "interventions": iv,
        "human_review": hr,
        "pad": pad,
        "attempts": attempts,
        "disturbance_suspects": disturbance_suspects(
            attempts, extra_places=extra_places),
        "designation": facts.get("designation"),
        "clearing_summary": clearing_summary,
        "truth_provenance": prov,
        "complete": not miss,
        "missing": miss,
    }
    return doc, miss


# ---------------------------------------------------------------- CLI
def _view_specs(specs):
    out = []
    for s in specs or []:
        if "=" in s:
            name, path = s.split("=", 1)
        else:
            name, path = os.path.splitext(os.path.basename(s))[0], s
        out.append((name, path))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="阶段 4 round.json 自动产出器（脚本写、永不手写）")
    ap.add_argument("--manifest", action="append", required=True,
                    help="一份块级尝试的 manifest.json（可重复，顺序 = 尝试顺序）")
    ap.add_argument("--facts", default=None,
                    help="人的观察事实（安全/干预/人眼复核/说法/真值来源）")
    ap.add_argument("--exec-log", default=None,
                    help="板端执行日志目录（<plan>.jsonl）")
    ap.add_argument("--view-truth", action="append", default=[],
                    help="跑后机位真值：name=path（可重复）")
    ap.add_argument("--colour", action="append", default=[],
                    help="垫面彩色量（measure_target_colour 输出或含 colour 的 verify.json），"
                         "与 --manifest 一一对应（可重复）")
    ap.add_argument("--pad", default=None, help="grid-pre.json（实测垫位姿）")
    ap.add_argument("--pad-spec", default=None,
                    help="垫参数 JSON：{footprint_m, thickness_m(可选)}；"
                         "不给厚度就用 pad solve 的垫面 z")
    ap.add_argument("--pad-thick", type=float, default=None,
                    help="覆盖垫厚（米）；足迹仍需 --pad-spec")
    ap.add_argument("--round-id", default=None)
    ap.add_argument("--out", required=True, help="输出 round.json 路径")
    args = ap.parse_args(argv)

    facts = {}
    if args.facts:
        try:
            facts = _load_json(args.facts)
        except (OSError, ValueError) as e:
            log("!! 读不了 --facts %s：%s" % (args.facts, e))
            return 2
    else:
        log("!! 没给 --facts：安全 / 干预 / 人眼复核 / 真值来源都无来源")

    manifests = []
    for p in args.manifest:
        try:
            manifests.append(_load_json(p))
        except (OSError, ValueError) as e:
            log("!! 读不了 --manifest %s：%s" % (p, e))
            return 2

    colours = []
    for p in args.colour:
        try:
            d = _load_json(p)
            colours.append(d.get("colour", d))
        except (OSError, ValueError) as e:
            log("!! 读不了 --colour %s：%s" % (p, e))
            colours.append(None)

    pad, pad_missing = load_pad(args.pad, args.pad_spec, args.pad_thick)

    if not facts:
        pad_missing = list(pad_missing) + ["没给 --facts（安全/干预/人眼/来源）"]
        facts = {}

    try:
        doc, miss = build_round(manifests, facts, args.exec_log,
                                _view_specs(args.view_truth), colours,
                                pad, pad_missing, round_id=args.round_id)
    except (OSError, ValueError, KeyError, TypeError) as e:
        log("!! 装配失败：%s" % e)
        return 2

    os.makedirs(os.path.dirname(os.path.abspath(args.out)) or ".",
                exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(doc, f, ensure_ascii=False, indent=1)
    log("→ %s（%s）" % (args.out,
                        "字段齐全" if doc["complete"] else "不完整"))
    for m in miss:
        log("   ! %s" % m)
    return 0 if doc["complete"] else 2


if __name__ == "__main__":
    sys.exit(main())

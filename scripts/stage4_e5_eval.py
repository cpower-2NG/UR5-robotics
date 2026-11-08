#!/usr/bin/env python3
"""E5 评估：真值 → 命令集 → 实例分割 → 指定解析 → 判分（S0/S1/S2 分开报）。

判据（契约 §3）：指定 ID ≥90%（S2 单独报）；中心 / 顶高 ≤2 mm、yaw ≤5°；
解析 100% 可判、0 误配。

两个定义先说清楚，否则数字没法读：
- **指定 ID 正确率** = 「解析器返回的 top-1 实例 == 命令要指的那块」/ 全部命令，
  **含** ambiguous 时仍返回的那一块（它毕竟给了答案）。
- **自主 top-1 率** = 其中 ``status == ok``（不追问、不需要人确认）且正确的比例。
  契约 §4.3 说自主 top-1 只单独报、不设硬门。
- **误配** = ``status == ok`` 却选错 —— 这条必须是 0。

命令从哪来：对每个真值块，先生成 **所有** 合法说法（at / 方位 / 关系 / 限定词），
再拿 **真值块表** 跑一遍解析器，**只保留在真值上唯一指向该块的命令**（其余记为
「真值上就不唯一」，跳过并计数，不静默丢）。这样命令集的合法性由解析器自己的
语义定义，而不是评估脚本另写一份判据 —— 语义本身另有 60 项单元测试钉着。

用法：
  python3 scripts/stage4_e5_eval.py s0=<帧目录> s1=<帧目录> s2=<帧目录> [--json out]
帧目录 = ``.../frames-s0/topcam-home``（含 color/depth/tf/truth.json）。
退出码：0 = 全部判据过；1 = 有判据未过；2 = 数据缺失/坏。
"""
import argparse
import json
import os
import sys

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import block_classes                                              # noqa: E402
import graspnet_candidates as gc                                  # noqa: E402
import stage4_specify as sp                                       # noqa: E402

MATCH_TOL_MM = 15.0        # 实例 ↔ 真值配对容差（半块 14.75，相邻块心 29.5）
GEOM_PASS = {"dxy_mm": 2.0, "dtop_mm": 2.0, "dyaw_deg": 5.0}
ID_PASS = 0.90

RELATIONS = ("left-of", "right-of", "front-of", "back-of")
POSITIONS = ("left", "right", "front", "back", "center")


# ---------------------------------------------------------------- 数据
def load_truth(frame_dir):
    with open(os.path.join(frame_dir, "truth.json"), encoding="utf-8") as f:
        return json.load(f)


def truth_records(truth):
    """真值块 → 解析器吃的块记录（``select`` 只认这几个字段）。"""
    out = []
    for b in truth["blocks"]:
        out.append({"idx": int(b["id"]), "class": b["color"],
                    "cx": b["x"] * 1000.0, "cy": b["y"] * 1000.0,
                    "top": b["top"] * 1000.0,
                    "yaw_rect": float(b["yaw"]),
                    "n": 800, "px_area_mm2": 0.93,
                    "ext_rect": [b["size"] * 1000.0, b["size"] * 1000.0],
                    "low": False, "class_share": 1.0})
    return out


def measure(frame_dir, split=True):
    """分割 + 判色（仿真帧用仿真色表）。返回 (块表, 台面 z)。"""
    frame = gc.load_frame(frame_dir)
    return gc.find_blocks(frame, split=split,
                          hue_table=block_classes.SIM_HUE_DEG)


def match(insts, truth):
    """贪心最近配对：真值块 ↔ 分割实例；超过 ``MATCH_TOL_MM`` 的不配。"""
    pool = [b for b in insts if not b.get("low") and sp._in_zone(b)]
    pairs, used = {}, set()
    for b in sorted(truth["blocks"], key=lambda t: t["id"]):
        best, bd = None, MATCH_TOL_MM
        for i, c in enumerate(pool):
            d = ((c["cx"] - b["x"] * 1000.0) ** 2
                 + (c["cy"] - b["y"] * 1000.0) ** 2) ** 0.5
            if d <= bd and i not in used:
                best, bd = i, d
        if best is not None:
            used.add(best)
            pairs[int(b["id"])] = pool[best]
    return pairs, [c for i, c in enumerate(pool) if i not in used]


# ---------------------------------------------------------------- 命令生成
def _candidate_commands(t, by_color):
    """一个真值块的所有合法说法（不做唯一性筛选）。"""
    cmds = ["at %d,%d" % (round(t["x"] * 1000), round(t["y"] * 1000))]
    cmds.append("%s center" % t["color"])
    for p in POSITIONS:
        if p != "center":
            cmds.append("%s %s" % (t["color"], p))
    for c2 in by_color:
        if c2 == t["color"]:
            continue
        for r in RELATIONS:
            cmds.append("%s %s %s" % (t["color"], r, c2))
    cmds.append("%s nearest" % t["color"])
    cmds.append("%s largest" % t["color"])
    cmds.append("%s most_visible" % t["color"])
    return cmds


def _indep_unique(text, t, truth):
    """**独立于解析器**的唯一性判据（评估器的自检，防止两边同源一起错）。

    只用真值坐标直接数谓词命中的块数，**另写一份选择逻辑**，不复用
    ``stage4_specify.select``。语义门槛（``REL_MARGIN_MM`` / ``TIE_MM``）是契约
    的一部分，两边共用常量；被独立重写的是「怎么数命中」这件事 —— 轴用错、方向
    弄反、忘了看候选，这里都会跟解析器打架。

    返回 True / False / None（None = 从真值判不了，例如 largest，真值里所有块同
    尺寸；不计入分歧）。
    """
    cmd = sp.parse_command(text)
    same = [b for b in truth["blocks"] if b["color"] == t["color"]]
    tie, rel_m = sp.TIE_MM, sp.REL_MARGIN_MM

    # 真值里的 x/y/top 是**米**（见 truth.json）；解析器里的记录是毫米。这里统一到
    # 毫米再比 —— 否则 5 mm 的门槛永远对 0.3 m 的差值不成立，全被误判成「不唯一」。
    def X(b): return b["x"] * 1000.0
    def Y(b): return b["y"] * 1000.0
    def TOP(b): return b["top"] * 1000.0

    kind = cmd["kind"]
    if kind == "at":
        return True
    if kind == "position":
        if cmd["position"] == "center":        # 离同色形心最近，且领先 ≥tie
            mx = sum(X(b) for b in same) / len(same)
            my = sum(Y(b) for b in same) / len(same)
            d = sorted(((X(b) - mx) ** 2 + (Y(b) - my) ** 2) ** 0.5
                       for b in same)
            dt = ((X(t) - mx) ** 2 + (Y(t) - my) ** 2) ** 0.5
            return dt == d[0] and (len(d) == 1 or d[1] - dt >= tie)
        ax, sign = {"left": (Y, 1), "right": (Y, -1),
                    "front": (X, 1), "back": (X, -1)}[cmd["position"]]
        v = sign * ax(t)
        hit = [b for b in same if sign * ax(b) >= v - tie]
        return len(hit) == 1 and hit[0] is t
    if kind == "stack":
        sign = 1 if cmd["where"] == "top" else -1
        v = sign * TOP(t)
        hit = [b for b in same if sign * TOP(b) >= v - tie]
        return len(hit) == 1 and hit[0] is t
    if kind == "relation":
        others = [b for b in truth["blocks"] if b["color"] == cmd["other"]]
        if not others or cmd["relation"] in ("on", "below"):
            return None                        # 单俯视下 on/below 真值判不了
        # 契约语义（见 stage4_specify 模块 docstring）：「两个颜色的全部有序对上
        # 判，取最稳定的一对」—— 所以这里数的**是成对的间隔**，不是「唯一满足的块」。
        ax, sign = {"left-of": (Y, 1), "right-of": (Y, -1),
                    "front-of": (X, 1), "back-of": (X, -1)}[cmd["relation"]]
        pairs = [(a, sign * (ax(a) - ax(o)))
                 for a in same for o in others
                 if sign * (ax(a) - ax(o)) >= rel_m]
        if not pairs:
            return False                       # 解析器这条命令不可能 ok
        pairs.sort(key=lambda p: -p[1])
        if pairs[0][0] is not t:
            return False                       # 最稳的一对不是目标 ⇒ 指错了
        if len(pairs) == 1:
            return True
        return (pairs[0][1] - pairs[1][1]) >= tie
    if kind == "color":
        if len(same) == 1:
            return True
        if cmd.get("qualifier") == "nearest":
            d = sorted((X(b) ** 2 + Y(b) ** 2) ** 0.5 for b in same)
            dt = (X(t) ** 2 + Y(t) ** 2) ** 0.5
            return dt == d[0] and (len(d) == 1 or d[1] - dt >= tie)
        return None                            # largest / most_visible 真值判不了
    return None


def gen_commands(truth):
    """生成命令集：只留**在真值块表上唯一指向该块**的说法。

    同时跑 ``_indep_unique`` 独立复核；返回的不一致条目由判分当失败处理。
    """
    recs = truth_records(truth)
    by_color = {}
    for b in truth["blocks"]:
        by_color.setdefault(b["color"], []).append(b["id"])
    cmds, skipped, disagree = [], 0, []
    for t in truth["blocks"]:
        for text in _candidate_commands(t, by_color):
            r = sp.select(recs, text)
            if r["status"] == "ok" and r["instance"] == int(t["id"]):
                cmds.append({"text": text, "target": int(t["id"])})
                ok = _indep_unique(text, t, truth)
                if ok is False:
                    disagree.append({"text": text, "target": int(t["id"])})
            else:
                skipped += 1
    return cmds, skipped, recs, disagree


# ---------------------------------------------------------------- 判分
def _geom(inst, t):
    import math
    dxy = ((inst["cx"] - t["x"] * 1000.0) ** 2
           + (inst["cy"] - t["y"] * 1000.0) ** 2) ** 0.5
    dtop = inst["top"] - t["top"] * 1000.0
    dyaw = gc.ang_diff_mod90(inst.get("yaw_rect", inst.get("yaw", 0.0)),
                             float(t["yaw"]))
    return {"dxy_mm": round(dxy, 2), "dtop_mm": round(dtop, 2),
            "dyaw_deg": round(dyaw, 2)}


def eval_frame(label, frame_dir, split=True):
    truth = load_truth(frame_dir)
    insts, _tz = measure(frame_dir, split=split)
    pairs, extra = match(insts, truth)
    # 解析器返回的是**分割实例的 idx**，命令的 target 是**真值 id** —— 两套编号
    # 完全不同（分割按 (cx,cy) 排序）。判分前必须过这道映射，否则全是假错配。
    inst_to_truth = {int(b["idx"]): tid for tid, b in pairs.items()}
    cmds, skipped, recs, disagree = gen_commands(truth)

    rows, counts = [], {"ok": 0, "ambiguous": 0, "none": 0, "bad": 0}
    correct = wrong_ok = auto_correct = 0
    for c in cmds:
        r = sp.select(insts, c["text"])
        status = r["status"]
        counts[status] = counts.get(status, 0) + 1
        got = inst_to_truth.get(int(r["instance"])) if r["instance"] is not None else None
        hit = got == int(c["target"])
        correct += int(hit)
        auto_correct += int(status == "ok" and hit)
        wrong_ok += int(status == "ok" and not hit)
        rows.append({"text": c["text"], "target": c["target"],
                     "status": status, "instance": r["instance"], "truth_id": got,
                     "correct": hit, "confidence": r["confidence"],
                     "reason": r.get("reason", "")})

    geom = []
    for tid, t in [(b["id"], b) for b in truth["blocks"]]:
        inst = pairs.get(int(tid))
        g = {"truth_id": int(tid), "color": t["color"],
             "found": inst is not None, "class_ok": None,
             "seg_id": None, "class": None}
        if inst is not None:
            g.update(_geom(inst, t))
            g["seg_id"] = int(inst["idx"])
            g["class"] = inst["class"]
            g["class_ok"] = inst["class"] == t["color"]
        geom.append(g)

    n_cmd = len(cmds)
    return {
        "label": label, "dir": frame_dir, "batch": truth.get("batch"),
        "counts": counts, "commands": rows,
        "n_commands": n_cmd, "n_skipped_not_unique": skipped,
        "n_indep_disagree": len(disagree), "indep_disagree": disagree,
        "n_truth": len(truth["blocks"]),
        "n_matched": len(pairs), "n_false_pos": len(extra),
        "true_pos": correct, "wrong_ok": wrong_ok,
        "id_rate": (correct / n_cmd) if n_cmd else None,
        "auto_top1_rate": (auto_correct / n_cmd) if n_cmd else None,
        "geom": geom,
    }


def _max(vals):
    return max(vals) if vals else None


def judge_batch(res):
    """单批判据 → (通过?, 失败项列表)。"""
    g = [x for x in res["geom"] if x["found"]]
    fails = []
    if res.get("n_indep_disagree"):
        fails.append("独立复核与解析器在 %d 条命令上不一致（评估器自身可疑）"
                     % res["n_indep_disagree"])
    if res["id_rate"] is None or res["id_rate"] < ID_PASS:
        fails.append("指定 ID %.1f%% < %.0f%%"
                     % (100 * (res["id_rate"] or 0), 100 * ID_PASS))
    if res["wrong_ok"]:
        fails.append("误配 %d 次（status=ok 却选错）" % res["wrong_ok"])
    if len(g) != res["n_truth"]:
        fails.append("漏检 %d 块" % (res["n_truth"] - len(g)))
    for k, lim in GEOM_PASS.items():
        v = _max([x[k] for x in g])
        if v is not None and v > lim:
            fails.append("%s 最大 %.2f > %.1f" % (k, v, lim))
    return not fails, fails


def render(results):
    out = ["| 批次 | 块 | 配对/漏检/多检 | 命令 | ok/歧义/none/bad | 指定 ID | 自主 top-1 "
           "| 误配 | 中心 max | 顶高 max | yaw max | 判定 |",
           "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in results:
        g = [x for x in r["geom"] if x["found"]]
        passed, fails = judge_batch(r)
        out.append("| %s | %d | %d/%d/%d | %d | %d/%d/%d/%d | %s | %s | %d | %s | %s | %s | %s |" % (
            r["label"], r["n_truth"], r["n_matched"],
            r["n_truth"] - r["n_matched"], r["n_false_pos"], r["n_commands"],
            r["counts"].get("ok", 0), r["counts"].get("ambiguous", 0),
            r["counts"].get("none", 0), r["counts"].get("bad", 0),
            "—" if r["id_rate"] is None else "%.1f%%" % (100 * r["id_rate"]),
            "—" if r["auto_top1_rate"] is None else "%.1f%%" % (100 * r["auto_top1_rate"]),
            r["wrong_ok"],
            "—" if not g else "%.2f" % _max([x["dxy_mm"] for x in g]),
            "—" if not g else "%.2f" % _max([x["dtop_mm"] for x in g]),
            "—" if not g else "%.2f" % _max([x["dyaw_deg"] for x in g]),
            "✅" if passed else "❌ " + "；".join(fails)))
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description="E5 实例分割 + 指定解析 评估")
    ap.add_argument("frames", nargs="+", help="label=帧目录，如 s0=.../topcam-home")
    ap.add_argument("--no-split", action="store_true",
                    help="关掉尺寸先验切分（对照 S2）")
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    specs = []
    for a in args.frames:
        if "=" not in a:
            ap.error("帧参数要写成 label=目录：%r" % a)
        label, d = a.split("=", 1)
        d = os.path.expanduser(d)
        if not os.path.isfile(os.path.join(d, "truth.json")):
            print("缺 truth.json：%s" % d, file=sys.stderr)
            return 2
        specs.append((label, d))

    results = [eval_frame(lbl, d, split=not args.no_split) for lbl, d in specs]
    print(render(results))
    if args.json:
        os.makedirs(os.path.dirname(os.path.abspath(args.json)) or ".",
                    exist_ok=True)
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump({"kind": "stage4-e5", "results": results},
                      f, ensure_ascii=False, indent=1)
        print("\n→ %s" % args.json)
    # 未通过的命令明细打出来（最多 10 条），别让人翻 JSON
    bad = [c for r in results for c in r["commands"]
           if not c["correct"] and c["status"] in ("ok", "ambiguous")]
    if bad:
        print("\n未命中明细（%d 条）：" % len(bad))
        for c in bad[:10]:
            print("  %-24s target=%s 命中真值=%s %s %s" % (
                c["text"], c["target"], c["truth_id"], c["status"], c["reason"]))
    return 0 if all(judge_batch(r)[0] for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())

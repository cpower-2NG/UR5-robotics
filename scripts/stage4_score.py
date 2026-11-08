#!/usr/bin/env python3
"""阶段 4 出口判分器（E13）：块级尝试 → 四条硬门 + 两条零容忍 + 人眼复核。

判据逐字取自契约 §4.3（S0–S2 硬出口）：

| 判据 | 定义 | 门 |
|---|---|---|
| 指定 ID 正确率 | 系统选中的实例 = 独立真值指定的实例 | ≥90% |
| 抓取成功率 | 指定块被抓出、抬到验证位、未掉、未抓错 | ≥80% |
| 放置格成功率 | 指定块落在指定格内（不压线、不越界） | ≥90% |
| 全桌合法终态率 | 所有块在合法格 / 自由区，不重叠、不落地、不越界 | ≥90% |
| 安全事件 | 碰撞、掉落、非预期运动、急停 | **0** |
| 人工干预 | 除授权 / 摆场景 / 确认外的动作 | **0** |
| 自主 top-1 | 不追问直接选中 | 单独报，**不设硬门** |

三条口径（契约 §4.1 / §4.2，别自己发明）：

1. **分母 = 该轮全部指定块**（块级尝试），不剔除失败块；只有 ``void``
   （外部原因未完成）不进分母，且要记原因。**同一判据连续 3 次 void ⇒ 停**。
2. **``rc=0`` 不算成功**。抓取成功必须由**俯视 + 腕部 + 人眼**三方确认
   （见 ``CONFIRMS``）；缺任何一路 ⇒ 这条尝试进 ``person_review``，不能算绿。
3. **测量来源不得复用控制感知**（E13 的存在理由）。真值必须带
   ``truth_source``，且只认 ``TRUTH_SOURCES`` 里的独立来源；控制链自己的读数
   写进来当"真值" ⇒ 直接 ``missing_data``。**这条是判分器的死穴**：如果真值和
   系统选择同源，判分器只是在给自己盖章 —— 阶段 4 的 E5 评估器已经栽过一次
   （台账第二百七十七轮，独立复核揪出 289 条分歧）。

退出码：0 = 该轮全过；1 = 有不过 / 待人评；2 = 数据缺失或坏。

轮次达标（契约 §4.4）：S0–S2 每批 3 布局 × 2 次，出口取**连续 3 轮**，
中间**不得改阈值 / 改代码**。``score_exit`` 干这件事：它拿每轮的
``thresholds`` 与 ``code_hash`` 比，变了就把连续计数清零重来。

用法：
  python3 scripts/stage4_score.py <round.json> [--json out] [--md out.md]
  python3 scripts/stage4_score.py round-01.json round-02.json round-03.json --exit
"""
import argparse
import hashlib
import json
import os
import sys

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))

# 契约 §4.3 的门。改这里 = 改判据，必须用户显式授权并记录生效轮次（契约 §4.4）。
DEFAULT_THRESHOLDS = {
    "id_rate": 0.90,
    "grasp_rate": 0.80,
    "place_rate": 0.90,
    "terminal_rate": 0.90,
    "safety_events": 0,
    "interventions": 0,
}

# 契约 §4.1：失败必须分类，不能只报「失败」。
FAIL_CLASSES = ("perception", "candidate", "ik", "planning",
                "execution", "placement", "human")

# 契约 §4.2.1：抓取成功要三路确认。
CONFIRMS = ("topcam", "wrist", "human")

# 允许当"独立真值"的来源。控制链（perception_3d / grasp_planner / 规划器）
# 的读数**不在这个名单里**，写进来即判数据坏。
TRUTH_SOURCES = ("topcam-independent", "human-annotation",
                 "layout-manifest+topcam")

SAFETY_KEYS = ("collisions", "drops", "unexpected_motion", "estop")

# 尝试级状态
A_OK, A_FAIL, A_REVIEW, A_VOID, A_MISSING = (
    "ok", "fail", "person_review", "void", "missing_data")

# 轮级 verdict → 退出码
_EXIT = {"ok": 0, "fail": 1, "person_review": 1, "missing_data": 2}

VOID_STREAK_STOP = 3


# ---------------------------------------------------------------- 小工具
def _num(v):
    """数字或 None。字符串数字也收（JSON 里常见）。"""
    if v is None or isinstance(v, bool):
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f


def _truth_instance(rec):
    """独立真值里「命令到底指哪一块」的实例号。"""
    t = rec.get("truth") or {}
    return t.get("instance_id")


def _selected_instance(rec):
    s = rec.get("selected") or {}
    return s.get("instance_id")


def code_hash():
    """判分器自身（含阈值）的指纹，用来判「中间有没有改代码」。"""
    h = hashlib.sha256()
    with open(os.path.abspath(__file__), "rb") as f:
        h.update(f.read())
    h.update(json.dumps(DEFAULT_THRESHOLDS, sort_keys=True).encode())
    return h.hexdigest()[:16]


# ---------------------------------------------------------------- 尝试级
def score_attempt(rec, i=0):
    """一次块级尝试 → 一行结果。"""
    row = {
        "i": i,
        "target": (rec.get("target") or {}).get("block_id"),
        "spec": rec.get("spec"),
        "status": A_OK,
        "reasons": [],
        "id_ok": None, "grasp_ok": None, "place_ok": None,
        "auto_top1": bool((rec.get("selected") or {}).get("auto_top1")),
        "failure_class": (rec.get("failure") or {}).get("class"),
    }

    void = rec.get("void") or {}
    if void.get("reason"):
        row["status"] = A_VOID
        row["reasons"].append("void:%s" % void["reason"])
        return row

    # ---- 真值来源闸门（E13 的死穴）
    tsrc = (rec.get("truth") or {}).get("truth_source")
    if tsrc not in TRUTH_SOURCES:
        row["status"] = A_MISSING
        row["reasons"].append("truth_source=%r 不在独立来源白名单" % (tsrc,))
        return row
    t_id, s_id = _truth_instance(rec), _selected_instance(rec)
    if t_id is None:
        row["status"] = A_MISSING
        row["reasons"].append("truth.instance_id 缺失")
        return row
    row["id_ok"] = (s_id is not None) and (int(s_id) == int(t_id))

    # ---- 抓取（三路确认，rc=0 不够）
    gr = rec.get("grasp") or {}
    conf = gr.get("confirm") or {}
    missing_conf = [k for k in CONFIRMS if conf.get(k) is not True]
    moved = bool(gr.get("lifted")) and not bool(gr.get("dropped")) \
        and not bool(gr.get("wrong_block")) and int(gr.get("exec_rc", 1)) == 0
    if not moved:
        # 没抬起来（或掉了 / 抓错 / rc≠0）就是失败，不需要确认也成立
        row["grasp_ok"] = False
    elif missing_conf:
        # 抬起来了但没三路确认 ⇒ 不能算成功，交人（不能默认绿）
        row["grasp_ok"] = None
        row["status"] = A_REVIEW
        row["reasons"].append("抓取未三路确认：缺 %s" % ",".join(missing_conf))
    else:
        row["grasp_ok"] = True

    # ---- 放置
    pl = rec.get("place") or {}
    row["place_ok"] = (pl.get("in_slot") is True
                       and pl.get("over_line") is not True
                       and pl.get("out_of_bounds") is not True)

    # ---- 失败必须分类
    failed = row["id_ok"] is False or row["grasp_ok"] is False \
        or row["place_ok"] is False
    if failed:
        if row["failure_class"] not in FAIL_CLASSES:
            row["status"] = A_REVIEW
            row["reasons"].append("失败未分类（failure.class=%r）"
                                  % (row["failure_class"],))
        elif row["status"] != A_REVIEW:
            row["status"] = A_FAIL
        if row["id_ok"] is False:
            row["reasons"].append("指定 ID 选错")
        if row["grasp_ok"] is False:
            row["reasons"].append("抓取失败")
        if row["place_ok"] is False:
            row["reasons"].append("放置格不中")
    return row


# ---------------------------------------------------------------- 轮级
def _terminal(obs):
    """全桌终态 → (ok?, 原因列表)。缺失 = 缺数据，不是失败。"""
    t = obs.get("terminal")
    if not t:
        return None, ["terminal 缺失"]
    blocks = t.get("blocks")
    if not isinstance(blocks, list) or not blocks:
        return None, ["terminal.blocks 缺失"]
    reasons = []
    illegal = [b for b in blocks
               if (b.get("zone") not in ("slot", "free")
                   or b.get("in_bounds") is not True
                   or b.get("legal") is False)]
    for k in ("overlaps", "off_table", "out_of_bounds"):
        v = _num(t.get(k))
        if v is None:
            return None, ["terminal.%s 缺失" % k]
        if v > 0:
            reasons.append("terminal.%s=%g" % (k, v))
    if illegal:
        reasons.append("非合法终态块 %d 个" % len(illegal))
    return (not reasons), reasons


def _safety(obs):
    s = obs.get("safety") or {}
    out, missing = 0, []
    for k in SAFETY_KEYS:
        v = _num(s.get(k))
        if v is None:
            missing.append(k)
            continue
        out += v
    return (None if missing else out), missing


def _interventions(obs):
    iv = obs.get("interventions") or {}
    v = _num(iv.get("count"))
    if v is None:
        return None
    return v


def _rate(num, den):
    return None if not den else num / float(den)


def _void_streak(rows):
    """最长连续 void 数（按尝试顺序）。同一判据连续 3 次 void ⇒ 停。"""
    best = cur = 0
    for r in rows:
        cur = cur + 1 if r["status"] == A_VOID else 0
        best = max(best, cur)
    return best


def score_round(obs, thresholds=None):
    """一轮 → 判分结果（不写文件）。"""
    th = dict(DEFAULT_THRESHOLDS)
    th.update(thresholds or {})
    raw = obs.get("attempts") or []
    rows = [score_attempt(r, i) for i, r in enumerate(raw)]
    n_targets = int(_num(obs.get("n_targets")) or len(raw))

    # void 不进分母；missing 也不能算进分母（它是数据坏，不是系统差）
    scored = [r for r in rows if r["status"] not in (A_VOID, A_MISSING)]
    den = len(scored)
    n_id = sum(1 for r in scored if r["id_ok"] is True)
    n_grasp = sum(1 for r in scored if r["grasp_ok"] is True)
    n_place = sum(1 for r in scored if r["place_ok"] is True)
    n_auto = sum(1 for r in scored if r["auto_top1"] and r["id_ok"] is True)

    term_ok, term_reasons = _terminal(obs)
    n_term = sum(1 for r in scored if term_ok is True)
    safety, safety_missing = _safety(obs)
    inter = _interventions(obs)
    review = obs.get("human_review") or {}

    rates = {
        "id_rate": _rate(n_id, den),
        "grasp_rate": _rate(n_grasp, den),
        "place_rate": _rate(n_place, den),
        "terminal_rate": _rate(n_term, den),
        "auto_top1_rate": _rate(n_auto, den),
        "void_rate": _rate(len(rows) - len(scored), len(rows)),
    }

    verdict, reason = "ok", ""
    if not raw:
        verdict, reason = "missing_data", "attempts 为空"
    elif len(raw) != n_targets:
        verdict, reason = ("missing_data",
                           "尝试数 %d ≠ 指定块数 %d" % (len(raw), n_targets))
    elif any(r["status"] == A_MISSING for r in rows):
        verdict, reason = "missing_data", "尝试缺真值/来源不合规"
    elif term_ok is None:
        verdict, reason = "missing_data", "；".join(term_reasons)
    elif safety is None:
        verdict, reason = "missing_data", "safety 缺字段：%s" % ",".join(safety_missing)
    elif inter is None:
        verdict, reason = "missing_data", "interventions.count 缺失"
    elif _void_streak(rows) >= VOID_STREAK_STOP:
        verdict, reason = ("fail",
                           "同一判据连续 %d 次 void ⇒ 停" % VOID_STREAK_STOP)
    elif safety > th["safety_events"]:
        verdict, reason = "fail", "安全事件 %g > %g" % (safety, th["safety_events"])
    elif inter > th["interventions"]:
        verdict, reason = "fail", "人工干预 %g > %g" % (inter, th["interventions"])
    else:
        # 四条率门（自主 top-1 不设门；void 不进分母，但全 void 时率是 None）
        for key, label in (("id_rate", "指定 ID"), ("grasp_rate", "抓取成功率"),
                           ("place_rate", "放置格成功率"),
                           ("terminal_rate", "全桌合法终态率")):
            v = rates[key]
            if v is None:
                verdict, reason = "missing_data", "%s 分母为 0（全 void？）" % label
                break
            if v < th[key]:
                verdict, reason = "fail", "%s %.1f%% < %.0f%%" % (
                    label, 100 * v, 100 * th[key])
                break
    if verdict == "ok":
        if review.get("approved") is not True:
            verdict, reason = "person_review", "人眼复核未批准"
        elif any(r["status"] == A_REVIEW for r in rows):
            verdict, reason = "person_review", "有尝试待人评"

    return {
        "version": 1, "kind": "stage4-score",
        "round": obs.get("round"), "batch": obs.get("batch"),
        "layout": obs.get("layout"), "repeat": obs.get("repeat"),
        "code_hash": obs.get("code_hash") or code_hash(),
        "thresholds": th,
        "n_targets": n_targets, "n_attempts": len(raw),
        "n_scored": den, "n_void": len(rows) - len(scored),
        "void_streak": _void_streak(rows),
        # 这一轮算不算「void 轮」：轮内连 3 次 void，或整轮全 void
        # （整轮全 void 在轮内数不出 3 连 —— 只有 2 块时永远数不到 3，
        #   所以整轮口径单独算一条）。连续 3 个 void 轮 ⇒ 停，见 score_exit。
        "void_round": (_void_streak(rows) >= VOID_STREAK_STOP
                       or (len(rows) > 0 and den == 0)),
        "counts": {"id_ok": n_id, "grasp_ok": n_grasp, "place_ok": n_place,
                   "auto_top1": n_auto, "terminal_ok": n_term},
        "rates": rates,
        "safety": safety, "safety_missing": safety_missing,
        "interventions": inter,
        "terminal_ok": term_ok, "terminal_reasons": term_reasons,
        "human_review": obs.get("human_review"),
        "attempts": rows,
        "verdict": verdict, "reason": reason,
    }


# ---------------------------------------------------------------- 出口（连续 3 轮）
def score_exit(rounds, need=3):
    """契约 §4.4：出口取**连续 3 轮**，中间不得改阈值 / 改代码。

    ``rounds`` 是 ``score_round`` 的产物（按时间先后）。返回连续计数与是否达标。
    计数规则：任一轮 ``verdict != "ok"``、或它的 ``thresholds`` / ``code_hash``
    与计数起点不一致 ⇒ 连续计数清零、从该轮重新起算。
    """
    streak, best, start = 0, 0, None
    vrun = 0                    # 连续几个「void 轮」
    stop_reason = ""
    marks = []
    for r in rounds:
        # 契约 §4.1：同一判据连续 3 次 void ⇒ 停。整轮 void 也算这一次。
        vrun = vrun + 1 if r.get("void_round") else 0
        if vrun >= VOID_STREAK_STOP:
            stop_reason = "同一判据连续 %d 轮 void ⇒ 停" % vrun
        same = (start is None
                or (r.get("thresholds") == start["thresholds"]
                    and r.get("code_hash") == start["code_hash"]))
        if r.get("verdict") == "ok" and same:
            streak += 1
            if start is None:
                start = {"thresholds": r.get("thresholds"),
                         "code_hash": r.get("code_hash"),
                         "round": r.get("round")}
            marks.append("ok")
        else:
            # 改判据 / 改代码 / 失败 —— 一律从这一轮重来
            streak = 1 if r.get("verdict") == "ok" else 0
            start = ({"thresholds": r.get("thresholds"),
                      "code_hash": r.get("code_hash"),
                      "round": r.get("round")} if streak else None)
            marks.append("restart:%s" % (r.get("reason") or "threshold_or_code_changed"))
        best = max(best, streak)
    return {"version": 1, "kind": "stage4-exit",
            "need": need, "streak": streak, "best": best,
            "void_run": vrun, "stop": bool(stop_reason),
            "stop_reason": stop_reason,
            "ok": (streak >= need) and not stop_reason, "marks": marks,
            "rounds": [{"round": r.get("round"), "batch": r.get("batch"),
                        "layout": r.get("layout"),
                        "verdict": r.get("verdict"),
                        "reason": r.get("reason")} for r in rounds]}


def exit_code(res):
    return _EXIT.get(res.get("verdict"), 2)


# ---------------------------------------------------------------- 报告
def _pct(v):
    return "—" if v is None else "%.1f%%" % (100 * v)


def _md(res):
    if res.get("kind") == "stage4-exit":
        out = ["# 阶段 4 出口连续轮次", "",
               "需要连续 **%d** 轮全过；当前连续 **%d** 轮，历史最好 **%d** 轮 ⇒ %s"
               % (res["need"], res["streak"], res["best"],
                  "**达标**" if res["ok"] else "**未达标**"), ""]
        if res.get("stop_reason"):
            out += ["> ⛔ %s" % res["stop_reason"], ""]
        out += ["| 轮 | 批 | 布局 | 判定 | 原因 |", "|---|---|---|---|---|"]
        for r in res["rounds"]:
            out.append("| %s | %s | %s | %s | %s |" % (
                r["round"], r["batch"], r["layout"], r["verdict"],
                r["reason"] or ""))
        return "\n".join(out) + "\n"

    th = res["thresholds"]
    rt = res["rates"]
    out = ["# 阶段 4 判分 · 第 %s 轮（%s / %s / 第 %s 次）" % (
        res["round"], res["batch"], res["layout"], res["repeat"]), "",
        "**判定：%s**%s" % (res["verdict"],
                         "（%s）" % res["reason"] if res["reason"] else ""), "",
        "分母 = %d 块（指定 %d，void %d，未进分母 %d）；"
        "安全事件 %s；人工干预 %s；自主 top-1 %s（不设门）"
        % (res["n_scored"], res["n_targets"], res["n_void"],
           res["n_attempts"] - res["n_scored"],
           "—" if res["safety"] is None else "%g" % res["safety"],
           "—" if res["interventions"] is None else "%g" % res["interventions"],
           _pct(rt["auto_top1_rate"])), "",
        "| 判据 | 命中/分母 | 实际 | 门 | 结论 |", "|---|---|---|---|---|"]
    for key, label, ck in (("id_rate", "指定 ID 正确率", "id_ok"),
                           ("grasp_rate", "抓取成功率", "grasp_ok"),
                           ("place_rate", "放置格成功率", "place_ok"),
                           ("terminal_rate", "全桌合法终态率", "terminal_ok")):
        v = rt[key]
        out.append("| %s | %d/%d | %s | ≥%.0f%% | %s |" % (
            label, res["counts"][ck], res["n_scored"], _pct(v),
            100 * th[key], "✅" if (v is not None and v >= th[key]) else "❌"))
    out.append("| 安全事件 | — | %s | =0 | %s |" % (
        res["safety"], "✅" if res["safety"] == 0 else "❌"))
    out.append("| 人工干预 | — | %s | =0 | %s |" % (
        res["interventions"], "✅" if res["interventions"] == 0 else "❌"))
    out += ["", "## 逐块", "",
            "| # | 目标块 | 指定说法 | ID | 抓取 | 放置 | 自主 | 状态 | 原因 |",
            "|---|---|---|---|---|---|---|---|---|"]
    for a in res["attempts"]:
        out.append("| %d | %s | %s | %s | %s | %s | %s | %s | %s |" % (
            a["i"], a["target"], a["spec"],
            {True: "✅", False: "❌"}.get(a["id_ok"], "—"),
            {True: "✅", False: "❌"}.get(a["grasp_ok"], "—"),
            {True: "✅", False: "❌"}.get(a["place_ok"], "—"),
            "✅" if a["auto_top1"] else "—",
            a["status"], "；".join(a["reasons"])))
    return "\n".join(out) + "\n"


def load_round(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def write_outputs(res, prefix):
    paths = []
    with open(prefix + "-score.json", "w", encoding="utf-8") as f:
        json.dump(res, f, ensure_ascii=False, indent=1)
    paths.append(prefix + "-score.json")
    with open(prefix + "-score.md", "w", encoding="utf-8") as f:
        f.write(_md(res))
    paths.append(prefix + "-score.md")
    return paths


def main(argv=None):
    ap = argparse.ArgumentParser(description="阶段 4 出口判分（E13）")
    ap.add_argument("rounds", nargs="+", help="round.json，按时间先后")
    ap.add_argument("--exit", action="store_true",
                    help="按契约 §4.4 算连续 3 轮达标")
    ap.add_argument("--need", type=int, default=3)
    ap.add_argument("--out-prefix", default=None,
                    help="写出 <prefix>-score.json/.md（单轮用）")
    ap.add_argument("--json", default=None)
    args = ap.parse_args(argv)

    try:
        obs_list = [load_round(p) for p in args.rounds]
    except (OSError, ValueError) as e:
        print("读不了轮次文件：%s" % e, file=sys.stderr)
        return 2

    results = [score_round(o) for o in obs_list]
    if args.exit:
        res = score_exit(results, need=args.need)
        print(_md(res))
        code = 0 if res["ok"] else 1
    else:
        res = results[0]
        print(_md(res))
        if args.out_prefix:
            for p in write_outputs(res, args.out_prefix):
                print("→ %s" % p)
        code = exit_code(res)
    if args.json:
        os.makedirs(os.path.dirname(os.path.abspath(args.json)) or ".",
                    exist_ok=True)
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(res, f, ensure_ascii=False, indent=1)
        print("→ %s" % args.json)
    return code


if __name__ == "__main__":
    sys.exit(main())

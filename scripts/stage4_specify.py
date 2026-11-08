#!/usr/bin/env python3
"""阶段 4 指定通道：**结构化命令 → 唯一实例**（契约 §1 的语法）。

只做两件事，分开可测：
  1. ``parse_command(text)`` —— 文本 → 结构化意图；语法外的输入**明确报错**，
     不猜、不降级（结构化语法是本阶段的核心，自由语言后置）。
  2. ``select(blocks, cmd)``   —— 意图 + 已分割的块表 → 实例；返回状态、实例 ID、
     回显字段（颜色 / base 中心 / 顶高 / yaw / 可见比例 / 置信度）。
     **0 匹配不猜、差距小要人确认**（契约 §6 失败模式表）。

坐标与方位（本模块**唯一定义处**，别处引用这句）
──────────────────────────────────────────────
全部用 **base 系**，单位毫米：``+x = 前``、``+y = 左``（ROS 惯例，机器人视角）。
- ``<color> <position>`` 的方位是在**同色实例之间**比的：``left`` = y 最大、
  ``right`` = y 最小、``front`` = x 最大、``back`` = x 最小、``center`` = 离这组
  同色实例的形心最近。只有一个同色实例时，方位条件恒成立（相当于多余的确认）。
- ``<color> <relation> <color>`` 在**两个颜色的全部有序对**上判，取最稳定的一对。

可见比例（不是判据，只用于 ``most_visible`` 排序与回显）
──────────────────────────────────────────────────────
``visible = n_px·px_area / block_mm² / VIS_FULL_REF``，封顶 1。``VIS_FULL_REF`` 是
**实测的满可见参考**：2026-11-08 在 E6 的 S0–S2 三帧十七块上量得 0.78–0.81
（顶面掩码只取 z > ztop−3 mm，边缘点被切掉，所以满可见也到不了 1）。它不是物理
可见率，只用来在同一帧内排序；跨相机/跨高度要先重新量。

用法：
  python3 scripts/stage4_specify.py <帧目录> "red left"
  python3 scripts/stage4_specify.py <帧目录> --check "yellow on blue"
"""
import argparse
import json
import os
import re
import sys

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import block_classes                                              # noqa: E402
import graspnet_candidates as gc                                  # noqa: E402

# 拾取区（base 系，米）：契约 §1 逐字沿用。与 stage4_score / 入口实验同一常量。
ZONE_M = (0.25, 0.575, 0.17, 0.42)

BLOCK_MM = 29.5
VIS_FULL_REF = 0.79          # 满可见参考，见模块 docstring

POSITIONS = ("left", "right", "front", "back", "center")
# 归一化后的限定词写法 → 规范名
QUALIFIERS = {"nearest": "nearest", "largest": "largest",
              "mostvisible": "most_visible"}

# 关系词归一化：允许多种写法，但只映射到下面这几个谓词；其余一律报错。
# 键都是 ``_norm`` + ``_PHRASE_SUBS`` 之后的形式（连字符/下划线已变空格、
# 多词短语已拼成一个词）。裸 ``left``/``right`` **不算**关系词 —— 那是方位，
# 关系必须写成 ``left-of``，否则 ``red left green`` 会有两种读法。
RELATIONS = {
    "leftof": "left-of",
    "rightof": "right-of",
    "frontof": "front-of",
    "backof": "back-of",
    "behind": "back-of",
    "on": "on",
    "above": "on",
    "ontopof": "on",
    "topof": "on",
    "under": "below",
    "below": "below",
    "underneath": "below",
}

# 多词短语先拼成一个词，再分词。顺序有意义（长的在前）。
_PHRASE_SUBS = [
    (r"\bin front of\b", "frontof"),
    (r"\bon top of\b", "ontopof"),
    (r"\bleft of\b", "leftof"),
    (r"\bright of\b", "rightof"),
    (r"\bfront of\b", "frontof"),
    (r"\bback of\b", "backof"),
    (r"\btop of\b", "topof"),
    (r"\bmost visible\b", "mostvisible"),
]

# 关系判定的最小间隔：两块中心在这个距离内就算「并排」而不是「在左/右」。
# 29.5 mm 的块并排时中心相距 29.5 mm；取 5 mm 只挡「几乎重合」的噪声对。
REL_MARGIN_MM = 5.0

# 排序类命令（方位 / 关系 / 限定词）的「够不够拉开」阈值：
# 第一名与第二名的差小于它 ⇒ 判为歧义，交人确认，不硬选。
TIE_MM = 5.0
TIE_REL = 0.05               # largest / most_visible 用相对差


class CommandError(ValueError):
    """语法外 / 不可能解析的输入。调用方必须把它当「不可判」，不能当「没有块」。"""


def _norm(text):
    """小写、词内连字符/下划线当空格、压空白 —— 但**不删字符**，删了就猜了。

    只在**两个字母之间**替换：``-12.5`` 的负号必须留住（否则 ``at -12,300``
    会静默变成 ``at 12,300``，选到桌子的另一头）。
    """
    s = str(text).strip().lower()
    s = re.sub(r"(?<=[a-z_])[_-](?=[a-z_])", " ", s)
    s = re.sub(r"\s+", " ", s)
    for pat, rep in _PHRASE_SUBS:
        s = re.sub(pat, rep, s)
    return s


def parse_command(text, classes=None):
    """文本 → 意图字典；语法外一律 ``CommandError``（带原话，方便人改）。

    **词表**是整张 ``PALETTE``（不只当前激活的类别表）：``yellow on blue`` 是契约里
    的合法说法，即使本批没有蓝块；``select`` 会回 ``none``（"没有 blue 块"），
    而不是把一句语法正确的话判成语法错。激活类别表只影响**能分割出什么**。
    """
    classes = list(classes or block_classes.DEFAULT_CLASSES)
    vocabulary = classes + [n for n in block_classes.PALETTE if n not in classes]
    raw = str(text)
    s = _norm(raw)
    if not s:
        raise CommandError("空命令")
    tokens = s.split(" ")

    # ---- at <x>,<y> / at <x> <y> --------------------------------------
    if tokens[0] == "at":
        rest = tokens[1:]
        if len(rest) == 1:
            coord = rest[0].split(",")
        elif len(rest) == 2 and "," not in rest[0]:
            coord = rest
        else:
            coord = [tok for part in rest for tok in part.split(",") if tok]
        if len(coord) != 2:
            raise CommandError("at 后面要两个数：at <x_mm>,<y_mm>，收到 %r" % raw)
        try:
            x, y = float(coord[0]), float(coord[1])
        except ValueError:
            raise CommandError("at 的坐标不是数字：%r" % raw)
        return {"kind": "at", "x": x, "y": y, "raw": raw}

    # ---- <color> ... ---------------------------------------------------
    if tokens[0] not in vocabulary:
        raise CommandError("不认识的颜色 %r（可用：%s）"
                           % (tokens[0], ", ".join(vocabulary)))
    color = tokens[0]
    body = tokens[1:]

    qualifier = None
    if body and body[-1] in QUALIFIERS:
        qualifier = QUALIFIERS[body[-1]]
        body = body[:-1]

    if not body:
        # 裸颜色：合法但通常歧义（多实例时由 select 判 ambiguous）
        return {"kind": "color", "color": color, "qualifier": qualifier,
                "raw": raw}

    if len(body) == 1:
        if body[0] in POSITIONS:
            return {"kind": "position", "color": color,
                    "position": body[0], "qualifier": qualifier, "raw": raw}
        if body[0] in ("top", "bottom"):
            return {"kind": "stack", "color": color,
                    "where": body[0], "qualifier": qualifier, "raw": raw}
        if body[0] in RELATIONS:
            raise CommandError("关系词 %r 后面缺少另一个颜色：%s <relation> <color>"
                               % (body[0], color))
        raise CommandError("看不懂 %r：期望 %s <left|right|front|back|center|top|"
                           "bottom|nearest|largest|most_visible>" % (raw, color))

    if len(body) == 2:
        rel = RELATIONS.get(body[0])
        other = body[1]
        if rel is None:
            raise CommandError("不认识的关系词 %r（可用：%s）"
                               % (body[0], ", ".join(sorted(set(RELATIONS.values())))))
        if other not in vocabulary:
            raise CommandError("关系后面的 %r 不是颜色（可用：%s）"
                               % (other, ", ".join(vocabulary)))
        return {"kind": "relation", "color": color, "relation": rel,
                "other": other, "qualifier": qualifier, "raw": raw}

    raise CommandError("词太多，看不懂：%r" % raw)


# ------------------------------------------------------------------ 选择
def _in_zone(b, zone_m=ZONE_M):
    x0, x1, y0, y1 = (v * 1000.0 for v in zone_m)
    return x0 <= b["cx"] <= x1 and y0 <= b["cy"] <= y1


def usable_blocks(blocks, zone_m=ZONE_M):
    """能参与指定的块：不是 low（低处碎片/平板）、在拾取区内。"""
    return [b for b in blocks if not b.get("low") and _in_zone(b, zone_m)]


def visible_ratio(b, block_mm=BLOCK_MM):
    px = b.get("px_area_mm2")
    if not px:
        return None
    v = float(b["n"]) * float(px) / (block_mm * block_mm) / VIS_FULL_REF
    return max(0.0, min(1.0, v))


def _echo(b, block_mm=BLOCK_MM):
    return {"id": int(b["idx"]), "color": b["class"],
            "x": round(float(b["cx"]), 1), "y": round(float(b["cy"]), 1),
            "top": round(float(b["top"]), 1),
            "yaw": round(float(b.get("yaw_rect", b.get("yaw", 0.0))), 1),
            "visible": (None if visible_ratio(b, block_mm) is None
                        else round(visible_ratio(b, block_mm), 2)),
            "class_share": b.get("class_share")}


def _result(status, block=None, reason="", candidates=(), confidence=0.0,
            block_mm=BLOCK_MM):
    out = {"status": status, "instance": None, "reason": reason,
           "confidence": round(float(confidence), 3),
           "candidates": [_echo(c, block_mm) for c in candidates]}
    if block is not None:
        out["instance"] = int(block["idx"])
        out["block"] = _echo(block, block_mm)
    return out


def _pick_best(scored, key):
    """``[(block, margin)]`` → 最优块；差 < TIE_* 判歧义。

    ``key``: 'mm' 用绝对差、'rel' 用相对差。返回 ``(block, margin, ambiguous)``；
    **只有一个候选时 margin 返回 None**（没有第二名可比，不能拿绝对坐标当间隔）。
    """
    if not scored:
        return None, None, False
    scored = sorted(scored, key=lambda t: -t[1])
    best, m1 = scored[0]
    m2 = scored[1][1] if len(scored) > 1 else None
    if m2 is None:
        return best, None, False
    if key == "rel":
        # m1/m2 是比例量（largest/most_visible）：相对差 < 5% 算平手
        gap = (m1 - m2) / m1 if m1 > 0 else 0.0
        amb = gap < TIE_REL
        margin = gap
    else:
        amb = (m1 - m2) < TIE_MM
        margin = m1 - m2
    return best, margin, amb


def _conf(margin, ref, ambiguous):
    """置信度：拉开一倍参考间隔封顶 1.0；歧义时压在 0.5 以下。

    ``ref <= 0`` 只会在阈值被人为改成 0 时出现（测试的变异场景），此时没法定
    0-1 的比例 —— 直接按「确定 / 不确定」给端点值，不要除零。
    """
    if margin is None:                 # 只有一个候选：没有第二名可比
        return 0.0 if ambiguous else 1.0
    if ref <= 0:
        return 0.0 if ambiguous else 1.0
    if ambiguous:
        return max(0.0, min(0.49, 0.49 * margin / ref))
    return max(0.5, min(1.0, 0.5 + 0.5 * min(1.0, margin / ref)))


def _score_qualifier(cands, qual):
    """限定词 → ``[(block, 分数)]``（分数越大越好）与「差」的量纲（mm / rel）。

    · ``nearest`` —— 离 base 原点（机器人底座）水平距离最近；
    · ``largest`` —— 顶面外接矩形面积最大；
    · ``most_visible`` —— 见 ``visible_ratio``。
    """
    if qual == "nearest":
        return [(b, -((b["cx"] ** 2 + b["cy"] ** 2) ** 0.5)) for b in cands], "mm"
    if qual == "largest":
        return [(b, b["ext_rect"][0] * b["ext_rect"][1]) for b in cands], "rel"
    if qual == "most_visible":
        return [(b, visible_ratio(b) or 0.0) for b in cands], "rel"
    raise CommandError("未知限定词 %r" % qual)


def _select_from(cands, qual, block_mm):
    """候选集 → 结果。``cands`` 已按主判据排好序。"""
    if len(cands) == 1:
        # 只有一个候选：有没有限定词都是确定的（限定词无从比较，不算不确定）
        return _result("ok", cands[0], confidence=1.0, candidates=cands,
                       block_mm=block_mm)
    if qual is None:
        return _result("ambiguous", cands[0],
                       reason="%d 个实例同样满足，缺方位/限定词" % len(cands),
                       candidates=cands, confidence=0.3, block_mm=block_mm)
    scored, key = _score_qualifier(cands, qual)
    ordered = [b for b, _ in sorted(scored, key=lambda t: -t[1])]
    best, margin, amb = _pick_best(scored, key)
    ref = TIE_MM if key == "mm" else 0.25
    return _result("ambiguous" if amb else "ok", best,
                   reason="" if not amb else "前两名按 %s 只差 %s"
                   % (qual, "%.1f mm" % margin if key == "mm"
                      else "%.0f%%" % (margin * 100)),
                   candidates=ordered,
                   confidence=_conf(margin, ref, amb), block_mm=block_mm)


def select(blocks, cmd, zone_m=ZONE_M, block_mm=BLOCK_MM):
    """已分割块表 + 意图 → 选择结果（dict）。

    返回字段：``status`` ∈ ok / ambiguous / none / bad；``instance``（块表里的 idx）；
    ``block``（回显）；``candidates``；``confidence``；``reason``。
    """
    if isinstance(cmd, str):
        try:
            cmd = parse_command(cmd)
        except CommandError as e:
            return {"status": "bad", "instance": None, "reason": str(e),
                    "confidence": 0.0, "candidates": []}

    pool = usable_blocks(blocks, zone_m)
    kind = cmd["kind"]

    if kind == "at":
        if not pool:
            return _result("none", reason="拾取区内没有块", block_mm=block_mm)
        near = sorted(pool, key=lambda b: (b["cx"] - cmd["x"]) ** 2
                                        + (b["cy"] - cmd["y"]) ** 2)
        d0 = ((near[0]["cx"] - cmd["x"]) ** 2
              + (near[0]["cy"] - cmd["y"]) ** 2) ** 0.5
        # 容差：块心到角点约 block_mm·0.707；取 0.8 倍块边，点中块面都算命中
        tol = block_mm * 0.8
        if d0 > tol:
            return _result("none", reason="离 (%.1f, %.1f) 最近的块有 %.1f mm，超过 %.0f"
                           % (cmd["x"], cmd["y"], d0, tol),
                           candidates=near[:3], block_mm=block_mm)
        scored = [(b, -((b["cx"] - cmd["x"]) ** 2 + (b["cy"] - cmd["y"]) ** 2) ** 0.5)
                  for b in near]
        best, margin, amb = _pick_best(scored, "mm")
        return _result("ambiguous" if amb else "ok", best,
                       reason="" if not amb else "两块离指定点几乎一样近",
                       candidates=near[:3],
                       confidence=_conf(margin, TIE_MM, amb), block_mm=block_mm)

    same = [b for b in pool if b["class"] == cmd["color"]]
    if not same:
        return _result("none", reason="拾取区内没有 %s 块" % cmd["color"],
                       block_mm=block_mm)

    if kind == "color":
        return _select_from(same, cmd.get("qualifier"), block_mm)

    if kind == "position":
        pos = cmd["position"]
        qual = cmd.get("qualifier")
        if pos == "left":
            scored = [(b, b["cy"]) for b in same]
        elif pos == "right":
            scored = [(b, -b["cy"]) for b in same]
        elif pos == "front":
            scored = [(b, b["cx"]) for b in same]
        elif pos == "back":
            scored = [(b, -b["cx"]) for b in same]
        else:                                   # center
            mx = sum(b["cx"] for b in same) / len(same)
            my = sum(b["cy"] for b in same) / len(same)
            scored = [(b, -((b["cx"] - mx) ** 2 + (b["cy"] - my) ** 2) ** 0.5)
                      for b in same]
        best, margin, amb = _pick_best(scored, "mm")
        if qual is not None and amb:
            # 方位分不出高下时，交给限定词；限定词也分不出就仍判歧义
            return _select_from(same, qual, block_mm)
        return _result("ambiguous" if amb else "ok", best,
                       reason="" if not amb else
                       "%s 色里有两个实例的 %s 坐标只差 %.1f mm"
                       % (cmd["color"], pos, margin),
                       candidates=same,
                       confidence=_conf(margin, TIE_MM, amb), block_mm=block_mm)

    if kind == "stack":
        if len(same) < 2:
            # 单实例：top/bottom 恒成立
            return _result("ok", same[0], confidence=1.0, candidates=same,
                           block_mm=block_mm)
        if cmd["where"] == "top":
            scored = [(b, b["top"]) for b in same]
        else:
            scored = [(b, -b["top"]) for b in same]
        best, margin, amb = _pick_best(scored, "mm")
        return _result("ambiguous" if amb else "ok", best,
                       reason="" if not amb else "同色实例顶高只差 %.1f mm" % margin,
                       candidates=same,
                       confidence=_conf(margin, TIE_MM, amb), block_mm=block_mm)

    if kind == "relation":
        other = [b for b in pool if b["class"] == cmd["other"]]
        if not other:
            return _result("none", reason="拾取区内没有参照颜色 %s" % cmd["other"],
                           block_mm=block_mm)
        rel = cmd["relation"]
        pairs = []
        for a in same:
            for b in other:
                if a is b:
                    continue
                m = _relation_margin(a, b, rel)
                if m is not None and m >= REL_MARGIN_MM:
                    pairs.append((a, m))
        if not pairs:
            return _result("none", reason="没有 %s 色的块 %s 任何 %s 色块"
                           % (cmd["color"], rel, cmd["other"]),
                           candidates=same, block_mm=block_mm)
        best, margin, amb = _pick_best(pairs, "mm")
        qual = cmd.get("qualifier")
        if qual is not None and amb:
            cands = []
            for a, _ in pairs:
                if a not in cands:
                    cands.append(a)
            return _select_from(cands, qual, block_mm)
        return _result("ambiguous" if amb else "ok", best,
                       reason="" if not amb else
                       "有两个 %s 块同时满足 %s，间隔只差 %.1f mm"
                       % (cmd["color"], rel, margin),
                       candidates=[a for a, _ in pairs][:4],
                       confidence=_conf(margin, TIE_MM, amb), block_mm=block_mm)

    return _result("bad", reason="未知意图 %r" % kind, block_mm=block_mm)


def _relation_margin(a, b, rel):
    """a 相对 b 满足 rel 的**间隔 mm**；不满足返回 None。"""
    if rel == "left-of":
        return a["cy"] - b["cy"]
    if rel == "right-of":
        return b["cy"] - a["cy"]
    if rel == "front-of":
        return a["cx"] - b["cx"]
    if rel == "back-of":
        return b["cx"] - a["cx"]
    if rel == "on":
        # 单俯视下压住的块整块看不见 ⇒ 这条在 E5 场景里**必然不命中**，
        # 留给 S3 的多视角 / 层号。仅当两块顶高差 ≈ 一个块边且水平贴近时才认。
        dz = a["top"] - b["top"]
        dxy = ((a["cx"] - b["cx"]) ** 2 + (a["cy"] - b["cy"]) ** 2) ** 0.5
        if dxy <= BLOCK_MM * 0.75 and 0.5 * BLOCK_MM <= dz <= 1.5 * BLOCK_MM:
            return dz
        return None
    if rel == "below":
        dz = b["top"] - a["top"]
        dxy = ((a["cx"] - b["cx"]) ** 2 + (a["cy"] - b["cy"]) ** 2) ** 0.5
        if dxy <= BLOCK_MM * 0.75 and 0.5 * BLOCK_MM <= dz <= 1.5 * BLOCK_MM:
            return dz
        return None
    return None


# ------------------------------------------------------------------ CLI
def main():
    ap = argparse.ArgumentParser(description="阶段 4 指定通道")
    ap.add_argument("frame", help="帧目录（含 color/depth/tf/candidates.json）")
    ap.add_argument("command", nargs="?", help="命令文本")
    ap.add_argument("--check", default=None, help="命令文本（与位置参数等价）")
    ap.add_argument("--classes", default=None, help="逗号分隔的类别表，默认 7 色")
    ap.add_argument("--hue-table", choices=("real", "sim"), default="real",
                    help="真机表 / 仿真表（仿真帧必须用 sim）")
    args = ap.parse_args()
    text = args.check if args.check is not None else args.command
    if text is None:
        ap.error("缺少命令文本")
    classes = (block_classes.parse_classes(args.classes) if args.classes
               else list(block_classes.DEFAULT_CLASSES))
    table = block_classes.SIM_HUE_DEG if args.hue_table == "sim" else None

    frame = gc.load_frame(args.frame)
    blocks, _tz = gc.find_blocks(frame, split=True, hue_table=table)
    try:
        cmd = parse_command(text, classes)
    except CommandError as e:
        print("BAD: %s" % e)
        return 2
    print("解析:", json.dumps(cmd, ensure_ascii=False))
    res = select(blocks, cmd)
    print("结果:", json.dumps(res, ensure_ascii=False, indent=1))
    return 0 if res["status"] == "ok" else 1


if __name__ == "__main__":
    sys.exit(main())

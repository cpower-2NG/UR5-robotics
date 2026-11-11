#!/usr/bin/env python3
"""抓放偏置补偿 b̂（阶段三 §4.1，2026-11-11 从 ``stage3_round.py`` 搬来）。

为什么单列一份
──────────────
阶段四正式轮也要用这套机制（``docs/阶段/阶段4-杂乱指定抓取/契约.md`` §4.7）：
机放的块落地后系统性偏向 base 的 (−x, −y)，阶段三实测均值 (−3.56, −2.56) mm、
|dxy| σ 0.84 mm。这个 offset 住在**块与夹爪之间**，不在「目标 → 臂」这条链上，
闭环把下一层目标改成「上一层实测中心」也消不掉；只有显式预补偿能压：

    b̂ = 实测块顶中心 − **命令**目标       命令目标 = 期望目标 − b̂

搬家而不是复制（AGENTS §3）：``stage3_round`` 仍从本模块 import 同名符号，
原位置留指针注释 —— 阶段三的历史命令与测试一字不改。

口径（不要改，改了 ``BIAS_ESTIMATE_VERSION`` 要跟着升）
────────────────────────────────────────────────────────
v1 = 矫正前（旧）、v2 = 矫正后（含 nudge 拖拽）、v3 = 矫正前（2026-11-06 起）。
两种样本不能相加：版本不符的旧文件当没有（回落到 ``--bias-seed``）。
"""

import json
import math
import os

#: 偏置上限（mm）。超过它**不是偏置，是失败**（块掉了 / 没抓住 / 塔被撞了），
#: 按失败停轮，不用它去推下一层 —— 拿一个 30 mm 的「偏置」补偿下一层，
#: 只会把塔越推越歪，而且事后从数据上看不出是哪一步坏的。
BIAS_LIMIT_MM = 15.0

#: 轮间累积的偏置估计（写在 run 目录下）。下一轮默认拿它当种子 ——
#: 不这样做，每轮第 1 层都只能不补，每轮都会重新造一次那个折角。
BIAS_ESTIMATE_FILE = "bias_estimate.json"

#: 估计的**语义版本**。2026-11-06 把 b̂ 从「矫正后（含拖拽）」改回「矫正前」：
#: 拖拽只在 nudge 层出现、下令时不可知，混进来是给每层加一次二元跳变
#: （20 层样本：矫正后 x 离均值最远 5.31 mm，矫正前 3.17 mm）。两种样本不能
#: 相加 —— 版本不符的旧文件当没有（回落到 ``--bias-seed``）。
#: v1 = 矫正前（旧）、v2 = 矫正后含拖拽、v3 = 矫正前（现行）。
BIAS_ESTIMATE_VERSION = 3
BIAS_ESTIMATE_MODEL = "constant_pre_nudge"


def bias_from_layer(measured_mm, commanded_mm, limit_mm=BIAS_LIMIT_MM):
    """一层放完后：``实测块顶中心 − 命令目标`` = 这一层的放置偏置 b̂。

    返回 ``(bias, why)``：正常时 ``bias=[bx, by]``（mm）、``why=None``；
    判不了时 ``bias=None``、``why`` 是原因（``measured_missing`` / 超上限）。

    **为什么必须有这一项**：机放的块落地后系统性偏向 base 的 (−x, −y) ——
    入口② open50/dwell0.5 五次均值 (−3.56, −2.56) mm、|dxy| σ 0.84 mm；今天三次
    机放对格心 (−3.4, +0.6)/(−2.8, +0.8)/(−0.9, +0.4)（方位角 170°/164°/156°，稳）。
    这个 offset 住**块与夹爪之间**，不在「目标 → 臂」这条链上 ⇒ 闭环组把下一层
    目标改成「上一层实测中心」也消不掉它。只有显式预补偿（命令 = 期望 − b̂）能压。
    """
    if measured_mm is None or commanded_mm is None:
        return None, "measured_missing"
    bx = float(measured_mm[0]) - float(commanded_mm[0])
    by = float(measured_mm[1]) - float(commanded_mm[1])
    mag = math.hypot(bx, by)
    if not math.isfinite(mag):
        return None, "bias_nonfinite"
    if mag > float(limit_mm):
        return None, ("bias_too_large: 实测比命令偏 %.2f mm（上限 %.1f）"
                      "—— 这不是偏置，是失败" % (mag, limit_mm))
    return [bx, by], None


def bias_note(mode):
    """`--bias-comp` 三种模式在逐层日志里各说各的（别把 fixed 说成「补偿关闭」）。

    2026-11-06 第 13 轮现场：`--bias-comp fixed` 明明在补（命令 = 期望 − b̂，
    逐层可验），日志却打印「补偿关闭（--bias-comp none）」——只看模式三分支，
    别再用「不是 adaptive 就是 none」这种二选一。
    """
    return {
        "adaptive": "下一层按历史均值预补偿",
        "fixed": "整轮固定先验补偿（--bias-comp fixed）",
        "none": "补偿关闭（--bias-comp none），只记账",
    }.get(mode, "未知模式 %r，按不补偿处理" % (mode,))


def applied_bias(bias_hist, mode="adaptive", seed=None):
    """这一层要用的 b̂（mm）：``种子 + 本轮已测各层`` 的均值。

    ``none`` ⇒ ``[0, 0]``（纯开环）。**种子是必需的，不是优化**：第 1 层没有
    本轮历史，不补；第 2 层补满 b̂ ⇒ 第 1↔2 层之间凭空多出一个 ≈|b̂| 的**折角**。
    判分看的是**相对**对中，4 mm 的折角直接把「≤5 mm」吃掉 —— 而物理上
    「相对第 1 层放准第 2 层」本来就要求先知道 b̂，轮内数据给不了，只能给先验。

    ``fixed`` ⇒ **只用种子，轮内不更新**（2026-11-05 第 6 轮后加的）。轮内自适应
    每层换一次 b̂，命令就跟着摆（round-06 实测跨层摆幅 x 1.18 / y 1.27 mm），
    而这一项和放置散差是**同一个量级**；把 b̂ 钉住，层间误差只剩
    ``b̂_k − b̂_{k−1}`` 里真实的那部分，命令项整个消失。历史仍照记，跨轮估计照更新。
    没给种子时退回 ``[0, 0]`` 并打印告警 —— 那是「以为在补、其实没补」。
    """
    if mode == "fixed":
        if seed:
            return [float(seed[0]), float(seed[1])]
        print("!! --bias-comp fixed 但没有任何 b̂ 先验（--bias-seed / "
              "bias_estimate.json）⇒ 这一轮其实没补")
        return [0.0, 0.0]
    if mode != "adaptive":
        return [0.0, 0.0]
    # 两种形状都收：`round.json` 里记的是 {"layer": k, "bias_mm": [...]}（2026-11-04
    # 首轮第 1 层后就是在这里崩的：旧代码只认 [bx, by]，第 2 层算补偿时 KeyError: 0），
    # 测试与离线调用则直接给 [bx, by]。
    vals = ([list(seed)] if seed else [])
    for h in (bias_hist or []):
        vals.append(list(h["bias_mm"]) if isinstance(h, dict) else list(h))
    if not vals:
        return [0.0, 0.0]
    n = float(len(vals))
    return [sum(float(b[0]) for b in vals) / n,
            sum(float(b[1]) for b in vals) / n]


def target_with_bias(desired_mm, bias_hist, mode="adaptive", seed=None):
    """``期望目标 − b̂``：命令给臂的目标。``desired`` 仍是要落到的那个点。"""
    b = applied_bias(bias_hist, mode, seed=seed)
    return [float(desired_mm[0]) - b[0], float(desired_mm[1]) - b[1]]


def commanded_target_m(desired_m, bias_hist, mode="adaptive", seed=None):
    """米制包装：臂与计划用 m，b̂ 用 mm —— 换算只在这一行，并由测试把守。"""
    t = target_with_bias([float(v) * 1000.0 for v in desired_m],
                         bias_hist, mode, seed=seed)
    return [t[0] / 1000.0, t[1] / 1000.0]


def parse_bias_seed(spec):
    """``"bx,by"``（mm，允许中文逗号）→ ``[bx, by]``；空 ⇒ ``None``。"""
    if spec is None:
        return None
    text = str(spec).strip().replace("，", ",")
    if not text:
        return None
    parts = [p.strip() for p in text.split(",")]
    if len(parts) != 2:
        raise ValueError("--bias-seed 要两个数（bx,by），实得 %r" % (spec,))
    try:
        return [float(parts[0]), float(parts[1])]
    except ValueError:
        raise ValueError("--bias-seed 不是数：%r" % (spec,))


def load_bias_estimate(path):
    """读轮间估计；坏文件当没有（**不猜**，但也不让一次坏文件炸掉整轮）。

    版本不符也当没有：``version 1`` 是旧「矫正前」、``version 2`` 是「矫正后
    （含拖拽）」，样本口径不同不能相加（见 ``BIAS_ESTIMATE_VERSION``）。
    """
    try:
        doc = json.load(open(path))
    except (OSError, ValueError):
        return None
    if doc.get("version") != BIAS_ESTIMATE_VERSION:
        return None
    val = doc.get("bias_mm")
    if not val or len(val) != 2:
        return None
    try:
        return [float(val[0]), float(val[1])]
    except (TypeError, ValueError):
        return None


def write_bias_estimate(rdir, doc):
    """把本轮的 b̂ 并进 ``<run>/bias_estimate.json``：下一轮第 1 层就有先验。

    没有测量（离线预检 / 第一层就停）⇒ 不写，返回 ``None``。
    """
    hist = [h["bias_mm"] for h in (doc.get("bias_history") or [])]
    if not hist:
        return None
    path = os.path.join(rdir, BIAS_ESTIMATE_FILE)
    try:
        prev = json.load(open(path))
    except (OSError, ValueError):
        prev = None
    if (prev or {}).get("version") != BIAS_ESTIMATE_VERSION:
        prev = None            # 旧口径的样本不能并进新口径（见常量注释）
    merged = merge_bias_estimate(prev, hist)
    if merged is None:
        return None
    n = float(len(hist))
    merged["rounds"] = list((prev or {}).get("rounds") or []) + [
        {"round": doc.get("round"), "n": len(hist),
         "bias_mm": [round(sum(float(h[0]) for h in hist) / n, 3),
                     round(sum(float(h[1]) for h in hist) / n, 3)]}]
    with open(path, "w") as fh:
        json.dump(merged, fh, ensure_ascii=False, indent=2)
        fh.write("\n")
    return merged


def merge_bias_estimate(prev, new_layers):
    """把本轮的 b̂ 并进轮间估计：``n`` 加权平均，返回落盘用的 dict。"""
    prev = prev or {}
    n0 = int(prev.get("n") or 0)
    vals = [list(map(float, prev.get("bias_mm") or [0.0, 0.0]))] * n0
    vals += [list(map(float, b)) for b in (new_layers or [])]
    if not vals:
        return None
    n = len(vals)
    return {"version": BIAS_ESTIMATE_VERSION, "model": BIAS_ESTIMATE_MODEL,
            "bias_mm": [round(sum(v[0] for v in vals) / n, 3),
                        round(sum(v[1] for v in vals) / n, 3)],
            "n": n,
            "rounds": list(prev.get("rounds") or [])}

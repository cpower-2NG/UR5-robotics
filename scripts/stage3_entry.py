#!/usr/bin/env python3
"""阶段 3 入口五实验的驱动（真机；每次 ``--go`` 都要**当次授权**）。

五项实验（契约 §3）
──────────────────
1. ``--mode zcal`` —— 塔位单块重复放置 ≥5 次，两个高度跟踪视点各 ≥3 帧；
   偏差 ``z_bias = 顶高 − 垫面 − 29.5``，取中位落盘 ``zcal.json``（``by_site``），
   每次残差 |≤2 mm| 才算过。顺带用拾取帧记一份拾取侧偏差。
2. ``--mode release`` —— 开口 30/50/40 mm × 停留 {0,0.5} s 各 5 次，
   每组量终态中心/yaw/顶高，算「实测 − 预测」的重复性（极差、σ）；
   最优组再补 ``--confirm`` 次。**数值闸门不在这里编**，数据出表由用户拍板。
3. ``--mode occlusion`` —— 人手先搭好 1/3/5 层塔，机器人夹着块停在
   ``place_approach`` 抓帧：下层塔顶还能不能被量出来。结果决定 eye-in-hand
   对照组做不做。**抓完这一帧就把这块放上去**（塔变成 h+1 层，由人重搭）。
4. ``--mode tolerance`` —— 人手把上块相对下块偏 0/3/5/8 mm 放上去，机器人
   **只动臂不碰夹爪**：量下层基准 → 量上层 → 静置 10 s 再量。
5. ``--mode nudge`` —— 开环放一块（入口② 最优组）→ 量 → 夹爪降到块两侧
   **只夹不抬** → 平移 δ / 原地转 δyaw → 松开 → 退开复测。δ = 目标 − 实测。
   记 push gain、有没有带转、有没有顶动邻块；判据 = 契约 §4.1 的 ≤5 mm / ≤5°。

真机纪律（每次会话重来）
────────────────────────
· ``--go`` 才动臂；执行器自己在板上核对 PLAYING / JTC active / robot_mode 7；
· 夹爪寄存器写入（初始化/开合）是**另一件事**的当次授权：急停就绪、
  两指整个行程内无手无遮挡；
· 任何一次执行判红 / 测量缺失 ⇒ 停在这一步，**不自动续跑**，交给人判读；
· 人搬块/搭塔的每一步都压成一行 ``>>> 需要人动手``，由 AI 原样转达；
  放行方式由 ``--ask-mode`` 定：``enter``（默认，等回车）/ ``file``（等哨兵
  文件 ``ack-NNN``，AI 驱动用）/ ``auto``（不停，只给离线演习）；
  **``--ask-mode auto`` 一律不许配 ``--go``**；
· 不 push。

不带 ``--go`` 只打印协议（不抓帧、不动臂）—— 离线计划预演用
``stage3_round.py --plan-only``。
"""
import argparse
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True

import numpy as np                                                # noqa: E402

import graspnet_candidates as gc                                   # noqa: E402
import precision_plan as pp                                        # noqa: E402
import precision_round as pr                                       # noqa: E402
import stage2_round as s2                                          # noqa: E402
import stage3_measure as m3                                        # noqa: E402
import stage3_round as s3r                                         # noqa: E402
import urkin                                                      # noqa: E402

MODES = ("zcal", "release", "occlusion", "tolerance", "nudge")

#: 入口线：校正后每次高度对 29.5 mm 的偏差绝对值上限。
ZCAL_TOL_MM = 2.0
#: 遮挡实验里「塔顶簇」的判据：top 落在期望顶高 ±这个值内。
OCCL_TOL_MM = 6.0
#: 塔位「空」的判据窗：比测量窗（50 mm）更紧。相邻格心最近 60 mm，
#: 阶段 2 的放置容差 10 mm ⇒ 邻块中心最远能到 ~50 mm，收紧到 40 mm 不误伤。
TOWER_CLEAR_RADIUS_MM = 40.0
#: 人手搭的塔顶 / 放的下块顶面，与理论值差超过它 ⇒ 层数不对或塔位没清空。
#: 半块高 14.75 mm，取 12 mm：漏一层（29.5）必被拦，读数偏差 +1~2 mm 不拦。
TOWER_HEIGHT_TOL_MM = 12.0

#: **机器人自己回位**（拆塔）时，回位块离拾取区已有块中心的最小距离。
#: 取 85 mm 而不是 `precision_plan.MIN_BLOCK_GAP`（60 mm）：后者是**规划**下限，
#: 回位按它贴边放，几毫米放置误差一加就变成 54–58.7 mm，零落地余量；
#: 2026-11-01 实测跑到第 6 次就再也选不出块。85 = 60 + 25 的落地余量。
PICK_RETURN_GAP_MM = 85.0
#: 回位块中心离拾取区边界的最小余量（块心，不是块边；`in_zone` 也按块心算）。
PICK_RETURN_MARGIN_MM = 25.0
#: 塔上那块在计划里的伪块序号 —— 拾取区的真 idx 从 0 起，−1 不会撞。
TOWER_BLOCK_IDX = -1


class EntryStop(Exception):
    """某一步判红：停住，不续跑。``rc`` 直接当退出码。"""

    def __init__(self, why, rc=2):
        super().__init__(why)
        self.why = why
        self.rc = rc


# ---------------------------------------------------------------- 纯逻辑（可测）
def group_stats(vals, digits=2):
    """一组数的重复性摘要；空组返回 ``{"n": 0}``。"""
    vals = [float(v) for v in vals if v is not None]
    if not vals:
        return {"n": 0}
    arr = np.asarray(vals, dtype=float)
    return {"n": int(arr.size),
            "mean": round(float(arr.mean()), digits),
            "median": round(float(np.median(arr)), digits),
            "min": round(float(arr.min()), digits),
            "max": round(float(arr.max()), digits),
            "range": round(float(arr.max() - arr.min()), digits),
            "std": round(float(arr.std(ddof=1)) if arr.size > 1 else 0.0,
                         digits)}


def zcal_doc(rows, pad_mm, block_h_mm=m3.BLOCK_TRUE_H_MM,
             tol_mm=ZCAL_TOL_MM, meta=None):
    """入口实验 1 的落盘格式 —— **``load_zcal`` 直接吃的就是这份**。

    ``rows``：每次放置一条 ``{"top_mm": …}``；``bias = top − 垫面 − 真高``。
    拾取侧偏差（若有）取 ``pick_bias_mm`` 的中位，作为 ``by_site.pick``。
    """
    pad = float(pad_mm)
    h = float(block_h_mm)
    biases, picks = [], []
    out_rows = []
    for i, r in enumerate(rows, 1):
        top = float(r["top_mm"])
        b = round(top - pad - h, 3)
        biases.append(b)
        if r.get("pick_bias_mm") is not None:
            picks.append(float(r["pick_bias_mm"]))
        out_rows.append(dict(r, rep=i, bias_mm=b))
    tower = float(np.median(biases)) if biases else 0.0
    residuals = [abs(b - tower) for b in biases]
    pick = float(np.median(picks)) if picks else None
    doc = {
        "version": 1, "model": "constant",
        "block_h_mm": h, "pad_z_mm": round(pad, 3),
        "by_site": {"tower": round(tower, 3),
                    "pick": None if pick is None else round(pick, 3)},
        "n": len(biases),
        "residual_mm": {"median": round(float(np.median(residuals)), 3)
                        if residuals else None,
                        "max": round(max(residuals), 3) if residuals else None},
        "tol_mm": float(tol_mm),
        # 判据是「**每一次**校正后的高度对真高都 ≤2 mm」，不是只看中位。
        "pass": bool(residuals) and max(residuals) <= float(tol_mm),
        "rows": out_rows,
    }
    if meta:
        doc["meta"] = meta
    return doc


def release_summary(groups):
    """开口×停留 → 每组的重复性；``best`` 按 (dxy 极差, dyaw 极差) 排序。

    ``groups = {组名: [{"dxy_mm","dyaw_deg","dz_mm"}, …]}``。
    """
    per = {}
    for name, rows in groups.items():
        per[name] = {
            "n": len(rows),
            "dxy_mm": group_stats([r["dxy_mm"] for r in rows]),
            "dyaw_deg": group_stats([r["dyaw_deg"] for r in rows]),
            # 彩色掩码的并行读数（与深度无关）；老行没有这个键，按缺数据处理。
            "dyaw_color_deg": group_stats([r.get("dyaw_color_deg")
                                           for r in rows]),
            "dz_mm": group_stats([r["dz_mm"] for r in rows]),
        }

    def key(name):
        s = per[name]
        return (s["dxy_mm"].get("range", float("inf")),
                s["dyaw_deg"].get("range", float("inf")))

    usable = [n for n, s in per.items() if s["n"]]
    best = min(usable, key=key) if usable else None
    return {"groups": per, "best": best,
            "order": sorted(usable, key=key)}


def occlusion_report(blocks, expected_top_mm, window_xy_mm,
                     radius_mm=m3.TOWER_RADIUS_MM, held_top_mm=None,
                     tol_mm=OCCL_TOL_MM, min_h_mm=m3.MIN_H_MM):
    """夹着块那一帧 → 「塔顶还在不在视野里」。

    只看塔窗内、非 low、高度 ≥``min_h_mm`` 的簇。**手里那块会挡在塔顶上方**
    （其顶面约比塔顶高 74.5 mm），所以按期望顶高分类，不拿「最高簇」当塔顶。
    """
    wx, wy = float(window_xy_mm[0]), float(window_xy_mm[1])
    inside = []
    for b in blocks:
        if b.get("low") or b.get("h", 0.0) < min_h_mm:
            continue
        d = math.hypot(b["cx"] - wx, b["cy"] - wy)
        if d <= float(radius_mm):
            inside.append({"idx": b["idx"], "cx": round(float(b["cx"]), 2),
                           "cy": round(float(b["cy"]), 2),
                           "top": round(float(b["top"]), 2),
                           "yaw_deg": (None if b.get("yaw_rect") is None
                                       else round(float(b["yaw_rect"]), 3)),
                           "yaw_trusted": bool(b.get("yaw_trusted")),
                           "dist_mm": round(d, 2)})
    tower = [b for b in inside if abs(b["top"] - float(expected_top_mm))
             <= float(tol_mm)]
    held = []
    if held_top_mm is not None:
        held = [b for b in inside if abs(b["top"] - float(held_top_mm))
                <= float(tol_mm)]
    return {"window_xy_mm": [wx, wy], "radius_mm": float(radius_mm),
            "expected_top_mm": round(float(expected_top_mm), 2),
            "held_top_mm": (None if held_top_mm is None
                            else round(float(held_top_mm), 2)),
            "n_window": len(inside), "clusters": inside,
            "tower_clusters": tower, "held_clusters": held,
            # 「量得到」只表示视野里有一簇落在塔顶高度上；中心/yaw 能不能用
            # 还要人看 —— 这就是本实验要回答的。
            "tower_top_visible": bool(tower)}


def tol_offset_mm(lower_center_mm, upper_center_mm):
    """人手偏置实验：实际偏移向量（上块中心 − 下块中心）与模长。"""
    dx = float(upper_center_mm[0]) - float(lower_center_mm[0])
    dy = float(upper_center_mm[1]) - float(lower_center_mm[1])
    return {"dx_mm": round(dx, 2), "dy_mm": round(dy, 2),
            "d_mm": round(math.hypot(dx, dy), 2)}


# ---------------------------------------------------------------- 后矫正（⑤）
#: 矫正后判据 = 契约 §4.1 的门（≤5 mm / ≤5°），**不新编数**。第 1 层没有下层，
#: 就按「目标 = 格心 + 0°」算；主轮里同样是这两个数，换成「相对下层实测」。
NUDGE_XY_TOL_MM = 5.0
NUDGE_YAW_TOL_DEG = 5.0
#: 排查「有没有顶动邻块」的半径（mm，离目标格心）。手指半开 50 mm ⇒
#: 半径 90 mm 已把两指行程全包住。
NUDGE_NEAR_MM = 90.0
#: 该半径内、离目标格心比这更近的簇，当成被矫正的那块自己
#: （块半对角线 17.7 mm；矫正量上限 15 mm，两者加起来仍 <25）。
NUDGE_SELF_MM = 25.0


def nudge_delta(measured_xy_mm, measured_yaw_deg, target_xy_mm,
                yaw_sign=1.0, delta_deg=0.0, yaw_quadrant=0):
    """「目标 − 实测」→ 要下的平移 δ(mm) 与 tool0 转角 δ(°)。纯函数。

    ``yaw_sign`` / ``delta_deg`` 与 `stage2_round.yaw_for_arm` **同一套映射**
    （``tool0_yaw = sign·θ + δ``）；目标块朝向是 0（格线方向），对应的 tool0
    角正好是 ``δ``。

    方块按 **90° 周期**折最短角（`stage2_round.signed_err_mod90` 同口径）：
    方块的正面/侧面互换，转 87° 与转 −3° 是同一件事。`fold_yaw_180` 那一次
    折是给夹爪挑一支更短的等价解，不是近似。

    返回里两个角分开写：
    · ``dyaw_deg`` —— **tool0** 要转多少（喂给 `build_nudge`）；
    · ``dyaw_block_deg`` —— 块**应跟着转**多少（= sign·dyaw_deg，用于对账）。

    ``yaw_quadrant``（0–3，`--nudge-yaw-quadrant`）把**接近朝向**再加 k·90°。
    方块对 90° 自对称 ⇒ 夹这一对侧面与夹那一对侧面是同一个合法抓取，所以
    这个数不改任何判据；它只改 tool0 绕 z 的朝向。用途是**方向帧判别**：
    零位移档实测的 ≈5 mm 侧推全落在世界 −y，而那些样本的指轴也都在 75–90°
    （≈世界 +y）——两者重合，分不出「推在夹爪系里」还是「推在世界系里」。
    把指轴转 90°，夹爪系的推就该跟着转到 ±x，世界系的推则仍留在 −y。
    ``dyaw`` 本身按 mod 90 折，绕 90° 转一次不变（块的等价解）。
    """
    sign = float(yaw_sign)
    quad = int(yaw_quadrant) % 4
    dx = float(target_xy_mm[0]) - float(measured_xy_mm[0])
    dy = float(target_xy_mm[1]) - float(measured_xy_mm[1])
    yaw_from = pp.fold_yaw_180(sign * float(measured_yaw_deg) + float(delta_deg)
                               + 90.0 * quad)
    yaw_to = pp.fold_yaw_180(float(delta_deg))
    dyaw = float(s2.signed_err_mod90(yaw_to, yaw_from))
    return {"dx_mm": dx, "dy_mm": dy, "d_mm": math.hypot(dx, dy),
            "dyaw_deg": dyaw, "dyaw_block_deg": sign * dyaw,
            "yaw_from_deg": float(yaw_from), "yaw_to_deg": float(yaw_to)}


def nearby_blocks(blocks, target_xy_mm, radius_mm=NUDGE_NEAR_MM,
                  exclude_mm=NUDGE_SELF_MM):
    """离目标格心 ``[exclude_mm, radius_mm]`` 内的**其它**块（诊断顶没顶邻块）。

    ``low`` 的簇（连接件 / 台面噪声）不算。
    """
    out = []
    for b in blocks or []:
        if b.get("low"):
            continue
        d = math.hypot(float(b["cx"]) - float(target_xy_mm[0]),
                       float(b["cy"]) - float(target_xy_mm[1]))
        if float(exclude_mm) <= d <= float(radius_mm):
            out.append({"idx": b.get("idx"),
                        "center_mm": [round(float(b["cx"]), 2),
                                      round(float(b["cy"]), 2)],
                        "top_mm": round(float(b.get("top", 0.0)), 2),
                        "dist_mm": round(d, 2)})
    return sorted(out, key=lambda r: r["dist_mm"])


def neighbour_shift(before_list, after_list, match_mm=30.0):
    """前后两批邻块按**最近匹配**配对，给出各自移动了多少。

    配不上（数量变了 / 最近的一个也超过 ``match_mm``）就记 ``None`` ——
    不猜「它大概就是那一块」。
    """
    out, used = [], set()
    for b in before_list or []:
        best, bd = None, None
        for i, a in enumerate(after_list or []):
            if i in used:
                continue
            d = math.hypot(a["center_mm"][0] - b["center_mm"][0],
                           a["center_mm"][1] - b["center_mm"][1])
            if bd is None or d < bd:
                best, bd = i, d
        if best is not None and bd <= float(match_mm):
            used.add(best)
            out.append({"before_idx": b["idx"],
                        "after_idx": after_list[best]["idx"],
                        "move_mm": round(bd, 2)})
        else:
            out.append({"before_idx": b["idx"], "after_idx": None,
                        "move_mm": None})
    return out


# nudge 的第二次闭合会把块沿**指轴**推走：Δ = (E·â)·â，â 是 tool0 的 x 轴。
# E 是**世界（base）系**里的一个固定矢量，不是夹爪系的固定量 —— 2026-11-01 低 yaw
# 判别档（指轴从 ±y 转到 ±x，位移也跟着从 −y 转到 −x）+ 12 行三模型最小二乘：
# 投影模型 RMS 1.22 mm，世界系固定位移 2.50、夹爪系固定 2.82。台账第一百八十九轮。
# â 与 −â 是**同一根指轴**（平行两指换手不算换姿态）⇒ Δ 对 180° 不变；
# 这条在物理上是必需的，也被测试钉住。
NUDGE_BIAS_XY_MM = (-2.05, -4.78)      # |E| = 5.20 mm，方向 −113.3°


def nudge_push_pred(bias_xy_mm, yaw_deg):
    """预测「闭合那一下」把块推走多少：``E`` 沿指轴的分量再沿指轴返回。

    只保留沿指轴的那一维：垂着指轴的方向上两指没有推力面，块不动
    （12 行实测的 ⟂ 分量 ≤0.34 mm，都在噪声地板里）。

    反向变异：把 ``yaw_deg`` 换成「指轴 = tool0 的 y」、或把分量换成 E 本身，
    都会让 `test_nudge_bias_comp_cancels_predicted_push` 变红。
    """
    ex, ey = float(bias_xy_mm[0]), float(bias_xy_mm[1])
    t = math.radians(float(yaw_deg))
    ax, ay = math.cos(t), math.sin(t)      # 指轴 = tool0 x（数据定的，见上）
    s = ex * ax + ey * ay
    return s * ax, s * ay


def nudge_bias_comp(spec):
    """`--nudge-bias-comp "x,y"` → ``(x, y)`` mm（base 系）；空 ⇒ 不补偿。

    不给默认值：E 只在 4 号格附近 8 mm 内估过（台账第一百八十九轮「未决 1」），
    静默套一个常数比不补偿更危险 —— 换位置若不成立，人会以为是别的地方坏了。
    """
    if spec is None or str(spec).strip() == "":
        return None
    vals = [float(v) for v in str(spec).replace("，", ",").split(",") if v.strip()]
    if len(vals) != 2:
        raise SystemExit("--nudge-bias-comp 要两个数（x,y，mm），拿到 %r" % (spec,))
    return (vals[0], vals[1])


def applied_nudge_command(cmd, zero=False, bias_xy_mm=None):
    """这次**真正发给计划**的指令；``cmd`` 是实测 δ，另存一份留作对账。

    ``zero=True``（`--nudge-zero`，诊断用）：平移与转角全置零，只走
    「接近 → 闭合 → 松开 → 退开」。2026-11-01 真机 5 次显示 nudge 自带一个
    ≈5.3 mm、与指令无关的恒定侧推（详见仓外台账第一百八十七轮）；这一档用来
    分辨它是出在这四个动作上，还是出在横移/扭转上。

    ``bias_xy_mm``（`--nudge-bias-comp`）：预补偿。闭合时块会被推走 Δ，
    就在**横移目标**里减掉 Δ（不动接近点），落点回到「实测 + 想要的 δ」。
    这一手只依赖「Δ 可预测」，不假定 E 是臂的定位残差还是测量的固定偏置
    —— 两者对补偿等价（台账第一百八十九轮 §未决）。
    """
    keys = ("dx_mm", "dy_mm", "d_mm", "dyaw_deg", "dyaw_block_deg",
            "yaw_from_deg", "yaw_to_deg")
    out = {k: float(cmd[k]) for k in keys if k in cmd}
    if zero:
        for k in ("dx_mm", "dy_mm", "d_mm", "dyaw_deg", "dyaw_block_deg"):
            if k in out:
                out[k] = 0.0
    if bias_xy_mm is not None and "dx_mm" in out and "dy_mm" in out:
        px, py = nudge_push_pred(bias_xy_mm, out.get("yaw_from_deg", 0.0))
        out["dx_mm"] -= px
        out["dy_mm"] -= py
        out["d_mm"] = math.hypot(out["dx_mm"], out["dy_mm"])
        out["push_pred_x_mm"] = px
        out["push_pred_y_mm"] = py
    return out


def nudge_row(rep, cmd, before, after, target_xy_mm,
              xy_tol_mm=NUDGE_XY_TOL_MM, yaw_tol_deg=NUDGE_YAW_TOL_DEG):
    """一次矫正的前后对账：实走 δ、push gain、带转角、终态过不过门。

    ``before`` / ``after`` 是矫正前后的 ``merged``（`stage3_measure` 融合值）。
    **yaw 判据一律走彩色掩码**（契约 §4.2.5），深度 ``yaw_deg`` 只报告：
    两个估计器逐次差 1.43 倍，拿深度当 ``ok`` 会把合格的矫正判红。

    ``ok=None``（``reason="no_colour_yaw"``）= 证明不了，转人评 —— 与
    `stage3_measure.pose_consistency` 同一口径。
    """
    def centre(m):
        return [float(m["center_base_mm"][0]), float(m["center_base_mm"][1])]

    def yaw_color(m):
        y = m.get("yaw_color_deg")
        return None if y is None else float(y)

    def err_vs_target(c):
        return math.hypot(float(target_xy_mm[0]) - c[0],
                          float(target_xy_mm[1]) - c[1])

    bc, ac = centre(before), centre(after)
    by, ay = yaw_color(before), yaw_color(after)
    yaw_err0 = None if by is None else float(s2.signed_err_mod90(by, 0.0))
    yaw_err1 = None if ay is None else float(s2.signed_err_mod90(ay, 0.0))
    dyaw_act = (None if (by is None or ay is None)
                else float(s2.signed_err_mod90(ay, by)))
    dyaw_cmd = float(cmd["dyaw_block_deg"])
    yaw_gain = (None if (dyaw_act is None or abs(dyaw_cmd) < 1e-9)
                else dyaw_act / dyaw_cmd)

    # 实走 δ：沿指令方向的有符号投影比（push gain）+ 侧向走偏（粘滑/推力偏）。
    dx, dy = ac[0] - bc[0], ac[1] - bc[1]
    d_act = math.hypot(dx, dy)
    cx, cy = float(cmd["dx_mm"]), float(cmd["dy_mm"])
    d_cmd = math.hypot(cx, cy)
    if d_cmd > 1e-9:
        gain = (dx * cx + dy * cy) / (d_cmd * d_cmd)
        perp = abs(-dx * cy + dy * cx) / d_cmd
    else:
        gain, perp = None, d_act

    e0, e1 = err_vs_target(bc), err_vs_target(ac)
    if yaw_err1 is None:
        ok, reason = None, "no_colour_yaw"
    else:
        ok = bool(e1 <= float(xy_tol_mm) and abs(yaw_err1) <= float(yaw_tol_deg))
        reason = "ok" if ok else "still_off"

    def state(c, e, y, ye):
        return {"center_mm": [round(c[0], 2), round(c[1], 2)],
                "xy_err_mm": round(e, 2), "yaw_color_deg": y,
                "yaw_err_deg": None if ye is None else round(ye, 2)}

    return {"rep": int(rep),
            "commanded": {k: round(float(v), 3) for k, v in cmd.items()},
            "before": dict(state(bc, e0, by, yaw_err0),
                           yaw_depth_deg=before.get("yaw_deg"),
                           top_mm=before.get("top_mm")),
            "after": dict(state(ac, e1, ay, yaw_err1),
                          yaw_depth_deg=after.get("yaw_deg"),
                          top_mm=after.get("top_mm")),
            "push": {"d_actual_mm": round(d_act, 2),
                     "gain_xy": None if gain is None else round(gain, 2),
                     "perp_mm": round(perp, 2),
                     "d_cmd_mm": round(d_cmd, 2)},
            "yaw": {"dyaw_actual_deg": (None if dyaw_act is None
                                        else round(dyaw_act, 2)),
                    "dyaw_expected_deg": round(dyaw_cmd, 2),
                    "gain_yaw": (None if yaw_gain is None
                                 else round(yaw_gain, 2))},
            "delta_xy_mm": round(e0 - e1, 2),
            "final_ok": ok, "reason": reason,
            "tol": {"xy_mm": float(xy_tol_mm),
                    "yaw_deg": float(yaw_tol_deg)}}


def nudge_pose_gate(before, after=None):
    """契约 §4.2.5 的多姿态一致性门 —— 过门返回 ``None``，否则返回人评理由。

    ``after`` 省略（``None``）= 只查矫正前那一次（还没动臂，用来决定夹不夹）。

    ``before`` / ``after`` 是 ``stage3_measure`` 的融合值；这里只读它自己记下的
    两视点差（``center_span_mm`` / ``yaw_color_span_deg``），不重算。

    为什么要单独补这一道（2026-11-01 实测）：nudge 这条路径原来只取融合后的
    ``center_base_mm``，把离散度记进 row.json 就没人看了。零位移诊断档第 1 次
    的两个视点相差 **11.17 mm**（超过契约的 10 mm 门），却仍被记成一条普通的
    ``status: ok``，混进了 ``n_final_ok``。几十次跑下来这种东西会把结论带偏，
    而且从汇总里根本看不出来。

    ``span`` 缺失 = 只有一个视点能测到 ⇒ **证明不了一致**，同样转人评
    （与 ``stage3_measure.pose_consistency`` 的 ``ok=None`` 同一口径）。
    """
    pairs = [("矫正前", before or {})]
    if after is not None:
        pairs.append(("矫正后", after or {}))
    bad = []
    for name, m in pairs:
        sp = m.get("center_span_mm")
        ys = m.get("yaw_color_span_deg")
        if sp is None or ys is None:
            bad.append("%s只有一个视点能测到（证明不了一致）" % name)
            continue
        if sp > m3.POSE_XY_TOL_MM:
            bad.append("%s两视点差 %.1f mm > %.0f mm"
                       % (name, sp, m3.POSE_XY_TOL_MM))
        if ys > m3.POSE_YAW_TOL_DEG:
            bad.append("%s两视点 yaw 差 %.1f° > %.0f°"
                       % (name, ys, m3.POSE_YAW_TOL_DEG))
    return "；".join(bad) or None


def nudge_summary(rows):
    """矫正实验汇总：终态过门比例 + push / 转角增益的重复性。

    两个比例分开报 —— 「放得太歪（超 15 mm/15°）压根没敢矫正」与「矫正了但
    没修回来」是两件事，混成一个数就看不出该改哪一头。
    """
    done = [r for r in rows or [] if r.get("status") == "ok"]
    ok = [r for r in done if r.get("final_ok") is True]

    def col(key, sub=None):
        vals = []
        for r in done:
            v = r.get(sub, {}).get(key) if sub else r.get(key)
            vals.append(v)
        return group_stats(vals)

    return {"n": len(rows or []), "n_nudged": len(done),
            "n_final_ok": len(ok),
            "n_skipped": len([r for r in rows or []
                              if r.get("status") != "ok"]),
            "xy_err_before": col("xy_err_mm", "before"),
            "xy_err_after": col("xy_err_mm", "after"),
            "yaw_err_before": col("yaw_err_deg", "before"),
            "yaw_err_after": col("yaw_err_deg", "after"),
            "gain_xy": col("gain_xy", "push"),
            "push_perp": col("perp_mm", "push"),
            "gain_yaw": col("gain_yaw", "yaw"),
            "tol": {"xy_mm": NUDGE_XY_TOL_MM, "yaw_deg": NUDGE_YAW_TOL_DEG}}


# ---------------------------------------------------------------- 板侧动作
def _dump(path, obj):
    json.dump(obj, open(path, "w"), ensure_ascii=False, indent=1)
    return path


#: 「要人动手」怎么放行。``enter`` 是原来的行为；``file`` 给 AI 驱动用；
#: ``auto`` 只给离线演习（``--go`` 时禁用，见 ``main``）。
_ASK = {"mode": "enter", "dir": None, "timeout_s": 0.0, "seq": 0}


def ask(text):
    """把「要人动手」压成一行，然后按 ``--ask-mode`` 决定什么时候往下走。

    ``enter``（默认）：有终端就等回车；没有终端（管道/后台）只提示不阻塞
    —— 老行为，**后台跑一步都不会停**，所以 AI 驱动只能走 ``file``。
    ``file``：先写 ``ask-NNN.txt``（提示原文），再等 ``ack-NNN`` 出现才继续。
    编号每次自增，所以放行不会串到上一条/下一条 —— 上一轮残留的 ack 先删掉。
    ``auto``：不停，直接继续（离线演习用；配 ``--go`` 会被 ``main`` 拒掉）。
    """
    print(">>> 需要人动手：" + text, flush=True)
    mode = _ASK["mode"]
    if mode == "auto":
        print("    （--ask-mode auto：不停，直接继续）", flush=True)
        return
    if mode == "file":
        d = _ASK["dir"]
        if not d:
            raise SystemExit("--ask-mode file 需要 --ask-dir 或 --run-dir")
        os.makedirs(d, exist_ok=True)
        n = int(_ASK["seq"]) + 1
        _ASK["seq"] = n
        need = os.path.join(d, "ask-%03d.txt" % n)
        ack = os.path.join(d, "ack-%03d" % n)
        if os.path.exists(ack):
            os.remove(ack)
        with open(need, "w", encoding="utf-8") as fh:
            fh.write(text + "\n")
        print("    做完后创建这个文件放行：%s" % ack, flush=True)
        t0 = time.time()
        while not os.path.exists(ack):
            if _ASK["timeout_s"] > 0 and time.time() - t0 > _ASK["timeout_s"]:
                print("!! 等人超时（%.1f s）：%s"
                      % (_ASK["timeout_s"], text), flush=True)
                raise SystemExit(3)
            time.sleep(0.5)
        try:
            os.remove(ack)
        except OSError:
            pass
        return
    try:
        input("    完成后按回车继续 …")
    except EOFError:
        print("    （非交互终端：直接继续 —— 确认已经做完再往下一步）")


def _survey_pair(top_mm, view_xy_mm, d_mm):
    return pp.stage3_survey_tool0(float(top_mm) / 1000.0,
                                  [view_xy_mm[0] / 1000.0,
                                   view_xy_mm[1] / 1000.0],
                                  distance=float(d_mm) / 1000.0)


def measure_pair(board, chain, scene, cdir, tag, pair, grid, window_xy_mm,
                 layer, z_bias_mm, move=True, require_found=True,
                 radius_mm=None):
    """在高度跟踪位姿对上测量一次（``move=False`` 表示已经停在 pair[0]）。

    ``require_found=False`` 供**塔位预检**用：量不到块才是通过的条件。
    """
    if move:
        rc = s3r.go_move(board, chain, cdir, tag + "-pose1", pair[0], _URDF)
        if rc != 0:
            raise EntryStop("%s 走到位姿 1 失败（rc=%d）" % (tag, rc), rc)
    n = int(scene.get("terminal_frames", 3))
    dirs1 = s3r.capture_frames(board, os.path.join(cdir, "frame-%s-pose1" % tag),
                               "entry-%s-p1" % tag, n)
    rc = s3r.go_move(board, chain, cdir, tag + "-pose2", pair[1], _URDF)
    if rc != 0:
        raise EntryStop("%s 走到位姿 2 失败（rc=%d）" % (tag, rc), rc)
    dirs2 = s3r.capture_frames(board, os.path.join(cdir, "frame-%s-pose2" % tag),
                               "entry-%s-p2" % tag, n)
    kw = {} if radius_mm is None else {"radius_mm": float(radius_mm)}
    meas = m3.measure_layer([("pose1", dirs1), ("pose2", dirs2)],
                            grid=grid, window_xy_mm=window_xy_mm,
                            layer=layer, z_bias_mm=float(z_bias_mm),
                            block_h_mm=m3.BLOCK_TRUE_H_MM, **kw)
    _dump(os.path.join(cdir, "measure-%s.json" % tag), meas)
    merged = meas.get("merged") or {}
    if require_found and not merged.get("found"):
        raise EntryStop("%s 测量缺失（塔窗内没有合格块）" % tag)
    return meas


def layer_height_check(measured_mm, expected_mm, tol_mm=TOWER_HEIGHT_TOL_MM):
    """人手摆好的高度对不对。返回 ``(ok, 差多少)``；差 = 实测 − 应有。"""
    d = round(float(measured_mm) - float(expected_mm), 2)
    return abs(d) <= float(tol_mm), d


def tower_occupancy_verdict(merged, cell_idx=m3.TOWER_CELL_IDX):
    """塔位测量的融合值 → 「还有块」的一句话；空着返回 ``None``。"""
    if not (merged or {}).get("found"):
        return None
    return ("塔位（格 %d）上还有块：顶 %.1f mm、中心 %s"
            % (int(cell_idx), merged["top_mm"], merged["center_base_mm"]))


def measure_tower(args, board, chain, scene, grid, cdir, tag="precheck"):
    """量一次塔位，**无论空不空**都返回 ``(meas, msg)``（``msg`` 为空 = 空位）。

    2026-10-31 现状层记着：垫上格 0/1/2/4/5/6/7 各一块，塔位（中心格 idx4）
    上正压着阶段 2 的一块。第 1 层计划按「表面 = 垫面」算 ``place_z``，
    不清格就会直接往下压 —— 所以在任何一次放置之前先量一遍。
    """
    view_xy = m3.grid_center(grid)
    pad_mm = m3.pad_z_mm(grid)
    pair = _survey_pair(pad_mm, view_xy, args.d_mm)
    meas = measure_pair(board, chain, scene, cdir, tag, pair, grid, view_xy,
                        layer=0, z_bias_mm=0.0, require_found=False,
                        radius_mm=TOWER_CLEAR_RADIUS_MM)
    msg = tower_occupancy_verdict(meas.get("merged"), args.tower_cell_idx)
    return meas, msg


def tower_precheck(args, board, chain, scene, grid, cdir, tag="precheck"):
    """塔位必须是**空的** —— 非空立刻停（fail-closed）。"""
    meas, msg = measure_tower(args, board, chain, scene, grid, cdir, tag)
    if msg:
        raise EntryStop(msg + " —— 先清空再跑")
    return meas


def resume_tower_action(msg, meas, last_merged, resume):
    """开工时塔位该怎么处理：``empty``（空）/ ``return``（自己拆回）/ ``stop``。

    续跑时塔上压着「上一次刚放下去、还没来得及拆」的那块是**正常**的：用
    当次实测当抓取目标（``meas["merged"]``），量不到结构才退回上次行里的
    ``last_merged``；两样都没有就停（fail-closed，不瞎摸）。
    """
    if not msg:
        return "empty"
    if not resume:
        return "stop"
    return "return" if ((meas or {}).get("merged") or last_merged) else "stop"


def tower_clear_now(board, scene, cdir, tag, grid, window_xy_mm,
                    radius_mm=TOWER_CLEAR_RADIUS_MM,
                    cell_idx=m3.TOWER_CELL_IDX):
    """人清完塔位后**不动臂**就地核一次：还有块就停。

    只在刚做完 ``measure_pair``（臂停在 survey 位姿 2）之后调用 ——
    那时相机本来就对着塔位，不必再走一趟。
    """
    n = int(scene.get("terminal_frames", 3))
    dirs = s3r.capture_frames(board, os.path.join(cdir, "frame-%s" % tag),
                              "entry-%s" % tag, n)
    meas = m3.measure_layer([("now", dirs)], grid=grid,
                            window_xy_mm=window_xy_mm, layer=0,
                            z_bias_mm=0.0, radius_mm=float(radius_mm))
    _dump(os.path.join(cdir, "measure-%s.json" % tag), meas)
    msg = tower_occupancy_verdict(meas.get("merged"), cell_idx)
    if msg:
        raise EntryStop(msg + " —— 还没清空，不往下跑")
    return meas


def pick_item(args, analysis, scene, zcal, n=1):
    """拾取帧 → 要抓的块；候选不够 / 太挤时转成 ``EntryStop``（不是 traceback）。

    顺序仍是「候选离块心最近」优先，但**跳过夹爪全开放不下的块**
    （邻块中心距 < ``MIN_BLOCK_GAP``）。2026-10-31 实测：回位块落点比计划偏几毫米，
    邻距从 60 缩到 59.2 mm；选到它就 PlanError，整条 40 次的扫参还没开始就废掉。
    """
    try:
        pool = s2.candidate_pool(analysis, scene["pick_zone"])
        usable = [x for x in pool
                  if x["p"].get("class") not in (None, "", "unknown")]
        items = s3r.build_items(analysis, scene, max(1, len(usable)),
                                arm=args.arm, zcal=zcal)
    except RuntimeError as exc:
        raise EntryStop("拾取区没有可用块：%s" % exc)
    ok = []
    for it in items:
        gap = pp.check_block_gap(it["block"], analysis.get("blocks", []))
        if gap is None or gap >= pp.MIN_BLOCK_GAP:
            ok.append(it)
            if len(ok) >= n:
                return ok
    if not ok:
        raise EntryStop("拾取区的块都挤在一起（最近邻 < %.0f mm）—— 挪开一块再跑"
                        % (pp.MIN_BLOCK_GAP * 1000.0))
    return ok


#: `measure_pair` 里 `go_move` 要 URDF；由 `main` 设一次。
_URDF = None


def capture_pick(args, board, scene, cdir, tag):
    """走到拾取巡检位 → 按 ``scene.pick_frames`` 连拍，返回 (首帧, 额外帧)。"""
    rc, frame_dir = s2.do_capture(board, "stage2pick",
                                  os.path.join(cdir, "frame-pick"),
                                  args.urdf, cdir, tag)
    if rc != 0:
        raise EntryStop("走到拾取巡检位失败（rc=%d）" % rc, rc)
    extras = []
    for j in range(1, max(1, int(scene.get("pick_frames", 1)))):
        fd = os.path.join(cdir, "frame-pick-%s" % "bcdefgh"[j - 1])
        board.capture(fd, "%s-pick-%d" % (tag, j + 1))
        extras.append(fd)
    return frame_dir, extras


def analyse_pick(frame_dir, extras, cdir):
    """拾取帧 → 分析结果（多帧融合 yaw）。"""
    analysis = s2.analyse_frame(frame_dir, cdir)
    if extras:
        fused = []
        for fd in extras:
            f = gc.load_frame(fd)
            eb, _tz = gc.find_blocks(f)
            fused.append(s2.add_pixel_yaw(f, eb))
        s2.fuse_block_yaw(analysis["blocks"], fused)
    return analysis


def pick_bias_of(analysis, item):
    """拾取侧偏差：被选中那块在帧里的 ``h − 29.5``（与历史复算同口径）。"""
    for b in analysis["blocks"]:
        if int(b["idx"]) == int(item["block_idx"]):
            h = b.get("h")
            if h is None:
                return None, b
            return round(float(h) - m3.BLOCK_TRUE_H_MM, 3), b
    return None, None


def load_rows(cdir):
    """``--resume``：把已跑完的 ``<tag>/row.json`` 读回来（按 rep 排序）。

    只收 ``rep`` 是整数的条目；坏文件跳过并留一句 —— 续跑宁可少一条让人补跑，
    也不要半条脏数据混进 z 标定 / 释放统计。
    """
    rows = []
    if not os.path.isdir(cdir):
        return rows
    for name in sorted(os.listdir(cdir)):
        path = os.path.join(cdir, name, "row.json")
        if not os.path.isfile(path):
            continue
        try:
            row = json.load(open(path))
        except ValueError:
            print("  续跑：跳过坏文件 %s" % path)
            continue
        if isinstance(row, dict) and isinstance(row.get("rep"), int):
            rows.append(row)
    rows.sort(key=lambda x: x["rep"])
    return rows


def free_slot_mm(blocks, zone_m, gap_mm=PICK_RETURN_GAP_MM,
                 margin_mm=PICK_RETURN_MARGIN_MM, step_mm=5.0):
    """拾取区里给「回位的块」找一个空位（格点搜索，纯逻辑）。

    ``zone_m`` = ``[x0, x1, y0, y1]``（**米**，base 系，同 ``scene["pick_zone"]``）。
    合格 = 离**每个**已有块中心 ≥ ``gap_mm``、离区边 ≥ ``margin_mm``；
    合格点里取**离区心最近**的（同分取离块更远者）—— 回位块尽量靠中间，
    不往边界角上堆，也不贴着要抓的那块。

    返回 ``{"xy_mm": [x, y], "gap_mm": 最近块距, "edge_mm": 离边最近}``；
    一个合格点都没有就返回 ``None`` —— 调用方 fail-closed，不硬塞。
    """
    x0, x1, y0, y1 = [float(v) * 1000.0 for v in zone_m]
    pts = [(float(b["cx"]), float(b["cy"])) for b in (blocks or [])]
    mid_x, mid_y = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    lo_x, hi_x = x0 + float(margin_mm), x1 - float(margin_mm)
    lo_y, hi_y = y0 + float(margin_mm), y1 - float(margin_mm)
    if lo_x > hi_x or lo_y > hi_y:
        return None
    best, best_key = None, None
    y = lo_y
    while y <= hi_y + 1e-6:
        x = lo_x
        while x <= hi_x + 1e-6:
            gap = (min(math.hypot(x - px, y - py) for px, py in pts)
                   if pts else float("inf"))
            edge = min(x - x0, x1 - x, y - y0, y1 - y)
            if gap >= float(gap_mm) and edge >= float(margin_mm):
                key = (round(math.hypot(x - mid_x, y - mid_y), 3), -gap)
                if best_key is None or key < best_key:
                    best_key = key
                    best = {"xy_mm": [round(x, 2), round(y, 2)],
                            "gap_mm": (None if gap == float("inf")
                                       else round(gap, 2)),
                            "edge_mm": round(edge, 2)}
            x += float(step_mm)
        y += float(step_mm)
    return best


def tower_block_item(merged, scene, zcal, arm="geometric"):
    """把「塔上那块」的实测值包装成 ``build_items`` 同款的 item（拆塔用）。

    ``merged`` = 上一轮塔顶的融合测量（``measure_pair(...)["merged"]``）。
    中心 / yaw 直接用；``h`` 用 ``顶高 − z 标定(tower)`` —— ``build_pick``
    的抓取高度整个锚在**台面**（``TABLE_PLANE_Z``），塔上的块只是 ``wz``
    比台面上的块大了一个块高（垫 + 29.5）。

    yaw 只有 ``yaw_rect`` 一个来源（没有像素最小外接矩形），按几何臂的
    ``sign·θ + δ`` 走；量不到 yaw 时 ``yaw_for_arm`` 会返回 0 并标
    ``geometric_missing`` —— 半开 50 mm 仍容得下 30 mm 方块的对角，不换全开。
    """
    scene = scene or {}
    cx, cy = [float(v) for v in merged["center_base_mm"]]
    h_mm = float(merged["top_mm"]) - float((zcal or {}).get("tower", 0.0))
    yaw = merged.get("yaw_deg")
    block = {"idx": TOWER_BLOCK_IDX, "cx": cx, "cy": cy,
             "top": h_mm, "h": h_mm,
             "yaw_pix": None, "yaw_rect": yaw,
             # ``yaw_for_arm`` 只在**这条**为 False 时标 untrusted
             "yaw_pix_trusted": bool(merged.get("yaw_trusted", True)),
             "class": merged.get("class")}
    cand = {"xyz_base": [cx / 1000.0, cy / 1000.0, h_mm / 1000.0],
            "dist_mm": 0.0,               # 块心就是候选点（grasp_at=块心）
            "angle_deg": 0.0 if yaw is None else float(yaw),
            "score": 1.0,
            # 只进计划 meta（开合宽度由 scene 的 pick/place_open_mm 定），
            # 塔上那块没有 GraspNet 候选宽度可读，给标称值。
            "width_mm": 30.0}
    yg, yp, src, why = s2.yaw_for_arm(arm, block, cand, scene)
    # 正方体四面对称：抓/放角按 90° 折（理由与 `build_items` 同一处）。
    # 折不动就是回位被拒 —— 2026-11-01 入口⑤ 第 6 次/诊断档第 2 次都是这么停的。
    yg, yp = pp.fold_yaw_90(yg), pp.fold_yaw_90(yp)
    return {"order": 1, "block_idx": TOWER_BLOCK_IDX,
            "class": merged.get("class"), "arm": arm,
            "block": block, "cand": cand,
            "grasp_at": scene.get("grasp_at", "block_center"),
            "block_h_raw_mm": round(h_mm, 3),
            "block_h_used_mm": round(h_mm, 3),
            "yaw_grasp_deg": round(float(yg), 3),
            "yaw_place_deg": round(float(yp), 3),
            "yaw_source": src, "yaw_note": why}


def place_one(args, board, chain, scene, grid, analysis, item, slot_mm,
              surface_mm, cdir, tag, survey_top_mm=None, view_xy_mm=None,
              z_bias_mm=0.0, plan_kwargs=None, exec_kwargs=None):
    """抓一块放到塔位 ``surface_mm`` 上并测量；返回记录。

    ``exec_kwargs`` 可带 ``upto`` —— 遮挡实验停在 ``place_approach``；
    测量在调用方做（停下时相机不在 survey 位）。
    """
    plan_kwargs = dict(plan_kwargs or {})
    exec_kwargs = dict(exec_kwargs or {})
    view_xy = view_xy_mm or slot_mm
    top_for_survey = (float(survey_top_mm) if survey_top_mm is not None
                      else float(surface_mm) + m3.BLOCK_TRUE_H_MM)
    pair = _survey_pair(top_for_survey, view_xy, args.d_mm)
    js = board.joints(os.path.join(cdir, "joints-%s" % tag))
    q_start = s2.chain_q(js)
    surface_m = float(surface_mm) / 1000.0
    carry_z = pp.stage3_carry_z(item["block_h_used_mm"] / 1000.0, surface_m)
    try:
        plan = s3r.layer_plan(chain, q_start, item, analysis["blocks"],
                              [slot_mm[0] / 1000.0, slot_mm[1] / 1000.0],
                              surface_m, survey_tool0=pair[0], carry_z=carry_z,
                              scene=scene, **plan_kwargs)
    except pp.PlanError as exc:
        raise EntryStop("计划期护栏拒绝（未动臂）：%s" % exc)
    plan_path = s3r.write_plan(os.path.join(cdir, "plan-%s.json" % tag), plan)
    rc = board.exec_plan(plan_path, os.path.join(cdir, "log-%s.jsonl" % tag),
                         go=True, **exec_kwargs)
    rec = {"tag": tag, "block_idx": item["block_idx"],
           "class": item.get("class"),
           "block_h_used_mm": item["block_h_used_mm"],
           "surface_z_mm": round(float(surface_mm), 3),
           "survey_tool0": plan["meta"]["survey_tool0"],
           "predicted_place_mm": [
               round(v * 1000.0, 2)
               for v in s3r.predicted_place(
                   item, [slot_mm[0] / 1000.0, slot_mm[1] / 1000.0])[0]],
           # 间隙取**这份计划实际用的**（plan meta），不取模块常量：
           # 否则换了 `place_clearance` 之后 Δz 会凭空多出 (5−实际) mm。
           "predicted_top_mm": round(float(surface_mm)
                                     + plan["meta"].get(
                                         "place_clearance_mm",
                                         pp.PLACE_CLEARANCE * 1000.0)
                                     + m3.BLOCK_TRUE_H_MM, 2),
           "exec_rc": rc, "pair": pair, "plan": os.path.basename(plan_path)}
    if rc != 0:
        raise EntryStop("exec 判红（rc=%d，%s）—— 停在原地，不续跑"
                        % (rc, os.path.basename(plan_path)), rc)
    if exec_kwargs.get("upto"):
        rec["stopped_at"] = exec_kwargs["upto"]
        return rec
    meas = measure_pair(board, chain, scene, cdir, tag, pair, grid, view_xy,
                        layer=1, z_bias_mm=z_bias_mm, move=False)
    rec["measured"] = meas
    return rec


def finish_plan(board, cdir, tag, plan_path, after):
    """把停在半路的计划跑完（``--from`` 下一条航点）。

    返回 ``(rc, 实际起跑的航点名)``。
    """
    plan = json.load(open(plan_path))
    nxt = pr.next_waypoint(plan, after)
    rc = board.exec_plan(plan_path, os.path.join(cdir, "log-%s.jsonl" % tag),
                         go=True, from_=nxt, append=True)
    return rc, nxt


def return_to_pick(args, board, chain, scene, cdir, tag, grid, slot_mm,
                   merged, zcal, rdir=None):
    """**机器人自己**把塔上那块抓下来、放回拾取区空位，再回核塔位清空。

    这是「拆塔」的最小形态，不引入新传感器：
    · 抓哪儿 = 上一轮塔顶实测（中心 / yaw / 顶高），``tower_block_item`` 包装；
    · 放哪儿 = **新抓一帧**拾取区，``free_slot_mm`` 选空位；
    · 收尾 = 计划的巡检位就设在塔上方，跑完**原地**``tower_clear_now`` 核空。

    计划从**当前实测关节角**起（刚在拾取巡检位抓完帧），一条计划走完
    「拾取位 → 塔上方抓块 → 抬到载运高 → 拾取区空位放下 → 回塔上方」。
    """
    rdir = rdir or cdir
    os.makedirs(rdir, exist_ok=True)
    frame_dir, extras = capture_pick(args, board, scene, rdir, tag + "-rp")
    analysis = analyse_pick(frame_dir, extras, rdir)
    spot = free_slot_mm(analysis["blocks"], scene["pick_zone"])
    if spot is None:
        raise EntryStop("拾取区放不下回位的块：没有离块 ≥%.0f mm、离边 ≥%.0f mm "
                        "的空位（先把块挪一挪再续跑）"
                        % (PICK_RETURN_GAP_MM, PICK_RETURN_MARGIN_MM))
    item = tower_block_item(merged, scene, zcal, arm="geometric")
    pair = _survey_pair(merged["top_mm"], merged["center_base_mm"], args.d_mm)
    js = board.joints(os.path.join(cdir, "joints-%s" % tag))
    q_start = s2.chain_q(js)
    carry_z = pp.stage3_carry_z(item["block_h_used_mm"] / 1000.0,
                                pp.TABLE_PLANE_Z)
    try:
        plan = s3r.layer_plan(chain, q_start, item, analysis["blocks"],
                              [spot["xy_mm"][0] / 1000.0,
                               spot["xy_mm"][1] / 1000.0],
                              pp.TABLE_PLANE_Z, survey_tool0=pair[0],
                              carry_z=carry_z, scene=scene,
                              release_dwell_s=0.0,
                              tower_half_edge=args.tower_half_edge)
    except pp.PlanError as exc:
        raise EntryStop("回位计划被护栏拒绝（未动臂）：%s" % exc)
    plan_path = s3r.write_plan(os.path.join(cdir, "plan-%s.json" % tag), plan)
    rec = {"tag": tag, "kind": "return",
           "spot_mm": spot["xy_mm"], "spot_gap_mm": spot["gap_mm"],
           "spot_edge_mm": spot["edge_mm"],
           "tower_top_mm": merged["top_mm"],
           "tower_center_mm": merged["center_base_mm"],
           "yaw_grasp_deg": item["yaw_grasp_deg"],
           "yaw_source": item["yaw_source"],
           "predicted_place_mm": [
               round(v * 1000.0, 2)
               for v in s3r.predicted_place(
                   item, [spot["xy_mm"][0] / 1000.0,
                          spot["xy_mm"][1] / 1000.0])[0]],
           "plan": os.path.basename(plan_path)}
    rc = board.exec_plan(plan_path, os.path.join(cdir, "log-%s.jsonl" % tag),
                         go=True)
    rec["exec_rc"] = rc
    _dump(os.path.join(cdir, "return-%s.json" % tag), rec)
    if rc != 0:
        raise EntryStop("回位执行判红（rc=%d）—— 停在原地不续跑；"
                        "块可能还在爪里或半放，先看示教器再决定"
                        % rc, rc)
    print("  回位：塔上那块 → 拾取区 %s（离最近块 %s mm，离边 %s mm，yaw %s）"
          % (spot["xy_mm"], spot["gap_mm"], spot["edge_mm"],
             item["yaw_source"]))
    tower_clear_now(board, scene, cdir, tag + "-clear", grid, slot_mm,
                    cell_idx=args.tower_cell_idx)
    return rec


# ---------------------------------------------------------------- 模式
def clear_tower_step(args, board, chain, scene, cdir, tag, grid, slot_mm,
                     placed, merged=None, zcal=None, rdir=None):
    """每一轮放置前：把塔上那块清掉，并**就地核一次**塔位空了。

    默认**机器人自己回位**（``return_to_pick``）——纯测试性的往返不该压给人。
    ``--no-self-return`` 退回人手动拿：``ask`` 一句 + ``tower_clear_now``。
    ``merged`` 缺（老调用/数据不全）时也退回人手动，并在日志里写明原因。
    """
    if not placed:
        return None
    if getattr(args, "no_self_return", False):
        ask("把这块从塔位拿走，放回拾取区；确认塔位清空")
        return tower_clear_now(board, scene, cdir, tag + "-clear", grid,
                               slot_mm, cell_idx=args.tower_cell_idx)
    if not merged:
        raise EntryStop("要机器人自己回位，但上一轮没有塔顶实测（merged 为空）"
                        " —— 用人手清塔后加 --no-self-return 再跑")
    return return_to_pick(args, board, chain, scene, cdir, tag, grid, slot_mm,
                          merged, zcal, rdir=rdir)


def mode_zcal(args, ctx):
    """① 塔位 z 标定：单块重复放置。"""
    board, chain, scene, grid, slot_mm, pad_mm, zcal = ctx
    cdir = os.path.join(args.run_dir, "entry-zcal")
    os.makedirs(cdir, exist_ok=True)
    if not getattr(args, "resume", False):
        # 续跑时塔位上**本来就压着**上一次的块（正是要机器人拆回来的那块），
        # 所以预检只在全新开跑时做。
        try:
            tower_precheck(args, board, chain, scene, grid, cdir)
        except EntryStop as exc:
            print("!! " + exc.why)
            return exc.rc
    rows, stop, placed = [], None, False
    last_merged = None
    if getattr(args, "resume", False):
        rows = load_rows(cdir)
        if rows:
            placed = True
            last_merged = rows[-1].get("merged") or rows[-1]
            print("  续跑：已有 %d 次结果，从第 %d 次接着跑"
                  % (len(rows), len(rows) + 1))
            if not getattr(args, "no_self_return", False):
                # 第一次「机器人自己拆塔」是**新的动作形态**：当次握手，不继承
                # 之前任何一次授权（AGENTS §1 红线 6）。
                ask("确认急停就绪、两指整个行程内无手无遮挡："
                    "接下来由机器人自己把塔上那块抓回拾取区")
    for r in range(len(rows) + 1, int(args.reps) + 1):
        tag = "zcal-%02d" % r
        rdir = os.path.join(cdir, tag)
        os.makedirs(rdir, exist_ok=True)
        try:
            clear_tower_step(args, board, chain, scene, cdir, tag, grid,
                             slot_mm, placed, merged=last_merged, zcal=zcal,
                             rdir=rdir)
            frame_dir, extras = capture_pick(args, board, scene, rdir, tag)
            analysis = analyse_pick(frame_dir, extras, rdir)
            item = pick_item(args, analysis, scene, zcal)[0]
            pick_bias, _b = pick_bias_of(analysis, item)
            # 第 1 层：表面 = 垫面；期望顶高 = 垫面 + 29.5（survey 位姿按它算）
            rec = place_one(args, board, chain, scene, grid, analysis, item,
                            slot_mm, pad_mm, rdir, tag,
                            survey_top_mm=pad_mm + m3.BLOCK_TRUE_H_MM,
                            z_bias_mm=0.0, plan_kwargs={"tower_half_edge":
                                                        args.tower_half_edge})
        except EntryStop as exc:
            stop = exc.why
            break
        placed = True
        merged = rec["measured"]["merged"]
        last_merged = merged
        row = {"rep": r, "top_mm": merged["top_mm"],
               "center_base_mm": merged["center_base_mm"],
               "yaw_deg": merged.get("yaw_deg"),
               "pick_bias_mm": pick_bias,
               "block_idx": item["block_idx"], "class": item.get("class"),
               # 整份融合观测 —— `--resume` 拿它当「塔上那块」，回位/拆塔要用
               "merged": merged}
        rows.append(row)
        _dump(os.path.join(rdir, "row.json"), row)
        print("  第 %d/%d 次：顶高 %.2f mm，中心 %s，偏差 %.2f mm"
              % (r, int(args.reps), merged["top_mm"],
                 merged["center_base_mm"],
                 merged["top_mm"] - pad_mm - m3.BLOCK_TRUE_H_MM))
    if placed and stop is None:
        # 收尾：最后一块也由机器人拿回拾取区，塔位留给下一项实验是空的
        # （不然下一项实验的塔位预检会直接判红）。失败不吞：记进 stop。
        try:
            clear_tower_step(args, board, chain, scene, cdir,
                             "zcal-%02d-end" % len(rows), grid, slot_mm, True,
                             merged=last_merged, zcal=zcal)
            placed = False
        except EntryStop as exc:
            stop = "收尾回位失败：%s" % exc.why
    doc = zcal_doc(rows, pad_mm, meta={
        "kind": "stage3-entry-zcal", "round_dir": cdir,
        "d_mm": float(args.d_mm), "arm": args.arm,
        "tower_cell_idx": int(args.tower_cell_idx),
        "stop_reason": stop,
    })
    path = os.path.join(cdir, "zcal.json")
    _dump(path, doc)
    print("\nz 标定：n=%d 中位 %s mm，残差最大 %s mm，%s"
          % (doc["n"], doc["by_site"]["tower"],
             doc["residual_mm"]["max"],
             "通过" if doc["pass"] else "**未过**（残差 >%.1f mm）"
             % ZCAL_TOL_MM))
    print("  落盘 → %s（`stage3_round.py --zcal` 直接吃）" % path)
    if stop:
        print("!! 停在：%s —— 不自动续跑" % stop)
        return 2
    return 0 if doc["pass"] else 1


def _release_row(item, rec, rep):
    """一次放置的「实测 − 预测」：Δxy、Δyaw（mod 90）、Δz（落稳下沉）。

    Δyaw 记**两套**：``dyaw_deg`` 走深度 ``yaw_rect``（老口径，契约现用），
    ``dyaw_color_deg`` 走彩色掩码（独立于深度）。两套都进表，事后对比再定用哪套。
    """
    merged = rec["measured"]["merged"]
    dx = merged["center_base_mm"][0] - rec["predicted_place_mm"][0]
    dy = merged["center_base_mm"][1] - rec["predicted_place_mm"][1]
    dyaw = (None if merged.get("yaw_deg") is None else
            round(float(s2.signed_err_mod90(
                merged["yaw_deg"], float(item["yaw_place_deg"]))), 3))
    dyaw_color = (None if merged.get("yaw_color_deg") is None else
                  round(float(s2.signed_err_mod90(
                      merged["yaw_color_deg"], float(item["yaw_place_deg"]))),
                        3))
    return {"rep": rep, "dxy_mm": round(math.hypot(dx, dy), 3),
            "dx_mm": round(dx, 3), "dy_mm": round(dy, 3),
            "dyaw_deg": dyaw,
            "dyaw_color_deg": dyaw_color,
            "dz_mm": round(merged["top_mm"] - rec["predicted_top_mm"], 3),
            "measured": merged,
            "predicted_place_mm": rec["predicted_place_mm"],
            "predicted_top_mm": rec["predicted_top_mm"],
            "block_idx": item["block_idx"]}


def mode_release(args, ctx):
    """② 释放扫参：开口 × 停留，各若干次。"""
    board, chain, scene, grid, slot_mm, pad_mm, zcal = ctx
    cdir = os.path.join(args.run_dir, "entry-release")
    os.makedirs(cdir, exist_ok=True)
    opens = [float(v) for v in str(args.opens).split(",") if v]
    dwells = [float(v) for v in str(args.dwells).split(",") if v]
    # 放置间隙（mm）。默认只跑 5 mm（`PLACE_CLEARANCE`，阶段 2 的老值）。
    clr_arg = getattr(args, "place_clearances", None)
    clearances = ([float(v) for v in str(clr_arg).split(",") if v]
                  if clr_arg else [5.0])
    groups = {}
    order = []
    for op in opens:
        for dw in dwells:
            for cl in clearances:
                # 5 mm 是默认值，组名保持老样子（已有 run 目录不用改名）
                name = ("open%.0f_dwell%.2f" % (op, dw) if cl == 5.0
                        else "open%.0f_dwell%.2f_clr%.1f" % (op, dw, cl))
                order.append((name, op, dw, cl))
    ask("确认急停就绪、两指整个行程内无手无遮挡：接下来连续夹放 %d 次"
        "（%d 组 × %d 次，最优组再补 %s 次），每次都由机器人自己把块拆回拾取区"
        % (int(args.reps) * len(order) + int(args.confirm or 0), len(order),
           int(args.reps), args.confirm or 0))
    resume = bool(getattr(args, "resume", False))
    done = {name: {row["rep"]: row for row in load_rows(os.path.join(cdir, name))}
            for name, _op, _dw, _cl in order}
    stop, placed, last_merged = None, False, None
    try:
        meas, msg = measure_tower(args, board, chain, scene, grid, cdir,
                                  tag=("resume-precheck" if resume else "precheck"))
        if resume:
            for name, _op, _dw, _cl in order:
                if done[name]:
                    last_merged = done[name][max(done[name])].get("measured")
        act = resume_tower_action(msg, meas, last_merged, resume)
        if act == "stop":
            raise EntryStop(msg + " —— 先清空再跑")
        if act == "return":
            print("  续跑：塔位非空 → 机器人自己把上次那块拆回拾取区")
            return_to_pick(args, board, chain, scene, cdir, "resume-clear",
                           grid, slot_mm, meas.get("merged") or last_merged,
                           zcal)
        # 走到这里塔位一定是空的（刚拆回或本来就空）
        placed, last_merged = False, None
    except EntryStop as exc:
        print("!! " + exc.why)
        return exc.rc
    for name, op, dw, cl in order:
        if stop:
            break
        reps = int(args.reps)
        gdir = os.path.join(cdir, name)
        os.makedirs(gdir, exist_ok=True)
        rows = groups[name] = [done[name][rep] for rep in sorted(done[name])]
        for r in range(1, reps + 1):
            tag = "%s-%02d" % (name, r)
            rdir = os.path.join(gdir, tag)
            os.makedirs(rdir, exist_ok=True)
            if r in done[name]:
                continue
            try:
                clear_tower_step(args, board, chain, scene, cdir, tag, grid,
                                 slot_mm, placed, merged=last_merged,
                                 zcal=zcal, rdir=rdir)
                frame_dir, extras = capture_pick(args, board, scene, rdir, tag)
                analysis = analyse_pick(frame_dir, extras, rdir)
                item = pick_item(args, analysis, scene, zcal)[0]
                rec = place_one(args, board, chain, scene, grid, analysis,
                                item, slot_mm, pad_mm, rdir, tag,
                                z_bias_mm=zcal["tower"],
                                plan_kwargs={
                                    "place_open_mm": op,
                                    "release_dwell_s": dw,
                                    "place_clearance": cl / 1000.0,
                                    "tower_half_edge": args.tower_half_edge})
            except EntryStop as exc:
                stop = exc.why
                break
            placed = True
            last_merged = rec["measured"]["merged"]
            row = _release_row(item, rec, r)
            rows.append(row)
            _dump(os.path.join(rdir, "row.json"), row)
            print("  %s 第 %d/%d 次：Δxy %.2f mm，Δyaw %s°，Δz %.2f mm"
                  % (name, r, reps, row["dxy_mm"], row["dyaw_deg"],
                     row["dz_mm"]))
        groups[name] = rows
        _dump(os.path.join(cdir, "summary.json"),
              {"kind": "stage3-entry-release", "groups_raw": groups,
               "summary": release_summary(groups), "stop_reason": stop})
    # 最优组补次数
    summ = release_summary(groups)
    best = summ.get("best")
    if stop is None and best and args.confirm and int(args.confirm) > 0:
        print("最优组是 %s；接下来补 %d 次确认（每次仍由机器人自己回位）"
              % (best, int(args.confirm)))
        op, dw, cl = [(o, d, c) for n, o, d, c in order if n == best][0]
        gdir = os.path.join(cdir, best)
        rows = groups.setdefault(best, [])
        for r in range(1, int(args.confirm) + 1):
            tag = "%s-confirm-%02d" % (best, r)
            rdir = os.path.join(gdir, tag)
            os.makedirs(rdir, exist_ok=True)
            if (int(args.reps) + r) in done[best]:
                continue
            try:
                clear_tower_step(args, board, chain, scene, cdir, tag, grid,
                                 slot_mm, placed, merged=last_merged,
                                 zcal=zcal, rdir=rdir)
                frame_dir, extras = capture_pick(args, board, scene, rdir, tag)
                analysis = analyse_pick(frame_dir, extras, rdir)
                item = pick_item(args, analysis, scene, zcal)[0]
                rec = place_one(args, board, chain, scene, grid, analysis,
                                item, slot_mm, pad_mm, rdir, tag,
                                z_bias_mm=zcal["tower"],
                                plan_kwargs={
                                    "place_open_mm": op,
                                    "release_dwell_s": dw,
                                    "place_clearance": cl / 1000.0,
                                    "tower_half_edge": args.tower_half_edge})
            except EntryStop as exc:
                stop = exc.why
                break
            placed = True
            last_merged = rec["measured"]["merged"]
            row = _release_row(item, rec, int(args.reps) + r)
            rows.append(row)
            _dump(os.path.join(rdir, "row.json"), row)
            print("  确认 %d/%d：Δxy %.2f mm，Δyaw %s°"
                  % (r, int(args.confirm), row["dxy_mm"], row["dyaw_deg"]))
    if placed and stop is None:
        # 收尾：最后一块也由机器人拿回拾取区（塔位留给下一项实验）。
        # 只在本轮**新放**过块时才有块要收；续跑时塔位在开工处已被核空。
        try:
            clear_tower_step(args, board, chain, scene, cdir, "release-end",
                             grid, slot_mm, True, merged=last_merged, zcal=zcal)
        except EntryStop as exc:
            stop = "收尾回位失败：%s" % exc.why
    summ = release_summary(groups)
    doc = {"kind": "stage3-entry-release", "grid_center_base_mm": slot_mm,
           "opens": opens, "dwells": dwells, "clearances": clearances,
           "reps": int(args.reps),
           "confirm": int(args.confirm) if args.confirm else 0,
           "groups_raw": groups, "summary": summ, "stop_reason": stop}
    path = _dump(os.path.join(cdir, "summary.json"), doc)
    print("\n释放扫参（按重复性排序）：")
    for name in summ["order"]:
        s = summ["groups"][name]
        print("  %-18s n=%-3d Δxy 极差 %.2f 中位 %.2f | Δyaw 极差 %s"
              % (name, s["n"], s["dxy_mm"].get("range", float("nan")),
                 s["dxy_mm"].get("median", float("nan")),
                 s["dyaw_deg"].get("range")))
    print("  数据 → %s" % path)
    print("  数值闸门不在这里编：把这张表带给用户拍板，再写进契约 §3-2")
    return 2 if stop else 0


def mode_occlusion(args, ctx):
    """③ 持块遮挡：夹着块停在 place_approach，看下层塔顶还在不在。"""
    board, chain, scene, grid, slot_mm, pad_mm, zcal = ctx
    heights = [int(v) for v in str(args.heights).split(",") if v]
    cdir = os.path.join(args.run_dir, "entry-occlusion")
    os.makedirs(cdir, exist_ok=True)
    try:
        tower_precheck(args, board, chain, scene, grid, cdir)
    except EntryStop as exc:
        print("!! " + exc.why)
        return exc.rc
    out, stop = [], None
    for h in heights:
        hdir = os.path.join(cdir, "layer%02d" % h)
        os.makedirs(hdir, exist_ok=True)
        tower_top_mm = pad_mm + h * m3.BLOCK_TRUE_H_MM
        try:
            # ---- 基准：人手搭好的塔，先按正常距离量一遍
            ask("请在塔位（格 %d）用 %d 块搭好直塔，然后离开工作区"
                % (int(args.tower_cell_idx), h))
            pair = _survey_pair(tower_top_mm, slot_mm, args.d_mm)
            base = measure_pair(board, chain, scene, hdir, "base", pair, grid,
                                slot_mm, layer=h, z_bias_mm=zcal["tower"])
            # 搭错层数 / 塔位没清空 ⇒ 后面按期望顶高分类会全错，就地停
            ok, dh = layer_height_check(base["merged"]["top_mm"],
                                        tower_top_mm)
            if not ok:
                raise EntryStop(
                    "%d 层塔顶量到 %.1f mm，应有 %.1f mm（差 %+.1f mm）——"
                    "塔位没清空或块数不对，拆掉重搭"
                    % (h, base["merged"]["top_mm"], tower_top_mm, dh))
            # ---- 夹着块停在 place_approach
            frame_dir, extras = capture_pick(args, board, scene, hdir, "pick")
            analysis = analyse_pick(frame_dir, extras, hdir)
            item = pick_item(args, analysis, scene, zcal)[0]
            rec = place_one(args, board, chain, scene, grid, analysis, item,
                            slot_mm, tower_top_mm, hdir, "hold",
                            survey_top_mm=tower_top_mm,
                            z_bias_mm=zcal["tower"],
                            plan_kwargs={"tower_xy_mm": slot_mm,
                                         "tower_top_mm": tower_top_mm,
                                         "tower_half_edge": args.tower_half_edge},
                            exec_kwargs={"upto": "place_approach"})
            n = int(scene.get("terminal_frames", 3))
            dirs = s3r.capture_frames(
                board, os.path.join(hdir, "frame-hold"),
                "entry-occl-l%02d-hold" % h, n)
            blocks, _tz = gc.find_blocks(gc.load_frame(dirs[0]),
                                         max_h_mm=m3.MAX_H_MM,
                                         min_h_mm=m3.MIN_H_MM)
            held_top_mm = (tower_top_mm + pp.PLACE_APPROACH_GAP * 1000.0
                           + pp.PLACE_CLEARANCE * 1000.0
                           + m3.BLOCK_TRUE_H_MM)
            rep = occlusion_report(blocks, tower_top_mm, slot_mm,
                                   held_top_mm=held_top_mm)
            rep.update({"layer": h, "frames": [os.path.basename(d)
                                               for d in dirs],
                        "baseline_merged": base["merged"],
                        "held_frame_dir": dirs[0]})
            _dump(os.path.join(hdir, "occlusion.json"), rep)
            out.append(rep)
            print("  %d 层：塔窗内 %d 簇；塔顶高度上 %d 簇 ⇒ %s"
                  % (h, rep["n_window"], len(rep["tower_clusters"]),
                     "看得见" if rep["tower_top_visible"] else "**看不见**"))
            # ---- 把这一块放上去（塔变 h+1 层，由人重搭）
            rc, nxt = finish_plan(board, hdir, "hold",
                                  os.path.join(hdir, "plan-hold.json"),
                                  "place_approach")
            if rc != 0:
                raise EntryStop("补完放置失败（rc=%d，从 %s）" % (rc, nxt), rc)
        except EntryStop as exc:
            stop = exc.why
            break
        if h != heights[-1]:
            ask("刚才这块已经放到塔顶（塔现在 %d 层）。请把塔拆掉，"
                "按下一档高度重搭" % (h + 1))
    doc = {"kind": "stage3-entry-occlusion", "heights": heights,
           "d_mm": float(args.d_mm), "results": out, "stop_reason": stop}
    path = _dump(os.path.join(cdir, "summary.json"), doc)
    visible = [r["layer"] for r in out if r["tower_top_visible"]]
    print("\n遮挡实验结果：%s"
          % ("、".join("%d 层可见" % v for v in visible) if visible
             else "所有测过的塔层都看不见塔顶"))
    print("  数据 → %s" % path)
    print("  eye-in-hand 对照组做不做由用户看这张表拍板")
    return 2 if stop else 0


def mode_tolerance(args, ctx):
    """④ 对中容差：人手偏置放置，机器人只动臂。"""
    board, chain, scene, grid, slot_mm, pad_mm, zcal = ctx
    offsets = [float(v) for v in str(args.offsets).split(",") if v]
    cdir = os.path.join(args.run_dir, "entry-tolerance")
    os.makedirs(cdir, exist_ok=True)
    try:
        tower_precheck(args, board, chain, scene, grid, cdir)
    except EntryStop as exc:
        print("!! " + exc.why)
        return exc.rc
    out, stop = [], None
    for nominal in offsets:
        odir = os.path.join(cdir, "off%02.0f" % nominal)
        os.makedirs(odir, exist_ok=True)
        try:
            # ---- 下块（基准）
            ask("请在塔位（格 %d）居中放**一块**，四边与垫子格线对齐，然后离开工作区"
                % int(args.tower_cell_idx))
            pair = _survey_pair(pad_mm + m3.BLOCK_TRUE_H_MM, slot_mm, args.d_mm)
            lower = measure_pair(board, chain, scene, odir, "lower", pair,
                                 grid, slot_mm, layer=1,
                                 z_bias_mm=zcal["tower"])
            ok, dh = layer_height_check(lower["merged"]["top_mm"],
                                        pad_mm + m3.BLOCK_TRUE_H_MM)
            if not ok:
                raise EntryStop(
                    "下块顶高 %.1f mm，单块应为 %.1f mm（差 %+.1f mm）——"
                    "塔位没清空？清掉重放"
                    % (lower["merged"]["top_mm"],
                       pad_mm + m3.BLOCK_TRUE_H_MM, dh))
            # ---- 上块（人手偏置）
            if nominal <= 0:
                ask("请在它正上方摆第二块（尽量对齐）")
            else:
                ask("请把第二块摆在下块正上方，沿 base x 正方向偏约 %g mm"
                    "（yaw 尽量对齐）" % nominal)
            pair2 = _survey_pair(pad_mm + 2 * m3.BLOCK_TRUE_H_MM, slot_mm,
                                 args.d_mm)
            upper = measure_pair(board, chain, scene, odir, "upper", pair2,
                                 grid, slot_mm, layer=2,
                                 z_bias_mm=zcal["tower"])
            ok, dh = layer_height_check(upper["merged"]["top_mm"],
                                        pad_mm + 2 * m3.BLOCK_TRUE_H_MM)
            if not ok:
                raise EntryStop(
                    "两层顶高 %.1f mm，应有 %.1f mm（差 %+.1f mm）——"
                    "第二块没放上去？"
                    % (upper["merged"]["top_mm"],
                       pad_mm + 2 * m3.BLOCK_TRUE_H_MM, dh))
            print("  静置 %.1f s 后复测" % float(args.hold_s))
            time.sleep(float(args.hold_s))
            pair3 = _survey_pair(pad_mm + 2 * m3.BLOCK_TRUE_H_MM, slot_mm,
                                 args.d_mm)
            after = measure_pair(board, chain, scene, odir, "after", pair3,
                                 grid, slot_mm, layer=2,
                                 z_bias_mm=zcal["tower"])
            lo = lower["merged"]
            up = upper["merged"]
            af = after["merged"]
            off = tol_offset_mm(lo["center_base_mm"], up["center_base_mm"])
            rec = {"nominal_mm": nominal, "offset": off,
                   "lower_center_mm": lo["center_base_mm"],
                   "lower_top_mm": lo["top_mm"],
                   "upper_center_mm": up["center_base_mm"],
                   "upper_top_mm": up["top_mm"],
                   "upper_yaw_deg": up.get("yaw_deg"),
                   "after_center_mm": af["center_base_mm"],
                   "after_top_mm": af["top_mm"],
                   "drift_xy_mm": round(math.hypot(
                       af["center_base_mm"][0] - up["center_base_mm"][0],
                       af["center_base_mm"][1] - up["center_base_mm"][1]), 2),
                   "drift_z_mm": round(af["top_mm"] - up["top_mm"], 2),
                   "dz_layer_mm": round(up["top_mm"] - lo["top_mm"], 2),
                   # 塔倒的机判：静置后顶高掉 10 mm 以上（契约 §3 的塌陷判据）
                   "collapsed": bool(af["top_mm"] - up["top_mm"] <= -10.0)}
            out.append(rec)
            _dump(os.path.join(odir, "row.json"), rec)
            print("  标称 %g mm → 实测 %.2f mm；静置漂移 xy %.2f / z %.2f mm%s"
                  % (nominal, off["d_mm"], rec["drift_xy_mm"],
                     rec["drift_z_mm"],
                     "；**塌了**" if rec["collapsed"] else ""))
            ask("请把这两块都拿走，塔位清空")
        except EntryStop as exc:
            stop = exc.why
            break
    doc = {"kind": "stage3-entry-tolerance", "offsets": offsets,
           "hold_s": float(args.hold_s), "results": out, "stop_reason": stop}
    path = _dump(os.path.join(cdir, "summary.json"), doc)
    print("\n对中容差：")
    for r in out:
        print("  标称 %-4g mm → 实测 %.2f mm，静置漂移 %.2f/%.2f mm%s"
              % (r["nominal_mm"], r["offset"]["d_mm"], r["drift_xy_mm"],
                 r["drift_z_mm"], "（塌）" if r["collapsed"] else ""))
    print("  数据 → %s" % path)
    return 2 if stop else 0


def window_candidates(meas):
    """塔位测量里 pose1 第一帧的候选簇（**含目标块自己**）。"""
    try:
        return list(meas["poses"][0]["frames"][0]["diag"]["candidates"])
    except (KeyError, IndexError, TypeError):
        return []


def scan_near(frame_dir, target_xy_mm, radius_mm=NUDGE_NEAR_MM,
              exclude_mm=NUDGE_SELF_MM):
    """一帧 → 离目标格心 ``[exclude, radius]`` 内的其它块（邻块诊断）。"""
    blocks, _tz = gc.find_blocks(gc.load_frame(frame_dir),
                                 max_h_mm=m3.MAX_H_MM,
                                 min_h_mm=m3.MIN_H_MM)
    return nearby_blocks(blocks, target_xy_mm, radius_mm=radius_mm,
                         exclude_mm=exclude_mm)


def _nudge_once(args, board, chain, scene, grid, cdir, rdir, tag, slot_mm,
                pad_mm, zcal, before, rec, sign, delta, rep):
    """放好一块之后：量过的那份 ``before`` → 只夹不抬地推到目标 → 复测。

    返回一行对账（``status`` 可能是 ``ok`` / ``no_colour_yaw`` /
    ``out_of_range`` / ``plan_rejected``）。执行判红才抛 ``EntryStop``
    —— 那是「块可能还在指间」，必须停。
    """
    merged = before["merged"]
    base = {"rep": int(rep), "tag": tag, "before_merged": merged,
            "placement": {"predicted_place_mm": rec.get("predicted_place_mm"),
                          "predicted_top_mm": rec.get("predicted_top_mm")}}
    if not merged.get("found"):
        raise EntryStop("%s 矫正前量不到块 —— 不瞎夹" % tag)
    yaw_c = merged.get("yaw_color_deg")
    if yaw_c is None or not merged.get("yaw_color_trusted"):
        # 契约 §4.2.5：彩色 yaw 缺失 ⇒ 人评，**不拿深度顶替**。
        return dict(base, status="no_colour_yaw",
                    reason="彩色 yaw 缺失/不可信（深度只报告，不顶替）")
    why = nudge_pose_gate(merged, None)
    if why:
        # 两个视点对「块在哪」都对不上，就别拿它当抓取目标 —— 不瞎夹。
        return dict(base, status="person_review", reason=why)

    cmd = nudge_delta(merged["center_base_mm"], yaw_c, slot_mm,
                      yaw_sign=sign, delta_deg=delta,
                      yaw_quadrant=int(getattr(args, "nudge_yaw_quadrant", 0)
                                       or 0))
    base["commanded"] = {k: round(float(v), 3) for k, v in cmd.items()}
    base["yaw_quadrant"] = int(getattr(args, "nudge_yaw_quadrant", 0) or 0)
    zero = bool(getattr(args, "nudge_zero", False))
    bias = nudge_bias_comp(getattr(args, "nudge_bias_comp", None))
    applied = applied_nudge_command(cmd, zero=zero, bias_xy_mm=bias)
    base["applied"] = {k: round(float(v), 3) for k, v in applied.items()}
    if bias is not None:
        base["bias_comp_mm"] = [round(float(bias[0]), 3),
                                round(float(bias[1]), 3)]
    if zero:
        base["note"] = "nudge-zero 诊断：实测 δ 只记录、不执行"
    elif (abs(applied["d_mm"]) > pp.NUDGE_MAX_SHIFT_MM
            or abs(applied["dyaw_deg"]) > pp.NUDGE_MAX_YAW_DEG):
        return dict(base, status="out_of_range",
                    reason="δ 超单次上限（%.0f mm / %.0f°）—— 该走完整抓放，"
                           "硬拽会把块拖到邻块上"
                           % (pp.NUDGE_MAX_SHIFT_MM, pp.NUDGE_MAX_YAW_DEG))

    # 目标块在塔窗候选里的编号：`check_block_gap` 靠 idx 把它自己排除掉。
    bcx, bcy = [float(v) for v in merged["center_base_mm"]]
    cands = window_candidates(before)
    idx = TOWER_BLOCK_IDX
    if cands:
        near = min(cands, key=lambda c: math.hypot(c["cx"] - bcx,
                                                   c["cy"] - bcy))
        d_near = math.hypot(near["cx"] - bcx, near["cy"] - bcy)
        if d_near > 10.0:
            raise EntryStop("%s 测量中心与帧内候选差 %.1f mm —— 对不上，"
                            "不夹" % (tag, d_near))
        idx = int(near["idx"])
    blocks = [{"idx": int(c["idx"]), "cx": float(c["cx"]),
               "cy": float(c["cy"])} for c in cands]
    if not blocks:
        blocks = [{"idx": idx, "cx": bcx, "cy": bcy}]

    # 复测用同一个位姿对：高度跟踪取**校正后**的塔顶（块不抬，前后同高）。
    top_mm = before.get("next_surface_est_mm")
    if top_mm is None:
        top_mm = merged["top_mm"]
    pair = _survey_pair(top_mm, slot_mm, args.d_mm)
    js = board.joints(os.path.join(cdir, "joints-%s" % tag))
    q_start = s2.chain_q(js)
    # 块高用**真高**：nudge 的夹持深度 = 块高 − 18.5 mm，z 偏差进不来；
    # 脚下那层表面 = 垫面（入口⑤ 只做第 1 层；k≥2 层要换 surface + tower_box）。
    block = {"idx": idx, "cx": bcx, "cy": bcy,
             "top": m3.BLOCK_TRUE_H_MM, "h": m3.BLOCK_TRUE_H_MM}
    try:
        plan = s3r.nudge_plan(chain, q_start, block, blocks,
                              shift_xy_mm=(applied["dx_mm"],
                                           applied["dy_mm"]),
                              dyaw_deg=applied["dyaw_deg"],
                              surface_m=float(pad_mm) / 1000.0,
                              yaw_block_deg=applied["yaw_from_deg"],
                              open_mm=float(args.nudge_open),
                              survey_tool0=pair[0])
    except pp.PlanError as exc:
        return dict(base, status="plan_rejected", reason=str(exc))
    plan_path = s3r.write_plan(os.path.join(rdir, "plan-nudge.json"), plan)
    rc = board.exec_plan(plan_path, os.path.join(rdir, "log-nudge.jsonl"),
                         go=True)
    if rc != 0:
        raise EntryStop("%s 矫正执行判红（rc=%d）—— 停在原地不续跑；"
                        "块可能还在指间，先看示教器再决定" % (tag, rc), rc)
    # nudge 计划最后一条航点就是 pair[0]（survey_tool0），不用再走一趟。
    after = measure_pair(board, chain, scene, rdir, tag + "-after", pair,
                         grid, slot_mm, layer=1, z_bias_mm=zcal["tower"],
                         move=False)
    nb0 = scan_near(os.path.join(rdir, "frame-%s-pose1" % tag), slot_mm)
    nb1 = scan_near(os.path.join(rdir, "frame-%s-after-pose1" % tag), slot_mm)
    row = nudge_row(rep, applied, merged, after["merged"], slot_mm)
    why = nudge_pose_gate(merged, after["merged"])
    if why:
        # 进不了机判：先判过门，再看它原本的 status（契约 §4.2.5）。
        row.update({"status": "person_review", "reason": why})
    row.update(base)
    row.update({"status": row.get("status") or "ok",
                "after_merged": after["merged"],
                "survey_tool0": pair[0],
                "plan": os.path.basename(plan_path),
                "plan_meta": plan["meta"],
                "neighbours_before": nb0, "neighbours_after": nb1,
                "neighbours_shift": neighbour_shift(nb0, nb1)})
    return row


def mode_nudge(args, ctx):
    """⑤ 放置后矫正：开环放一块 → 量 → **只夹不抬**推到目标 → 退开再量。

    契约 §3 第 5 项。δ = 目标 − 实测；≥10 次；每次记**指令 δ 与实走 δ**、
    push gain、有没有带转、有没有顶动邻块；判据就是 §4.1 的 ≤5 mm / ≤5°。

    「放」用入口② 选出的最优释放组（默认开口 50 / 停留 0 / 间隙 3 mm）——
    矫正要修的是**真实产线**会出现的偏差，不是人为制造的偏差。
    每次仍由机器人自己把块拆回拾取区（纯测试的往返不压给人）。
    """
    board, chain, scene, grid, slot_mm, pad_mm, zcal = ctx
    cdir = os.path.join(args.run_dir, "entry-nudge")
    os.makedirs(cdir, exist_ok=True)
    sign = float(scene.get("yaw_sign", 1.0))
    delta = float(scene.get("finger_axis_delta_deg", 0.0))
    reps = int(args.reps)
    if reps < 10:
        print("!! 契约 §3 第 5 项要求 ≥10 次；本次只排了 %d 次 —— 这一轮不能算过"
              % reps)
    resume = bool(getattr(args, "resume", False))
    done = ({row["rep"]: row for row in load_rows(cdir)} if resume else {})
    rows = [done[k] for k in sorted(done)]
    placed = bool(rows)
    last_merged = ((rows[-1].get("after_merged") or rows[-1].get("merged"))
                   if rows else None)
    if rows:
        print("  续跑：已有 %d 次结果，从第 %d 次接着跑"
              % (len(rows), rows[-1]["rep"] + 1))
    ask("确认急停就绪、两指整个行程内无手无遮挡：接下来连续 %d 次"
        "「开环放一块 → 只夹不抬地矫正 → 退开再量」，每次都由机器人自己"
        "把块拆回拾取区" % (reps - len(rows)))
    try:
        meas, msg = measure_tower(args, board, chain, scene, grid, cdir,
                                  tag=("resume-precheck" if resume
                                       else "precheck"))
        act = resume_tower_action(msg, meas, last_merged, resume)
        if act == "stop":
            raise EntryStop(msg + " —— 先清空再跑")
        if act == "return":
            print("  续跑：塔位非空 → 机器人自己把上次那块拆回拾取区")
            return_to_pick(args, board, chain, scene, cdir, "resume-clear",
                           grid, slot_mm, meas.get("merged") or last_merged,
                           zcal)
        # 走到这里塔位一定是空的（刚拆回或本来就空）
        placed, last_merged = False, None
    except EntryStop as exc:
        print("!! " + exc.why)
        return exc.rc
    stop = None
    for r in range(1, reps + 1):
        if r in done:
            continue
        tag = "nudge-%02d" % r
        rdir = os.path.join(cdir, tag)
        os.makedirs(rdir, exist_ok=True)
        try:
            clear_tower_step(args, board, chain, scene, cdir, tag, grid,
                             slot_mm, placed, merged=last_merged, zcal=zcal,
                             rdir=rdir)
            frame_dir, extras = capture_pick(args, board, scene, rdir, tag)
            analysis = analyse_pick(frame_dir, extras, rdir)
            item = pick_item(args, analysis, scene, zcal)[0]
            rec = place_one(args, board, chain, scene, grid, analysis, item,
                            slot_mm, pad_mm, rdir, tag, z_bias_mm=zcal["tower"],
                            plan_kwargs={
                                "place_open_mm": float(args.nudge_open),
                                "release_dwell_s": float(args.nudge_dwell),
                                "place_clearance": float(args.nudge_clearance)
                                / 1000.0,
                                "tower_half_edge": args.tower_half_edge})
        except EntryStop as exc:
            stop = exc.why
            break
        placed = True
        last_merged = rec["measured"]["merged"]
        try:
            row = _nudge_once(args, board, chain, scene, grid, cdir, rdir, tag,
                              slot_mm, pad_mm, zcal, rec["measured"], rec,
                              sign, delta, r)
        except EntryStop as exc:
            stop = exc.why
            break
        rows.append(row)
        _dump(os.path.join(rdir, "row.json"), row)
        if row.get("after_merged"):
            last_merged = row["after_merged"]
        print(format_nudge_line(tag, row))
    if placed and stop is None:
        # 收尾：最后一块也由机器人拿回拾取区（塔位留给下一项实验）
        try:
            clear_tower_step(args, board, chain, scene, cdir, "nudge-end",
                             grid, slot_mm, True, merged=last_merged,
                             zcal=zcal)
        except EntryStop as exc:
            stop = "收尾回位失败：%s" % exc.why
    summ = nudge_summary(rows)
    doc = {"kind": "stage3-entry-nudge", "grid_center_base_mm": slot_mm,
           "pad_z_mm": pad_mm, "reps": reps,
           "release": {"open_mm": float(args.nudge_open),
                       "dwell_s": float(args.nudge_dwell),
                       "clearance_mm": float(args.nudge_clearance)},
           "yaw_quadrant": int(getattr(args, "nudge_yaw_quadrant", 0) or 0),
           "bias_comp_mm": nudge_bias_comp(
               getattr(args, "nudge_bias_comp", None)),
           "max_shift_mm": pp.NUDGE_MAX_SHIFT_MM,
           "max_yaw_deg": pp.NUDGE_MAX_YAW_DEG,
           "rows": rows, "summary": summ, "stop_reason": stop}
    path = _dump(os.path.join(cdir, "summary.json"), doc)
    print("\n后矫正（判据 = 契约 §4.1：≤%.0f mm / ≤%.0f°）："
          % (NUDGE_XY_TOL_MM, NUDGE_YAW_TOL_DEG))
    print("  矫正 %d/%d 次，终态过门 %d 次；没矫正 %d 次"
          % (summ["n_nudged"], summ["n"], summ["n_final_ok"],
             summ["n_skipped"]))
    for key, label in (("xy_err_before", "Δxy 前"), ("xy_err_after", "Δxy 后"),
                       ("yaw_err_before", "Δyaw 前"),
                       ("yaw_err_after", "Δyaw 后")):
        s = summ[key]
        if s.get("n"):
            print("  %-8s 中位 %.2f 极差 %.2f（n=%d）"
                  % (label, s["median"], s["range"], s["n"]))
    for key, label in (("gain_xy", "push gain"), ("push_perp", "侧向走偏 mm"),
                       ("gain_yaw", "转角 gain")):
        s = summ[key]
        if s.get("n"):
            print("  %-12s 中位 %.2f 极差 %.2f（n=%d）"
                  % (label, s["median"], s["range"], s["n"]))
    print("  数据 → %s" % path)
    print("  通过与否带这张表给用户拍板；修不动就如实记失败，不改主命题阈值")
    return 2 if stop else 0


def _fmt(v):
    return "—" if v is None else "%.2f" % float(v)


def format_nudge_line(tag, row):
    """一次矫正的一行摘要。

    2026-11-01 真机事故：原版把 ``_fmt(...)``（**字符串**）塞进 ``%.2f``，
    第 2 次一走到成功分支就 ``TypeError`` 把整个 10 次跑打断（前两次的数据
    已经在盘上，但臂停在半路）。所以这里一律 ``%s`` —— 数值格式由 ``_fmt``
    负责，缺值显示「—」，不再让格式化本身成为崩溃点。
    """
    if row.get("status") != "ok":
        return ("  %s 没矫正（%s）：%s"
                % (tag, row.get("status"), row.get("reason")))
    before = row.get("before") or {}
    after = row.get("after") or {}
    push = row.get("push") or {}
    yaw = row.get("yaw") or {}
    return ("  %s 前 %s mm / %s° → 后 %s mm / %s°；"
            "push gain %s，带转 %s°（指令 %s°）%s"
            % (tag, _fmt(before.get("xy_err_mm")),
               _fmt(before.get("yaw_err_deg")),
               _fmt(after.get("xy_err_mm")), _fmt(after.get("yaw_err_deg")),
               push.get("gain_xy"), _fmt(yaw.get("dyaw_actual_deg")),
               _fmt(yaw.get("dyaw_expected_deg")),
               "；**过了**" if row.get("final_ok") else "；仍超门"))


# ---------------------------------------------------------------- 上下文/CLI
def resolve_zcal_path(run_dir, explicit, mode):
    """z 标定文件从哪来：显式 > ``<run-dir>/entry-zcal/zcal.json`` > **停**。

    忘了传 ``--zcal`` 时宁可停，也不能按 0 偏差静默跑 40 次放置 ——
    这类「没报错但用了默认值」的失效本项目吃过大亏（0x0100 读状态、死键）。
    """
    if explicit:
        return explicit
    if mode == "zcal":
        return None                       # 标定模式本身在产出它
    cand = os.path.join(run_dir, "entry-zcal", "zcal.json")
    if not os.path.isfile(cand):
        raise EntryStop("缺 z 标定（%s）—— 先跑 --mode zcal，或用 --zcal 指一份"
                        % cand)
    return cand


def load_ctx(args):
    """场景 / 网格 / 塔位 / z 标定 / URDF —— 四个模式共用。"""
    global _URDF
    _URDF = args.urdf
    try:
        scene = s2.load_scene(args.scene)
    except ValueError as exc:
        raise EntryStop("场景坏：%s" % exc)
    grid_path = args.grid or scene.get("grid_json")
    if not grid_path:
        raise EntryStop("缺 grid.json —— 塔基、垫面、格线方向都要它")
    try:
        grid = s2.load_grid(grid_path)
        s2.check_grid_order(scene, grid)
    except ValueError as exc:
        raise EntryStop("网格坏：%s" % exc)
    slot_mm = m3.grid_center(grid, args.tower_cell_idx)
    pad_mm = m3.pad_z_mm(grid)
    zcal = s3r.load_zcal(resolve_zcal_path(args.run_dir, args.zcal, args.mode))
    if args.mode != "zcal" and zcal.get("pass") is False:
        raise EntryStop("z 标定未过（残差 >%.1f mm）—— 先补测，别带偏差跑"
                        % float(ZCAL_TOL_MM))
    chain = urkin.Chain(args.urdf)
    return scene, grid, slot_mm, pad_mm, zcal, chain


def protocol(mode):
    head = ("开工前先量一次塔位：非空立刻停（阶段 2 的块还压着中心格时要先清格）；"
            "拾取区没有可用块也停。")
    return head + {
        "zcal": "① z 标定：塔位单块重复放置 --reps 次（默认 5），"
                "两个高度跟踪视点各连拍 scene.terminal_frames 帧（默认 3）；"
                "每次由**机器人自己**把塔上那块抓回拾取区空位"
                "（--no-self-return 才退回人手动），放回后就地核一次「塔位真的空了」。",
        "release": "② 释放扫参：开口 --opens × 停留 --dwells，每组 --reps 次；"
                   "最优组再 --confirm 次。每次由**机器人自己**回位（同上）。",
        "occlusion": "③ 持块遮挡：人手搭 1/3/5 层塔，机器人先量基准（塔高不对就停），"
                     "再夹块停在 place_approach 抓帧；抓完把这块放上去，由人重搭。",
        "tolerance": "④ 对中容差：人手把上块偏 --offsets 放上去，"
                     "机器人只动臂测量 + 静置复测（下块/两层高度对不上就停）。",
        "nudge": "⑤ 后矫正：开环放一块（用入口② 最优组）→ 量 → **只夹不抬**"
                 "推到格心/格线 → 退开再量，--reps 次（契约要求 ≥10）；"
                 "每次由**机器人自己**把块拆回拾取区。判据 = §4.1 的 ≤5 mm / ≤5°。",
    }[mode]


def main(argv=None):
    ap = argparse.ArgumentParser(description="阶段 3 入口五实验驱动")
    ap.add_argument("--mode", required=True, choices=MODES)
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--scene", default=None)
    ap.add_argument("--grid", default=None)
    ap.add_argument("--zcal", default=None, help="已有的 z 标定（zcal 模式不用）")
    ap.add_argument("--arm", default="geometric",
                    choices=("geometric", "zero", "graspnet"))
    ap.add_argument("--tower-cell-idx", type=int, default=m3.TOWER_CELL_IDX)
    ap.add_argument("--tower-half-edge", type=float, default=0.020)
    ap.add_argument("--d-mm", type=float,
                    default=pp.STAGE3_SURVEY_DEFAULT_D * 1000.0)
    ap.add_argument("--reps", type=int, default=5)
    ap.add_argument("--opens", default="50,70,95")
    ap.add_argument("--dwells", default="0,0.5")
    ap.add_argument("--place-clearances", default=None,
                    help="放置时块底离表面的间隙（mm，逗号分隔）；不给=5 mm 老行为")
    ap.add_argument("--confirm", type=int, default=10)
    ap.add_argument("--heights", default="1,3,5")
    ap.add_argument("--offsets", default="0,3,5,8")
    ap.add_argument("--hold-s", type=float, default=10.0)
    ap.add_argument("--nudge-open", type=float, default=50.0,
                    help="⑤ 矫正前开环放置用的开口（mm）；默认入口② 最优组")
    ap.add_argument("--nudge-dwell", type=float, default=0.0,
                    help="⑤ 矫正前开环放置用的停留（s）")
    ap.add_argument("--nudge-clearance", type=float, default=3.0,
                    help="⑤ 矫正前开环放置用的间隙（mm）；默认入口② 最优组")
    ap.add_argument("--nudge-zero", action="store_true",
                    help="⑤ 诊断档：实测 δ 只记录、不执行，只走「接近→闭合→"
                         "松开→退开」，用来查 nudge 里那个恒定侧推出在哪一步")
    ap.add_argument("--nudge-bias-comp", type=str, default=None, metavar="EX,EY",
                    help="⑤ 预补偿：闭合那一下会把块沿指轴推走 (E·â)·â，E 给在 base 系"
                         "（mm）。横移目标里减掉预测的 Δ。现值 \"-2.05,-4.78\""
                         "（台账第一百八十九轮），但只在 4 号格附近估过。"
                         "**必须用等号式**（--nudge-bias-comp=-2.05,-4.78）："
                         "负号开头、空格分开会被 argparse 当成另一个选项")
    ap.add_argument("--nudge-yaw-quadrant", type=int, default=0, choices=(0, 1, 2, 3),
                    help="⑤ 接近朝向再加 k·90°（方块 90° 自对称，仍是合法抓取）。"
                         "用来做方向帧判别：夹爪系里的侧推会跟着转，世界系里的不转")
    ap.add_argument("--no-self-return", action="store_true",
                    help="把「塔上那块」交回人手动拿走；默认由机器人自己抓回拾取区")
    ap.add_argument("--resume", action="store_true",
                    help="zcal/release/nudge：复用 run-dir 里已有的 row.json，"
                         "从没做的那一位接着跑；塔上若还压着上次那块，"
                         "先由机器人自己拆回")
    ap.add_argument("--ask-mode", default="enter",
                    choices=("enter", "file", "auto"),
                    help="「需要人动手」怎么放行：enter=等回车（默认；没终端就不停）；"
                         "file=等哨兵文件 ack-NNN（AI 驱动用）；auto=不停（不许配 --go）")
    ap.add_argument("--ask-dir", default=None,
                    help="--ask-mode file 的哨兵目录；默认 <run-dir>/_ask")
    ap.add_argument("--ask-timeout-s", type=float, default=0.0,
                    help="--ask-mode file 等多久算超时；0=一直等（默认）")
    ap.add_argument("--go", action="store_true",
                    help="真跑；**每次都当次授权**，闸门在 precision_exec")
    args = ap.parse_args(argv)
    if args.go and args.ask_mode == "auto":
        raise SystemExit("--ask-mode auto 不许配 --go：那等于把「需要人动手」"
                         "全跳过还动臂")
    _ASK["mode"] = args.ask_mode
    _ASK["dir"] = args.ask_dir or os.path.join(args.run_dir, "_ask")
    _ASK["timeout_s"] = float(args.ask_timeout_s)

    if not args.go:
        print(protocol(args.mode))
        print("（没加 --go：只打印协议，不动臂、不抓帧）")
        return 0

    try:
        scene, grid, slot_mm, pad_mm, zcal, chain = load_ctx(args)
    except EntryStop as exc:
        print("!! " + exc.why)
        return exc.rc
    os.makedirs(args.run_dir, exist_ok=True)
    board = pr.Board(s2.BOARD_ALIAS, args.run_dir)
    print(protocol(args.mode))
    # 动臂前的第一道确认也走 ask：file 模式下它就是一个必须被放行的哨兵，
    # 而不是一行没人看的提示。
    ask("确认急停就绪、两指行程内无手无遮挡；执行器在板上还会核对 "
        "PLAYING / JTC / robot_mode")
    board.push()
    ctx = (board, chain, scene, grid, slot_mm, pad_mm, zcal)
    fn = {"zcal": mode_zcal, "release": mode_release,
          "occlusion": mode_occlusion, "tolerance": mode_tolerance,
          "nudge": mode_nudge}[args.mode]
    return fn(args, ctx)


if __name__ == "__main__":
    sys.exit(main())

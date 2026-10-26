#!/usr/bin/env python3
"""夹着块的时候量一次：块的顶面中心到底在不在 tool0 轴上。

为什么非要这一帧
----------------
2026-10-26 第 4 个循环，块放进 0 号槽后顶面中心离槽心 9.69 mm，而且是**单方向**
（用户的原话：「夹爪位置和方块中心相比远离靶纸」）。这个 9.69 mm 只有两种来路：

    A. 抓的时候就没抓正（候选/块心估计偏，或外参在这一侧偏）
    B. 抓正了，块在指垫之间滑了（途中加速，或放下时被台面顶了一下）

只看放置帧分不开 A 和 B —— 两种都表现为「块不在槽心」。在 **lift 停住、臂静止、
块还夹在爪里**的时候抓一帧，量「块顶面中心在 tool0 坐标系里的 xy 偏移」就分开了：

    偏移 ≈ 计划值     ⇒ 抓的时候是正的，问题在抓之后（B）
    偏移比计划大得多 ⇒ 抓的时候就没抓正（A）

`precision_round.py --held-capture` 会在这个时刻自动抓一帧（`frames/held-NN`）。

量法
----
不聚类、也不整块算质心：只取**高位**（台面 + 80 mm 以上，远高于任何立着的块）
且**颜色属于目标类**的点，再取最高 3 mm 那一层算中心。指垫是黑的（S/V 太低），
这条颜色约束天然把夹爪排除在外 —— 而按「离相机最近的那一坨」取会很危险：
指垫就贴在块的侧面，深度上分不开。

用法
  python3 measure_held.py <帧目录> --urdf robot.urdf                 # 自动判类
  python3 measure_held.py <帧目录> --urdf robot.urdf --class cyan --plan pick.json
"""
import argparse
import json
import os
import sys

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import block_classes                                            # noqa: E402
import graspnet_candidates as gc                                # noqa: E402
import urkin                                                    # noqa: E402

import numpy as np                                              # noqa: E402

HIGH_ABOVE_TABLE = 0.080     # 立着的块最高 35 mm；80 mm 之上只有「被举起来的」
TOP_STRATUM = 0.003          # 与 find_blocks 同一把尺子：顶面取最高 3 mm
S_MIN, V_MIN = 60, 40        # 与 find_blocks / palette_survey 同值


def rgb_to_hsv_vec(rgb):
    """整张图一次算完的 RGB→HSV。数值与 ``graspnet_candidates.rgb_to_hsv255``
    （逐色 ``colorsys``）一致 —— 逐像素调 colorsys 太慢，而两套实现一旦不一致，
    这里选出来的像素与 find_blocks 选出来的就不是同一批，读数没法互相比。"""
    a = np.asarray(rgb, dtype=np.float64) / 255.0
    mx = a.max(-1)
    mn = a.min(-1)
    d = mx - mn
    h = np.zeros_like(mx)
    m = d > 1e-12
    r, g, b = a[..., 0], a[..., 1], a[..., 2]
    i = m & (mx == r)
    h[i] = ((g[i] - b[i]) / d[i]) % 6.0
    i = m & (mx == g)
    h[i] = (b[i] - r[i]) / d[i] + 2.0
    i = m & (mx == b)
    h[i] = (r[i] - g[i]) / d[i] + 4.0
    h *= 60.0
    s = np.where(mx > 1e-12, d / np.maximum(mx, 1e-12) * 255.0, 0.0)
    return h, s, mx * 255.0


def class_of_hue_vec(hue, names):
    """逐像素判类（向量化）：环形最近色相。只用来选像素，不用来定类别。"""
    hues = np.array([block_classes.MEASURED_HUE_DEG[n] for n in names], float)
    diff = np.abs(hue[..., None] - hues) % 360.0
    diff = np.minimum(diff, 360.0 - diff)
    return diff.argmin(-1), diff.min(-1)


def tool0_pose(frame, urdf):
    js = frame.get("joints")
    if not js:
        return None
    q = dict(zip(js["name"], js["position"]))
    return urkin.Chain(urdf).fk(q)


def measure(frame_dir, urdf, want_class=None, plan_path=None, verbose=True):
    frame = gc.load_frame(frame_dir)
    R, t, K = frame["R_bc"], frame["t_bc"], frame["K"]
    P, pix = gc.backproject(frame["depth"], K)
    Pb = P @ R.T + t
    tz = gc.table_plane_z(Pb)

    hue, sat, val = rgb_to_hsv_vec(frame["color"])
    hsv_pt = hue[pix[:, 1], pix[:, 0]], sat[pix[:, 1], pix[:, 0]], \
        val[pix[:, 1], pix[:, 0]]
    usable = (hsv_pt[1] >= S_MIN) & (hsv_pt[2] >= V_MIN)
    high = Pb[:, 2] > tz + HIGH_ABOVE_TABLE

    names = list(block_classes.DEFAULT_CLASSES)
    idx, _ = class_of_hue_vec(hsv_pt[0], names)
    base_mask = usable & high
    if base_mask.sum() < 200:
        raise SystemExit("台面上方 %.0f mm 以上几乎没有彩色点（%d 个）—— "
                         "臂真的停在 lift 吗？"
                         % (HIGH_ABOVE_TABLE * 1000, int(base_mask.sum())))

    # 自动判类：按饱和度加权的多数票，与 find_blocks 同口径
    if want_class is None:
        w = np.zeros(len(names))
        for i, n in enumerate(names):
            w[i] = hsv_pt[1][base_mask & (idx == i)].sum()
        want_class = names[int(w.argmax())]
    if want_class not in names:
        raise SystemExit("类别 %r 不在表里（有 %s）" % (want_class, names))
    sel = base_mask & (idx == names.index(want_class))
    if sel.sum() < 200:
        raise SystemExit("高位点上没有 %s 的点（%d 个）—— 夹的是这一块吗？"
                         % (want_class, int(sel.sum())))

    Q = Pb[sel]
    ztop = float(np.percentile(Q[:, 2], 97))
    F = Q[Q[:, 2] > ztop - TOP_STRATUM]
    c = F[:, :2].mean(0)
    spread = float(np.ptp(F[:, :2], axis=0).max()) * 1000
    centre = np.array([c[0], c[1], ztop])

    T = tool0_pose(frame, urdf)
    if T is None:
        raise SystemExit("帧里没有 joint_states.json —— 算不出 tool0")
    Rt, tt = T[:3, :3], T[:3, 3]
    # 横向偏移投影到 tool0 系（此刻 tool0 竖直朝下 ⇒ 与 base 的 xy 同向，
    # 所以这组数可以直接和放置帧的 dx/dy 相减）。
    d_tool = Rt.T @ (centre - tt)
    # 竖直方向**在 base 里**算：tool0 的 +z 轴朝下（Rz≈(0,0,-1)），在 tool 系里
    # 「下方」是 +z，写出来每次都要翻一次符号 —— 直接用 base 的 z 差，没有歧义。
    below = float(tt[2] - centre[2])

    out = {"frame": os.path.abspath(frame_dir), "class": want_class,
           "table_z_mm": round(tz * 1000, 2),
           "n_points_high": int(base_mask.sum()), "n_points_class": int(sel.sum()),
           "block_centre_base_mm": [round(float(v * 1000), 2) for v in centre],
           "block_top_spread_mm": round(spread, 1),
           "tool0_base_mm": [round(float(v * 1000), 2) for v in tt],
           "tool0_z_axis": [round(float(v), 4) for v in Rt[:, 2]],
           "block_in_tool_mm": [round(float(v * 1000), 2) for v in d_tool],
           "block_below_tool_mm": round(below * 1000, 2),
           "still": (frame.get("joints") or {}).get("still"),
           "p2p_deg": (frame.get("joints") or {}).get("joint_p2p_deg")}

    if plan_path:
        meta = json.load(open(plan_path)).get("meta", {})
        gp = meta.get("grasp_point")
        if meta.get("block_xy_mm") and gp:
            # 计划把 tool0 放在 grasp_point 上；块心是**抓帧**估的那个点。
            # 所以「完美执行」时这个偏移不是 0，而是抓帧那两个数之差。
            # 抓帧的时刻 tool0 竖直朝下，两个偏移都在 base 的 xy 里，
            # 与放置帧的 dx/dy 是同一套坐标，可以直接相减。
            want_xy = np.array([(meta["block_xy_mm"][0] - gp[0] * 1000.0),
                                (meta["block_xy_mm"][1] - gp[1] * 1000.0)])
            out["plan_expected_xy_mm"] = [round(float(v), 2) for v in want_xy]
            out["vs_plan_xy_mm"] = [round(float((d_tool[i] * 1000 - want_xy[i])), 2)
                                    for i in range(2)]
            out["vs_plan_norm_mm"] = round(float(np.linalg.norm(
                d_tool[:2] * 1000 - want_xy)), 2)
        if meta.get("grasp_z") is not None and meta.get("wz_used_mm") is not None:
            # 抓的时候块顶面在 tool0 下方多远 —— 计划算得出来，帧里也量得出来。
            # 两个数对不上说明深度尺度或外参在 lift 位姿变了（不是 xy 的事）。
            out["plan_below_tool_mm"] = round(
                (meta["grasp_z"] - meta["wz_used_mm"] / 1000.0) * 1000, 2)

    if verbose:
        print("帧 %s" % frame_dir)
        print("  台面 z=%+.1f mm   高位点 %d   其中 %s %d 点   静止=%s（峰峰 %.3f°）"
              % (tz * 1000, int(base_mask.sum()), want_class, int(sel.sum()),
                 out["still"], out["p2p_deg"] or 0.0))
        print("  块心 base (%.1f, %.1f, %.1f) mm   顶面 xy 展开 %.1f mm"
              % (centre[0] * 1000, centre[1] * 1000, centre[2] * 1000, spread))
        print("  tool0 base (%.1f, %.1f, %.1f) mm   z 轴 %s"
              % (tt[0] * 1000, tt[1] * 1000, tt[2] * 1000,
                 np.round(Rt[:, 2], 3).tolist()))
        print("  **块心相对 tool0 轴**  (dx, dy) = (%+.1f, %+.1f) mm"
              "   [与放置帧 dx/dy 同一套坐标，可直接相减]"
              % (d_tool[0] * 1000, d_tool[1] * 1000))
        print("  块顶面在 tool0 下方 %+.1f mm%s"
              % (below * 1000,
                 "" if "plan_below_tool_mm" not in out
                 else "（计划 %.1f mm，差 %+.1f mm）"
                 % (out["plan_below_tool_mm"],
                    below * 1000 - out["plan_below_tool_mm"])))
        if "plan_expected_xy_mm" in out:
            e = out["plan_expected_xy_mm"]
            print("  计划要求的偏移      (%+.1f, %+.1f) mm（=抓帧那个块心 − 抓点）"
                  % (e[0], e[1]))
            print("  ⇒ 超出计划的部分    (%+.1f, %+.1f) mm   |%.1f| mm"
                  "（滑移 + 抓帧估计误差，两者这里分不开）"
                  % (out["vs_plan_xy_mm"][0], out["vs_plan_xy_mm"][1],
                     out["vs_plan_norm_mm"]))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("frame")
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--class", dest="cls", default=None)
    ap.add_argument("--plan", default=None, help="计划 JSON（比对计划要求的偏移）")
    ap.add_argument("--json", default=None)
    args = ap.parse_args()
    out = measure(args.frame, args.urdf, args.cls, args.plan)
    if args.json:
        json.dump(out, open(args.json, "w"), ensure_ascii=False, indent=1)
        print("  → %s" % args.json)
    return 0


if __name__ == "__main__":
    sys.exit(main())

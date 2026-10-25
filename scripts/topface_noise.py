#!/usr/bin/env python3
"""顶面抓取点的**误差结构**：一帧的抖动有多大、哪一半在抖、多帧融合能救多少。

为什么单独一个脚本（2026-10-25）
================================
`eval_baseline.py --est pc` 只报一个「3D 中位 3.5 mm」。这个数在**决定下一步
该往哪使劲**时不够用：它把三件不同的事混在一起——

  1. **单帧抖动**（precision）：同一块、同一视角，重拍一次会差多少；
  2. **视角相关系统偏置**（bias）：换个臂位姿再看，同一块的高度读数会整体
     平移多少 —— 这部分**过滤不掉**，只能靠近距或标定；
  3. **真值模型自身的不确定**：真值把每块都当成「边长=高=方块高」的立方体，
     逐块高度其实**没测过**（现场只量过一次 28–29 mm）。

本脚本量第 1 项并把方差拆成「局部桌面」和「高出桌面」两半，顺带给出**时间
融合**能到多少（半半对照，不用外部真值）。第 2 项的判决实验要动积木（见
`docs/待执行命令.md`），第 3 项要卡尺。

指标为什么这样取
================
· 同一个块跨帧的真实高度不变 ⇒ **逐块跨帧标准差**就是单帧抖动，不需要真值；
· 方差分解回答「该打磨哪一半」：局部桌面（检测框外扩 40 px 里拟的平面）与
  高出桌面（顶面相对它的高度）各自贡献多少；
· 融合收益用**不相交的两组帧**各取中位再比（A−B 的散布 = √2·σ_N）。
  ⚠️ 千万别用滚动中位：窗口与自身重叠，N 越大 std 越接近 0 —— 那是自证式绿灯
  （本脚本第一版就是这么错的，N=9 时 std 显示 0.00，实际单帧 σ 是 2.5 mm）。
  ⚠️ 两组各取**中位**，所以它估的是**中位融合**的 σ。高斯噪声下中位比均值
  差一截，故对照要用 σ₁·√(MEDIAN_VAR[n])（小 n 的准确值，见下面的表）；
  `√N` 那列是**均值融合**的理论值，两列一起印，别把半半对照的数和 √N 直接
  比（那是苹果比橘子）。

已知结论（2026-10-25，腕部 0.7 m，10 帧 × 9 块）
=================================================
· 单帧 σ（Z）中位 **2.5 mm**，range 到 10 mm；
· 拆开：局部桌面那一半 σ≈1.1 mm，高出桌面那一半 σ≈2.5 mm ⇒ 顶面深度是主源；
· 六个估计变体（z 带长大重拟、LSQ、宽 6 mm、彩色亮层拟平面、亮层中位、
  亮层 p75/p85/p90）σ 都落在 2.0–2.6 mm，且**σ 更小的都带 2–5 mm 系统偏置**
  ⇒ 单帧精度已经压在 D435i 在 0.7 m 的噪声地板上，换算法赚不到；
· 半半对照：N=5 帧融合后 σ≈1.0 mm（中位融合理论 1.5，均值融合理论 1.2；
  实测**低于两种理论** —— 真实噪声不是高斯的，重尾时中位比高斯理论更省；
  这两个数只当数量级参考，不当判据）。
  ⚠️ 前提是**相邻帧独立**。静态场景里 RealSense 的深度引擎不保证这一点，
  所以融合收益必须**在板上用连续帧实测**再决定要不要开（见待执行命令）。

用法
====
    python3 scripts/topface_noise.py --frame <帧目录> [--frame <帧目录> ...] \
        [--out noise.json] [--pc-min-score 0.30]

帧目录由 `scripts/capture_baseline.py` 生成。**同一视角的重复帧**给单帧抖动；
**不同视角**的帧不要混在一起算 σ（那会把系统偏置算成抖动）—— 一次只喂一组。
"""

import argparse
import json
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
import eval_baseline as E  # noqa: E402  复用加载/板位姿/点云/估计这一整套
import pc_grasp           # noqa: E402

GROUP_TOL_MM = 10.0     # 同一个块的跨帧像素位置只差几像素；base 系位置差几毫米
MIN_FRAMES = 3          # 少于 3 帧的块不参与 σ
FUSE_N = (3, 5)         # 半半对照要评估的融合帧数
# 高斯下「n 个样本取中位」的方差系数 Var(median)/σ²。小 n 与渐近式 π/(2n)
# 差得**不小**（n=3 是 0.448 而非 0.524，差 8%），而本脚本的融合档都在小 n，
# 所以用实测值而不是渐近式 —— 否则「理论」先偏了，实测对不上还会被当成噪声。
# 数值：3e6 组蒙特卡洛，相对误差 <0.3%。
MEDIAN_VAR = {2: 0.4998, 3: 0.4479, 4: 0.2983, 5: 0.2866}


def collect(frame_dirs, pc_min_score=pc_grasp.MIN_SCORE, square_mm=None,
            block_h_mm=None):
    """逐帧跑 pc 估计 → 每条估计一条记录（base 系）。

    记录字段：``frame`` / ``bx,by,bz``（base 系，米）/ ``above_m``（高出局部
    桌面）/ ``plane_z``（局部桌面在该像素处的高度，base 系）/ ``score``。
    """
    square_mm = E.SQUARE_MM_DEFAULT if square_mm is None else square_mm
    block_h_mm = E.BLOCK_H_MM_DEFAULT if block_h_mm is None else block_h_mm
    board = E.make_board(square_mm)
    out = []
    for d in frame_dirs:
        f = E.load_frame(d)
        pose = E.board_pose(f["img"], f["K"], f["D"], board)
        if pose is None:
            print(f"[skip] {d}: 板没解出来", file=sys.stderr)
            continue
        f["_board_pose"] = (np.asarray(pose["R"]), np.asarray(pose["t"]).reshape(3))
        P, valid = E.point_cloud(f)
        f["_cloud"] = (P, valid)
        gp = pc_grasp.global_table_plane(P, valid, table_hint=0.0)
        name = os.path.basename(os.path.normpath(d))
        for det in f["detections"]["detections"]:
            score = float((det.get("best") or {}).get("score", 0.0))
            if score < pc_min_score:
                continue
            box = (int(round(det["cx"] - det["size_x"] / 2)),
                   int(round(det["cx"] + det["size_x"] / 2)),
                   int(round(det["cy"] - det["size_y"] / 2)),
                   int(round(det["cy"] + det["size_y"] / 2)))
            plane = pc_grasp.local_table_plane(P, valid, box, global_plane=gp)
            e = E.estimate_grasp_pc(f, det, plane)
            if e is None:
                continue
            p = np.asarray(e["p_base"], float)
            n, c = plane
            plane_z = (c - n[0] * p[0] - n[1] * p[1]) / n[2]
            out.append({"frame": name, "bx": float(p[0]), "by": float(p[1]),
                        "bz": float(p[2]), "above_m": float(e["z_above_mm"]) / 1e3,
                        "plane_z": float(plane_z), "score": score})
    return out


def group_blocks(entries, tol_mm=GROUP_TOL_MM):
    """按 base 系 (x, y) 把跨帧的估计聚成块，返回 ``[(中心xy, [记录...]), ...]``。"""
    groups = []
    for r in entries:
        xy = np.array([r["bx"], r["by"]]) * 1e3
        hit = None
        for g in groups:
            if np.linalg.norm(xy - g["xy"]) < tol_mm:
                hit = g
                break
        if hit is None:
            groups.append({"xy": xy, "rows": [r]})
        else:
            hit["rows"].append(r)
            hit["xy"] = np.mean([[q["bx"] * 1e3, q["by"] * 1e3]
                                 for q in hit["rows"]], axis=0)
    return [(g["xy"], g["rows"]) for g in groups]


def split_half_diff(values, n):
    """一组帧的「前 n 个中位 − 后 n 个中位」。``None`` 表示帧不够。

    返回的是**差**而不是 σ：两个中位各自带 σ_N 的抖动，故差的方差 = 2σ_N²。
    聚合必须在**块之间**做（见 ``summarize`` 里的 RMS），单块的
    ``|差|/√2`` 不是无偏估计 —— 它服从半正态，中位只有 0.67σ，会系统性
    报小（正是本项目反复踩的「绿灯为错理由而亮」）。
    """
    if len(values) < 2 * n:
        return None
    return float(np.median(values[:n]) - np.median(values[-n:]))


def summarize(entries, min_frames=MIN_FRAMES):
    """误差结构汇总。返回可直接 json.dumps 的 dict。"""
    blocks = []
    for xy, rows in group_blocks(entries):
        if len(rows) < min_frames:
            continue
        bz = np.array([r["bz"] for r in rows]) * 1e3            # mm
        above = np.array([r["above_m"] for r in rows]) * 1e3
        plane = np.array([r["plane_z"] for r in rows]) * 1e3
        blk = {"xy_mm": [round(float(v), 1) for v in xy],
               "n": len(rows),
               "z_mean_mm": round(float(bz.mean()), 2),
               "z_std_mm": round(float(bz.std(ddof=1)), 2),
               "z_range_mm": round(float(bz.max() - bz.min()), 2),
               "plane_z_mean_mm": round(float(plane.mean()), 2),
               "plane_z_std_mm": round(float(plane.std(ddof=1)), 2),
               "above_mean_mm": round(float(above.mean()), 2),
               "above_std_mm": round(float(above.std(ddof=1)), 2),
               "frames": [r["frame"] for r in rows]}
        order = sorted(range(len(rows)), key=lambda i: rows[i]["frame"])
        blk["_bz_ordered"] = [float(bz[i]) for i in order]
        blocks.append(blk)

    summary = {"blocks": len(blocks), "frames": sorted({r["frame"] for r in entries})}
    if not blocks:
        return summary
    z_std = [b["z_std_mm"] for b in blocks]
    summary["single_frame_sigma_mm"] = round(float(np.median(z_std)), 2)
    summary["sigma_parts_mm"] = {
        "local_plane": round(float(np.median([b["plane_z_std_mm"] for b in blocks])), 2),
        "above_plane": round(float(np.median([b["above_std_mm"] for b in blocks])), 2),
    }
    means = [b["z_mean_mm"] for b in blocks]
    summary["block_mean_spread_mm"] = round(float(max(means) - min(means)), 2)

    fusion = {}
    for n in FUSE_N:
        diffs = [d for d in (split_half_diff(b["_bz_ordered"], n) for b in blocks)
                 if d is not None]
        if len(diffs) >= 3:                      # 块太少时 RMS 本身不可信
            # 差平方的均值开方 = √2·σ_N ⇒ σ_N = RMS(差)/√2
            sigma = float(np.sqrt(np.mean(np.square(diffs)) / 2.0))
            fusion[str(n)] = {
                "sigma_mm": round(sigma, 2),
                # 两个理论参考：半半对照比的是**中位**，高斯下 σ_N = σ₁·√(MEDIAN_VAR[n])；
                # √N 那列是**均值**融合，留着只为看清「中位多付的那一份」。
                "median_theory_mm": round(float(
                    summary["single_frame_sigma_mm"]
                    * np.sqrt(MEDIAN_VAR.get(n, np.pi / (2.0 * n)))), 2),
                "sqrt_n_theory_mm": round(summary["single_frame_sigma_mm"]
                                          / np.sqrt(n), 2),
                "blocks": len(diffs)}
    if fusion:
        summary["temporal_fusion"] = fusion
    for b in blocks:
        b.pop("_bz_ordered", None)
    summary["per_block"] = blocks
    return summary


def _print(summary):
    print(f"帧: {len(summary['frames'])} 个，块: {summary['blocks']} 个")
    if not summary["blocks"]:
        print("没有可统计的块（每块至少 %d 帧）" % MIN_FRAMES)
        return
    print(f"单帧 σ（Z，逐块中位）: {summary['single_frame_sigma_mm']:.2f} mm")
    p = summary["sigma_parts_mm"]
    print(f"方差拆解: 局部桌面 σ={p['local_plane']:.2f} mm / "
          f"高出桌面 σ={p['above_plane']:.2f} mm")
    print(f"块均值极差: {summary['block_mean_spread_mm']:.2f} mm"
          f"（= 逐块真实差异 + 视角相关偏置，两者本脚本分不开）")
    for n, d in (summary.get("temporal_fusion") or {}).items():
        print(f"时间融合 N={n}: σ≈{d['sigma_mm']:.2f} mm"
              f"（中位理论 {d['median_theory_mm']:.2f} / 均值理论 "
              f"{d['sqrt_n_theory_mm']:.2f}，{d['blocks']} 块）")
    print()
    print(f"{'block xy (mm)':>18} {'n':>3} {'z mean±σ':>16} {'局部桌面σ':>10} "
          f"{'高出桌面σ':>10}")
    for b in summary["per_block"]:
        lbl = "(%d, %d)" % (round(b["xy_mm"][0]), round(b["xy_mm"][1]))
        print(f"{lbl:>18} {b['n']:3d} {b['z_mean_mm']:7.2f}±{b['z_std_mm']:4.2f} "
              f"{b['plane_z_std_mm']:10.2f} {b['above_std_mm']:10.2f}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frame", action="append", required=True,
                    help="帧目录（capture_baseline.py 生成的）；可重复")
    ap.add_argument("--out", help="把汇总写成 JSON")
    ap.add_argument("--pc-min-score", type=float, default=pc_grasp.MIN_SCORE)
    ap.add_argument("--block-h-mm", type=float, default=E.BLOCK_H_MM_DEFAULT)
    ap.add_argument("--square-mm", type=float, default=E.SQUARE_MM_DEFAULT)
    args = ap.parse_args(argv)

    entries = collect(args.frame, pc_min_score=args.pc_min_score,
                      square_mm=args.square_mm, block_h_mm=args.block_h_mm)
    summary = summarize(entries)
    _print(summary)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as fp:
            json.dump(summary, fp, ensure_ascii=False, indent=1)
        print(f"\n→ {args.out}")


if __name__ == "__main__":
    main()

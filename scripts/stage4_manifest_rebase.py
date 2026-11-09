#!/usr/bin/env python3
"""把 manifest 里**没被搬动**的块，期望位置换成俯视跑前实测（判分基线同源）。

为什么要这一步
──────────────
``compare_layout`` 的「不撞」判据是**位移 ≤5 mm**：拿跑前期望位置与跑后真值比。
E9 真机半段的规划感知走**腕部链**（用户 2026-11-09 拍板），跑后判分走**俯视**
（同一拍板）。两套量具对同一块静止积木的系统偏差实测 **12–40 mm**（俯视外参
自 11-08 起陈旧，误差随位置变）⇒ 静止块会被判成「被碰了」，**假红**。

位移是两个数之差。只要两个数出自**同一台相机、同一套外参**，系统偏差就整体
抵消，5 mm 公差才在量噪声。所以：把未被搬动块的期望位置换成俯视**跑前**实测。

没有被放宽的东西
────────────────
· 公差一个数都不动（仍 5 mm / 40 mm）；
· **被搬动的块不重写** —— 它们的期望位置是计划落格（垫面几何，不是测量值），
  与跑后真值的比较照旧；
· yaw、z、size、颜色、id 全部保留腕部值（俯视的 yaw 不可用，见台账第二百八十四轮）；
· 基线对不上（配对距离 > ``max_mm``、或层数结构不一致使 |Δz| > ``max_z_mm``）
  ⇒ **报错退出**，不静默保留腕部值。
"""
import argparse
import json
import math
import sys

#: 期望位置与基线位置的配对距离上限（毫米）。块间距 ≥100 mm，塔的两层靠 z 分开
#: （29.5 mm）⇒ 60 mm 既能容下 12–40 mm 的系统偏差，又不会错配。
MAX_MM = 60.0
#: 同一块在两路真值里的 z 差上限（毫米）。层数结构不一致时它会先炸。
MAX_Z_MM = 10.0


def rebase_final_layout(manifest, baseline_blocks, max_mm=MAX_MM,
                        max_z_mm=MAX_Z_MM):
    """返回 ``(新 manifest, 报告)``。**不修改入参**。

    ``baseline_blocks`` 是俯视**跑前**真值的 ``blocks``（或整个真值 dict）。
    配对按**三维最近、一对一**贪心 —— 与 ``stage4_clear_run._match_blocks``
    同一套口径：两份真值的 id 各按位置重排，按 id 对齐会把塔的上下层错配。
    """
    if isinstance(baseline_blocks, dict):
        baseline_blocks = baseline_blocks.get("blocks") or []
    post = {str(i): b for i, b in enumerate(baseline_blocks)}
    moved = {str(a.get("block")) for a in manifest.get("actions") or []}

    layout = [dict(e) for e in manifest.get("final_layout") or []]
    todo = [e for e in layout if str(e.get("id")) not in moved]

    pairs = []
    for e in todo:
        for pi, p in post.items():
            d = math.sqrt((float(p["x"]) - float(e["x"])) ** 2
                          + (float(p["y"]) - float(e["y"])) ** 2
                          + (float(p["z"]) - float(e["z"])) ** 2) * 1000.0
            pairs.append((d, str(e.get("id")), pi))
    pairs.sort()

    used_e, used_p, pick = set(), set(), {}
    for d, ei, pi in pairs:
        if ei in used_e or pi in used_p:
            continue
        used_e.add(ei)
        used_p.add(pi)
        pick[ei] = (pi, d)

    rows = []
    for e in todo:
        ei = str(e.get("id"))
        if ei not in pick:
            raise ValueError("块 #%s 在基线与计划的配对里落单（基线 %d 块、"
                             "待判 %d 块）—— 数量对不上，不重写"
                             % (ei, len(post), len(todo)))
        pi, d = pick[ei]
        b = post[pi]
        if d > max_mm:
            raise ValueError("块 #%s 与基线最近块相距 %.1f mm > %.0f —— "
                             "两份真值不是同一个场景/场景已变，不重写"
                             % (ei, d, max_mm))
        dz = (float(b["z"]) - float(e["z"])) * 1000.0
        if abs(dz) > max_z_mm:
            raise ValueError("块 #%s 的 z 差 %.1f mm > %.0f —— 层数结构不一致"
                             "（俯视把塔读成单块？），重写会搬错层，不重写"
                             % (ei, dz, max_z_mm))
        rows.append({"id": ei, "baseline_index": int(pi),
                     "from_mm": [round(float(e["x"]) * 1000, 2),
                                 round(float(e["y"]) * 1000, 2)],
                     "to_mm": [round(float(b["x"]) * 1000, 2),
                               round(float(b["y"]) * 1000, 2)],
                     "d_mm": round(d, 2), "dz_mm": round(dz, 2)})
        e["x"] = float(b["x"])
        e["y"] = float(b["y"])

    rep = {"rebased": len(rows), "moved_kept": sorted(moved),
           "max_d_mm": max([r["d_mm"] for r in rows] or [0.0]),
           "max_dz_mm": max([abs(r["dz_mm"]) for r in rows] or [0.0]),
           "rows": rows}
    return dict(manifest, final_layout=layout), rep


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--manifest", required=True)
    ap.add_argument("--baseline", required=True,
                    help="俯视**跑前**真值 JSON（判分相机、与跑后同一套外参）")
    ap.add_argument("--out", default=None,
                    help="写到哪（缺省 = 原地覆盖 --manifest）")
    ap.add_argument("--max-mm", type=float, default=MAX_MM)
    ap.add_argument("--max-z-mm", type=float, default=MAX_Z_MM)
    ap.add_argument("--check", action="store_true",
                    help="只打印将改什么，不写文件")
    a = ap.parse_args(argv)

    with open(a.manifest, encoding="utf-8") as f:
        man = json.load(f)
    with open(a.baseline, encoding="utf-8") as f:
        base = json.load(f)
    try:
        out, rep = rebase_final_layout(man, base, a.max_mm, a.max_z_mm)
    except ValueError as e:
        print("!! %s" % e)
        return 1

    print("重写 %d 块（被搬动的 %d 块保留计划落格）：最大配对距 %.1f mm，"
          "最大 |Δz| %.1f mm"
          % (rep["rebased"], len(rep["moved_kept"]), rep["max_d_mm"],
             rep["max_dz_mm"]))
    for r in rep["rows"]:
        print("  #%-3s 基线#%-3d  (%8.2f,%8.2f) → (%8.2f,%8.2f) mm  "
              "d=%.1f dz=%+.1f"
              % (r["id"], r["baseline_index"], r["from_mm"][0],
                 r["from_mm"][1], r["to_mm"][0], r["to_mm"][1], r["d_mm"],
                 r["dz_mm"]))
    if a.check:
        print("（--check：没有写文件）")
        return 0
    out["final_layout_rebase"] = dict(
        rep, baseline=a.baseline, note="未被搬动块的期望位置 = 俯视跑前实测；"
                                       "判分相机与基线同源，系统偏差抵消")
    p = a.out or a.manifest
    with open(p, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=1)
    print("→ %s" % p)
    return 0


if __name__ == "__main__":
    sys.exit(main())

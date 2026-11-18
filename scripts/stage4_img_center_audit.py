#!/usr/bin/env python3
"""阶段 4 块心差场复核（2026-11-18，用户拍板第 4 项的一部分）。

输入：带 ``meas.img_xy_mm`` 的真值 JSON（``stage4_truth_from_frame --consensus``，
2026-11-18 起 SAM 链会记录）。输出：逐块「图像块心 − 深度方框中心」的差，
以及两种**全局模型**的拟合残差：

  A. 纯平移：``d = t``
  B. 平移 + 绕竖直轴的小旋转：``d = t + ε·ẑ × (p − c0)``
     （相机/手眼外参若有小角度误差 ε，块心差会随位置切向增长；位置见
     ``p``，单位毫米，``ε`` 用弧度、报告里换算成度）

判读：B 明显优于 A（残差 RMS 至少减半）⇒ 差场是**全局标定项**（方向随
位置转）；两模型残差都大、逐块又稳定 ⇒ 是**逐块测量偏置**（S2:5 实测
深度顶面点集单侧缺边，属这一类）。真正的多视点分离要等机械臂可动时
按队列条目拍「survey + approach」两组（不动臂做不出第二视点）。

本工具纯离线、只读 JSON；``test_stage4_img_center_audit.py`` 用合成差场
钉住两个模型的解。
"""
import argparse
import json
import math
import sys

import numpy as np


def fit_models(points_mm, deltas_mm):
    """``(N,2)`` 块位与差（毫米）→ 两个全局模型的解与残差。

    返回::

        {"translation": {t_mm, rms_mm, max_mm, n},
         "rotation":    {t_mm, eps_rad, eps_deg, rms_mm, max_mm, n},
         "better": "translation" | "rotation" | "tie"}
    """
    P = np.asarray(points_mm, float)
    D = np.asarray(deltas_mm, float)
    if len(P) != len(D) or len(P) < 1:
        raise ValueError("points/deltas 长度不一致或为空")
    n = len(P)
    c0 = P.mean(axis=0)
    out = {"translation": {}, "rotation": {}, "n": n}

    t = D.mean(axis=0)
    res = D - t
    rms = float(np.sqrt(np.mean(np.sum(res ** 2, axis=1))))
    out["translation"] = {
        "t_mm": [round(float(t[0]), 3), round(float(t[1]), 3)],
        "rms_mm": round(rms, 3),
        "max_mm": round(float(np.max(np.hypot(res[:, 0], res[:, 1]))), 3),
    }

    # d = t + ε·ẑ × (p − c0) = t + ε·(−(y−c0y), (x−c0x))
    jx = -(P[:, 1] - c0[1])
    jy = (P[:, 0] - c0[0])
    A = np.zeros((2 * n, 3))
    b = D.reshape(-1)
    A[0::2, 0] = 1.0
    A[1::2, 1] = 1.0
    A[0::2, 2] = jx
    A[1::2, 2] = jy
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    tx, ty, eps = float(sol[0]), float(sol[1]), float(sol[2])
    pred = np.stack([tx + eps * jx, ty + eps * jy], axis=1)
    res2 = D - pred
    rms2 = float(np.sqrt(np.mean(np.sum(res2 ** 2, axis=1))))
    out["rotation"] = {
        "t_mm": [round(tx, 3), round(ty, 3)],
        "eps_rad": round(eps, 6),
        "eps_deg": round(math.degrees(eps), 4),
        "rms_mm": round(rms2, 3),
        "max_mm": round(float(np.max(np.hypot(res2[:, 0], res2[:, 1]))), 3),
        "lever_mm": round(float(np.max(np.hypot(P[:, 0] - c0[0],
                                                P[:, 1] - c0[1]))), 2),
    }
    a, r = out["translation"]["rms_mm"], out["rotation"]["rms_mm"]
    if abs(out["rotation"]["eps_deg"]) < 0.005:
        # ε 在数值噪声级 ⇒ 旋转模型退化成平移，别硬报旋转。
        out["better"] = "translation"
    elif r <= a * 0.5:
        out["better"] = "rotation"
    elif a <= r * 0.5:
        out["better"] = "translation"
    else:
        out["better"] = "tie"
    return out


def audit_truth(doc):
    """真值 JSON → 逐块行 + 模型拟合（只用带 ``img_xy_mm`` 的块）。"""
    rows = []
    for b in doc.get("blocks") or []:
        m = b.get("meas") or {}
        im = m.get("img_xy_mm")
        if not im or not m.get("img_ok", True):
            continue
        x, y = float(b["x"]) * 1000.0, float(b["y"]) * 1000.0
        dx, dy = float(im[0]) - x, float(im[1]) - y
        rows.append({"color": b.get("color"), "layer": b.get("layer", 0),
                     "xy_mm": [round(x, 2), round(y, 2)],
                     "img_mm": [float(im[0]), float(im[1])],
                     "delta_mm": [round(dx, 3), round(dy, 3)],
                     "dist_mm": round(math.hypot(dx, dy), 3),
                     "img_frames": m.get("img_frames")})
    out = {"blocks": rows}
    if len(rows) >= 3:
        out["fit"] = fit_models([r["xy_mm"] for r in rows],
                                [r["delta_mm"] for r in rows])
    else:
        out["fit"] = None
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--truth", required=True, help="真值 JSON（consensus 输出）")
    ap.add_argument("--out", default=None)
    args = ap.parse_args(argv)
    doc = json.load(open(args.truth, encoding="utf-8"))
    rep = audit_truth(doc)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as fh:
            json.dump(rep, fh, ensure_ascii=False, indent=1)
    print(json.dumps(rep, ensure_ascii=False, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())

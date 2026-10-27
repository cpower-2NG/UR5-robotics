#!/usr/bin/env python3
"""把「预测的抓取角」叠到相机画面上，和「量到的块面角」直接对比。

用途
────
计划器只用角度数字说话，看不出它对不对。这个脚本把两样东西画在同一张
彩色图上，人眼一眼判：

* **绿线**：块面角 θ_pix（``yaw_pix``：像素最小外接矩形，**计划器真正用的那一个**）；
* **洋红线**：抓取角 ψ（``round.json`` 里该块的 ``yaw_grasp_deg``，
  即 tool0 绕 z 的角）——「爪打算怎么夹」；
* 两条线都画成 **mod 90 的两条轴**（θ 与 θ+90、ψ 与 ψ+90），因为方块的
  方块面四个方向等价；比较的是**相对夹角**，不是绝对值。
* **橙线**（细）：``yaw_rect``（点云 3D 最小外接矩形）—— 同一块面的另一个估计；
  两条「量到的」线本该平行，差得大就说明这一块的几何估计本身不可信。
* 灰框：块顶面的投影轮廓（``ext_rect`` 有就用，否则用 ``ext``）。

判读：绿线与洋红线**平行** = 预测对齐；相差 δ 度就是这一块会转 δ 度。

用法
────
    python3 scripts/stage2_yaw_overlay.py \
        --frame  <run>/b2/frame-pick4 \
        --candidates <run>/b2/candidates.json \
        --round <run>/round-02            # 可选：带上才有洋红线
        --out <run>/b2/overlay-pick4.png \
        --block "#2"                      # 可选：只画某些块（"#idx,#idx"）

``--round`` 里没有该块（不是本轮抓的），就只画绿线与灰框。
"""

import argparse
import json
import os
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import graspnet_candidates as gc        # noqa: E402
import stage2_geom as sg                # noqa: E402

FONT_CANDIDATES = [
    "/mnt/c/Windows/Fonts/msyh.ttc",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
]


def _font(size):
    for p in FONT_CANDIDATES:
        if os.path.exists(p):
            try:
                return ImageFont.truetype(p, size)
            except OSError:
                continue
    return ImageFont.load_default()


def project(frame, pts_mm, z_mm=None):
    """base mm 点（N×2 或 N×3）→ 像素 (u, v)；相机后方的点返回 nan。"""
    P = np.asarray(pts_mm, dtype=float)
    if P.shape[1] == 2:
        z = 0.0 if z_mm is None else float(z_mm)
        P = np.concatenate([P, np.full((len(P), 1), z)], axis=1)
    P = P / 1000.0
    R, t, K = frame["R_bc"], frame["t_bc"], frame["K"]
    Q = (R.T @ (P - t).T).T
    z = Q[:, 2]
    ok = z > 1e-6
    uv = np.full((len(P), 2), np.nan)
    uv[ok, 0] = K[0, 0] * Q[ok, 0] / z[ok] + K[0, 2]
    uv[ok, 1] = K[1, 1] * Q[ok, 1] / z[ok] + K[1, 2]
    return uv


def _axis_ends(cx, cy, deg, half_mm):
    """以 (cx,cy) 为中心、方向 deg 的 base 系线段两端（mm）。"""
    a = np.deg2rad(float(deg))
    d = np.array([np.cos(a), np.sin(a)]) * half_mm
    return (cx - d[0], cy - d[1]), (cx + d[0], cy + d[1])


def _rect_corners(cx, cy, deg, ext_mm):
    a = np.deg2rad(float(deg))
    c, s = np.cos(a), np.sin(a)
    hx, hy = float(ext_mm[0]) / 2.0, float(ext_mm[1]) / 2.0
    pts = []
    for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        dx, dy = sx * hx, sy * hy
        pts.append((cx + c * dx - s * dy, cy + s * dx + c * dy))
    return pts


def _mod90_diff(a, b):
    """a 与 b 的 mod 90 有符号差，落在 (-45, 45]。"""
    d = (float(a) - float(b)) % 90.0
    return d - 90.0 if d > 45.0 else d


def _parse_blocks(s):
    if not s:
        return None
    out = set()
    for tok in str(s).split(","):
        tok = tok.strip().lstrip("#")
        if tok:
            out.add(int(tok))
    return out


def _round_items(round_dir):
    """round.json → {block_idx: item}。没有就返回空。"""
    if not round_dir:
        return {}
    p = os.path.join(round_dir, "round.json")
    if not os.path.exists(p):
        return {}
    doc = json.load(open(p))
    return {int(it["block_idx"]): it for it in doc.get("items", [])}


def draw(frame_dir, cand_json, round_dir=None, out_png=None, only=None,
         half_mm=34.0, title=""):
    frame = gc.load_frame(frame_dir)
    cand = json.load(open(cand_json))
    blocks = cand.get("blocks", [])
    items = _round_items(round_dir)

    img = Image.fromarray(frame["color"]).convert("RGB")
    dr = ImageDraw.Draw(img)
    f_big, f_small = _font(20), _font(15)

    rows = []
    for b in blocks:
        idx = int(b.get("idx", -1))
        if only is not None and idx not in only:
            continue
        if b.get("low"):
            continue
        h = float(b.get("h") or 0.0)
        if h < 15.0:
            continue
        cx, cy = float(b["cx"]), float(b["cy"])
        top = float(b.get("top") or 0.0)
        # θ_pix 现算：candidates.json 落盘时还没有 yaw_pix（那是 analyse_frame
        # 之后才补的），而计划器用的正是它 —— 现场算才能和计划对得上。
        try:
            _r = sg.block_rect_yaw(frame, b)
        except Exception:                                          # noqa: BLE001
            _r = None
        theta_pix = None if _r is None else _r["yaw_deg"]
        theta_pix_ok = bool(_r and _r.get("trusted"))
        theta = theta_pix if theta_pix is not None else b.get("yaw_rect")
        theta_src = "pix" if theta_pix is not None else "rect"
        if theta is None:
            theta, theta_src = b.get("yaw"), "pca"
        theta_alt = b.get("yaw_rect") if theta_src == "pix" else None
        ext = b.get("ext_rect") or b.get("ext")
        cls = b.get("class") or "?"

        # 灰框：块顶面投影
        if ext and theta is not None:
            corners = _rect_corners(cx, cy, theta, ext)
            uv = project(frame, corners, z_mm=top)
            if np.isfinite(uv).all():
                dr.line([tuple(p) for p in uv] + [tuple(uv[0])],
                        fill=(170, 170, 170), width=2)

        # 绿线：量到的块面角（mod 90 两条轴）
        if theta is not None:
            for k in (0.0, 90.0):
                a, bb = _axis_ends(cx, cy, float(theta) + k, half_mm)
                uv = project(frame, [a, bb], z_mm=top)
                if np.isfinite(uv).all():
                    dr.line([tuple(uv[0]), tuple(uv[1])],
                            fill=(0, 220, 0), width=3)

        # 橙线（细）：另一个几何估计 yaw_rect
        if theta_alt is not None:
            for k in (0.0, 90.0):
                a, bb = _axis_ends(cx, cy, float(theta_alt) + k, half_mm * 0.8)
                uv = project(frame, [a, bb], z_mm=top)
                if np.isfinite(uv).all():
                    dr.line([tuple(uv[0]), tuple(uv[1])],
                            fill=(255, 160, 0), width=2)

        # 洋红线：预测抓取角
        psi = items.get(idx, {}).get("yaw_grasp_deg")
        if psi is not None:
            for k in (0.0, 90.0):
                a, bb = _axis_ends(cx, cy, float(psi) + k, half_mm)
                uv = project(frame, [a, bb], z_mm=top)
                if np.isfinite(uv).all():
                    dr.line([tuple(uv[0]), tuple(uv[1])],
                            fill=(255, 0, 220), width=3)

        uv_c = project(frame, [(cx, cy)], z_mm=top)[0]
        if np.isfinite(uv_c).all():
            u, v = float(uv_c[0]), float(uv_c[1])
            r = 5
            dr.ellipse([u - r, v - r, u + r, v + r], outline=(255, 255, 0),
                       width=2)
            tag = "#%d %s θ%.1f(%s%s)" % (idx, cls, float(theta), theta_src,
                                          "" if theta_pix_ok else "!")
            if theta_alt is not None:
                tag += " r%+.1f" % _mod90_diff(theta_alt, theta)
            if psi is not None:
                d = _mod90_diff(psi, theta) if theta is not None else None
                tag += " ψ%.1f Δ%+.1f°" % (float(psi), d)
            dr.text((u + 8, v - 10), tag, font=f_small, fill=(255, 255, 0),
                    stroke_width=3, stroke_fill=(0, 0, 0))
            rows.append((idx, cls, theta, theta_src, theta_alt, psi,
                         _mod90_diff(psi, theta) if (psi is not None and
                                                     theta is not None) else None))

    legend = [
        "green  = measured block face theta (yaw_pix, what the planner used)",
        "orange = other estimate yaw_rect (3D rect; should parallel green)",
        "magenta= predicted grasp yaw psi (round.json yaw_grasp_deg)",
        "grey   = projected top-face rectangle",
    ]
    y = 8
    for line in legend:
        dr.text((10, y), line, font=f_small, fill=(255, 255, 255),
                stroke_width=3, stroke_fill=(0, 0, 0))
        y += 20
    if title:
        dr.text((10, y + 4), title, font=f_big, fill=(255, 255, 0),
                stroke_width=3, stroke_fill=(0, 0, 0))

    if out_png:
        os.makedirs(os.path.dirname(os.path.abspath(out_png)), exist_ok=True)
        img.save(out_png)
    return img, rows


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--frame", required=True)
    ap.add_argument("--candidates", required=True)
    ap.add_argument("--round", default=None,
                    help="含 round.json 的轮次目录（给了才画预测角）")
    ap.add_argument("--out", default=None)
    ap.add_argument("--block", default=None, help='只画这些块："#0,#2"')
    ap.add_argument("--half-mm", type=float, default=34.0,
                    help="轴线半长（mm，默认 34）")
    ap.add_argument("--title", default="")
    args = ap.parse_args()

    out = args.out or os.path.join(os.path.abspath(args.frame),
                                   "yaw-overlay.png")
    _img, rows = draw(args.frame, args.candidates, args.round, out,
                      only=_parse_blocks(args.block), half_mm=args.half_mm,
                      title=args.title)
    print("写出 %s" % out)
    for idx, cls, theta, src, alt, psi, d in rows:
        alt_s = "     —  " if alt is None else "θr%7.2f°" % alt
        if psi is None:
            print("  #%-2d %-12s θ %7.2f° (%s) %s   无预测角"
                  % (idx, cls, theta, src, alt_s))
        else:
            print("  #%-2d %-12s θ %7.2f° (%s) %s  ψ %7.2f°  Δ %+6.2f°"
                  % (idx, cls, theta, src, alt_s, psi, d))
    return 0


if __name__ == "__main__":
    sys.exit(main())

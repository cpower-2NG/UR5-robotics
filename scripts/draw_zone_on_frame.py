#!/usr/bin/env python3
"""**把 `base_link` 里的区画到真照片上** —— 给「照着划区」做人工复核用。

为什么需要它
============
`pick_zone` / `place_zone` 是 `base_link` 系里的 3D 盒子（`planner_node.in_zone`
三轴闭区间）。包线图（`plot_table_envelope.py`）画的是**示意图**，不是照片 ——
它能告诉你「哪一格安全」，但**告诉不了你那一格在台面上的哪个实物位置**。
现场要定放置区，看的就是这个：「这块地方有没有东西挡着、够不够放一排」。

所以这个工具把区的四角投影回像素，叠在**真照片**上。

方向怎么来的（这是最容易错的一步）
==================================
留档 `docs/calib/extrinsics-2026-10-02.json` 是 **`base_link ← camera_link`**，
而针孔投影要的是**光学系**（`camera_color_optical_frame`：z 朝前、x 朝右、y 朝下）。
两者差一个**固定**旋转（REP-103/105）：`opt_z=link_x`、`opt_x=−link_y`、`opt_y=−link_z`。

⚠️ 直接用 `camera_link` 的 R/t 去套针孔公式会画出一个**看着像样但整个歪掉**的框。
所以本脚本带一条**可检验的不变式**（每次运行都打印）：

    光轴（camera_link 的 +x，转到 base 系）与**台面法线**的夹角
    —— 文档记的是 **35.9°**（与台面 54.1°），实测这份留档是 **35.73°/54.27°**。

对不上就说明系搞错了，**不是**「差一点点」。退出码 2。

跑法
====
    python3 scripts/draw_zone_on_frame.py \\
        --image docs/calib/清场示意-2026-10-02.png \\
        --info .scratch/rounds/r00/info.json \\
        --extrinsics docs/calib/extrinsics-2026-10-02.json \\
        --zone pick:0.40,0.75,-0.45,0.45 \\
        --zone place:0.60,0.75,0.20,0.40 \\
        --axes 0.15 --out /tmp/zone_overlay.png

**只读**：读图片/内参/外参，写一张 png。不发轨迹、不碰相机、不碰机械臂。
"""

import argparse
import json
import math
import os
import sys

sys.dont_write_bytecode = True

import numpy as np

# camera_link → 光学系（REP-103/105）。列 = 光学系三个轴在 link 系里的表示：
#   opt_x(右) = −link_y, opt_y(下) = −link_z, opt_z(前) = link_x
R_LINK_TO_OPTICAL = np.array([[0.0, 0.0, 1.0],
                              [-1.0, 0.0, 0.0],
                              [0.0, -1.0, 0.0]])

OPTICAL_FRAMES = ("camera_color_optical_frame", "camera_optical_frame")


def quat_to_matrix(q):
    """``[x, y, z, w]`` → 3×3。与 `auto_label_grid.quat_to_matrix` 同一约定。"""
    x, y, z, w = [float(v) for v in q]
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        raise ValueError("四元数长度为 0")
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def compose_base_optical(doc):
    """外参文件 → ``(R, t)``，语义是 **``base_link ← 光学系``**。

    接受两种 `frame_child`：已经是光学系（原样用），或者是 `camera_link`
    （左乘那个固定旋转）。**别的帧名一律拒绝** —— 猜一个帧关系正是本仓库
    反复在修的那一类「话说了没查过的事」。
    """
    # ⚠️ 键名有两种：留档那份写 `frame_child`，标注那份写 `camera_frame`。
    #    两个都认；都没有就拒绝（猜帧关系是本仓库反复在修的那一类）。
    child = str(doc.get("frame_child") or doc.get("camera_frame") or "")
    R = (np.array(doc["R"], dtype=float) if "R" in doc
         else quat_to_matrix(doc["q"] if "q" in doc else doc["rotation"]))
    t = np.array([float(v) for v in (doc.get("t") or doc["translation"])], dtype=float)
    if R.shape != (3, 3):
        raise ValueError(f"旋转不是 3×3：{R.shape}")
    if child in OPTICAL_FRAMES:
        return R, t
    if child == "camera_link":
        return R @ R_LINK_TO_OPTICAL, t
    raise ValueError(f"不认识的 frame_child={child!r}："
                     f"只认 camera_link 与 {OPTICAL_FRAMES}（拒绝猜）")


def axis_angles(R):
    """光轴在 base 系里与**台面法线**、与**台面**的夹角（度）。

    光轴 = `camera_link` 的 +x 在 base 系里的方向 = `R_base←optical` 的第 3 列。
    文档记的实测值是「与法线 35.9°／与台面 54.1°」—— 这是本脚本的自检。
    """
    a = np.asarray(R, dtype=float)[:, 2]
    n = np.linalg.norm(a)
    if n < 1e-12:
        return None, None
    cos_v = min(1.0, abs(float(a[2])) / n)
    to_normal = math.degrees(math.acos(cos_v))
    return to_normal, 90.0 - to_normal


def project(R, t, p_base, K):
    """``base_link`` 的点 → 像素 ``(u, v)``；在相机背后时返回 ``None``。

    ``K`` = ``(fx, fy, cx, cy)``。**背后必须返回 None 而不是裁一个数出来** ——
    裁剪出来的坐标会横跨画面，看着像个巨大的目标（`auto_label_grid` 的
    `project_box` 有同一条注释）。
    """
    p_cam = np.asarray(R, dtype=float).T @ (np.asarray(p_base, dtype=float)
                                            - np.asarray(t, dtype=float))
    if p_cam[2] <= 1e-9:
        return None
    fx, fy, cx, cy = K
    return (fx * p_cam[0] / p_cam[2] + cx, fy * p_cam[1] / p_cam[2] + cy)


def parse_zone(spec):
    """``名字:xmin,xmax,ymin,ymax[,z]`` → ``(名字, [四角], z)``。"""
    if ":" not in spec:
        raise ValueError(f"区要写成 名字:xmin,xmax,ymin,ymax[,z]，收到 {spec!r}")
    name, nums = spec.split(":", 1)
    v = [float(x) for x in nums.split(",")]
    if len(v) not in (4, 5):
        raise ValueError(f"{name}: 要 4 或 5 个数（多了 z），收到 {len(v)} 个")
    xmin, xmax, ymin, ymax = v[:4]
    z = v[4] if len(v) == 5 else 0.0
    if xmin >= xmax or ymin >= ymax:
        raise ValueError(f"{name}: 上下界反了（xmin<xmax、ymin<ymax）")
    corners = [(xmin, ymin, z), (xmax, ymin, z), (xmax, ymax, z), (xmin, ymax, z)]
    return name.strip(), corners, z


def main():
    ap = argparse.ArgumentParser(
        description="把 base_link 里的区投影到真照片上（只读，画一张 png）")
    ap.add_argument("--image", required=True)
    ap.add_argument("--info", required=True, help="含 fx,fy,cx,cy（或 K）的 json")
    ap.add_argument("--extrinsics", required=True, help="留档那类外参 json")
    ap.add_argument("--zone", action="append", default=[],
                    help="名字:xmin,xmax,ymin,ymax[,z]；可给多次")
    ap.add_argument("--axes", type=float, default=0.0,
                    help=">0 时画出 base 原点与 x/y 轴的这段长度（m）")
    ap.add_argument("--out", required=True)
    ap.add_argument("--expect-normal-deg", type=float, default=35.9,
                    help="光轴与台面法线的期望夹角（文档实测 35.9）")
    ap.add_argument("--tol-deg", type=float, default=3.0,
                    help="上面那条的容差（度）。超了判系搞错了")
    args = ap.parse_args()

    from PIL import Image, ImageDraw

    doc = json.load(open(args.extrinsics, encoding="utf-8"))
    info = json.load(open(args.info, encoding="utf-8"))
    K = (float(info["fx"]), float(info["fy"]), float(info["cx"]), float(info["cy"]))
    W, H = int(info["width"]), int(info["height"])

    R, t = compose_base_optical(doc)
    to_normal, to_plane = axis_angles(R)

    print("=" * 74)
    print(f"外参   {args.extrinsics}   （{doc.get('frame_parent')} ← {doc.get('frame_child')}）")
    print(f"内参   fx={K[0]:.1f} fy={K[1]:.1f} cx={K[2]:.0f} cy={K[3]:.0f}  {W}×{H}")
    print(f"相机光心在 base = {np.round(t, 4).tolist()} m")
    print(f"★ 光轴 vs **台面法线** {to_normal:.2f}°  /  vs **台面** {to_plane:.2f}°"
          f"   （期望 {args.expect_normal_deg:.1f}°，容差 {args.tol_deg:.1f}°）")
    if abs(to_normal - args.expect_normal_deg) > args.tol_deg:
        print("!! 与文档对不上 ⇒ **帧关系搞错了**，画出来的框会整个歪掉。拒绝出图。")
        return 2
    print("   ⇒ 帧关系验过（这条不变式不过，后面画什么都不算数）")

    im = Image.open(args.image).convert("RGB")
    if im.size != (W, H):
        print(f"!! 图片是 {im.size}，内参是 {W}×{H} —— 不是同一台/同一档分辨率")
        return 2
    dr = ImageDraw.Draw(im)

    def px(p_base):
        return project(R, t, p_base, K)

    # ---- 先画 base 原点与坐标轴：**这一笔是给眼睛复核投影对不对用的** ----
    if args.axes > 0:
        o = px((0.0, 0.0, 0.0))
        if o:
            for vec, col, lab in (((args.axes, 0, 0), (255, 60, 60), "+x"),
                                  ((0, args.axes, 0), (60, 200, 60), "+y")):
                e = px(vec)
                if e:
                    dr.line([o, e], fill=col, width=5)
                    dr.text((e[0] + 4, e[1] + 4), lab, fill=col)
            dr.ellipse([o[0] - 7, o[1] - 7, o[0] + 7, o[1] + 7],
                       outline=(255, 255, 255), width=4)
            dr.text((o[0] + 10, o[1] + 10), "base_link origin", fill=(255, 255, 255))
            print(f"  base 原点投到像素 {tuple(round(v) for v in o)}")

    # ---- 区 ----
    palette = [(255, 215, 0), (0, 200, 255), (255, 0, 255), (0, 255, 128)]
    for i, spec in enumerate(args.zone):
        try:
            name, corners, z = parse_zone(spec)
        except ValueError as e:
            print(f"!! {e}")
            return 2
        pts = [px(c) for c in corners]
        if any(p is None for p in pts):
            print(f"!! {name}: 有角点在**相机背后**（或正好在光心平面上）—— 拒绝画")
            return 2
        col = palette[i % len(palette)]
        dr.line(list(pts) + [pts[0]], fill=col, width=6, joint="curve")
        # 标签band放在第一个角上，避免压住框线
        lx, ly = pts[0]
        dr.rectangle([lx, ly - 26, lx + 12 + 9 * len(name), ly], fill=col)
        dr.text((lx + 6, ly - 20), name, fill=(0, 0, 0))
        zs = f"  z={z:+.4f}" if z else "  z=0（台面）"
        print(f"  {name}: 四角像素 {[tuple(round(v) for v in p) for p in pts]}{zs}")

    im.save(args.out)
    print(f"\n图已写出：{args.out}")
    print("⚠️ 框画在**台面平面 z=0** 上；方块有厚度，实物顶面会比这个框略窄一点。")
    return 0


if __name__ == "__main__":
    sys.exit(main())

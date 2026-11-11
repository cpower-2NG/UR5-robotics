#!/usr/bin/env python3
"""阶段 4 摆位引导：把布局目标**画到腕部相机的实拍照片上**。

为什么（2026-11-11 用户定）
==========================
A3 1:1 摆位图要 A3 纸 + 1:1 打印；现场只有 A4（1:1 放不下 325×250 mm 的拾取区）。
改成：腕部相机在拾取区中心上方 500 mm 垂直向下拍一张**空台面**，把目标块的
足迹 / 朝向 / 颜色叠在照片上，人看着屏幕摆块。照片是近正射（500 mm 高、
1280×720，约 1.82 px/mm），只用来「大致位置」——手摆目标本来就是 ±3–5 mm /
±10°，不需要 1:1。

照片怎么来（两个模式）
======================
1. ``--plan``（只出计划，不动臂）：从板上活关节出一份 survey 机位计划 +
   一份**回原位**计划 + ``setup-view.sh``。执行时臂到位 → 就地拍一组腕部 burst
   → 回原位（人再摆块），全程要当次运动令牌（AGENTS 红线 6）。
2. 默认 annotate：读腕部 burst 里的一帧 + 该帧的 K / R / t（手眼链），把布局
   目标投影上去，写 PNG + 同名 JSON。**离线、不动臂。**

用法
====
    # A) 出拍照/回原位计划（开发机；不动臂，执行时另走 STOP→PLAY 授权）
    python3 scripts/stage4_placement_view.py --plan \\
        --urdf <robot.urdf> --from-frame <joints 目录> \\
        --out-dir <run>/setup-view --board-dir ~/precision/plans/e9/setup-view \\
        --board-log-dir ~/precision/logs/e9/setup-view

    # B) 把布局画到照片上（离线）
    python3 scripts/stage4_placement_view.py --batch S0 --layout 1 \\
        --burst <run>/setup-view/wrist-pick --out <run>/setup-view/s0-1.png

坐标口径：布局表是 base 毫米（`stage4_layouts`），足迹画在台面 z=0；投影复用
``stage4_clear_run._base_to_uv``（全仓唯一的 base→像素实现），块角复用
``stage4_layout_sheet.block_corners``（与摆位图同源）。本工具不写运动几何、
不 import ROS。

退出码：0 = 写出；1 = 写出了但**有目标投出画幅**（照片不能引导那一块）；
2 = 输入缺失 / 读不懂 / 计划预检不过（不写）。
"""
import argparse
import glob
import json
import math
import os
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np                                                # noqa: E402
from PIL import Image, ImageDraw, ImageFont                       # noqa: E402

import graspnet_candidates as gc                                  # noqa: E402
import precision_plan as pp                                       # noqa: E402
import stage4_clear_run as cr                                     # noqa: E402
import stage4_layout_sheet as ls                                  # noqa: E402
import stage4_layouts as lay                                      # noqa: E402

#: 画图用的 RGB（只是显示色，不是测量口径；测量色相在 `block_classes`）。
COLOR_RGB = {"red": (220, 60, 60), "orange": (240, 145, 40),
             "tan": (200, 160, 110), "yellow": (235, 215, 50),
             "yellowgreen": (160, 205, 70), "cyan": (70, 200, 210),
             "purple": (150, 90, 200)}
DEFAULT_RGB = (200, 200, 200)

#: 英文名，给没有中文字体的机器兜底。
COLOR_EN = {"red": "red", "orange": "orange", "tan": "tan",
            "yellow": "yellow", "yellowgreen": "ygreen",
            "cyan": "cyan", "purple": "purple"}

FONT_CANDIDATES = (os.environ.get("STAGE4_SHEET_FONT") or "",
                   ls.WINDOWS_CJK_FONT,
                   "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")


# ------------------------------------------------------------------ 字体
def pick_pil_font(size_px):
    """CJK 字体 → ``(font, has_cjk)``；一份都找不到就退 Pillow 默认（无中文）。"""
    for path in FONT_CANDIDATES:
        if path and os.path.exists(path):
            try:
                f = ImageFont.truetype(path, int(size_px))
                # msyh / DejaVu 都能加载中文码位；用「字形是否缺」判太绕，
                # 这里按文件类型判：DejaVu 明确没有 CJK。
                return f, "DejaVu" not in os.path.basename(path)
            except OSError:
                continue
    return ImageFont.load_default(), False


# ------------------------------------------------------------------ 几何
def project_mm(xy_mm, z_mm, K, R, t):
    """base 毫米 + 台面高度 → 像素（相机后方/投不到返回 None）。"""
    return cr._base_to_uv((float(xy_mm[0]) / 1000.0, float(xy_mm[1]) / 1000.0,
                           float(z_mm) / 1000.0), K, R, t)


def px_per_mm(rec, size_mm=None):
    """块足迹在像面上的比例（相邻两角距离 / 边长）。"""
    size_mm = float(size_mm if size_mm is not None else
                    lay.BLOCK_SIZE_M * 1000.0)
    a, b = rec["corners_uv"][0], rec["corners_uv"][1]
    if a is None or b is None:
        return None
    return round(math.hypot(a[0] - b[0], a[1] - b[1]) / size_mm, 4)


def block_targets(blocks, K, R, t, z_mm=0.0, size_mm=None, width=None,
                  height=None):
    """布局块 → 投影像素记录（角点 / 中心 / 对角线端点 / 是否出画幅）。"""
    size_mm = float(size_mm if size_mm is not None else
                    lay.BLOCK_SIZE_M * 1000.0)
    out = []
    for b in blocks:
        x_mm = float(b["x"]) * 1000.0
        y_mm = float(b["y"]) * 1000.0
        yaw = float(b.get("yaw", 0.0))
        corners = ls.block_corners(x_mm, y_mm, yaw, size_mm)
        uvs = [project_mm(p, z_mm, K, R, t) for p in corners]
        ctr = project_mm((x_mm, y_mm), z_mm, K, R, t)
        clipped = ctr is None or any(
            u is None or (width is not None and
                          not (-1.0 <= u[0] <= width + 1.0)) or
            (height is not None and not (-1.0 <= u[1] <= height + 1.0))
            for u in uvs)
        rec = {
            "id": int(b.get("id", len(out))),
            "color": str(b.get("color", "unknown")),
            "x_mm": round(x_mm, 2), "y_mm": round(y_mm, 2),
            "yaw_deg": round(yaw, 3),
            "corners_mm": [[round(p[0], 2), round(p[1], 2)] for p in corners],
            "corners_uv": [None if u is None else [round(u[0], 2),
                                                   round(u[1], 2)]
                           for u in uvs],
            "center_uv": (None if ctr is None
                          else [round(ctr[0], 2), round(ctr[1], 2)]),
            "diag_uv": [None if u is None else [round(u[0], 2), round(u[1], 2)]
                        for u in (uvs[0], uvs[2])],
            "clipped": bool(clipped),
        }
        rec["px_per_mm"] = px_per_mm(rec, size_mm)
        out.append(rec)
    return out


def zone_lines(zone_mm, step_mm=100.0):
    """拾取区内的网格线（base 毫米），返回 ``[(p0, p1, kind)]``。"""
    x0, x1, y0, y1 = [float(v) for v in zone_mm]
    lines = [((x0, y0), (x1, y0), "border"),
             ((x1, y0), (x1, y1), "border"),
             ((x1, y1), (x0, y1), "border"),
             ((x0, y1), (x0, y0), "border")]
    step = float(step_mm)
    if step <= 0:
        return lines
    x = math.ceil(x0 / step) * step
    while x < x1 - 1e-6:
        lines.append(((x, y0), (x, y1), "grid"))
        x += step
    y = math.ceil(y0 / step) * step
    while y < y1 - 1e-6:
        lines.append(((x0, y), (x1, y), "grid"))
        y += step
    return lines


def build_doc(blocks, zone_mm, K, R, t, burst=None, frame=None,
              batch=None, layout_id=None, width=None, height=None,
              z_mm=0.0):
    """标注文档（纯函数；画图与测试都吃它）。"""
    recs = block_targets(blocks, K, R, t, z_mm=z_mm, width=width,
                         height=height)
    grid = []
    for p0, p1, kind in zone_lines(zone_mm):
        a = project_mm(p0, z_mm, K, R, t)
        b = project_mm(p1, z_mm, K, R, t)
        grid.append({"kind": kind,
                     "p0_mm": [round(p0[0], 2), round(p0[1], 2)],
                     "p1_mm": [round(p1[0], 2), round(p1[1], 2)],
                     "p0_uv": None if a is None else [round(a[0], 2),
                                                       round(a[1], 2)],
                     "p1_uv": None if b is None else [round(b[0], 2),
                                                       round(b[1], 2)]})
    return {
        "kind": "stage4-placement-view",
        "batch": batch,
        "layout": layout_id,
        "burst": burst,
        "frame": frame,
        "image_wh": [width, height],
        "table_z_mm": z_mm,
        "zone_mm": [float(v) for v in zone_mm],
        "camera": {"K": [[float(v) for v in row] if hasattr(row, "__len__")
                         else float(row) for row in np.asarray(K, float)],
                   "R_bc": [[float(v) for v in row]
                            for row in np.asarray(R, float)],
                   "t_bc_m": [float(v) for v in np.asarray(t, float)]},
        "grid": grid,
        "blocks": recs,
        "clipped": [b["id"] for b in recs if b["clipped"]],
        "verdict": "看照片摆块：彩色框内 = 目标足迹（±3–5 mm / ±10°）；"
                   "拍完把纸/胶带撤掉，别压在块下",
    }


# ------------------------------------------------------------------ 画图
def _text_box(draw, xy, text, font, fill=(255, 255, 255, 255),
              bg=(0, 0, 0, 190), pad=4):
    l, t, r, b = draw.textbbox((0, 0), text, font=font)
    w, h = r - l, b - t
    x, y = xy
    draw.rectangle([x - pad, y - pad, x + w + pad, y + h + pad * 2], fill=bg)
    draw.text((x - l, y - t), text, font=font, fill=fill)
    return (x - pad, y - pad, x + w + pad, y + h + 2 * pad)


def _rects_hit(a, b):
    return not (a[2] <= b[0] or b[2] <= a[0] or a[3] <= b[1] or b[3] <= a[1])


def _uv_bbox(rec):
    pts = [u for u in rec["corners_uv"] if u is not None]
    if not pts:
        return None
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return (min(xs), min(ys), max(xs), max(ys))


def draw_doc(img, doc, cjk=True, title=None):
    """把标注文档画到 RGB 图上，返回 RGBA 合成图。"""
    base = img.convert("RGB").convert("RGBA")
    W, H = base.size
    overlay = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    d = ImageDraw.Draw(overlay)
    f_small, _ = pick_pil_font(18)
    f_info, _ = pick_pil_font(20)
    f_title, _ = pick_pil_font(26)

    for g in doc["grid"]:
        a, b = g["p0_uv"], g["p1_uv"]
        if a is None or b is None:
            continue
        if g["kind"] == "border":
            d.line([tuple(a), tuple(b)], fill=(0, 230, 230, 210), width=3)
        else:
            d.line([tuple(a), tuple(b)], fill=(255, 255, 255, 70), width=1)

    # ---- 图例先算好尺寸，挑一个**不压目标**的角（左上优先）。
    lines = [title or "%s 摆位引导" % (doc.get("layout") or "")]
    lines.append("网格 100 mm · 彩色框 = 目标足迹 · 白线 = 朝向")
    for rec in doc["blocks"]:
        if cjk:
            name = ls.COLOR_ZH.get(rec["color"], rec["color"])
        else:
            name = COLOR_EN.get(rec["color"], rec["color"])
        lines.append("#%d %s  (%.0f, %.0f)  %.0f°"
                     % (rec["id"], name, rec["x_mm"], rec["y_mm"],
                        rec["yaw_deg"]))
    box_w = 0
    for i, line in enumerate(lines):
        f = f_title if i == 0 else f_info
        l, t, r, b = d.textbbox((0, 0), line, font=f)
        box_w = max(box_w, r - l)
    box_w += 24
    box_h = 34 * len(lines) + 18
    boxes = [_uv_bbox(rec) for rec in doc["blocks"]]
    best = None
    for x, y in ((8, 8), (W - box_w - 8, 8),
                 (8, H - box_h - 8), (W - box_w - 8, H - box_h - 8)):
        rect = (x, y, x + box_w, y + box_h)
        score = sum(1 for b in boxes
                    if b is not None and _rects_hit(rect, b))
        if best is None or score < best[0]:
            best = (score, rect)
    legend = best[1]
    d.rectangle(list(legend), fill=(0, 0, 0, 175),
                outline=(255, 255, 255, 120), width=2)
    y = legend[1] + 6
    for i, line in enumerate(lines):
        d.text((legend[0] + 10, y), line,
               font=(f_title if i == 0 else f_info),
               fill=(255, 255, 255, 255))
        y += 34
    placed = [legend]

    for rec in doc["blocks"]:
        rgb = COLOR_RGB.get(rec["color"], DEFAULT_RGB)
        poly = [tuple(u) for u in rec["corners_uv"] if u is not None]
        if len(poly) >= 3:
            d.polygon(poly + [poly[0]], fill=rgb + (80,))
            d.line(poly + [poly[0]], fill=rgb + (255,), width=4)
        dg = rec["diag_uv"]
        if all(u is not None for u in dg):
            d.line([tuple(dg[0]), tuple(dg[1])], fill=(255, 255, 255, 230),
                   width=2)
        ctr = rec["center_uv"]
        if ctr is not None:
            cx, cy = ctr
            d.line([(cx - 7, cy), (cx + 7, cy)], fill=(255, 255, 255, 255),
                   width=2)
            d.line([(cx, cy - 7), (cx, cy + 7)], fill=(255, 255, 255, 255),
                   width=2)
            away = (1.0 if cx >= W / 2.0 else -1.0,
                    -1.0 if cy >= H / 2.0 else 1.0)
            lx = cx + away[0] * 58
            ly = cy + away[1] * 46
            lx = min(max(8, lx), W - 250)
            ly = min(max(8, ly), H - 40)
            if cjk:
                name = ls.COLOR_ZH.get(rec["color"], rec["color"])
                text = "#%d %s (%.0f, %.0f) mm" % (rec["id"], name,
                                                   rec["x_mm"], rec["y_mm"])
            else:
                text = "#%d %s (%.0f, %.0f)" % (
                    rec["id"], COLOR_EN.get(rec["color"], rec["color"]),
                    rec["x_mm"], rec["y_mm"])
            l, t, r, b = d.textbbox((0, 0), text, font=f_small)
            tw, th = r - l + 8, b - t + 8
            ax = (lx - tw if away[0] > 0 else lx)
            ay = ly if away[1] > 0 else ly - th
            rect = (ax - 4, ay - 4, ax + tw + 4, ay + th + 4)
            for _ in range(6):            # 压到已放的框就往下让
                if not any(_rects_hit(rect, p) for p in placed):
                    break
                ay += 26
                rect = (ax - 4, ay - 4, ax + tw + 4, ay + th + 4)
            ay = min(max(2.0, ay), H - th - 6.0)
            d.line([(cx, cy), (ax + tw / 2.0, ay + th / 2.0)],
                   fill=rgb + (220,), width=2)
            placed.append(_text_box(d, (ax, ay), text, f_small))

    # 比例尺：绘图处局部的 px/mm（竖直下视时全场近似常数）
    spm = next((b["px_per_mm"] for b in doc["blocks"]
                if not b["clipped"] and b["px_per_mm"]), None)
    if spm:
        L = 50.0 * spm
        x0, y0 = 24, H - 34
        d.line([(x0, y0), (x0 + L, y0)], fill=(255, 255, 255, 255), width=4)
        d.line([(x0, y0 - 7), (x0, y0 + 7)], fill=(255, 255, 255, 255), width=3)
        d.line([(x0 + L, y0 - 7), (x0 + L, y0 + 7)],
               fill=(255, 255, 255, 255), width=3)
        _text_box(d, (x0 + 6, y0 + 10), "50 mm（%.2f px/mm）" % spm, f_small,
                  bg=(0, 0, 0, 175))

    if doc.get("clipped"):
        _text_box(d, (W - 300, H - 40),
                  "!! 目标出画幅：%s" % doc["clipped"], f_small,
                  fill=(255, 120, 120, 255))
    out = Image.alpha_composite(base, overlay)
    return out.convert("RGB")


# ------------------------------------------------------------------ 读帧
def camera_from_burst(burst, frame=0):
    """腕部 burst → ``(K, R, t, color_path, width, height, idx)``。"""
    pat = os.path.join(burst, "color_*.png")
    paths = sorted(glob.glob(pat))
    if not paths:
        raise ValueError("burst 里没有 color_*.png：%s" % burst)
    if frame is None:
        frame = 0
    want = os.path.join(burst, "color_%03d.png" % int(frame))
    if want not in paths:
        raise ValueError("burst 里没有帧 %03d：%s" % (int(frame), burst))
    fr = gc.load_frame(burst, tip=gc.TIP, chain=gc.CHAIN, idx=int(frame))
    with Image.open(want) as im:
        W, H = im.size
    return (np.asarray(fr["K"], float), np.asarray(fr["R_bc"], float),
            np.asarray(fr["t_bc"], float), want, W, H, int(frame))


def camera_from_json(path):
    """``{"K": 3×3, "R_bc": 3×3, "t_bc_m": [x,y,z]}`` → 同 burst 的四元组。"""
    with open(path, encoding="utf-8") as f:
        doc = json.load(f)
    K = np.asarray(doc["K"], float).reshape(3, 3)
    R = np.asarray(doc["R_bc"], float).reshape(3, 3)
    t = np.asarray(doc["t_bc_m"], float).reshape(3)
    return K, R, t


# ------------------------------------------------------------------ 计划
def return_plan_doc(view_plan, q_start, purpose="setup-view 回原位（不拍照）"):
    """survey 机位计划 → **原路返回到 q_start** 的计划（纯函数，好测）。

    运动只有关节空间插值，所以把前向航点倒着走一遍就是合法回程；终点用工整
    的起始关节（``q_start``），不用前向计划的第一个航点近似。
    """
    wps = view_plan["waypoints"]
    if not wps:
        raise ValueError("前向计划没有航点")
    back = []
    for i in range(len(wps) - 2, -1, -1):
        back.append({"name": "return-%s" % wps[i]["name"],
                     "q": [float(v) for v in wps[i]["q"]],
                     "duration": float(wps[i]["duration"])})
    prev = wps[-1]["q"]
    back.append({"name": "return-start",
                 "q": [float(v) for v in q_start],
                 "duration": pp.duration_for(q_start, prev)})
    return {
        "version": 2,
        "meta": {"kind": "move", "purpose": purpose},
        "q_start": [float(v) for v in prev],
        "waypoints": back,
    }


def plan_main(args):
    """出 ``01-view-pick.json`` + ``02-return.json`` + ``setup-view.sh``。"""
    import urkin
    try:
        chain = urkin.Chain(args.urdf)
        if args.from_frame:
            js = json.load(open(os.path.join(args.from_frame,
                                             "joint_states.json"),
                                 encoding="utf-8"))
            q_start = [float(js["position"][js["name"].index(n)])
                       for n in chain.move_joints]
        elif args.q_start:
            q_start = [float(v) for v in args.q_start.split(",")]
        else:
            print("!! 要给 --from-frame 或 --q-start")
            return 2
        os.makedirs(args.out_dir, exist_ok=True)
        pick_view = cr.E9_VIEWS[0]
        plans, _q_end = cr.plan_views(chain, q_start, [pick_view],
                                      args.out_dir, start=0)
    except (ValueError, KeyError, OSError, pp.PlanError) as exc:
        print("!! 机位预检不过 / 输入缺失：%s" % exc)
        print("   计划**没有**写出 —— 不要执行。")
        return 2
    vp = json.load(open(plans[0]["path"], encoding="utf-8"))
    ret = return_plan_doc(vp, q_start)
    ret_path = os.path.join(args.out_dir, "02-return.json")
    with open(ret_path, "w", encoding="utf-8") as f:
        json.dump(ret, f, ensure_ascii=False, indent=1)

    board_burst = (args.board_burst_dir or
                   "$HOME/precision/e9/setup-view/wrist-pick")
    rec_view = dict(plans[0])
    rec_return = {"index": 2, "name": "02-return.json", "path": ret_path,
                  "kind": "move", "view": None, "block": None,
                  "n_waypoints": len(ret["waypoints"])}
    sh = cr.run_script_text([rec_view, rec_return], args.board_dir,
                            args.board_log_dir,
                            burst_dirs={pick_view["name"]: board_burst},
                            token_min_remaining=args.min_remaining)
    sh_path = os.path.join(args.out_dir, "setup-view.sh")
    with open(sh_path, "w", encoding="utf-8") as f:
        f.write(sh)
    os.chmod(sh_path, 0o755)
    print("survey 机位：相机 (%0.3f, %0.3f) 高 %0.3f m；%d 航点"
          % (plans[0]["slot"][0], plans[0]["slot"][1],
             pick_view["height"], plans[0]["n_waypoints"]))
    print("回原位：%d 航点（终点 = 计划起点）" % len(ret["waypoints"]))
    print("拍完的 burst 在板上：%s" % board_burst)
    print("→ %s" % sh_path)
    print("  执行要当次 STOP→PLAY 授权：")
    print("    python3 scripts/real_motion_authorize.py --step stop")
    print("    python3 scripts/real_motion_authorize.py --step play")
    print("    ssh <board> 'bash %s/setup-view.sh <token>'" % args.board_dir)
    return 0


# ------------------------------------------------------------------ 标注
def annotate_main(args):
    try:
        if args.burst:
            K, R, t, img_path, W, H, idx = camera_from_burst(
                args.burst, args.frame)
        elif args.image and args.camera_json:
            K, R, t = camera_from_json(args.camera_json)
            img_path, idx = args.image, None
            with Image.open(img_path) as im:
                W, H = im.size
        else:
            print("!! 要么给 --burst，要么同时给 --image 与 --camera-json")
            return 2
        if args.scene:
            with open(args.scene, encoding="utf-8") as f:
                # 场景真值（stage4_truth_from_frame）与 stage4_layouts 同口径：
                # x/y **米**、yaw **度**。别把它当弧度换算。
                blocks = json.load(f).get("blocks") or []
            batch = layout_id = None
        else:
            batch, layout_id = args.batch, int(args.layout)
            blocks = lay.layout_blocks(batch, layout_id)
        # E9_VIEWS 的 zone 是**米**；本工具内部一律毫米。
        zone_mm = ([float(v) for v in args.zone.split(",")]
                   if args.zone else
                   [float(v) * 1000.0 for v in cr.E9_VIEWS[0]["zone"]])
        doc = build_doc([dict(b) for b in blocks], zone_mm, K, R, t,
                        burst=args.burst or args.image, frame=idx,
                        batch=batch, layout_id=layout_id,
                        width=W, height=H, z_mm=float(args.table_z_mm))
    except (ValueError, KeyError, OSError) as exc:
        print("!! %s" % exc)
        return 2

    title = ("%s:%s 摆位引导" % (batch, layout_id)) if batch else "摆位引导"
    font, cjk = pick_pil_font(20)
    with Image.open(img_path) as im:
        out = draw_doc(im, doc, cjk=cjk, title=title)
    out_dir = os.path.dirname(os.path.abspath(args.out))
    if not os.path.isdir(out_dir):
        print("!! 输出目录不存在：%s" % out_dir)
        return 2
    out.save(args.out)
    jpath = args.json or os.path.splitext(args.out)[0] + ".json"
    with open(jpath, "w", encoding="utf-8") as f:
        json.dump(doc, f, ensure_ascii=False, indent=1)

    n_ok = sum(1 for b in doc["blocks"] if not b["clipped"])
    print("摆位引导：%d/%d 块在画幅内；%s"
          % (n_ok, len(doc["blocks"]),
             "比例 %0.2f px/mm" % doc["blocks"][0]["px_per_mm"]
             if doc["blocks"] and doc["blocks"][0]["px_per_mm"] else "无比例"))
    for b in doc["blocks"]:
        print("  #%d %-11s (%7.1f, %7.1f) mm  yaw %6.1f°%s"
              % (b["id"], b["color"], b["x_mm"], b["y_mm"], b["yaw_deg"],
                 "  !! 出画幅" if b["clipped"] else ""))
    print("→ %s" % args.out)
    print("→ %s" % jpath)
    return 1 if doc["clipped"] else 0


def build_parser():
    ap = argparse.ArgumentParser(
        description="阶段 4 摆位引导：布局目标画到腕部相机照片上 / 出拍照计划")
    ap.add_argument("--plan", action="store_true",
                    help="出 survey 机位 + 回原位计划与 setup-view.sh（不动臂）")
    ap.add_argument("--urdf", default=None)
    ap.add_argument("--from-frame", default=None, help="含 joint_states.json 的目录")
    ap.add_argument("--q-start", default=None, help="六个关节角（弧度，逗号分隔）")
    ap.add_argument("--out-dir", default=None, help="--plan 的产物目录")
    ap.add_argument("--board-dir", default="$HOME/precision/plans/e9/setup-view")
    ap.add_argument("--board-log-dir", default="$HOME/precision/logs/e9/setup-view")
    ap.add_argument("--board-burst-dir", default=None,
                    help="板上 burst 目录（缺省 $HOME/precision/e9/setup-view/wrist-pick）")
    ap.add_argument("--min-remaining", type=float, default=120.0,
                    help="setup-view.sh 开头的板端令牌剩余守卫（秒）")

    ap.add_argument("--burst", default=None, help="腕部 burst 目录（含 tf/joint）")
    ap.add_argument("--frame", type=int, default=0)
    ap.add_argument("--image", default=None, help="直接给一张照片（配 --camera-json）")
    ap.add_argument("--camera-json", default=None,
                    help='{"K":..,"R_bc":..,"t_bc_m":[..]}')
    ap.add_argument("--scene", default=None, help="布局 JSON（缺省用 --batch/--layout）")
    ap.add_argument("--batch", default="S0", choices=sorted(lay.BATCHES))
    ap.add_argument("--layout", type=int, default=None)
    ap.add_argument("--zone", default=None, metavar="X0,X1,Y0,Y1")
    ap.add_argument("--table-z-mm", type=float, default=0.0,
                    help="足迹所在高度（台面 = 0；给垫上就填垫厚）")
    ap.add_argument("--out", default=None, help="标注图输出（PNG）")
    ap.add_argument("--json", default=None, help="标注文档输出（缺省与 PNG 同名）")
    return ap


def main(argv=None):
    ap = build_parser()
    args = ap.parse_args(argv)
    if args.plan:
        if not (args.urdf and args.out_dir):
            ap.error("--plan 要给 --urdf 与 --out-dir")
        return plan_main(args)
    if args.out is None:
        ap.error("标注模式要给 --out")
    if not args.scene and args.layout is None:
        ap.error("要给 --layout（或 --scene）")
    return annotate_main(args)


if __name__ == "__main__":
    sys.exit(main())

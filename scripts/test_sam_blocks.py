#!/usr/bin/env python3
"""``sam_blocks`` 自检：纯逻辑单测（注入假预测器，不依赖 torch/GPU）。

合成场景：俯视相机（600 px 焦距）看平面台面，两块 29.5 mm 贴合方块
（orange + tan，色相差 8° = S1 真机故障对），深度图给出台面 + 两块顶面。
假预测器按掩码矩形返回身份掩码 —— 钉住的是分层逻辑（实例切分/身份投票/
覆盖回退/钳制），不是 SAM 模型本身（那由 burst 回放盖）。

跑法：python3 scripts/test_sam_blocks.py
"""
import math
import os
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np                                       # noqa: E402

import sam_blocks as sb                                 # noqa: E402

N_PASS = N_FAIL = 0
FAILS = []


def check(name, cond, detail=''):
    global N_PASS, N_FAIL
    if cond:
        N_PASS += 1
        print('  PASS  %s' % name)
    else:
        N_FAIL += 1
        FAILS.append(name)
        print('  FAIL  %s%s' % (name, '' if detail == '' else '   ← %s'
                                % (detail,)))


# ---------------------------------------------------------------- 合成场景
H, W = 100, 120
K = np.array([[600.0, 0, 60.0], [0, 600.0, 50.0], [0, 0, 1.0]])
R_BC = np.diag([1.0, -1.0, -1.0]).astype(float)
T_BC = np.array([0.0, 0.0, 1.0])
CAM = {"K": K, "R_bc": R_BC, "t_bc": T_BC}

#: 块在像素系的矩形（base 29.5 mm ⇒ 17.7 px）
UA, UB = 45, 63          # A（orange）u 45..62
UC, UD = 63, 81          # B（tan）u 63..80
VA, VB = 35, 53

# 真机实测渲染色（第 349 轮测量）：rel_sat orange≈215 / tan≈132 —— tan 是
# 低饱和米色，与 orange 的区分靠 (hue, rel_sat) 2D 二义消除，不靠纯 hue。
ORANGE = np.array([230, 66, 0])     # hue ≈ 20.5°  rel_sat ≈ 255
TAN = np.array([200, 146, 96])      # hue ≈ 28.6°  rel_sat ≈ 132（低饱和米色）


def make_big_frame():
    """两块 40×40 px 的大顶面（像素数过 SAM 门，专测调用形态）。"""
    depth = np.full((H, W), 1.0)
    color = np.full((H, W, 3), 220, dtype=np.uint8)
    for (u0, u1), col in (((10, 50), ORANGE), ((70, 110), TAN)):
        depth[20:60, u0:u1] = 1.0 - 0.0295
        color[20:60, u0:u1] = col
    return {"depth": depth, "color": color, "K": K, "R_bc": R_BC,
            "t_bc": T_BC, "dir": None}


def make_frame():
    depth = np.full((H, W), 1.0)                       # 台面
    color = np.full((H, W, 3), 220, dtype=np.uint8)    # 浅灰桌面
    depth[VA:VB, UA:UB] = 1.0 - 0.0295                 # A 顶面
    depth[VA:VB, UB:UD] = 1.0 - 0.0295                 # B 顶面（贴合）
    color[VA:VB, UA:UB] = ORANGE
    color[VA:VB, UB:UD] = TAN
    return {"depth": depth, "color": color, "K": K, "R_bc": R_BC,
            "t_bc": T_BC, "dir": None}


class FakeSam:
    """假预测器：按预设矩形返回掩码（u0,v0,u1,v1 列表）。"""

    def __init__(self, rects):
        self.rects = rects
        self.calls = []

    def masks_for(self, points, image_path=None):
        self.calls.append([tuple(round(v) for v in p) for p in points])
        out = np.zeros((len(self.rects), H, W), dtype=bool)
        for i, (u0, v0, u1, v1) in enumerate(self.rects):
            out[i, v0:v1, u0:u1] = True
        return out


def main():
    print('=' * 68)
    print('[1] discover_prompts：base 系长轴端点投影（第 346 轮的坑）')
    row = {"cx": -10.0, "cy": 10.0, "top": 29.5, "ext_rect": [59.0, 29.5],
           "yaw_rect": 0.0}
    pts = sb.discover_prompts(row, CAM)
    check('两个提示点都在图内且分居长轴两端',
          len(pts) == 2 and abs(pts[0][0] - pts[1][0]) > 1
          and abs(pts[0][1] - pts[1][1]) < 1e-6
          and all(0 <= u < W and 0 <= v < H for u, v in pts), str(pts))

    print('\n[2] 拆分主路径：假预测器给左右两片 ⇒ orange/tan 两行')
    fr = make_frame()
    pred = FakeSam([(30, 30, UB, VB + 5), (UB, 30, 100, VB + 5)])
    rows, tz = sb.find_blocks_sam(fr, pred)
    check('台面拟合 ≈ 0（量化噪声 ±2 mm 内，与网格链同口径）',
          abs(tz) < 0.005, tz)
    check('两行（贴合对拆开）', len(rows) == 2,
          [(r['class'], round(r['cx'], 1)) for r in rows])
    if len(rows) == 2:
        cls_set = {r['class'] for r in rows}
        check('身份 = {orange, tan}（色相差 8° 不再互吃）',
              cls_set == {'orange', 'tan'}, str(cls_set))
        d = math.hypot(rows[0]['cx'] - rows[1]['cx'],
                       rows[0]['cy'] - rows[1]['cy'])
        check('对距 ≈ 29.5 mm（钳制或像素直出）',
              abs(d - 29.5) < 1.5, round(d, 2))
        check('schema 关键字段齐',
              all(k in rows[0] for k in
                  ('cx', 'cy', 'uv', 'top', 'h', 'yaw', 'ext_rect', 'class',
                   'class_share', 'votes', 'split_src', 'split_n',
                   'mixed_pair_unsplit', 'low')),
              str(sorted(rows[0])))
        check('split_src ∈ {sam, grid+size}、low = False',
              all(r['split_src'] in ('sam', 'grid+size')
                  and r['low'] is False for r in rows),
              [r['split_src'] for r in rows])
    # 预拆路径（grid+size）可能在 SAM 之前就把对拆开了（合成块很小，
    # 达不到 SAM 簇的 2×PIECE_MIN_PX 门）⇒ 调用可以为空；有调用时必是两端点。
    check('SAM 调用（若有）每次收到长轴两端',
          all(len(c) == 2 for c in pred.calls), str(pred.calls))

    print('\n[3] 单掩码盖两块 ⇒ 尺寸先验拆开（同色/异色同一路径）')
    fr = make_frame()
    pred = FakeSam([(30, 30, 100, VB + 5)])            # 一片盖两块
    rows, _ = sb.find_blocks_sam(fr, pred)
    # 2026-11-17：单片盖两块不再只靠「混合票 ⇒ 丢」；先按外接矩形长边/块边
    # 做尺寸先验几何拆分（S2:1 同色对真机数据点），拆不动才保留混合票。
    check('单片盖两块 ⇒ 尺寸先验拆成两行、身份 orange/tan',
          len(rows) == 2
          and {r['class'] for r in rows} == {'orange', 'tan'},
          [(r['class'], r['split_src']) for r in rows])
    check('拆分来源 = grid+size / sam+size',
          all(r['split_src'] in ('grid+size', 'sam+size') for r in rows),
          [r['split_src'] for r in rows])

    print('\n[4] 预测器无掩码 ⇒ 回退网格行（fail-closed 同现状）')
    fr = make_frame()
    pred = FakeSam([])
    rows, _ = sb.find_blocks_sam(fr, pred)
    check('回退：行仍在且带 sam_fallback',
          len(rows) >= 1 and any('sam_fallback' in r for r in rows),
          [(r.get('class'), r.get('sam_fallback')) for r in rows])

    print('\n[5] clamp_pairs：带内钳到块边、uv 重投影、带外不动')
    cam = CAM
    a = {"cx": -10.0, "cy": 0.0, "top": 29.5, "uv": [60.0, 50.0],
         "low": False}
    b = {"cx": -10.0, "cy": 26.3, "top": 29.5, "uv": [60.0, 34.0],
         "low": False}
    d = sb.clamp_pairs([a, b], cam)
    got = math.hypot(a['cx'] - b['cx'], a['cy'] - b['cy'])
    check('26.3 mm 带内 ⇒ 钳到 29.5', abs(got - 29.5) < 0.01,
          round(got, 2))
    check('uv 重投影（x 不变 ⇒ uv u 列不动，v 沿 y 推开）',
          abs(a['uv'][0] - b['uv'][0]) < 0.5
          and abs(a['uv'][0] - 54.0) < 0.6
          and abs(a['uv'][1] - b['uv'][1]) > 2, str((a['uv'], b['uv'])))
    c = {"cx": -10.0, "cy": 0.0, "top": 29.5, "uv": [60.0, 50.0],
         "low": False}
    e = {"cx": -10.0, "cy": 35.0, "top": 29.5, "uv": [60.0, 29.0],
         "low": False}
    d2 = sb.clamp_pairs([c, e], cam)
    got2 = math.hypot(c['cx'] - e['cx'], c['cy'] - e['cy'])
    check('35 mm 带外（>1.02×块边）⇒ 不钳', d2 is None
          and abs(got2 - 35.0) < 0.01, (d2, round(got2, 2)))

    # 2026-11-18 S2:5 回归：28.7 mm（0.973×块边）必须钳回 29.5（旧上界 0.95
    # 让它漏网，两块中心各偏 ~0.4 mm ⇒ 抓取偏心带起 partner）。
    f = {"cx": -10.0, "cy": 0.0, "top": 29.5, "uv": [60.0, 50.0],
         "low": False}
    g = {"cx": -10.0, "cy": 28.7, "top": 29.5, "uv": [60.0, 31.0],
         "low": False}
    d3 = sb.clamp_pairs([f, g], cam)
    got3 = math.hypot(f['cx'] - g['cx'], f['cy'] - g['cy'])
    check('28.7 mm（S2:5 实证）⇒ 钳到 29.5（旧上界漏钳的回归）',
          d3 is not None and abs(got3 - 29.5) < 0.01,
          (d3, round(got3, 2)))

    print('\n[6] mixed_votes：25% 口径')
    check('双类 60/40 ⇒ True',
          sb.mixed_votes({'cyan': 60, 'tan': 40}, 100) is True)
    check('次要类 15% ⇒ False',
          sb.mixed_votes({'orange': 80, 'tan': 15}, 95) is False)
    check('单类 ⇒ False', sb.mixed_votes({'red': 100}, 100) is False)

    print('\n[7] measure_dir 的 seg 校验')
    try:
        import stage4_truth_from_frame as tff
        tff.measure_dir('/nonexistent', seg='bogus')
        check('seg=bogus ⇒ ValueError', False, '没抛')
    except ValueError as e:
        check('seg=bogus ⇒ ValueError（点名收到值）', 'bogus' in str(e),
              str(e))
    except Exception as e:                            # noqa: BLE001
        check('seg=bogus ⇒ ValueError', False, repr(e))

    print('\n[8] 同色贴合对的尺寸先验拆分（S2:1 真机数据点）')

    def _pair_piece(n=30, shift=0.0295):
        xs = np.linspace(-0.01475, 0.01475, n)
        ys = np.linspace(-0.01475, 0.01475, n)
        gx, gy = np.meshgrid(xs, ys)
        base = np.column_stack([gx.ravel(), gy.ravel(),
                                np.full(gx.size, 0.03)])
        second = base.copy()
        second[:, 0] += shift
        Pb = np.vstack([base, second]) if shift else base
        W, H = 200, 200
        pix = np.array([(i % W, i // W) for i in range(len(Pb))], int)
        color = np.zeros((H, W, 3), np.uint8)
        color[..., 0], color[..., 1], color[..., 2] = 200, 20, 20  # red
        rect = sb.gc.min_area_rect_2d(Pb[:, :2])
        return {"idx": np.arange(len(Pb)), "rect": rect}, Pb, pix, color

    pc, Pb, pix, color = _pair_piece()
    pieces = sb.split_same_colour_piece(pc, Pb, pix, color)
    check('同色贴合对（29.5×59 mm 顶面）⇒ 拆成两片', len(pieces) == 2,
          [p.get('n') for p in pieces])
    if len(pieces) == 2:
        d = math.hypot(pieces[0]['c'][0] - pieces[1]['c'][0],
                       pieces[0]['c'][1] - pieces[1]['c'][1]) * 1000.0
        check('两片中心距 ≈ 块边（29.5 mm）', abs(d - 29.5) <= 2.0,
              round(d, 2))
        check('两片身份都是 red', all(p.get('cls') == 'red' for p in pieces),
              [p.get('cls') for p in pieces])
    pc1, Pb1, pix1, color1 = _pair_piece(shift=0.0)
    check('单块（29.5×29.5）⇒ 原样一片（不硬切）',
          len(sb.split_same_colour_piece(pc1, Pb1, pix1, color1)) == 1)

    print('\n[6] 提示点合并 + 图像块心并行（2026-11-20 计算段优化）')

    class PointFake:
        def __init__(self, batch):
            if batch:
                self.supports_batch = True
            self.calls = []

        def masks_for(self, points, image_path=None):
            self.calls.append([(round(float(u), 1), round(float(v), 1))
                               for u, v in points])
            out = np.zeros((len(points), H, W), dtype=bool)
            for i, (u, v) in enumerate(points):
                u0, u1 = max(0, int(u) - 9), min(W, int(u) + 9)
                v0, v1 = max(0, int(v) - 9), min(H, int(v) + 9)
                out[i, v0:v1, u0:u1] = True
            return out

    def _k(rows):
        return [(r.get('class'), round(float(r['cx']), 2),
                 round(float(r['cy']), 2), r.get('img_ok')) for r in rows]

    pa = PointFake(False)
    ra, _ = sb.find_blocks_sam(make_big_frame(), pa)
    pb = PointFake(True)
    rb, _ = sb.find_blocks_sam(make_big_frame(), pb)
    check('逐行调用 2 次 / 批处理只调 1 次',
          len(pa.calls) == 2 and len(pb.calls) == 1,
          (len(pa.calls), len(pb.calls)))
    check('批处理与逐行输出一致（类/中心/img_ok）', _k(ra) == _k(rb),
          (_k(ra), _k(rb)))
    check('批处理一次收到全部提示点（2 行 × 2 点）',
          len(pb.calls[0]) == 4, str(pb.calls))
    r1, _ = sb.find_blocks_sam(make_big_frame(), PointFake(False), img_jobs=1)
    r2, _ = sb.find_blocks_sam(make_big_frame(), PointFake(False), img_jobs=4)
    check('图像块心并行与串行输出一致（img 字段）',
          [(r.get('img_ok'), r.get('img_cx'), r.get('img_cy')) for r in r1]
          == [(r.get('img_ok'), r.get('img_cx'), r.get('img_cy'))
              for r in r2])
    check('UltralyticsSam 声明支持批处理',
          sb.UltralyticsSam.supports_batch is True)

    print('\n%d PASS / %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('失败项：' + '; '.join(FAILS))
    return 0 if N_FAIL == 0 else 1


if __name__ == '__main__':
    sys.exit(main())

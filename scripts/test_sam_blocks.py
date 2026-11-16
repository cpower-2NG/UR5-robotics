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

ORANGE = np.array([255, 87, 0])    # hue ≈ 20.5°
TAN = np.array([255, 121, 0])      # hue ≈ 28.5°


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
        check('split_src = sam、low = False',
              all(r['split_src'] == 'sam' and r['low'] is False
                  for r in rows))
    check('提示点被记录（预测器收到长轴两端）',
          len(pred.calls) == 1 and len(pred.calls[0]) == 2,
          str(pred.calls))

    print('\n[3] 单掩码盖两块 ⇒ 混合票标记（build_truth 会丢）')
    fr = make_frame()
    pred = FakeSam([(30, 30, 100, VB + 5)])            # 一片盖两块
    rows, _ = sb.find_blocks_sam(fr, pred)
    check('单片 ⇒ 1 行且 mixed_pair_unsplit = True',
          len(rows) == 1 and rows[0]['mixed_pair_unsplit'] is True,
          [(r['class'], r['mixed_pair_unsplit']) for r in rows])

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
    check('35 mm 带外（>0.95×块边）⇒ 不钳', d2 is None
          and abs(got2 - 35.0) < 0.01, (d2, round(got2, 2)))

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

    print('\n%d PASS / %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('失败项：' + '; '.join(FAILS))
    return 0 if N_FAIL == 0 else 1


if __name__ == '__main__':
    sys.exit(main())

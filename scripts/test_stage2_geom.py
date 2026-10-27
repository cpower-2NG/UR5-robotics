#!/usr/bin/env python3
"""阶段 2 顶面像素矩形的自检（纯合成，不读存档帧）。

钉住三件事：
  1. 轴对齐矩形 → yaw mod 90 ≈ 0；
  2. 像素里转 30° → 基底 yaw mod 90 ≈ 60°（图像 y 向下、基底 y 向上，
     所以符号相反；45° 自映射）；转回去必须变红 —— 防「永远返回 0」；
  3. 合成深度帧上，半径内的块被选中、半径过小会丢掉（变异测试）。

跑法：
    python3 scripts/test_stage2_geom.py
"""

import math
import os
import sys

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import cv2                                                       # noqa: E402
import numpy as np                                               # noqa: E402

import stage2_geom as sg                                         # noqa: E402

CASES = []


def case(fn):
    CASES.append(fn)
    return fn


def _K():
    return np.array([[500.0, 0.0, 320.0],
                     [0.0, 500.0, 240.0],
                     [0.0, 0.0, 1.0]], dtype=np.float64)


def _pose():
    return np.diag([1.0, -1.0, -1.0]), np.array([0.0, 0.0, 0.5])


def _rect_mask(w=100, h=60, deg=0.0, shape=(480, 640)):
    m = np.zeros(shape, dtype=np.uint8)
    c = np.array([shape[1] / 2.0, shape[0] / 2.0])
    a = math.radians(deg)
    loc = np.array([[-w / 2, -h / 2], [w / 2, -h / 2],
                    [w / 2, h / 2], [-w / 2, h / 2]])
    R = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    cv2.fillPoly(m, [(loc @ R.T + c).astype(np.int32)], 255)
    return m


@case
def test_axis_aligned_mask():
    K = _K()
    R, t = _pose()
    r = sg.mask_rect_yaw(_rect_mask(), K, R, t, 0.0)
    assert r is not None
    assert sg.yaw_mod90(r['yaw_deg']) < 1.0, r
    assert abs(r['ext_mm'][0] - 100.0) < 1.0, r['ext_mm']
    assert abs(r['ext_mm'][1] - 60.0) < 1.0, r['ext_mm']
    assert r['fill'] > 0.9, r


@case
def test_rotated_mask_maps_to_60_mod90():
    """像素 +30°（y 向下）→ 基底 −30°（y 向上）→ mod 90 = 60°。"""
    K = _K()
    R, t = _pose()
    r30 = sg.mask_rect_yaw(_rect_mask(deg=30.0), K, R, t, 0.0)
    assert r30 is not None
    assert abs(sg.yaw_mod90(r30['yaw_deg']) - 60.0) < 1.5, r30
    r60 = sg.mask_rect_yaw(_rect_mask(deg=60.0), K, R, t, 0.0)
    assert abs(sg.yaw_mod90(r60['yaw_deg']) - 30.0) < 1.5, r60
    # 45° 自映射；三条互相不能相等（防常量返回）。
    r45 = sg.mask_rect_yaw(_rect_mask(deg=45.0), K, R, t, 0.0)
    assert abs(sg.yaw_mod90(r45['yaw_deg']) - 45.0) < 1.5, r45
    vals = [sg.yaw_mod90(x['yaw_deg']) for x in (r30, r45, r60)]
    assert len(set(round(v, 1) for v in vals)) == 3, vals


@case
def test_empty_and_tiny_mask():
    K = _K()
    R, t = _pose()
    assert sg.mask_rect_yaw(np.zeros((480, 640), np.uint8), K, R, t, 0.0) is None
    tiny = np.zeros((480, 640), np.uint8)
    cv2.circle(tiny, (320, 240), 3, 255, -1)
    assert sg.mask_rect_yaw(tiny, K, R, t, 0.0) is None, '3 px 圆不该当矩形'


@case
def test_ang_span_mod90():
    assert sg.ang_span_mod90([]) is None
    assert abs(sg.ang_span_mod90([1.0, 89.0]) - 2.0) < 1e-9
    assert abs(sg.ang_span_mod90([0.0, 45.0]) - 45.0) < 1e-9
    assert abs(sg.ang_span_mod90([10.0, 40.0, 70.0])
               - 60.0) < 1e-9, '圆周上 30° 间隔的三点，极差是 60 不是 30'
    assert abs(sg.ang_span_mod90([10.0, 30.0, 50.0]) - 40.0) < 1e-9
    assert sg.ang_span_mod90([None, 10.0]) == 0.0


def _synth_frame():
    """台面 base z=0、块顶 base z=30 mm；块在图像里是 40×30 px 矩形。"""
    h, w = 480, 640
    depth = np.full((h, w), 0.500, dtype=np.float32)
    depth[240 - 15:240 + 15, 320 - 20:320 + 20] = 0.470
    R, t = _pose()
    return {'depth': depth, 'color': np.zeros((h, w, 3), np.uint8),
            'K': _K(), 'R_bc': R, 't_bc': t}


@case
def test_block_rect_on_synthetic_depth():
    frame = _synth_frame()
    block = {'cx': 0.0, 'cy': 0.0, 'top': 30.0, 'h': 30.0}
    r = sg.block_rect_yaw(frame, block)
    assert r['yaw_deg'] is not None
    assert sg.yaw_mod90(r['yaw_deg']) < 1.0, r
    assert r['trusted'] is True, r
    assert r['n_pixels'] >= 1000, r
    assert abs(r['ext_mm'][0] - 0.470 * 40 / 500 * 1000) < 2.0, r['ext_mm']
    assert abs(r['ext_mm'][1] - 0.470 * 30 / 500 * 1000) < 2.0, r['ext_mm']


@case
def test_block_rect_radius_mutation():
    """半径缩到 5 mm → 只剩中心一小块，不能报 trusted。"""
    frame = _synth_frame()
    block = {'cx': 0.0, 'cy': 0.0, 'top': 30.0, 'h': 30.0}
    r = sg.block_rect_yaw(frame, block, radius_mm=5.0)
    assert r['trusted'] is False, r


# ---------------------------------------------------------------- runner
def main():
    npass = nfail = 0
    for fn in CASES:
        try:
            fn()
        except AssertionError as e:
            print('FAIL %s: %s' % (fn.__name__, e))
            nfail += 1
        except Exception as e:                                   # noqa: BLE001
            print('ERROR %s: %s: %s' % (fn.__name__, type(e).__name__, e))
            nfail += 1
        else:
            print('PASS %s' % fn.__name__)
            npass += 1
    print('\n%d PASS, %d FAIL' % (npass, nfail))
    return 1 if nfail else 0


if __name__ == '__main__':
    sys.exit(main())

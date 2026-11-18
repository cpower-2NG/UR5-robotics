#!/usr/bin/env python3
"""``stage4_image_center`` 自检：合成图上的方块边缘对齐精度与 fail-closed。

判据（用户 2026-11-18：块心准出闸门要「落在图像里块的实际中心」）：
- 已知块心/朝向/边长的合成方块，投影成 1280×720 彩色图 + 掩码；
- 图像空间细化后块心误差 **< 0.45 mm**、yaw 误差 **< 1.5°**（mod 90，
  方块四重对称 ⇒ yaw 判据天然弱；0.45 mm 栅格地板在 56 px 方块上折 ~1.3°）；
  0.45 mm ≈ 0.85 px：**合成图栅格化地板实测 ~0.25 mm**（把画出的方块自己
  的最小外接矩形中心反投影，和数学中心差 (−0.21,+0.20)，见 make_scene
  注释）；算法相对「画出来的方块中心」只差 ~0.05 mm。
- **掩码单侧侵蚀/偏移不影响块心**（2026-11-18 S2:5 实证：深度方框中心
  被可见侧面带上沿拉偏 1–3.5 mm；图像拟合只看彩色边线，掩码只提供初值）；
- 无颜色边 / 掩码太小 / 初值离真值太远 ⇒ ``ok=False``（不给数）。

跑法：python3 scripts/test_stage4_image_center.py
"""
import math
import os
import sys

import cv2
import numpy as np

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import stage4_clear_run as cr                                     # noqa: E402
import stage4_image_center as ic                                  # noqa: E402

N_PASS = N_FAIL = 0
FAILS = []

K = np.array([[900.0, 0.0, 640.0], [0.0, 900.0, 360.0], [0.0, 0.0, 1.0]])
R_BC = np.diag([1.0, -1.0, -1.0])
T_BC = np.array([0.45, 0.30, 0.50])
TOP_Z = 0.0295


def check(name, cond, detail=''):
    global N_PASS, N_FAIL
    if cond:
        N_PASS += 1
        print('  PASS  %s' % name)
    else:
        N_FAIL += 1
        FAILS.append(name)
        print('  FAIL  %s%s' % (name, ('   ← %s' % (detail,)) if detail else ''))


def _corners(cx, cy, yaw_deg, size):
    h = size / 2.0
    a = math.radians(yaw_deg)
    ca, sa = math.cos(a), math.sin(a)
    return [(cx + sx * h * ca - sy * h * sa, cy + sx * h * sa + sy * h * ca)
            for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]


def make_scene(cx=0.45, cy=0.30, yaw=20.0, size=0.0295, bg=30):
    """合成方块：4× 超采样绘制再降采样（整数像素直画有 ~0.5 px 的量化偏置，
    会把测量地板抬到 ~0.26 mm；2026-11-18 实测这一项曾把测试判红）。"""
    corners = _corners(cx, cy, yaw, size)
    uv = [cr._base_to_uv((x, y, TOP_Z), K, R_BC, T_BC) for x, y in corners]
    ss = 4
    uv4 = np.array([[int(round(u * ss)), int(round(v * ss))]
                    for u, v in uv], np.int32)
    big = np.full((720 * ss, 1280 * ss, 3), bg, np.uint8)
    cv2.fillConvexPoly(big, uv4, (200, 200, 200))
    color = cv2.resize(big, (1280, 720), interpolation=cv2.INTER_AREA)
    big_mask = np.zeros((720 * ss, 1280 * ss), np.uint8)
    cv2.fillConvexPoly(big_mask, uv4, 255)
    mask = cv2.resize(big_mask, (1280, 720), interpolation=cv2.INTER_AREA)
    mask_uv = np.argwhere(mask > 127)[:, ::-1]        # (u, v)
    return mask_uv, color


def main():
    print('=' * 68)
    print('[1] 图像空间块心：合成方块（已知 0.450, 0.300 / yaw 20° / 29.5 mm）')
    mask_uv, color = make_scene()
    r = ic.refine_center(mask_uv, color, K, R_BC, T_BC, TOP_Z)
    check('ok=True', r.get('ok') is True, r.get('why'))
    if r.get('ok'):
        dx = r['x_mm'] - 450.0
        dy = r['y_mm'] - 300.0
        check('块心误差 < 0.45 mm', math.hypot(dx, dy) < 0.45,
              (round(r['x_mm'], 2), round(r['y_mm'], 2)))
        dyaw = abs(((r['yaw_deg'] - 20.0 + 45.0) % 90.0) - 45.0)
        check('yaw 误差 < 1.5°（mod 90）', dyaw < 1.5,
              (r['yaw_deg'], round(dyaw, 2)))
        check('四角在画幅内',
              all(0 <= u < 1280 and 0 <= v < 720 for u, v in r['corners_uv']),
              r['corners_uv'])
        check('支撑边数 ≥3（四条合成边）', r.get('strong_sides', 0) >= 3,
              r.get('support'))
    print('[2] 偏移 60 mm 的块：块心仍准（位置无关）')
    mask_uv2, color2 = make_scene(cx=0.51, cy=0.36, yaw=55.0)
    r2 = ic.refine_center(mask_uv2, color2, K, R_BC, T_BC, TOP_Z)
    check('ok=True', r2.get('ok') is True, r2.get('why'))
    if r2.get('ok'):
        err = math.hypot(r2['x_mm'] - 510.0, r2['y_mm'] - 360.0)
        check('块心误差 < 0.45 mm', err < 0.45,
              (round(r2['x_mm'], 2), round(r2['y_mm'], 2)))

    print('[3] 掩码单侧侵蚀（2026-11-18 S2:5 回归：掩码只提供初值）')
    # 右侧 30% 的掩码像素被吃掉 ⇒ 掩码矩形中心明显左偏；颜色边线完整。
    cx0 = float(np.mean([p[0] for p in mask_uv]))
    keep = mask_uv[mask_uv[:, 0] < cx0 + 0.35 * (mask_uv[:, 0].max() - cx0)]
    r3 = ic.refine_center(keep, color, K, R_BC, T_BC, TOP_Z)
    check('ok=True', r3.get('ok') is True, r3.get('why'))
    if r3.get('ok'):
        err = math.hypot(r3['x_mm'] - 450.0, r3['y_mm'] - 300.0)
        check('单侧侵蚀后块心仍 < 0.45 mm（初值偏了也不跟）', err < 0.45,
              (round(r3['x_mm'], 2), round(r3['y_mm'], 2),
               r3.get('shift_mm')))

    print('[4] fail-closed：无边 / 掩码太小 / 初值离真值太远')
    flat = np.full((720, 1280, 3), 40, np.uint8)
    r4 = ic.refine_center(mask_uv, flat, K, R_BC, T_BC, TOP_Z)
    check('无颜色边 ⇒ ok=False（边支撑）', r4.get('ok') is False,
          r4.get('why'))
    r5 = ic.refine_center(np.zeros((4, 2), int), color, K, R_BC, T_BC, TOP_Z)
    check('4 点掩码且无初值 ⇒ ok=False', r5.get('ok') is False, r5.get('why'))
    r6 = ic.refine_center(None, color, K, R_BC, T_BC, TOP_Z,
                          cx0_mm=458.0, cy0_mm=300.0, yaw0_deg=20.0)
    check('初值偏 8 mm ⇒ ok=False（位移上限）', r6.get('ok') is False,
          r6.get('why'))

    print('\n%d PASS / %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('失败项：' + '; '.join(FAILS))
    return 0 if N_FAIL == 0 else 1


if __name__ == '__main__':
    sys.exit(main())

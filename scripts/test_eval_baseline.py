#!/usr/bin/env python3
"""`eval_baseline.py` 的自检：**合成图**，已知块在哪。

为什么单列
==========
真值分割换过原理（2026-10-25：从「YOLO 框里按颜色分」→「按深度分层」，
再补「颜色轮廓 + 立方体拟合」），而那一步错了**不会报错**：它会安安静静
地多数一块、少数一块，误差统计照样给出好看的数字。所以这里用合成图把
「找到几块、中心在哪、贴边怎么算」钉住。

⚠️ 合成图只验**逻辑**，验不了真相机。真帧的分割只能用真实帧看（见台账）。
"""

import os
import sys

import numpy as np

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import eval_baseline as eb                                    # noqa: E402

_failed = []


def check(name, cond, detail=''):
    print("  [%s] %s%s" % ("PASS" if cond else "FAIL", name,
                           ("  " + detail) if detail else ""))
    if not cond:
        _failed.append(name)


def section(t):
    print("\n" + t)


# 相机朝下看板：板系 z 轴朝相机 ⇒ 板面 z_board=0，块顶 z_board=+dz
R_CB = np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])
T_CB = np.array([0.0, 0.0, 0.400])
K = np.array([[900.0, 0, 640.0], [0, 900.0, 360.0], [0, 0, 1.0]])
D = np.zeros((5, 1))


def synth(blocks, w=1280, h=720):
    """blocks: [(x0, x1, y0, y1, 高出板面的米数)]

    ⚠️ 深度图按**真实 PNG 的约定**给：uint16、单位**毫米**（`segment_blocks_depth`
    内部 /1000 换成米）。第一版这里给的是米，于是整层都落在掩膜外、一块都找不到 ——
    测试自己错了，函数没错。
    """
    depth = np.full((h, w), 400, np.uint16)
    for x0, x1, y0, y1, dz in blocks:
        depth[y0:y1, x0:x1] = np.uint16(round(400 - dz * 1000.0))
    return depth


section("[1] 两块都找到，质心落在块中心（±3 px）")
d = synth([(200, 280, 300, 380, 0.0285), (760, 840, 360, 440, 0.0285)])
got = eb.segment_blocks_depth(d, K, D, R_CB, T_CB)
check("找到 2 块", len(got) == 2, str([(round(g["u"]), round(g["v"])) for g in got]))
if len(got) == 2:
    got.sort(key=lambda g: g["u"])
    check("第一块在 (240, 340)", abs(got[0]["u"] - 240) <= 3
          and abs(got[0]["v"] - 340) <= 3,
          "(%.1f, %.1f)" % (got[0]["u"], got[0]["v"]))
    check("第二块在 (800, 400)", abs(got[1]["u"] - 800) <= 3
          and abs(got[1]["v"] - 400) <= 3,
          "(%.1f, %.1f)" % (got[1]["u"], got[1]["v"]))
    check("高度落在 28.5 mm 附近（±3）",
          all(abs(g["z_top_mm"] - 28.5) <= 3 for g in got),
          str([g["z_top_mm"] for g in got]))
    check("都不算贴边", all(not g["truncated"] for g in got))

section("[2] 贴图像边缘的块要标 truncated")
d = synth([(0, 60, 300, 380, 0.0285)])
got = eb.segment_blocks_depth(d, K, D, R_CB, T_CB)
check("找到 1 块", len(got) == 1)
check("truncated=True", bool(got) and got[0]["truncated"])

section("[3] 纯平面：一块都不许有")
got = eb.segment_blocks_depth(synth([]), K, D, R_CB, T_CB)
check("0 块", len(got) == 0, str(len(got)))

section("[4] 深度野值（0 与 1.0 m 的点）不破坏分割")
d = synth([(200, 280, 300, 380, 0.0285), (760, 840, 360, 440, 0.0285)])
rng = np.random.default_rng(0)
ys = rng.integers(300, 380, 60)
xs = rng.integers(200, 280, 60)
d[ys, xs] = 1000          # 毫米：1 m
d[rng.integers(0, 720, 80), rng.integers(0, 1280, 80)] = 0.0
got = eb.segment_blocks_depth(d, K, D, R_CB, T_CB)
check("仍然找到 2 块", len(got) == 2, str(len(got)))

section("[5] 高了/矮了都不进这一层（层边界 16–45 mm）")
got = eb.segment_blocks_depth(synth([(200, 280, 300, 380, 0.060)]),
                              K, D, R_CB, T_CB)
check("60 mm 的块不算（>45）", len(got) == 0)
got = eb.segment_blocks_depth(synth([(200, 280, 300, 380, 0.008)]),
                              K, D, R_CB, T_CB)
check("8 mm 的块不算（<16）", len(got) == 0)


# ─────────────────── 颜色轮廓 + 立方体模型 ───────────────────

def synth_color(cubes, w=1280, h=720, side_mm=28.5):
    """cubes: [(x_m, y_m, yaw_rad)]（板系）。

    彩色图：白底 + 纯红立方体投影；深度图：桌面 400 mm，块面 400 − 高。
    ⚠️ 块面深度是「整块同一个值」，只为了让 z_med 过 8 mm 的门；
    拟合只读彩色掩膜，不读深度数值。
    """
    import cv2
    img = np.full((h, w, 3), 255, np.uint8)
    depth = np.full((h, w), 400, np.uint16)
    for x, y, yaw in cubes:
        P = eb._cube_corners(x, y, yaw, side_mm / 1000.0, side_mm / 1000.0)
        hull = np.round(eb._cube_hull(P, K, R_CB, T_CB)).astype(np.int32)
        mask = np.zeros((h, w), np.uint8)
        cv2.fillConvexPoly(mask, hull, 1)
        img[mask == 1] = (0, 0, 255)              # BGR 纯红 → S=255
        depth[mask == 1] = np.uint16(round(400 - side_mm))
    return img, depth


section("[6] 颜色轮廓：一个立方体，x/y 恢复到 ±2 mm")
img, d = synth_color([(0.100, 0.050, np.radians(20.0))])
got = eb.segment_blocks_color(img, d, K, D, R_CB, T_CB, 28.5)
check("找到 1 块", len(got) == 1, str(len(got)))
if len(got) == 1:
    g = got[0]
    check("x 恢复（100 mm）", abs(g["x"] - 0.100) * 1000 <= 2.0,
          "%.2f mm" % (g["x"] * 1000))
    check("y 恢复（50 mm）", abs(g["y"] - 0.050) * 1000 <= 2.0,
          "%.2f mm" % (g["y"] * 1000))
    check("IoU ≥ 0.8", g["iou"] >= 0.8, "%.3f" % g["iou"])
    check("不算贴边", not g["truncated"])

section("[7] 颜色轮廓：两块分开的立方体都找到")
img, d = synth_color([(0.100, 0.050, 0.3), (0.220, 0.150, 0.8)])
got = eb.segment_blocks_color(img, d, K, D, R_CB, T_CB, 28.5)
check("找到 2 块", len(got) == 2, str(len(got)))
if len(got) == 2:
    got.sort(key=lambda g: g["x"])
    check("按 x 排序后 100/220 mm",
          abs(got[0]["x"] - 0.100) * 1000 <= 2.0
          and abs(got[1]["x"] - 0.220) * 1000 <= 2.0,
          str([round(g["x"] * 1000, 1) for g in got]))

section("[8] 纯白桌面：一块都没有")
img = np.full((720, 1280, 3), 255, np.uint8)
d = np.full((720, 1280), 400, np.uint16)
got = eb.segment_blocks_color(img, d, K, D, R_CB, T_CB, 28.5)
check("0 块", len(got) == 0, str(len(got)))

section("[9] 低于板面 9 mm 的彩色斑不算积木（z_med < 8 mm）")
import cv2                                                     # noqa: E402
img = np.full((720, 1280, 3), 255, np.uint8)
d = np.full((720, 1280), 400, np.uint16)
cv2.rectangle(img, (400, 300), (480, 380), (0, 0, 255), -1)
d[300:380, 400:480] = 409                  # z_board = −9 mm
got = eb.segment_blocks_color(img, d, K, D, R_CB, T_CB, 28.5)
check("0 块", len(got) == 0, str(len(got)))

print("\n" + "=" * 66)
if _failed:
    print("❌ %d 项没过：" % len(_failed))
    for n in _failed:
        print("   - " + n)
    sys.exit(1)
print("✅ 全部通过（%s）" % os.path.basename(__file__))
sys.exit(0)

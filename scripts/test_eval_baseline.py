#!/usr/bin/env python3
"""`eval_baseline.py` 的自检：**合成深度图**，已知块在哪。

为什么单列
==========
真值分割换了原理（2026-10-25：从「YOLO 框里按颜色分」改成「按深度分层」），
而那一步错了**不会报错**：它会安安静静地多数一块、少数一块，误差统计照样
给出好看的数字。所以这里用合成图把「找到几块、质心在哪、贴边怎么算」钉住。

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

print("\n" + "=" * 66)
if _failed:
    print("❌ %d 项没过：" % len(_failed))
    for n in _failed:
        print("   - " + n)
    sys.exit(1)
print("✅ 全部通过（%s）" % os.path.basename(__file__))
sys.exit(0)

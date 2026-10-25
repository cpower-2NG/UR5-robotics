#!/usr/bin/env python3
"""`eval_baseline.py` 的自检：**合成图**，已知块在哪。

为什么单列
==========
真值分割换过原理（2026-10-25：从「YOLO 框里按颜色分」→「按深度分层」，
再补「颜色轮廓 + 立方体拟合」），而那一步错了**不会报错**：它会安安静静
地多数一块、少数一块，误差统计照样给出好看的数字。所以这里用合成图把
「找到几块、中心在哪、贴边怎么算」钉住。

⚠️ 合成图只验**逻辑**，验不了真相机。真帧的分割只能用真实帧看（见台账）。

后三节（[10]–[12]）另把 `--est pc` 的点云估计钉住：平面拟合、三块立方体的
板系 x/y/顶面高、以及「贴桌面的彩色块必须被高度闸门拒掉」。
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

# ─────────────────── 点云估计（--est pc）───────────────────
# 合成帧让 base 系与板系**差一个非对称的刚体变换**：若 p_base↔板系 的某次
# 变换写反（R 用成 R.T、平移符号反），round-trip 就不再抵消，这里会红。
# 渲染仍按正俯视做（板→相机用 R_CB/T_CB），避免斜视时「侧面混进顶面」的
# 投影误差干扰判定。

def _Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def _Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


R_BC = _Rz(np.radians(37.0)) @ _Ry(np.radians(12.0)) @ np.diag([1.0, -1, -1])
T_BC = -R_BC @ T_CB                     # 板原点 → base 原点，桌面落在 base z≈0


def synth_pc(objs, w=1280, h=720, side_mm=28.5):
    """objs: [(x_m, y_m, yaw, 顶面高_m)]（板系）→ (img, depth, boxes)。

    顶面高 = 0 表示「贴在桌上的彩色块」——用来验高度闸门。

    ⚠️ 不能把整块轮廓填成同一个颜色/同一个深度：相机在光轴外俯视立方体时，
    近侧（顶面）在像面里被放大，轮廓里会露出一圈**侧面**。第一版整块填纯红 +
    顶面深度，结果顶面的「最亮层」把侧面也算进去，质心被拉向光心 3–7 mm
    —— 测试自己错了。现在顶面亮（V=255）、侧面暗（V=128），侧面深度给
    半高（真实值介于桌面与顶面之间），Otsu 能分开两者。
    """
    import cv2
    img = np.full((h, w, 3), 255, np.uint8)
    depth = np.full((h, w), 400, np.uint16)
    boxes = []
    for x, y, yaw, hm in objs:
        P = eb._cube_corners(x, y, yaw, side_mm / 1000.0, hm)
        hull_all = eb._cube_hull(P, K, R_CB, T_CB)
        hull_top = eb._cube_hull(P[4:], K, R_CB, T_CB)     # 顶面与像面平行
        mask = np.zeros((h, w), np.uint8)
        cv2.fillConvexPoly(mask, np.round(hull_all).astype(np.int32), 1)
        img[mask == 1] = (0, 0, 128)                       # 暗红 → 侧面
        depth[mask == 1] = np.uint16(round(400 - hm * 500.0))
        top = np.zeros((h, w), np.uint8)
        cv2.fillConvexPoly(top, np.round(hull_top).astype(np.int32), 1)
        img[top == 1] = (0, 0, 255)                        # 亮红 → 顶面
        depth[top == 1] = np.uint16(round(400 - hm * 1000.0))
        boxes.append((int(hull_all[:, 0].min()), int(hull_all[:, 0].max()),
                      int(hull_all[:, 1].min()), int(hull_all[:, 1].max())))
    return img, depth, boxes


def _make_pc_frame(img, depth):
    """最小可用帧：点云 + 板位姿 + 非对称 TF（base 系与板系不同）。

    TF 用四元数写（真实帧的样子），这里顺带验一次换算。
    """
    (m00, m01, m02), (m10, m11, m12), (m20, m21, m22) = R_BC
    qw = np.sqrt(max(0.0, 1.0 + m00 + m11 + m22)) / 2.0
    qx = np.sqrt(max(0.0, 1.0 + m00 - m11 - m22)) / 2.0 * np.sign(m21 - m12)
    qy = np.sqrt(max(0.0, 1.0 - m00 + m11 - m22)) / 2.0 * np.sign(m02 - m20)
    qz = np.sqrt(max(0.0, 1.0 - m00 - m11 + m22)) / 2.0 * np.sign(m10 - m01)
    entry = {"rotation": {"x": qx, "y": qy, "z": qz, "w": qw},
             "translation": {k: float(v) for k, v in
                             zip(("x", "y", "z"), T_BC)}}
    f = {"dir": "<synth>", "K": K, "D": D, "img": img, "depth": depth,
         "tf": {"ok": {"base_link:camera_color_optical_frame": entry}}}
    R_chk, t_chk = eb._tf_matrix(entry)
    if not (np.allclose(R_chk, R_BC, atol=1e-6) and np.allclose(t_chk, T_BC)):
        raise AssertionError("四元数↔矩阵换算不对：\n%s\nvs\n%s" % (R_chk, R_BC))
    f["_cloud"] = eb.point_cloud(f)
    f["_board_pose"] = (R_CB, T_CB)
    return f


section("[10] _fit_plane：桌面上 20% 野值，仍恢复 z=12 mm 平面（±1.5 mm）")
rng = np.random.default_rng(3)
P_in = np.column_stack([rng.uniform(-0.2, 0.2, 4000),
                        rng.uniform(-0.2, 0.2, 4000),
                        rng.normal(0.012, 0.0008, 4000)])
P_out = np.column_stack([rng.uniform(-0.2, 0.2, (400, 2))])
P_out = np.column_stack([P_out, rng.uniform(-0.05, 0.06, len(P_out))])
fit = eb._fit_plane(np.vstack([P_in, P_out]))
check("有解", fit is not None)
if fit is not None:
    n_f, c_f, cnt_f = fit
    check("法向竖直（|n_z| > 0.99）", abs(n_f[2]) > 0.99, "%.4f" % n_f[2])
    check("平面高度 12 mm（±1.5）", abs(abs(c_f) - 0.012) * 1000 <= 1.5,
          "%.2f mm" % (abs(c_f) * 1000))
    check("内点 ≥ 3500", cnt_f >= 3500, str(cnt_f))

section("[11] 点云估计：三块立方体，板系 x/y/顶面高都恢复（±2 mm）")
img, d, boxes = synth_pc([(0.100, 0.050, np.radians(20.0), 0.0285),
                          (-0.180, -0.060, 0.0, 0.0285),
                          (0.220, -0.090, np.radians(55.0), 0.0285)])
f = _make_pc_frame(img, d)
P, valid = f["_cloud"]
gp = eb._global_table_plane(P, valid)
check("全局兜底桌面存在", gp is not None)
for (bx0, bx1, by0, by1), (tx, ty, tyaw, th) in zip(
        boxes, [(0.100, 0.050, 0.0, 0.0285),
                (-0.180, -0.060, 0.0, 0.0285),
                (0.220, -0.090, 0.0, 0.0285)]):
    det = {"cx": 0.5 * (bx0 + bx1), "cy": 0.5 * (by0 + by1),
           "size_x": bx1 - bx0, "size_y": by1 - by0,
           "best": {"score": 0.9, "class_id": "cube"}}
    plane = eb._local_table_plane(P, valid, (bx0, bx1, by0, by1),
                                  global_plane=gp)
    e = eb.estimate_grasp_pc(f, det, plane)
    tag = "(%.0f, %.0f) mm" % (tx * 1000, ty * 1000)
    check("块 %s 有估计" % tag, e is not None)
    if e is not None:
        pb = e["p_board"]
        check("  板系 x/y（±2 mm）",
              abs(pb[0] - tx) * 1000 <= 2.0 and abs(pb[1] - ty) * 1000 <= 2.0,
              "(%+.2f, %+.2f) mm" % ((pb[0] - tx) * 1000,
                                     (pb[1] - ty) * 1000))
        check("  顶面高 28.5 mm（±2）", abs(pb[2] - th) * 1000 <= 2.0,
              "%.2f mm" % (pb[2] * 1000))
        check("  离桌面高度 ≈ 28.5 mm", abs(e["z_above_mm"] - 28.5) <= 2.0,
              "%.2f mm" % e["z_above_mm"])

section("[12] 贴桌面的彩色块（高 0）被高度闸门拒掉")
img, d, boxes = synth_pc([(0.100, 0.050, 0.0, 0.0)])
f = _make_pc_frame(img, d)
P, valid = f["_cloud"]
gp = eb._global_table_plane(P, valid)
bx0, bx1, by0, by1 = boxes[0]
det = {"cx": 0.5 * (bx0 + bx1), "cy": 0.5 * (by0 + by1),
       "size_x": bx1 - bx0, "size_y": by1 - by0,
       "best": {"score": 0.9, "class_id": "cube"}}
plane = eb._local_table_plane(P, valid, (bx0, bx1, by0, by1), global_plane=gp)
e = eb.estimate_grasp_pc(f, det, plane)
check("返回 None", e is None, "" if e is None else str(e["z_above_mm"]))

print("\n" + "=" * 66)
if _failed:
    print("❌ %d 项没过：" % len(_failed))
    for n in _failed:
        print("   - " + n)
    sys.exit(1)
print("✅ 全部通过（%s）" % os.path.basename(__file__))
sys.exit(0)

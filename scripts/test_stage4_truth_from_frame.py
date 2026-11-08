#!/usr/bin/env python3
"""``stage4_truth_from_frame`` 自检：射线 ∩ 平面 / 过滤 / 编号 / 台面值透传。

为什么值得单测：这个工具是「真机块位真值」的唯一出口 —— 一旦毫米当米、块心 z
算错、或位置退回深度反投影，E8 探针的碰撞场景会整体偏移，而探针照样会报出
一串看起来正常的数。上游 ``find_blocks`` 是现成实现（另有自检），这里只钉住
本工具多出来的那层几何与换算。

**重点钉的是「位置对深度偏差免疫」**：2026-11-08 实测俯视深度系统性读远
3.0–3.3%，如果位置跟着深度走，同一块换个深度读数就会平移几十毫米。本工具
只从 ``uv`` 出发，所以把行的深度值改掉、位置必须一字不动 —— 这条坏了，真机
真值就整体不可信。

跑法：
    python3 scripts/test_stage4_truth_from_frame.py
"""
import os
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np                                                # noqa: E402

import stage4_truth_from_frame as tf                             # noqa: E402

N_PASS = N_FAIL = 0
FAILS = []

# 合成相机：装在 (0.4, 0.3, 1.5) m（拾取区内），正下方看；cam x→base x，
# cam y→base −y，cam z→base −z（det = +1，是真旋转）。600 px 焦距，主点 (320, 240)。
CAM = {"K": np.array([[600.0, 0.0, 320.0],
                      [0.0, 600.0, 240.0],
                      [0.0, 0.0, 1.0]]),
       "R_bc": np.array([[1.0, 0.0, 0.0],
                         [0.0, -1.0, 0.0],
                         [0.0, 0.0, -1.0]]),
       "t_bc": np.array([0.4, 0.3, 1.5])}


def check(name, cond, detail=''):
    global N_PASS, N_FAIL
    if cond:
        N_PASS += 1
        print('  PASS  %s' % name)
    else:
        N_FAIL += 1
        FAILS.append(name)
        print('  FAIL  %s%s'
              % (name, '' if detail == '' else '   ← %r' % (detail,)))


def row(idx, uv=(320.0, 240.0), cx=400.0, cy=300.0, top=32.0, h=None,
        cls="red", low=False, yaw=0.0):
    """``find_blocks`` 的一行（毫米 / 度，与真实输出同单位）。"""
    return {"idx": idx, "cx": cx, "cy": cy, "top": top,
            "h": top if h is None else h, "yaw": yaw,
            "class": cls, "class_share": 0.9, "ext": [29.0, 28.6],
            "ext_rect": [29.1, 28.9], "uv": list(uv), "low": low}


# ---------------------------------------------------------------- 几何
def section_ray_plane():
    print('\n[1] 射线 ∩ 平面：主点正对相机光轴 ⇒ 落在相机脚下')
    t = tf.build_truth([row(0, uv=(320.0, 240.0))], CAM, table_top=0.0)
    b = t["blocks"][0]
    check("x ≈ 相机 x（0.400）", abs(b["x"] - 0.400) < 1e-9, b["x"])
    check("y ≈ 相机 y（0.300）", abs(b["y"] - 0.300) < 1e-9, b["y"])
    check("top = 台面 + 块高", abs(b["top"] - 0.0295) < 1e-12, b["top"])
    check("z = top − size/2（块心）",
          abs(b["z"] - (0.0295 - 0.01475)) < 1e-12, b["z"])
    check("size 用默认 29.5 mm", abs(b["size"] - 0.0295) < 1e-12, b["size"])
    check("yaw 原样透传", b["yaw"] == 0.0, b["yaw"])

    # u+60px：平面在 1.4705 m 下方 ⇒ Δx = 0.1 × 1.4705 = 147 mm
    b2 = tf.build_truth([row(0, uv=(380.0, 240.0))], CAM)["blocks"][0]
    check("右移 60 px ⇒ x 增 147.05 mm",
          abs((b2["x"] - 0.400) - 0.14705) < 1e-4, (b2["x"] - 0.4) * 1000)
    check("右移 60 px 不动 y", abs(b2["y"] - 0.300) < 1e-9, b2["y"])

    # v+40px：cam y 向下、R 把 cam y 映到 base −y ⇒ base y 减小
    b3 = tf.build_truth([row(0, uv=(320.0, 280.0))], CAM)["blocks"][0]
    check("下移 40 px ⇒ y 减 98.03 mm",
          abs((0.300 - b3["y"]) - 0.09803) < 1e-4, (0.3 - b3["y"]) * 1000)

    # 台面抬高：平面更近 ⇒ 同一像素 x 偏移变小（几何正确性）
    b4 = tf.build_truth([row(0, uv=(380.0, 240.0))], CAM,
                        table_top=0.1)["blocks"][0]
    check("台面抬高 100 mm ⇒ 同一像素 Δx 变小",
          (b4["x"] - 0.400) < 0.14705 and (b4["x"] - 0.400) > 0.137,
          (b4["x"] - 0.4) * 1000)
    check("台面抬高 ⇒ top = 0.1 + 0.0295",
          abs(b4["top"] - 0.1295) < 1e-12, b4["top"])


def section_depth_immunity():
    print('\n[2] 位置对深度偏差免疫（深度只进 meas，不进几何）')
    a = tf.build_truth([row(0, uv=(355.0, 275.0), cx=400.0, cy=300.0,
                            top=32.0)], CAM)["blocks"][0]
    # 同一块，深度整体读远 3.3%（cx/cy/top 都往后推）
    b = tf.build_truth([row(0, uv=(355.0, 275.0), cx=413.2, cy=309.9,
                            top=78.9)], CAM)["blocks"][0]
    check("x 不受深度影响", a["x"] == b["x"], (a["x"], b["x"]))
    check("y 不受深度影响", a["y"] == b["y"], (a["y"], b["y"]))
    check("top 不受深度影响", a["top"] == b["top"], (a["top"], b["top"]))
    check("深度值留在 meas 里", abs(b["meas"]["depth_top_mm"] - 78.9) < 1e-9,
          b["meas"])


def section_reject():
    print('\n[3] 射线打不到平面 ⇒ 丢这一行（不是算出一个假点）')
    # 相机在台面下方、朝下看：lam ≤ 0
    below = dict(CAM, t_bc=np.array([0.4, 1.0, -0.5]))
    check("相机在平面下方 ⇒ 0 块",
          tf.build_truth([row(0)], below)["blocks"] == [])
    # 射线与平面平行：R 把光轴 (0,0,1) 映到 base (0,−1,0)（z 分量为 0）
    side = {"K": CAM["K"],
            "R_bc": np.array([[1.0, 0.0, 0.0],
                              [0.0, 0.0, -1.0],
                              [0.0, 1.0, 0.0]]),
            "t_bc": np.array([0.4, 1.0, 1.5])}
    check("射线平行于平面 ⇒ 0 块",
          tf.build_truth([row(0)], side)["blocks"] == [])
    out = tf.ray_plane_xy((320.0, 240.0), CAM["K"], CAM["R_bc"],
                          CAM["t_bc"], 0.0295)
    check("打得到时返回二元组", out is not None and len(out) == 2, out)


# ---------------------------------------------------------------- 拾取区/非积木
def section_zone():
    print('\n[4] 拾取区按射线后的 x,y 判；区外幻影与 class=unknown 都丢掉并计数')
    inside = tf.build_truth([row(0, uv=(320.0, 240.0))], CAM)
    check("区内的块保留", len(inside["blocks"]) == 1
          and inside["dropped_out_of_zone"] == 0, inside)
    # u 右移到 x ≈ 0.4+0.6 m，明显出区
    out = tf.build_truth([row(0, uv=(560.0, 240.0))], CAM)
    check("区外的检测被丢掉", out["blocks"] == [] and
          out["dropped_out_of_zone"] == 1, out)
    kept = tf.build_truth([row(0, uv=(560.0, 240.0))], CAM,
                          keep_outside=True)
    check("--keep-outside 时保留", len(kept["blocks"]) == 1,
          kept["blocks"])
    unk = tf.build_truth([row(0, cls="unknown")], CAM)
    check("class=unknown 默认丢掉（整簇无饱和像素 ⇒ 不是彩块）",
          unk["blocks"] == [] and unk["dropped_not_block"] == 1, unk)
    unk_kept = tf.build_truth([row(0, cls="unknown")], CAM,
                              drop_unknown=False)
    check("--keep-unknown 时保留", len(unk_kept["blocks"]) == 1, unk_kept)
    weak = tf.build_truth([row(0)], CAM, min_class_share=0.95)
    check("min_class_share 能滤掉弱票簇", weak["blocks"] == []
          and weak["dropped_not_block"] == 1, weak)


# ---------------------------------------------------------------- 过滤/编号
def section_filter():
    print('\n[5] 低处诊断簇不进真值；编号连续且按 xy 排序')
    rows = [row(0, uv=(320.0, 240.0)),                 # x=0.400
            row(1, uv=(260.0, 240.0)),                 # x=0.400−147=0.253
            row(2, uv=(300.0, 240.0), top=8.0, h=8.0, low=True)]
    t = tf.build_truth(rows, CAM)
    check("low=True 的簇被丢掉", len(t["blocks"]) == 2,
          [b["id"] for b in t["blocks"]])
    check("编号从 0 连续", [b["id"] for b in t["blocks"]] == [0, 1],
          [b["id"] for b in t["blocks"]])
    check("按 (x, y) 排序后 x=0.253 在前",
          t["blocks"][0]["x"] < t["blocks"][1]["x"],
          [round(b["x"], 4) for b in t["blocks"]])


# ---------------------------------------------------------------- 警告而非失败
def section_warn():
    print('\n[6] h 与边长对不上：不失败，只打 warn 字段')
    b = tf.build_truth([row(0, top=61.0, h=57.0)], CAM)["blocks"][0]
    check("warn_h_mm 存在且 ≈ 57 − 29.5", "warn_h_mm" in b
          and abs(b["warn_h_mm"] - 27.5) < 0.01, b.get("warn_h_mm"))
    check("top 仍按台面+size 算（不拿噪声高度改碰撞盒）",
          abs(b["top"] - 0.0295) < 1e-12, b["top"])
    ok = tf.build_truth([row(0, top=33.0, h=33.0)], CAM)["blocks"][0]
    check("对得上的块没有 warn", "warn_h_mm" not in ok)


# ---------------------------------------------------------------- 空真值
def section_empty():
    print('\n[7] 只有低处簇 ⇒ 0 块（调用方必须判失败）')
    t = tf.build_truth([row(0, top=8.0, h=8.0, low=True)], CAM)
    check("真值块数为 0", t["blocks"] == [], t["blocks"])


# ---------------------------------------------------------------- 多帧共识
def section_consensus():
    print('\n[8] 多帧共识：同块聚一簇、幻影按帧数剔、颜色多数、方块的 yaw 折 90°')
    def one(uv, cls="red", yaw=0.0):
        return tf.build_truth([row(0, uv=uv, cls=cls, yaw=yaw)], CAM)

    # 同一块在 4 帧里的抖动（±3 px ≈ ±7 mm，真机实测 6–9 mm）应聚成一块
    frames = [one((320.0, 240.0)), one((323.0, 241.0)),
              one((317.0, 239.0)), one((321.0, 238.0))]
    c = tf.consensus(frames, min_frames=3)
    check("抖动聚成 1 块", len(c["blocks"]) == 1, c["blocks"])
    b = c["blocks"][0]
    check("x 取中位（落在 4 帧范围内）",
          min(fr["blocks"][0]["x"] for fr in frames) - 1e-9 <= b["x"]
          <= max(fr["blocks"][0]["x"] for fr in frames) + 1e-9, b["x"])
    check("记录出现帧数与散布", b["meas"]["n_frames"] == 4
          and b["meas"]["x_spread_mm"] > 0, b["meas"])

    # 只在 1 帧出现的幻影（离主簇 73 mm）应被帧数门剔掉
    phantom = tf.build_truth([row(0, uv=(350.0, 240.0))], CAM,
                             keep_outside=True)
    c2 = tf.consensus(frames[:3] + [phantom], min_frames=3)
    check("1/4 帧的孤点被剔掉", len(c2["blocks"]) == 1
          and c2["blocks"][0]["meas"]["n_frames"] == 3, c2["blocks"])

    # 颜色多数票
    votes = [one((320.0, 240.0), cls="purple"),
             one((322.0, 240.0), cls="purple"),
             one((318.0, 240.0), cls="red")]
    c3 = tf.consensus(votes, min_frames=3)
    check("颜色取多数（purple 2 : red 1）", c3["blocks"][0]["color"] == "purple",
          c3["blocks"][0]["color"])

    # 方块 yaw：2°/92°/182° 是同一个盒子，折 90° 后中位应 ≈ 2°
    ys = [one((320.0, 240.0), yaw=2.0), one((320.0, 240.0), yaw=92.0),
          one((320.0, 240.0), yaw=182.0)]
    c4 = tf.consensus(ys, min_frames=3)
    check("yaw 折进 [0,90) 后 ≈ 2°", abs(((c4["blocks"][0]["yaw"] - 2.0 + 45)
                                           % 90) - 45) < 1e-6,
          c4["blocks"][0]["yaw"])


def main():
    section_ray_plane()
    section_depth_immunity()
    section_reject()
    section_zone()
    section_filter()
    section_warn()
    section_empty()
    section_consensus()
    print('\n%d PASS / %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('失败项：' + ', '.join(FAILS))
    return 0 if N_FAIL == 0 else 1


if __name__ == '__main__':
    sys.exit(main())

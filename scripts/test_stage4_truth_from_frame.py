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
    # 同一块，深度整体读远 3.3%（cx/cy/top 都往后推）。
    # h 是**相对拟合台面**的高度，深度整体读远时台面一起被推远 ⇒ h 不变
    # （h 若跟着 top 走，78.9 mm 会被 layer_count 读成三层塔）。
    b = tf.build_truth([row(0, uv=(355.0, 275.0), cx=413.2, cy=309.9,
                            top=78.9, h=32.0)], CAM)["blocks"][0]
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

    # 自定义保留区：E9 真机半段要让**垫面与北侧自由区**也进真值（块落到垫上以后
    # 它在拾取区外，按 ZONE_M 会被丢掉 ⇒ 后续场次看不见它，规划会撞）
    wide = (0.20, 0.80, -0.30, 0.65)
    edge = tf.build_truth([row(0, uv=(460.0, 240.0))], CAM,
                          zone_m=wide)          # x ≈ 0.743 m
    check("--zone 加宽后拾取区外的块保留", len(edge["blocks"]) == 1,
          edge["blocks"])
    check("真值里记下用的是哪个区", edge["zone_m"] == list(wide),
          edge["zone_m"])
    still_out = tf.build_truth([row(0, uv=(460.0, 240.0))], CAM)
    check("不给 --zone 时仍按拾取区丢", still_out["blocks"] == []
          and still_out["dropped_out_of_zone"] == 1, still_out)


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
    # 45 mm 既不是一块（29.5）也不是两层（59）—— 残差 14 mm > 0.4×29.5，
    # 所以不认成塔（35 mm 以上就够触发「不像单块」了，见 layer_count）
    t = tf.build_truth([row(0, top=45.0, h=45.0)], CAM)
    check("不像塔 ⇒ 仍是一块", len(t["blocks"]) == 1, len(t["blocks"]))
    b = t["blocks"][0]
    check("warn_h_mm 存在且 ≈ 45 − 29.5", "warn_h_mm" in b
          and abs(b["warn_h_mm"] - 15.5) < 0.01, b.get("warn_h_mm"))
    check("meas 里写明为什么没展开", "layer_note" in b["meas"], b["meas"])
    check("top 仍按台面+size 算（不拿噪声高度改碰撞盒）",
          abs(b["top"] - 0.0295) < 1e-12, b["top"])
    ok = tf.build_truth([row(0, top=33.0, h=33.0)], CAM)["blocks"][0]
    check("对得上的块没有 warn", "warn_h_mm" not in ok)


# ---------------------------------------------------------------- 塔的分层
def section_layers():
    print('\n[9] 塔：h 是塔高 ⇒ 展开成 n 块；顶层实测、下层 coaxial 推定')
    # 二层塔：h = 59.4 mm（真机实测 62.9，见 layer_count 注释）
    t = tf.build_truth([row(0, uv=(380.0, 240.0), top=59.4, h=59.4,
                            yaw=12.0)], CAM)
    check("展开成 2 块", len(t["blocks"]) == 2,
          [(b["layer"], b["top"]) for b in t["blocks"]])
    lo, hi = t["blocks"][0], t["blocks"][1]
    check("先塔底后塔顶（id 按层升）", lo["layer"] == 0 and hi["layer"] == 1,
          [lo["layer"], hi["layer"]])
    check("塔顶层 top = 台面 + 2×size",
          abs(hi["top"] - 0.059) < 1e-12, hi["top"])
    check("塔底 top = 台面 + size", abs(lo["top"] - 0.0295) < 1e-12, lo["top"])
    check("塔顶 z = 59 − size/2", abs(hi["z"] - 0.04425) < 1e-12, hi["z"])
    check("塔底 z = 29.5 − size/2", abs(lo["z"] - 0.01475) < 1e-12, lo["z"])
    check("两层 xy/yaw 相同（coaxial）",
          lo["x"] == hi["x"] and lo["y"] == hi["y"] and lo["yaw"] == hi["yaw"])
    check("塔顶标 measured_top_face",
          hi["meas"]["from"] == "measured_top_face", hi["meas"])
    check("塔底标 coaxial_with_top_face",
          lo["meas"]["from"] == "coaxial_with_top_face"
          and lo["meas"]["tower_layers"] == 2, lo["meas"])

    # 顶层的位置必须按 z=59 的平面解 —— 按 z=29.5 解会沿视线偏 20+ mm
    right = tf.ray_plane_xy((380.0, 240.0), CAM["K"], CAM["R_bc"],
                            CAM["t_bc"], 0.059)
    wrong = tf.ray_plane_xy((380.0, 240.0), CAM["K"], CAM["R_bc"],
                            CAM["t_bc"], 0.0295)
    check("塔顶 xy 用 z=59 平面（不是 z=29.5）",
          abs(hi["x"] - right[0]) < 1e-12
          and abs(hi["x"] - wrong[0]) > 1e-3,
          (hi["x"] - 0.400, right[0] - 0.400, wrong[0] - 0.400))

    # 三层塔
    t3 = tf.build_truth([row(0, h=88.5, top=88.5)], CAM)
    check("h=88.5 ⇒ 3 块，塔顶 top=88.5",
          len(t3["blocks"]) == 3
          and abs(t3["blocks"][-1]["top"] - 0.0885) < 1e-12,
          [(b["layer"], b["top"]) for b in t3["blocks"]])

    # 关掉就回到旧行为
    off = tf.build_truth([row(0, h=59.4, top=59.4)], CAM, layers=False)
    check("--no-layers ⇒ 1 块（旧行为）", len(off["blocks"]) == 1,
          len(off["blocks"]))

    # 单块不受影响
    one = tf.build_truth([row(0, h=30.0, top=30.0)], CAM)["blocks"]
    check("单块仍是 1 块、plane 不变", len(one) == 1
          and abs(one[0]["top"] - 0.0295) < 1e-12
          and one[0]["meas"]["from"] == "measured_top_face", one)

    # 共识：coaxial 两层的 xy 完全相同，只按 xy 聚会并成一块
    frames = [tf.build_truth([row(0, uv=(380.0, 240.0), h=59.4, top=59.4)],
                             CAM),
              tf.build_truth([row(0, uv=(381.0, 240.0), h=59.4, top=59.4)],
                             CAM),
              tf.build_truth([row(0, uv=(379.0, 240.0), h=59.4, top=59.4)],
                             CAM)]
    c = tf.consensus(frames, min_frames=3)
    check("共识后仍是 2 块（按三维距离分上下层）", len(c["blocks"]) == 2,
          [(b["layer"], round(b["top"], 4)) for b in c["blocks"]])
    check("共识保留层号与 top",
          [b["layer"] for b in c["blocks"]] == [0, 1]
          and abs(c["blocks"][1]["top"] - 0.059) < 1e-9,
          [(b["layer"], b["top"]) for b in c["blocks"]])


# ---------------------------------------------------------------- 空真值
def section_empty():
    print('\n[7] 只有低处簇 ⇒ 0 块（调用方必须判失败）')
    t = tf.build_truth([row(0, top=8.0, h=8.0, low=True)], CAM)
    check("真值块数为 0", t["blocks"] == [], t["blocks"])


# ---------------------------------------------------------------- 多帧共识
def section_consensus_parallel():
    """``--consensus`` 的进程池路径：结果必须与串行**逐字相同**。

    2026-11-12：20 帧串行 30.4 s，进程池 16 进程 7.0 s（本机 16 核）。
    加速的前提是不能改变语义 —— 每帧互相独立（只读自己的 PNG + tf json），
    所以只并行「逐帧量测」，聚类合并仍在一个进程里按原顺序做。

    这里不碰真帧：把 ``measure_dir`` / ``build_truth`` 换成确定性的假实现，
    让 6 帧给出同一个块（±2 mm 抖动）。串行与 3 进程的 JSON 必须一字不差。
    fork 起子进程（Linux 默认），假实现随父进程内存带过去。
    """
    print('\n[14] --consensus 并行：串行 vs 进程池结果逐字相同')
    import json
    import shutil
    import tempfile

    d = tempfile.mkdtemp(prefix='tfpar-')
    tmp = tempfile.mkdtemp(prefix='tfpar-out-')
    try:
        for i in range(6):
            open(os.path.join(d, 'color_%03d.png' % i), 'wb').close()

        real_m, real_b = tf.measure_dir, tf.build_truth

        def fake_measure(frame_dir, index=None, split=True, yaw_method='colour',
                         camera='wrist'):
            return {'idx': index}, [], 0.0

        def fake_build(rows, frame, table_top=0.0, block_size=0.0295,
                       batch='b', index=0, keep_outside=False,
                       drop_unknown=True, min_class_share=0.0, layers=True,
                       zone_m=None):
            i = frame['idx']
            return {'batch': batch, 'index': i, 'source': 'fake',
                    'block_size_m': block_size, 'table_top_m': table_top,
                    'zone_m': list(tf.sp.ZONE_M if zone_m is None else zone_m),
                    'dropped_out_of_zone': 0, 'dropped_not_block': 0,
                    'blocks': [{'x': 0.400 + 0.001 * i, 'y': 0.300,
                                'z': 0.0, 'top': block_size, 'layer': 0,
                                'color': 'red', 'size': block_size,
                                'yaw': 2.0 * i,
                                'meas': {'yaw_source': 'colour_rect',
                                         'from': 'fake', 'h_mm': 29.5}}]}

        tf.measure_dir, tf.build_truth = fake_measure, fake_build
        try:
            outs = {}
            for jobs in ('1', '3'):
                out = os.path.join(tmp, 'truth-j%s.json' % jobs)
                rc = tf.main(['--frame', d, '--consensus', '--min-frames', '3',
                              '--jobs', jobs, '--out', out])
                outs[jobs] = (rc, open(out, encoding='utf-8').read()
                              if os.path.exists(out) else None)
        finally:
            tf.measure_dir, tf.build_truth = real_m, real_b

        check("串行与 3 进程都返 0", outs['1'][0] == 0 and outs['3'][0] == 0,
              {k: v[0] for k, v in outs.items()})
        check("真值文件内容逐字相同", outs['1'][1] == outs['3'][1]
              and outs['1'][1] is not None,
              (outs['1'][1] or '')[:200])
        got = json.loads(outs['3'][1] or '{}')
        check("6 帧聚成 1 块（并行没丢帧）",
              len(got.get('blocks', [])) == 1
              and got['blocks'][0]['meas']['n_frames'] == 6, got.get('blocks'))
    finally:
        shutil.rmtree(d, ignore_errors=True)
        shutil.rmtree(tmp, ignore_errors=True)


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


def synth_frame(size_m=0.032, yaw_deg=0.0, with_depth=True):
    """正下方看的合成俯视帧：台面 + 一块正方形顶面（尺寸/朝向可控）。

    ``CAM`` 的相机 z→base −z、t_z=1.5 ⇒ 深度值 = 1.5 − base_z。方块顶面在
    base z = size_m，其余是台面（z=0）。
    """
    import cv2
    H, W = 480, 640
    depth = np.full((H, W), 1.5) if with_depth else np.zeros((H, W))
    if with_depth:
        half = 600.0 * size_m / 2.0          # 600 px/m ⇒ 边长像素的一半
        c, s = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
        poly = np.array([(320.0 + c * dx - s * dy, 240.0 + s * dx + c * dy)
                         for dx, dy in ((-half, -half), (half, -half),
                                        (half, half), (-half, half))],
                        np.float32)
        mask = np.zeros((H, W), np.uint8)
        cv2.fillPoly(mask, [poly.astype(np.int32)], 1)
        depth[mask > 0] = 1.5 - size_m
    return {"K": CAM["K"], "R_bc": CAM["R_bc"], "t_bc": CAM["t_bc"],
            "depth": depth, "color": np.zeros((H, W, 3), np.uint8)}


def section_colour_yaw():
    print('\n[10] yaw 口径：彩色像素掩码优先（PCA 只作掩码坏掉时的回退）')
    for ang in (0.0, 30.0, 62.0):
        fr = synth_frame(0.032, ang)
        rows = [row(0, top=32.0, h=32.0, yaw=150.0)]
        n_ok, n = tf.colour_yaw_rows(fr, rows)
        # 这张合成相机 R=diag(1,−1,−1)：图像里转 +φ，base 系是 −φ。
        want = (-ang) % 90.0
        check("φ=%.0f° 掩码 trusted" % ang, n_ok == 1 and n == 1, rows[0])
        y = rows[0]["yaw"] % 90.0
        check("φ=%.0f° yaw 测得 %.1f°（期望 %.1f°，±2°）" % (ang, y, want),
              abs(((y - want + 45) % 90) - 45) < 2.0, y)
        check("φ=%.0f° yaw_source=colour_rect" % ang,
              rows[0]["yaw_source"] == "colour_rect", rows[0]["yaw_source"])
        check("φ=%.0f° PCA 原值留在 yaw_pca" % ang,
              abs(rows[0]["yaw_pca"] - 150.0) < 1e-9, rows[0].get("yaw_pca"))

    # 掩码坏掉（这里：块心远在窗口外）⇒ 不失败，回退 PCA 并写明来源
    fr = synth_frame(0.032, 0.0)
    rows = [row(0, cx=100.0, cy=100.0, top=32.0, h=32.0, yaw=150.0)]
    tf.colour_yaw_rows(fr, rows)
    check("掩码不 trusted ⇒ yaw 保留 PCA", abs(rows[0]["yaw"] - 150.0) < 1e-9,
          rows[0]["yaw"])
    check("掩码不 trusted ⇒ yaw_source=depth_pca",
          rows[0]["yaw_source"] == "depth_pca" and bool(rows[0].get("yaw_note")),
          rows[0].get("yaw_note"))

    # build_truth 把来源写进顶层 meas（下层 coaxial 跟着顶层）
    fr = synth_frame(0.032, 20.0)
    rows = [row(0, top=59.0, h=59.0, yaw=1.0)]
    tf.colour_yaw_rows(fr, rows)
    t = tf.build_truth(rows, CAM)
    check("build_truth 顶层 meas.yaw_source=colour_rect",
          t["blocks"][0]["meas"]["yaw_source"] == "colour_rect",
          t["blocks"][0]["meas"].get("yaw_source"))
    check("build_truth 下层 meas 跟着顶层来源",
          t["blocks"][1]["meas"].get("yaw_source") == "colour_rect",
          t["blocks"][1]["meas"].get("yaw_source"))


def section_consensus_yaw():
    print('\n[11] 共识只用彩色 yaw：PCA 帧不得把共识角拖偏')
    def one(uv, yaw, src):
        r = row(0, uv=uv, yaw=yaw)
        r["yaw_source"] = src
        return tf.build_truth([r], CAM)

    frames = [one((320.0, 240.0), 3.0, "colour_rect"),
              one((321.0, 240.0), 5.0, "colour_rect"),
              one((319.0, 240.0), 70.0, "depth_pca")]
    c = tf.consensus(frames, min_frames=3)
    b = c["blocks"][0]
    check("yaw 取彩色帧（≈4°），不被 70° 的 PCA 拖走",
          abs(((b["yaw"] - 4.0 + 45) % 90) - 45) < 2.0, b["yaw"])
    check("共识 yaw_source=colour_rect",
          b["meas"]["yaw_source"] == "colour_rect", b["meas"])
    check("记录彩色帧数", b["meas"]["yaw_colour_frames"] == 2, b["meas"])

    c2 = tf.consensus([one((320.0, 240.0), 40.0, "depth_pca")], min_frames=1)
    check("一帧彩色都没有 ⇒ 回退 depth_pca",
          c2["blocks"][0]["meas"]["yaw_source"] == "depth_pca",
          c2["blocks"][0]["meas"].get("yaw_source"))


def section_camera():
    print('\n[12] --camera 只换 tf 链/K，几何一层不动')
    check("wrist 用 gc.TIP/gc.CHAIN",
          tf.CAMERAS["wrist"] == (tf.gc.TIP, tf.gc.CHAIN), tf.CAMERAS["wrist"])
    check("topcam 用 TOPCAM_TIP/TOPCAM_CHAIN",
          tf.CAMERAS["topcam"] == (tf.gc.TOPCAM_TIP, tf.gc.TOPCAM_CHAIN),
          tf.CAMERAS["topcam"])

    # 载荷参数真的转发到 load_frame（不打桩到 find_blocks；拿不到帧就说明转发对了）
    seen = {}

    class _Stop(Exception):
        pass

    def fake_load(frame_dir, tip=None, chain=None, idx=None):
        seen.update(tip=tip, chain=chain, idx=idx)
        raise _Stop

    real = tf.gc.load_frame
    tf.gc.load_frame = fake_load
    try:
        for cam, want in (("wrist", tf.CAMERAS["wrist"]),
                          ("topcam", tf.CAMERAS["topcam"])):
            seen.clear()
            try:
                tf.measure_dir("x", index=7, camera=cam)
            except _Stop:
                pass
            check("%s 转发 tip/chain/idx 到 load_frame" % cam,
                  (seen.get("tip"), seen.get("chain"), seen.get("idx"))
                  == (want[0], want[1], 7), seen)
        seen.clear()
        try:
            tf.measure_dir("x", index=7)        # 缺省 = wrist（v0.6）
        except _Stop:
            pass
        check("缺省相机 = wrist",
              (seen.get("tip"), seen.get("chain"))
              == (tf.gc.TIP, tf.gc.CHAIN), seen)
    finally:
        tf.gc.load_frame = real

    try:
        tf.measure_dir("x", camera="lidar")
        raised = None
    except ValueError as e:
        raised = str(e)
    check("未知相机名报错而不是静默用俯视",
          raised is not None and "lidar" in raised, raised)


def section_s11_free_colour_split():
    """s11 ``wrist-post/free`` 原始帧重放：两块不同色的接触块必须量出两块。

    这条是 §20.4 的现场回归 —— 修之前 ``split_top_face`` 的尺寸先验拒绝切
    （短边被边缘侵蚀压到 20.3 mm），两块量成一块、中心落在中间，判分报「缺块」。
    帧在仓外，没有就 SKIP（不假装通过）。
    """
    print('\n[13] s11 free 机位重放：6.5 mm 缝的不同色接触块')
    d = os.path.expanduser(
        '~/robotics/notes/ur5-vision-grasp/runs/2026-11-10-stage4-entry/'
        'e9-s11/wrist-post/free')
    if not os.path.isdir(d):
        print('  SKIP：缺帧 %s' % d)
        return
    frame, rows, _ = tf.measure_dir(d, index=0, camera='wrist')
    cut = [r for r in rows if r.get('split_src') == 'colour']
    check("拆出 2 个颜色块", len(cut) == 2,
          [(r.get('class'), r.get('split_src')) for r in rows])
    check("按长边顺序编号 split_i=0,1",
          sorted(r.get('split_i') for r in cut) == [0, 1],
          [r.get('split_i') for r in cut])
    t = tf.build_truth(rows, frame, keep_outside=True)
    blocks = t['blocks']
    check("真值 2 块（低分诊断簇不算块）", len(blocks) == 2,
          [(b['color'], round(b['x'], 4), round(b['y'], 4)) for b in blocks])
    if len(blocks) != 2:
        return
    want = {'orange': (0.47821, 0.53862), 'yellow': (0.51411, 0.53637)}
    err = max(max(abs(b['x'] - want[b['color']][0]),
                  abs(b['y'] - want[b['color']][1])) for b in blocks)
    # 3 mm 是口径差（彩色窗口质心 vs 顶面射线），不是量具误差；5 mm 留余量。
    check("与彩色窗口测量差 ≤ 5 mm", err <= 0.005, round(err * 1000.0, 2))
    if len(blocks) >= 2:
        sep = np.hypot(blocks[0]['x'] - blocks[1]['x'],
                       blocks[0]['y'] - blocks[1]['y'])
        check("两块中心距 35.5–37.5 mm（现场 35.97）",
              0.0355 <= sep <= 0.0375, round(sep * 1000.0, 2))


# ------------------------------------------------- 贴合对修复（S1 首轮）
def section_contact_pair_fixes():
    """台账第三百三十九轮修复的口径钉（S1 首轮两种阻塞）。

    ④ build_truth 丢「未拆混合簇」（票面双类 ≥25%，位置在两块中间——放进
    共识会把整簇心与两块心并成 Chimera）；① 颜色拆分触发门 1.5→1.1×块边
    （缩水掩码的贴合对不再被挡在门外）；② 质心钳制在 ``find_blocks`` 内，
    用 s1-1 / s1-4 真帧重放钉（``zcode_replay_pair_fix.py``），合成侧不重复。
    """
    print('\n[14] 贴合对修复：④ 未拆混合簇丢弃 / ① 触发门放宽')
    mixed = row(0, cls="cyan")
    mixed["class_share"] = 0.59
    mixed["mixed_pair_unsplit"] = True
    t = tf.build_truth([mixed], CAM)
    check("④ 标记行不进真值、计数 1",
          t["blocks"] == [] and t["dropped_unsplit_pair"] == 1,
          t["dropped_unsplit_pair"])
    clean = row(1, cls="cyan")
    clean["class_share"] = 0.95
    t2 = tf.build_truth([clean], CAM)
    check("④ 干净行照常进真值、计数 0",
          len(t2["blocks"]) == 1 and t2["dropped_unsplit_pair"] == 0,
          t2["dropped_unsplit_pair"])

    # ① 触发门：合成两色顶面点集，min-area rect 长边 40 mm —— 旧门
    # （1.5×29.5 = 44.25）拒拆、新门（1.1×29.5 = 32.45）放行。
    import graspnet_candidates as gc
    n_half = 120
    rng = np.random.default_rng(7)
    xs = np.concatenate([rng.normal(0.0, 0.004, n_half),
                         rng.normal(0.020, 0.004, n_half)])
    F = np.stack([xs, np.zeros(2 * n_half), np.full(2 * n_half, 0.03)], axis=1)
    # 色板两格：cyan hue≈193°、yellowgreen hue≈54°（对齐 MEASURED_HUE_DEG）。
    color = np.array([[[0, 200, 255], [201, 180, 0]]], dtype=np.uint8)
    qpix = np.stack([np.concatenate([np.zeros(n_half, int),
                                     np.ones(n_half, int)]),
                     np.zeros(2 * n_half, int)], axis=1)
    rect = {"ext_mm": [40.0, 20.0], "yaw_deg": 0.0, "fill": 1.0}
    pieces = gc.split_top_face_by_colour(F, qpix, color, rect)
    check("① 长边 40 mm（<1.5×、≥1.1×块边）拆出 2 片",
          len(pieces) == 2, [len(p) for p in pieces])
    if len(pieces) == 2:
        med = sorted(float(np.median(F[p_, 0])) for p_ in pieces)
        check("① 两片沿长轴分开 ~20 mm（≥ sep 门 13.3 mm）",
              abs(med[1] - med[0]) > 0.015, round((med[1] - med[0]) * 1000, 1))
    old = gc.COLOUR_SPLIT_GATE
    try:
        gc.COLOUR_SPLIT_GATE = 1.5
        pieces_old = gc.split_top_face_by_colour(F, qpix, color, rect)
    finally:
        gc.COLOUR_SPLIT_GATE = old
    check("① 同一输入在旧门（1.5×）下不拆（A/B 口径钉）",
          len(pieces_old) == 1, [len(p) for p in pieces_old])


def main():
    section_ray_plane()
    section_depth_immunity()
    section_reject()
    section_zone()
    section_filter()
    section_warn()
    section_empty()
    section_consensus()
    section_consensus_parallel()
    section_layers()
    section_colour_yaw()
    section_consensus_yaw()
    section_camera()
    section_s11_free_colour_split()
    section_contact_pair_fixes()
    print('\n%d PASS / %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('失败项：' + ', '.join(FAILS))
    return 0 if N_FAIL == 0 else 1


if __name__ == '__main__':
    sys.exit(main())

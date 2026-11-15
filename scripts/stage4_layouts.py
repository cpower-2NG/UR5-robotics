#!/usr/bin/env python3
"""阶段 4 正式批次（S0 首批）的**布局唯一出处**：块的颜色 / 位置 / 朝向。

为什么单独一个文件
------------------
「布局即真值」：正式轮不跑前实测块位，layout manifest 就是判分用的真值。
那就必须有**一份**写得明明白白的表，而不是每轮现场从 `gen_scene` 的随机种子
重算一遍 —— 重算当然会得到同样的数（种子固定），但没人能在纸上核对它。
这里把选中的三个布局逐块写死，并留一条测试：与 `gen_scene.make_layout('S0', n)`
逐块对拍（误差 < 0.05 mm / 0.05°）。两边一旦漂移，测试直接红，不会出现
「纸上摆 471.2、仿真里是别的数」这种事。

选中的三个布局（用户 2026-11-11 拍板，五选三）
----------------------------------------------
* S0:1 —— 散布最开（268×177 mm，最小间距 95.1 mm）；
* S0:2 —— **唯一含红块**的布局（红块只在这一轮用上）；
* S0:3 —— 次分散（241×151 mm），与上面两个合起来覆盖全部 7 色。
排除 S0:4：最小间距 60.4 mm，贴 60 mm 护栏。

S1/S2 选中布局（2026-11-15，zcode 按契约 v0.8 誊入）
----------------------------------------------------
* S1（异色接触）取 **1 / 3 / 4**：贴合对 = orange+red / orange+tan /
  yellowgreen+cyan，三对不同色对、方位分散。排除 S1:2 —— 其贴合对是
  tan+tan **同色**（``_add_contact_pair`` 两次独立取色撞色），与批次定义
  「异色接触」不符；S1:5（cyan+yellow）备用。
* S2（同色接触）取 **1 / 2 / 5**：贴合对 = red / tan / yellow，三种色对、
  位置西/南/东分布。排除 S2:3 / S2:4 —— 色对与已选重复。
* 坐标与 ``gen_scene.make_layout`` 逐位对拍（``compare_with_gen_scene``，
  容差 0.05 mm / 0.05°；表存 3 位小数避免 1 位小数舍入踩线）。

坐标口径：**base 系毫米 / 度**，块心；边长 29.5 mm，台面上（layer 0）。
格位映射（块 id 升序 → 格）在 `stage4_clear_model.assign_cells` /
``DEFAULT_CELLS_BY_BATCH``（S1/S2 六块六格），不在本文件。
**2026-11-15 契约 v0.8**：真机改自由摆位 —— 本表只是**身份/颜色集合先验**
与仿真对拍资产，不再要求现场照表摆块；S0_ROUNDS 的「三布局×2」连击排期
随 v0.8 集中批次轮一并退役（常量保留只作记录）。
"""
import math
import os
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import stage4_clear_model as cm                                  # noqa: E402

#: 正式批次的积木边长（米）。与 gen_scene.BLOCK_M 同源（测试对拍）。
BLOCK_SIZE_M = float(cm.BLOCK_M)

#: S0 选中的三个布局：(id, 颜色, x_mm, y_mm, yaw_deg)。id 与 gen_scene 一致（0–4）。
S0_SELECTED = {
    1: (
        (0, "yellowgreen", 552.7, 197.8, 40.7),
        (1, "yellow", 292.7, 375.0, 18.2),
        (2, "cyan", 415.5, 290.8, 16.3),
        (3, "orange", 507.8, 313.8, 100.5),
        (4, "purple", 284.4, 215.9, 126.9),
    ),
    2: (
        (0, "yellowgreen", 471.2, 290.4, 51.5),
        (1, "yellow", 394.3, 323.0, 5.2),
        (2, "red", 371.4, 228.4, 62.9),
        (3, "orange", 281.8, 371.9, 56.7),
        (4, "tan", 447.9, 194.8, 158.5),
    ),
    3: (
        (0, "purple", 503.6, 292.8, 93.9),
        (1, "yellow", 548.2, 232.8, 101.7),
        (2, "yellowgreen", 391.8, 361.3, 109.6),
        (3, "tan", 311.1, 273.2, 50.4),
        (4, "orange", 551.9, 384.1, 74.9),
    ),
}

#: 正式轮次的顺序：三个布局各跑两次（S0:1 ×2 → S0:2 ×2 → S0:3 ×2）。
#: **2026-11-15 v0.8 起退役**（连击 streak 废止，改集中批次轮）；保留只作记录。
S0_ROUNDS = ((1, 1), (1, 2), (2, 1), (2, 2), (3, 1), (3, 2))

#: S1（异色接触）选中布局：贴合对全异色（orange+red / orange+tan /
#: yellowgreen+cyan）。排除 S1:2（贴合对 tan+tan 同色，不符批次定义）。
S1_SELECTED = {
    1: (
        (0, "orange", 437.309, 396.555, 174.766),
        (1, "red", 407.932, 399.246, 174.766),
        (2, "tan", 386.366, 263.554, 161.481),
        (3, "yellowgreen", 343.969, 316.381, 40.317),
        (4, "purple", 389.307, 196.443, 15.555),
        (5, "yellow", 280.606, 382.216, 46.488),
    ),
    3: (
        (0, "orange", 339.030, 343.526, 121.327),
        (1, "tan", 323.692, 368.726, 121.327),
        (2, "yellowgreen", 429.408, 194.457, 163.236),
        (3, "cyan", 447.120, 308.301, 34.306),
        (4, "yellow", 514.543, 244.727, 5.834),
        (5, "red", 298.074, 267.159, 125.569),
    ),
    4: (
        (0, "yellowgreen", 499.125, 365.945, 60.160),
        (1, "cyan", 513.804, 391.534, 60.160),
        (2, "tan", 306.541, 212.362, 58.394),
        (3, "orange", 344.342, 317.942, 157.947),
        (4, "yellow", 555.327, 272.121, 167.945),
        (5, "red", 388.014, 190.413, 139.162),
    ),
}

#: S2（同色接触）选中布局：贴合对 red / tan / yellow（三色对、西/南/东分布）。
#: 排除 S2:3 / S2:4 —— 色对与已选重复。
S2_SELECTED = {
    1: (
        (0, "red", 291.983, 244.345, 36.343),
        (1, "red", 315.745, 261.828, 36.343),
        (2, "yellowgreen", 528.515, 188.792, 1.587),
        (3, "purple", 479.602, 328.801, 120.609),
        (4, "cyan", 543.862, 308.072, 156.089),
        (5, "tan", 405.804, 265.815, 88.517),
    ),
    2: (
        (0, "tan", 396.957, 344.334, 70.655),
        (1, "tan", 406.729, 372.168, 70.655),
        (2, "purple", 300.017, 401.172, 112.322),
        (3, "orange", 307.141, 281.263, 69.233),
        (4, "yellowgreen", 472.555, 276.950, 172.851),
        (5, "red", 399.510, 272.017, 45.588),
    ),
    5: (
        (0, "yellow", 463.645, 362.537, 86.328),
        (1, "yellow", 465.534, 391.977, 86.328),
        (2, "cyan", 515.758, 273.472, 143.755),
        (3, "orange", 380.108, 360.210, 139.409),
        (4, "yellowgreen", 398.177, 226.982, 153.000),
        (5, "red", 501.356, 206.305, 63.241),
    ),
}

BATCHES = {"S0": S0_SELECTED, "S1": S1_SELECTED, "S2": S2_SELECTED}


def layout_blocks(batch, layout):
    """布局 → 真值块（米、base 系；字段与 ``gen_scene`` 的 scene_truth 相同）。

    ``z`` 是块心离台面高度（layer 0 ⇒ 边长/2），``top`` 是块顶高度。
    """
    if batch not in BATCHES:
        raise ValueError("未知正式批次 %r（可选 %s）" % (batch, sorted(BATCHES)))
    table = BATCHES[batch]
    if layout not in table:
        raise ValueError("%s 没有选中的布局 %r（选中 %s）"
                         % (batch, layout, sorted(table)))
    size = BLOCK_SIZE_M
    blocks = []
    for bid, color, x_mm, y_mm, yaw in table[layout]:
        blocks.append({
            "id": int(bid),
            "color": str(color),
            "layer": 0,
            "size": size,
            "x": float(x_mm) / 1000.0,
            "y": float(y_mm) / 1000.0,
            "z": size / 2.0,
            "top": size,
            "yaw": float(yaw),
        })
    return blocks


def scene_truth(batch, layout, pad_pose=None):
    """探针 / 判分共用的 scene_truth 文档（形状同 ``gen_scene.write_scene``）。"""
    blocks = layout_blocks(batch, layout)
    doc = {
        "batch": batch,
        "index": int(layout),
        "block_size_m": BLOCK_SIZE_M,
        "zone_m": list(cm.PICK_ZONE),
        "blocks": blocks,
    }
    if pad_pose is not None:
        doc["pad_pose"] = pad_pose
    return doc


#: 同层间距带（×BLOCK）。与 ``gen_scene.validate_layout`` 同口径：
#: <0.72 ⇒ 重叠（硬红）；[0.72, 1.02] ⇒ **贴合带**（S0 禁止；S1/S2 允许，
#: 全表至多 1 对）；≥1.02×BLOCK ⇒ 非接触（表级自检到「不重叠」为止 ——
#: ≥60 mm 的安全间距是 S0 的表级要求；S1/S2 的邻块间隙真门在探针 OBB，
#: 表级不再重复设 60 mm 线，否则 gen_scene 的 59.5 mm 合法布局会被误拒）。
_CONTACT_LO = 0.72
_CONTACT_HI = 1.02


def validate_blocks(batch, blocks, min_gap_m=0.060):
    """布局自检的**判定主体**（吃块表）。

    与 ``validate_layout`` 的分工：那里查表取标称块，这里做全部判定 ——
    对临时块表（测试变异 / 将来的实测自检）也能用同一套口径。
    """
    errs = []
    x0, x1, y0, y1 = cm.PICK_ZONE
    m = BLOCK_SIZE_M / 2.0
    for b in blocks:
        if not (x0 + m <= b["x"] <= x1 - m and y0 + m <= b["y"] <= y1 - m):
            errs.append("块 %d 出了拾取区（%.1f, %.1f mm）"
                        % (b["id"], b["x"] * 1000, b["y"] * 1000))
    contacts = 0
    for i in range(len(blocks)):
        for j in range(i + 1, len(blocks)):
            a, c = blocks[i], blocks[j]
            if abs(a.get("layer", 0) - c.get("layer", 0)) >= 1:
                continue          # 不同层竖向不构成碰撞（gen_scene 同口径）
            d = math.hypot(a["x"] - c["x"], a["y"] - c["y"])
            if d < _CONTACT_LO * BLOCK_SIZE_M:
                errs.append("块 %d/%d 重叠（中心距 %.1f mm < %.1f mm）"
                            % (a["id"], c["id"], d * 1000,
                               _CONTACT_LO * BLOCK_SIZE_M * 1000))
            elif d <= _CONTACT_HI * BLOCK_SIZE_M:
                if batch == "S0":
                    errs.append("S0 出现接触块 %d/%d（%.1f mm）"
                                % (a["id"], c["id"], d * 1000))
                else:
                    contacts += 1
            elif batch == "S0" and d < min_gap_m:
                errs.append("块 %d/%d 中心距 %.1f mm < %.0f mm"
                            % (a["id"], c["id"], d * 1000, min_gap_m * 1000))
    if batch in ("S1", "S2") and contacts != 1:
        errs.append("%s 应有恰好 1 个贴合对，实得 %d 个（场景定义：一个贴合对"
                    " + 其余摆散）" % (batch, contacts))
    return errs


def validate_layout(batch, layout, min_gap_m=0.060):
    """布局自检：块心离拾取区边界留半块 + 同层间距按**批次**判。

    S0（无接触）：两两 ≥ ``min_gap_m``，出现贴合即红（原行为不变）。
    S1/S2（接触批）：贴合带内允许**恰好 1 对**（场景定义 = 一个贴合对 +
    其余摆散）；重叠带一律红；更近的算重叠、更远的不再设 60 mm 线
    （真门在探针，理由见 ``_CONTACT_LO/_CONTACT_HI`` 注释）。
    判定主体在 ``validate_blocks``（吃块表，测试可对临时块表用同一口径）。
    """
    return validate_blocks(batch, layout_blocks(batch, layout), min_gap_m)


def contact_pairs(blocks):
    """块表里同层**贴合对**的 ``(id_a, id_b)`` 列表（中心距 ∈ 贴合带）。

    吃块列表而不是 (batch, layout)——对标称表与跑前实测真值同样可用
    （给现场摆块提示用：S1「X 与 Y 两块贴住」/ S2「同色 X 两块贴住」）。
    """
    out = []
    for i in range(len(blocks)):
        for j in range(i + 1, len(blocks)):
            a, c = blocks[i], blocks[j]
            if abs(a.get("layer", 0) - c.get("layer", 0)) >= 1:
                continue
            d = math.hypot(a["x"] - c["x"], a["y"] - c["y"])
            if _CONTACT_LO * BLOCK_SIZE_M <= d <= _CONTACT_HI * BLOCK_SIZE_M:
                out.append((a["id"], c["id"]))
    return out


def compare_with_gen_scene(batch="S0"):
    """与 ``gen_scene.make_layout`` 逐块对拍；返回差异列表（空 = 一致）。"""
    import gen_scene as gs                                      # cm 已把它放上 sys.path

    diffs = []
    for layout in sorted(BATCHES[batch]):
        theirs = {int(b["id"]): b for b in gs.make_layout(batch, layout)}
        mine = {b["id"]: b for b in layout_blocks(batch, layout)}
        if set(theirs) != set(mine):
            diffs.append("%s:%d 块 id 集合不同：%s vs %s"
                         % (batch, layout, sorted(theirs), sorted(mine)))
            continue
        for bid in sorted(mine):
            a, b = mine[bid], theirs[bid]
            if a["color"] != b["color"]:
                diffs.append("%s:%d 块 %d 颜色 %s vs %s"
                             % (batch, layout, bid, a["color"], b["color"]))
            dx = abs(a["x"] - float(b["x"])) * 1000.0
            dy = abs(a["y"] - float(b["y"])) * 1000.0
            dyaw = abs(a["yaw"] - float(b["yaw"]))
            if dx > 0.05 or dy > 0.05 or dyaw > 0.05:
                diffs.append(
                    "%s:%d 块 %d 位置差 (%.3f, %.3f) mm / yaw %.3f°"
                    % (batch, layout, bid, dx, dy, dyaw))
    return diffs


def main(argv=None):
    import argparse
    import json

    ap = argparse.ArgumentParser(description="S0 布局表（查看 / 自检）")
    ap.add_argument("--batch", default="S0", choices=sorted(BATCHES))
    ap.add_argument("--layout", type=int, default=None)
    ap.add_argument("--json", action="store_true", help="输出 scene_truth JSON")
    ap.add_argument("--check", action="store_true",
                    help="与 gen_scene 对拍 + 布局自检；不一致 ⇒ 退出 1")
    args = ap.parse_args(argv)

    if args.check:
        diffs = compare_with_gen_scene(args.batch)
        for layout in sorted(BATCHES[args.batch]):
            diffs += validate_layout(args.batch, layout)
        if diffs:
            print("!! 布局自检不通过：")
            for d in diffs:
                print("   " + d)
            return 1
        print("布局自检通过：%d 个布局，与 gen_scene 逐块一致%s"
              % (len(BATCHES[args.batch]),
                 "、间距 ≥60 mm" if args.batch == "S0"
                 else "、贴合对 = 恰好 1 对（接触批口径）"))
        return 0

    if args.layout is not None:
        layout = args.layout
    else:
        rounds = S0_ROUNDS if args.batch == "S0" else None
        print("%s 选中布局：%s%s"
              % (args.batch, sorted(BATCHES[args.batch]),
                 ("；轮次顺序（v0.8 前旧口径）：%s"
                  % ", ".join("%s:%d-r%d" % (args.batch, l, r)
                              for l, r in rounds)) if rounds else
                 "（v0.8：真机自由摆位，本表只是身份/颜色集合先验）"))
        return 0
    doc = scene_truth(args.batch, layout)
    if args.json:
        print(json.dumps(doc, ensure_ascii=False, indent=1))
    else:
        for b in doc["blocks"]:
            print("  块 %d  %-12s x=%7.1f y=%7.1f yaw=%6.1f°"
                  % (b["id"], b["color"], b["x"] * 1000.0,
                     b["y"] * 1000.0, b["yaw"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())

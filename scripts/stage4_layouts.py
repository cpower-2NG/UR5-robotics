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

坐标口径：**base 系毫米 / 度**，块心；边长 29.5 mm，台面上（layer 0）。
格位映射（块 id 升序 → 8、6、4、2、0）在 `stage4_clear_model.assign_cells`，
不在本文件 —— 那是「怎么放」，这里是「真值在哪」。
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
S0_ROUNDS = ((1, 1), (1, 2), (2, 1), (2, 2), (3, 1), (3, 2))

BATCHES = {"S0": S0_SELECTED}


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


def validate_layout(batch, layout, min_gap_m=0.060):
    """布局自检：块心离拾取区边界留半块、同层两两间距 ≥ ``min_gap_m``。"""
    blocks = layout_blocks(batch, layout)
    errs = []
    x0, x1, y0, y1 = cm.PICK_ZONE
    m = BLOCK_SIZE_M / 2.0
    for b in blocks:
        if not (x0 + m <= b["x"] <= x1 - m and y0 + m <= b["y"] <= y1 - m):
            errs.append("块 %d 出了拾取区（%.1f, %.1f mm）"
                        % (b["id"], b["x"] * 1000, b["y"] * 1000))
    for i in range(len(blocks)):
        for j in range(i + 1, len(blocks)):
            a, c = blocks[i], blocks[j]
            d = math.hypot(a["x"] - c["x"], a["y"] - c["y"])
            if d < min_gap_m:
                errs.append("块 %d/%d 中心距 %.1f mm < %.0f mm"
                            % (a["id"], c["id"], d * 1000, min_gap_m * 1000))
    return errs


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
        print("布局自检通过：%d 个布局，与 gen_scene 逐块一致、间距 ≥60 mm"
              % len(BATCHES[args.batch]))
        return 0

    if args.layout is not None:
        layout = args.layout
    else:
        print("%s 选中布局：%s；轮次顺序：%s"
              % (args.batch, sorted(BATCHES[args.batch]),
                 ", ".join("%s:%d-r%d" % (args.batch, l, r)
                           for l, r in S0_ROUNDS)))
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

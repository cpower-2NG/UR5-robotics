#!/usr/bin/env python3
"""O1 评估器自检：mask IoU、一对一分配、按批合并与判据。

不需要仿真帧：用合成深度图（相机与 base 同系、正上方看）检查
``gt_top_masks`` 的足迹/高度/yaw 判据，用合成 mask 检查分配规则。
跑法：
    python3 scripts/test_stage4_o1_eval.py
"""
import os
import sys

import numpy as np

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import stage4_o1_eval as o1                                     # noqa: E402

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
        print('  FAIL  %s%s' % (name, '   ← %s' % (detail,) if detail else ''))


def flat_frame(h=100, w=100, z=1.0):
    """相机系 = base 系，深度全 1.0 m；fx=1000 ⇒ 1 px = 1 mm。"""
    return {"depth": np.full((h, w), z, dtype=np.float64),
            "K": np.array([[1000.0, 0.0, 0.0],
                           [0.0, 1000.0, 0.0],
                           [0.0, 0.0, 1.0]]),
            "R_bc": np.eye(3), "t_bc": np.zeros(3)}


def block(bid, x_mm, y_mm, yaw=0.0, size=0.02, top=1.0):
    return {"id": bid, "color": "red", "x": x_mm / 1000.0,
            "y": y_mm / 1000.0, "yaw": yaw, "size": size, "top": top,
            "z": top - size / 2.0}


def section_iou():
    print('\n[1] mask IoU')
    a = np.zeros((10, 10), dtype=bool)
    a[2:5, 2:5] = True
    b = np.zeros((10, 10), dtype=bool)
    b[2:5, 2:5] = True
    c = np.zeros((10, 10), dtype=bool)
    c[5:8, 5:8] = True
    d = np.zeros((10, 10), dtype=bool)
    d[2:5, 2:7] = True
    check('同一 mask IoU=1', o1.iou(a, b) == 1.0)
    check('不相交 IoU=0', o1.iou(a, c) == 0.0)
    check('部分重叠 = 9/15', abs(o1.iou(a, d) - 9.0 / 15.0) < 1e-9,
          o1.iou(a, d))
    check('空 mask 不炸（union=0 ⇒ 0）', o1.iou(np.zeros((3, 3), bool),
                                              np.zeros((3, 3), bool)) == 0.0)


def section_gt_masks():
    print('\n[2] 真值顶面 mask：足迹 / 高度 / yaw')
    fr = flat_frame()
    # 块心放在 50.5 mm ⇒ 20 mm 块正好盖住像素 41..60（20×20 px），
    # 边界落在半像素上，不会因浮点比较丢一行。
    truth = {"blocks": [block(0, 50.5, 50.5)]}
    g = o1.gt_top_masks(fr, truth)[0]
    check('轴对齐 20 mm 块 = 400 px', int(g.sum()) == 400, int(g.sum()))
    ys, xs = np.nonzero(g)
    check('质心在 (50.5,50.5)', abs(xs.mean() - 50.5) < 0.2
          and abs(ys.mean() - 50.5) < 0.2,
          (xs.mean(), ys.mean()))

    g45 = o1.gt_top_masks(fr, {"blocks": [block(0, 50.5, 50.5, yaw=45.0)]})[0]
    check('旋转 45° 面积仍 ≈400 px', 350 <= g45.sum() <= 450,
          int(g45.sum()))
    check('旋转确实改变了像素集（不是把 yaw 当装饰）',
          bool((g45 != g).any()))
    ys45, xs45 = np.nonzero(g45)
    check('旋转后质心仍在 (50.5,50.5)', abs(xs45.mean() - 50.5) < 0.8
          and abs(ys45.mean() - 50.5) < 0.8)

    far = o1.gt_top_masks(fr, {"blocks": [block(0, 50.5, 50.5, top=1.1)]},
                          dz_top=0.05, dz_bot=0.05)[0]
    check('高度带不含顶面 ⇒ 空 mask', int(far.sum()) == 0, int(far.sum()))

    off = o1.gt_top_masks(fr, {"blocks": [block(0, 150, 150)]})[0]
    check('足迹在画面外 ⇒ 空 mask', int(off.sum()) == 0)


def _mask(shape, ys, xs):
    m = np.zeros(shape, dtype=bool)
    m[ys, xs] = True
    return m


def section_assign():
    print('\n[3] 一对一分配：并簇不能代表任何一块')
    shape = (20, 20)
    gts = {0: _mask(shape, slice(2, 10), slice(2, 10)),
           1: _mask(shape, slice(2, 10), slice(10, 18))}
    # 两个干净实例
    pool = [{"idx": 0}, {"idx": 1}]
    masks = {0: _mask(shape, slice(2, 10), slice(2, 10)),
             1: _mask(shape, slice(2, 10), slice(10, 18))}
    rep, pairs = o1.assign(pool, masks, gts)
    check('两块都对上', rep == {0: 0, 1: 1}, (rep, pairs))

    # 合并簇：一簇横跨两块 → 与每块 IoU 都 < 0.75
    merged = {0: _mask(shape, slice(2, 10), slice(2, 18))}
    rep, pairs = o1.assign([{"idx": 0}], merged, gts)
    check('合并簇 IoU≈0.5 < 0.75 ⇒ 不分配', rep == {} and not pairs,
          (rep, pairs))

    # 多检：一个真值 + 一个无关实例
    gts1 = {7: _mask(shape, slice(2, 10), slice(2, 10))}
    masks2 = {0: _mask(shape, slice(2, 10), slice(2, 10)),
              1: _mask(shape, slice(14, 19), slice(14, 19))}
    rep, pairs = o1.assign([{"idx": 0}, {"idx": 1}], masks2, gts1)
    check('多检实例不抢真值', rep == {0: 7}, (rep, pairs))


def section_pool_judge():
    print('\n[4] 按批合并与判据')
    def frame_res(batch, correct, n_cmd, misbind=0, iou=0.9):
        return {"batch": batch, "dir": batch, "n_truth": 2,
                "n_commands": n_cmd, "n_skipped_not_unique": 0,
                "n_indep_disagree": 0,
                "variants": {v: {
                    "n_instances": 2, "n_matched": 2, "n_false_pos": 0,
                    "iou_best": {"0": iou, "1": iou}, "iou_mean": iou,
                    "iou_median": iou, "iou_ge_075": 2, "merged": 0,
                    "missed": 0, "counts": {"ok": n_cmd}, "n_commands": n_cmd,
                    "correct": correct, "ok_correct": correct,
                    "misbind": misbind, "id_rate": correct / n_cmd,
                    "auto_top1_rate": correct / n_cmd, "commands": [],
                } for v in o1.VARIANTS}}
    pooled = o1.pool_results([frame_res("S1", 9, 10), frame_res("S1", 9, 10)])
    row = pooled[0]
    check('两帧命令数合并（20 不是 10）', row["n_commands"] == 20,
          row["n_commands"])
    check('ID 率按计数合并 = 90%',
          abs(row["variants"]["multi"]["id_rate"] - 0.9) < 1e-9,
          row["variants"]["multi"]["id_rate"])
    check('S1 90% ⇒ 通过', o1.judge_batch(row, "multi")[0])

    bad = o1.pool_results([frame_res("S2", 8, 10)])[0]
    ok, fails = o1.judge_batch(bad, "multi")
    check('S2 80% ⇒ 不过', not ok and any("指定 ID" in f for f in fails), fails)

    bad2 = o1.pool_results([frame_res("S2", 10, 10, misbind=1)])[0]
    ok, fails = o1.judge_batch(bad2, "multi")
    check('误配 1 次 ⇒ 不过', not ok and any("误配" in f for f in fails), fails)

    s3 = o1.pool_results([frame_res("S3", 0, 10)])[0]
    check('S3 只报不判', o1.judge_batch(s3, "multi")[0])

    # 目标块顶面被上层压到只剩薄边（重度遮挡）⇒ 命令不进「无遮挡」分母
    inv = frame_res("S3", 5, 10)
    inv["n_gt_occluded"] = 1
    inv["variants"]["multi"].update({
        "correct_unoccluded": 5, "n_commands_unoccluded": 5,
        "n_commands_occluded": 5, "misbind_occluded": 0})
    row = o1.pool_results([inv])[0]
    check('遮挡块不进「无遮挡 top-1」分母',
          row["variants"]["multi"]["id_rate_unoccluded"] == 1.0,
          row["variants"]["multi"]["id_rate_unoccluded"])
    check('遮挡目标的错选单独记，不混进误配',
          row["variants"]["multi"]["misbind"] == 0
          and row["n_gt_occluded"] == 1,
          (row["variants"]["multi"]["misbind"], row["n_gt_occluded"]))


def main():
    section_iou()
    section_gt_masks()
    section_assign()
    section_pool_judge()
    print('\n%d PASS / %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('失败：' + '；'.join(FAILS))
    return 0 if N_FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())

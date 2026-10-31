#!/usr/bin/env python3
"""stage3_measure 的纯逻辑测试：塔窗选取、融合、一致性、**表面语义**。

跑法：
  python3 scripts/test_stage3_measure.py

不碰相机、不碰板子：帧用假的 ``measure_frame`` 替身喂进去。

为什么单独盯 ``surface_est_mm`` / ``next_surface_est_mm``
────────────────────────────────────────────────────────
``surface_est = top − 29.5 − bias`` 是**本层块脚下的面**；下一层要放的
面是 ``top − bias``，两者差整整一块（29.5 mm）。2026-10-31 首版编排脚本
就拿前者当下一层表面 —— 第 2 层会按垫面高度放。这里的用例就是这个缺陷的哨。
"""
import os
import sys

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np                                                # noqa: E402

import stage3_measure as m3                                       # noqa: E402

CASES = []


def case(fn):
    CASES.append(fn)
    return fn


def assert_close(a, b, tol=1e-6, why=""):
    assert abs(float(a) - float(b)) <= tol, \
        "%s: %r vs %r（容差 %g）" % (why or "数值", a, b, tol)


GRID = {
    "version": 1, "kind": "stage2-grid", "pitch_mm": 60.0,
    "cells": [{"idx": i, "row": i // 3, "col": i % 3,
               "center_base_mm": [422.65 + (i % 3 - 1) * 60.0,
                                  -114.86 + (i // 3 - 1) * 60.0]}
              for i in range(9)],
    "grid_center_base_mm": [422.65, -114.86],
    "grid_yaw_base_deg": 179.809,
    "T_base_pad": [[1, 0, 0, 0.42265], [0, 1, 0, -0.11486],
                   [0, 0, 1, 0.002], [0, 0, 0, 1]],
}


def blk(idx, cx, cy, top, h, low=False, yaw=0.0, trusted=True, cls="cyan"):
    return {"idx": idx, "cx": cx, "cy": cy, "top": top, "h": h,
            "low": low, "yaw_rect": yaw, "yaw_trusted": trusted, "class": cls}


# ---------------------------------------------------------------- 网格
@case
def test_grid_center_and_pad():
    assert m3.grid_center(GRID, 4) == [422.65, -114.86]
    assert_close(m3.pad_z_mm(GRID), 2.0)
    try:
        m3.grid_center(GRID, 99)
    except ValueError:
        pass
    else:
        raise AssertionError("缺格 99 必须报错，不能静默给一个中心")


# ---------------------------------------------------------------- 塔窗
@case
def test_pick_tower_top_takes_highest_in_window():
    blocks = [blk(1, 420.0, -115.0, 31.0, 29.0),
              blk(2, 425.0, -110.0, 60.0, 58.0),      # 塔顶
              blk(3, 422.0, -114.0, 200.0, 198.0, low=True),   # low 不参与
              blk(4, 600.0, 0.0, 31.0, 29.0),         # 窗外
              blk(5, 422.0, -114.0, 5.0, 5.0)]        # 高度不足
    best, diag = m3.pick_tower_top(blocks, [422.65, -114.86])
    assert best is not None and best["idx"] == 2, best
    assert diag["n_window"] == 2, diag
    # 窗外那个再高也不能选
    blocks[3]["top"] = 300.0
    best2, _ = m3.pick_tower_top(blocks, [422.65, -114.86])
    assert best2["idx"] == 2, "窗外块被选进来了"


@case
def test_pick_tower_top_empty_window_is_none():
    best, diag = m3.pick_tower_top([blk(1, 0.0, 0.0, 30.0, 29.0)],
                                   [422.65, -114.86])
    assert best is None and diag["n_window"] == 0


# ---------------------------------------------------------------- 融合
@case
def test_fuse_frames_median_and_yaw():
    recs = [
        {"found": True, "frame": "a", "center_base_mm": [420.0, -115.0],
         "top_mm": 30.0, "yaw_deg": 1.0, "yaw_trusted": True},
        {"found": True, "frame": "b", "center_base_mm": [422.0, -114.0],
         "top_mm": 31.0, "yaw_deg": 2.0, "yaw_trusted": True},
        {"found": True, "frame": "c", "center_base_mm": [424.0, -113.0],
         "top_mm": 32.0, "yaw_deg": 3.0, "yaw_trusted": True},
    ]
    f = m3.fuse_frames(recs)
    assert f["found"] and f["n_found"] == 3
    assert f["center_base_mm"] == [422.0, -114.0], f
    assert_close(f["top_mm"], 31.0)
    assert f["yaw_span_deg"] == 2.0, f
    assert 1.0 < f["yaw_deg"] < 3.0, f
    # 任一帧不可信 ⇒ 整层不可信（宁可人评）
    recs[2]["yaw_trusted"] = False
    assert m3.fuse_frames(recs)["yaw_trusted"] is False
    # 全找不到
    assert m3.fuse_frames([{"found": False}])["found"] is False


@case
def test_pose_consistency_states():
    a = {"found": True, "center_base_mm": [422.0, -115.0], "top_mm": 31.0,
         "yaw_deg": 1.0, "yaw_color_deg": 0.5}
    b = {"found": True, "center_base_mm": [424.0, -116.0], "top_mm": 32.0,
         "yaw_deg": 2.5, "yaw_color_deg": 1.5}
    ok = m3.pose_consistency([a, b])
    assert ok["ok"] is True, ok
    assert_close(ok["dxy_mm"], np.hypot(2.0, 1.0), 0.01)
    assert_close(ok["dz_mm"], 1.0, 0.01)
    far = dict(b, center_base_mm=[445.0, -115.0])
    bad = m3.pose_consistency([a, far])
    assert bad["ok"] is False and bad["reason"] == "pose_mismatch", bad
    one = m3.pose_consistency([a, {"found": False}])
    assert one["ok"] is None and one["reason"] == "single_pose", one


@case
def test_pose_consistency_yaw_uses_color():
    """yaw 判据 = 彩色（用户 2026-11-01 拍板）；深度再离谱也不能翻红，反之亦然。"""
    # 深度两视点差 20°（噪声大），彩色只差 0.5° ⇒ 应判合格
    a = {"found": True, "center_base_mm": [422.0, -115.0], "top_mm": 31.0,
         "yaw_deg": 0.0, "yaw_color_deg": 1.0}
    b = {"found": True, "center_base_mm": [422.5, -115.5], "top_mm": 31.4,
         "yaw_deg": 20.0, "yaw_color_deg": 1.5}
    r = m3.pose_consistency([a, b])
    assert r["ok"] is True, r
    assert r["dyaw_deg"] == 20.0 and r["dyaw_color_deg"] == 0.5, r
    # 反过来：彩色差 8°（超 5° 门），深度只差 0.2° ⇒ 应判不合格
    c = dict(b, yaw_deg=0.2, yaw_color_deg=9.0)
    r2 = m3.pose_consistency([a, c])
    assert r2["ok"] is False and r2["reason"] == "pose_mismatch", r2
    # 彩色 yaw 缺失 ⇒ ok=None（不拿深度顶替，也不当合格）
    d = {"found": True, "center_base_mm": [422.0, -115.0], "top_mm": 31.0,
         "yaw_deg": 1.0}
    e = dict(d)
    r3 = m3.pose_consistency([d, e])
    assert r3["ok"] is None and r3["reason"] == "no_color_yaw", r3
    assert r3["dyaw_color_deg"] is None, r3


# ---------------------------------------------------------------- 表面语义
FAKE = {
    "p1": {"found": True, "frame": "p1", "center_base_mm": [422.0, -115.0],
           "top_mm": 32.0, "yaw_deg": 0.5, "yaw_color_deg": 0.6,
           "yaw_trusted": True},
    "p2": {"found": True, "frame": "p2", "center_base_mm": [423.0, -115.0],
           "top_mm": 32.4, "yaw_deg": 1.0, "yaw_color_deg": 1.0,
           "yaw_trusted": True},
}


def fake_measure_frame(frame_dir, **kw):
    return dict(FAKE[os.path.basename(frame_dir)])


@case
def test_measure_layer_surface_semantics():
    """本层脚下表面与下一层放置面**必须差一个块高**，且 z 偏差只扣一次。"""
    orig = m3.measure_frame
    m3.measure_frame = fake_measure_frame
    try:
        rec = m3.measure_layer([("pose1", ["p1"]), ("pose2", ["p2"])],
                               window_xy_mm=[422.65, -114.86], layer=2,
                               z_bias_mm=1.2, block_h_mm=29.5)
    finally:
        m3.measure_frame = orig
    top = rec["merged"]["top_mm"]
    assert rec["merged"]["found"] and rec["merged"]["n_found"] == 2, rec
    # 偏差符号：surface = top − 29.5 − 1.2（偏高要把读数减掉，不是加上）
    assert_close(rec["surface_est_mm"], top - 29.5 - 1.2, 0.01, "surface_est")
    assert_close(rec["next_surface_est_mm"], top - 1.2, 0.01, "next_surface")
    assert_close(rec["next_surface_est_mm"] - rec["surface_est_mm"], 29.5,
                 0.01, "两层表面必须差一整块")
    assert rec["merged"]["frames_used"] == ["pose1", "pose2"], \
        "融合记录要带视点名，不能是一串 None"
    assert rec["consistency"]["ok"] is True, rec["consistency"]


@case
def test_measure_layer_missing_top_has_no_surface():
    orig = m3.measure_frame
    m3.measure_frame = lambda d, **kw: {"found": False,
                                        "frame": os.path.basename(d)}
    try:
        rec = m3.measure_layer([("pose1", ["p1"])],
                               window_xy_mm=[0, 0], layer=1)
    finally:
        m3.measure_frame = orig
    assert rec["merged"]["found"] is False
    assert "surface_est_mm" not in rec and "next_surface_est_mm" not in rec, \
        "量不到就不能编一个表面出来（fail-closed）"


@case
def test_bias_from_single():
    # 读数顶高 30.7、垫面 2.0、真高 29.5 ⇒ 读数偏低 0.8 mm
    assert_close(m3.bias_from_single(30.7, 2.0), -0.8)
    assert_close(m3.bias_from_single(32.2, 2.0), 0.7)


@case
def test_scan_biases_summary():
    """历史帧/入口实验 1 的偏差表：逐块 h − 29.5，low 不算，中位/极差要对。"""
    orig_load, orig_find = m3.gc.load_frame, m3.gc.find_blocks
    fake = {
        "a": [blk(0, 0.5, 0.3, 30.7, 30.7, cls="red"),
              blk(1, 0.4, 0.2, 28.0, 28.0, cls="cyan"),
              blk(2, 0.3, 0.1, 90.0, 90.0, low=True)],
        "b": [blk(0, 0.5, 0.3, 30.0, 30.0, cls="red")],
    }
    m3.gc.load_frame = lambda d: d
    m3.gc.find_blocks = lambda f, **kw: (fake[f], 0.0)
    try:
        res = m3.scan_biases(["a", "b"])
    finally:
        m3.gc.load_frame, m3.gc.find_blocks = orig_load, orig_find
    s = res["summary"]
    assert s["n_blocks"] == 3 and s["n_frames"] == 2, s
    assert [r["bias_mm"] for r in res["rows"]] == [1.2, -1.5, 0.5], res["rows"]
    assert s["min_mm"] == -1.5 and s["max_mm"] == 1.2, s
    assert s["median_mm"] == 0.5 and s["n_positive"] == 2, s


# ---------------------------------------------------------------- 跑
def main():
    failed = 0
    for fn in CASES:
        try:
            fn()
            print("PASS  %s" % fn.__name__)
        except Exception as exc:                                  # noqa: BLE001
            failed += 1
            print("FAIL  %s\n      %s: %s"
                  % (fn.__name__, type(exc).__name__, exc))
    print("\n%d/%d PASS" % (len(CASES) - failed, len(CASES)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())

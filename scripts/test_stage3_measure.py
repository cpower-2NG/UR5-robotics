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


@case
def test_pick_tower_top_can_gate_by_expected_height():
    """eye-in-hand 在 carry 处：手里那块更高，**按高度认塔顶**，不取最高簇。

    入口③ 三层实测：持块顶 170 / 215 / 273 mm，塔顶 31.8 / 90.8 / 149.8 mm。
    不给期望顶高时「最高簇」就是手里那块 —— 这正是旧记录假阴性的来源。
    """
    blocks = [blk(1, 422.0, -114.0, 150.0, 148.0),   # 手里那块（更高）
              blk(2, 423.0, -115.0, 90.8, 89.0),     # 塔顶
              blk(3, 421.0, -113.0, 40.0, 38.0)]     # 更低的邻层
    top, _ = m3.pick_tower_top(blocks, [422.0, -114.0])
    assert top["idx"] == 1, "不给期望时旧行为是取最高簇"
    top2, diag2 = m3.pick_tower_top(blocks, [422.0, -114.0],
                                    expected_top_mm=90.8)
    assert top2 is not None and top2["idx"] == 2, top2
    assert_close(diag2["expected_top_mm"], 90.8, 1e-9)
    assert diag2["n_window"] == 1, diag2
    # 期望值落在两簇之间 ⇒ 一簇都不选（**fail-closed**，不退回「最高簇」）
    top3, diag3 = m3.pick_tower_top(blocks, [422.0, -114.0],
                                    expected_top_mm=120.0)
    assert top3 is None and diag3["n_window"] == 0, diag3


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
def test_fuse_frames_ignores_untrusted_color_yaw():
    """不可信帧的彩色 yaw **不进圆均值** —— 否则一致性会拿污染值报「一致」。

    可信判据（``all()``）与数值聚合是两件事：只让标志挡门、却让值参与平均，
    就会出现「``metric_yaw`` 说不可信、``pose_consistency`` 说两视点一致」的
    自相矛盾报告。
    """
    good = lambda y: {"found": True, "frame": "g%d" % y,   # noqa: E731
                      "center_base_mm": [422.0, -115.0], "top_mm": 32.0,
                      "yaw_color_deg": y, "yaw_color_trusted": True}
    bad = {"found": True, "frame": "bad", "center_base_mm": [422.0, -115.0],
           "top_mm": 32.0, "yaw_color_deg": 40.0, "yaw_color_trusted": False}

    # 3 帧里 2 帧可信 ⇒ **算这个视点投了票**（2026-11-03 起，台账第 222 轮）；
    # 但坏帧的值仍不许进均值。
    f = m3.fuse_frames([good(1.0), good(1.2), bad])
    assert f["yaw_color_trusted"] is True, f
    assert abs(f["yaw_color_deg"] - 1.1) < 0.2, \
        ("不可信帧污染了均值", f["yaw_color_deg"])
    assert f["yaw_color_votes"] == [2, 3], f["yaw_color_votes"]

    # 2 帧里只有 1 帧可信 ⇒ 投票不足，仍不可信
    f2 = m3.fuse_frames([good(1.0), bad])
    assert f2["yaw_color_trusted"] is False, f2
    assert f2["yaw_color_votes"] == [1, 2], f2["yaw_color_votes"]

    # 单帧视点没有「第二票」可补，仍按那一帧自己的可信度
    assert m3.fuse_frames([good(1.0)])["yaw_color_trusted"] is True
    assert m3.fuse_frames([bad])["yaw_color_trusted"] is False

    # 全不可信 ⇒ 不给均值（否则 `pose_consistency` 会拿它算出「一致」）
    g = m3.fuse_frames([bad, dict(bad, frame="bad2")])
    assert g["yaw_color_deg"] is None, g
    assert g["yaw_color_votes"] == [0, 2], g["yaw_color_votes"]
    assert g["yaw_color_trusted"] is False, g


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
           "yaw_trusted": True, "yaw_color_trusted": True},
    "p2": {"found": True, "frame": "p2", "center_base_mm": [423.0, -115.0],
           "top_mm": 32.4, "yaw_deg": 1.0, "yaw_color_deg": 1.0,
           "yaw_trusted": True, "yaw_color_trusted": True},
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


# ------------------------------------------------- 颜色掩码 / 贴块异物救援
# 这里的帧是**合成**的：一台正俯视相机看 z=0 平面上的一块正方形色块，
# 可选在一条边上贴一块同色异物。用它盯两件事：
#   1) 干净块必须可信、yaw 打在真值上；
#   2) 贴了异物后掩码判假（复现 nudge-07/09 的 fill 0.71–0.79），
#      给了 ``side_mm`` 的救援必须把异物切掉、把 yaw 拉回来。
F_SYNTH = 600.0
SYNTH_WH = (480, 640)


def _synth_frame(yaw_deg=0.0, tab_mm=0.0, tab_w_mm=9.0,
                 side_mm=29.5, cam_h_mm=360.0, cx_mm=50.0, cy_mm=20.0,
                 block_rgb=(20, 90, 100), shadow=None, shade=None):
    """正俯视相机 + z=0 平面上的正方形色块（可带贴边异物）→ 假 frame。"""
    h, w = SYNTH_WH
    K = np.array([[F_SYNTH, 0.0, (w - 1) / 2.0],
                  [0.0, F_SYNTH, (h - 1) / 2.0],
                  [0.0, 0.0, 1.0]])
    R = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
    t = np.array([cx_mm / 1000.0, cy_mm / 1000.0, cam_h_mm / 1000.0])
    vs, us = np.mgrid[0:h, 0:w]
    rays = np.stack([(us - K[0, 2]) / K[0, 0],
                     (vs - K[1, 2]) / K[1, 1],
                     np.ones(us.shape)], -1)
    dirs = rays @ R.T                                       # 相机 → base
    s = (0.0 - t[2]) / dirs[..., 2]
    P = dirs * s[..., None] + t                             # (h,w,3) base
    a = np.radians(yaw_deg)
    e1 = np.array([np.cos(a), np.sin(a)])
    e2 = np.array([-np.sin(a), np.cos(a)])
    rel = P[..., :2] - t[:2]
    x = rel @ e1
    y = rel @ e2
    half = side_mm / 2000.0
    on = (np.abs(x) <= half) & (np.abs(y) <= half)
    if tab_mm:
        on |= ((x <= -half) & (x >= -half - tab_mm / 1000.0)
               & (np.abs(y) <= tab_w_mm / 2000.0))
    img = np.full((h, w, 3), 150, dtype=np.uint8)
    if shadow:
        # 与块**贴边**的一条灰影：颜色是「块被自己的阴影压暗」还是「灰地面」
        # 在通道差判据下分不开，正是 COLOR_SAT_TOL 要挡的那个东西。
        sw = shadow.get("w_mm", 12.0) / 1000.0
        sl = shadow.get("len_mm", side_mm * 1.4) / 2000.0
        sh_on = ((x >= half) & (x <= half + sw) & (np.abs(y) <= sl))
        img[sh_on] = shadow.get("rgb", (100, 95, 90))
    img[on] = block_rgb
    if shade:
        # 块**自己**被阴影压暗的那一半：色相与饱和度不变、只有明度掉。
        img[on & (x >= 0.0)] = shade.get("rgb", (70, 53, 37))
    blk_ = {"uv": [(w - 1) / 2.0, (h - 1) / 2.0], "top": 0.0,
            "cx": cx_mm, "cy": cy_mm, "h": side_mm}
    return {"color": img, "depth": None, "K": K, "R_bc": R, "t_bc": t,
            "_block": blk_}


def _yaw_err_mod90(a, b):
    d = (float(a) - float(b)) % 90.0
    return min(d, 90.0 - d)


@case
def test_color_rect_clean_square_trusted():
    fr = _synth_frame(yaw_deg=17.0)
    r = m3.color_rect_yaw(fr, fr["_block"], side_mm=29.5)
    assert r["trusted"], r
    assert 0.85 <= r["fill"] <= 1.0, r
    assert _yaw_err_mod90(r["yaw_deg"], 17.0) <= 1.0, r
    assert 27.0 <= max(r["ext_mm"]) <= 32.0, r


@case
def test_color_rect_tab_is_untrusted_and_rescued():
    """贴了异物 ⇒ 原掩码判假；给 side_mm 就得救回来，不给就得判假。"""
    fr = _synth_frame(yaw_deg=17.0, tab_mm=9.0)
    bad = m3.color_rect_yaw(fr, fr["_block"], side_mm=None)
    assert not bad["trusted"], ("贴异物却没判假，用例失效", bad)
    assert max(bad["ext_mm"]) > 31.0, bad
    good = m3.color_rect_yaw(fr, fr["_block"], side_mm=29.5)
    assert good["trusted"], good
    assert good.get("rescue"), ("没走救援路径", good)
    assert _yaw_err_mod90(good["yaw_deg"], 17.0) <= 2.0, good
    assert max(good["ext_mm"]) <= 34.0, good
    # 反向变异：把「最多能切多少」压到 0 ⇒ 救援必须放弃、判回不可信。
    orig = m3.COLOR_RESCUE_MAX_EXCL
    m3.COLOR_RESCUE_MAX_EXCL = 0.0
    try:
        dead = m3.color_rect_yaw(fr, fr["_block"], side_mm=29.5)
    finally:
        m3.COLOR_RESCUE_MAX_EXCL = orig
    assert not dead["trusted"], ("变异没让用例变红，用例是空转", dead)


@case
def test_color_rect_rescue_rejects_wrong_size_prior():
    """side_mm 给错（远小于块）时，救援不许硬救 —— 覆盖判据要能挡住。"""
    fr = _synth_frame(yaw_deg=17.0, tab_mm=9.0)
    r = m3.color_rect_yaw(fr, fr["_block"], side_mm=15.0)
    assert not r["trusted"], ("边长先验给错还判可信", r)


@case
def test_color_rect_ignores_adjacent_grey_shadow():
    """块旁贴边的**灰影**不许进掩码 —— 它和暗黄块的通道差只有 34，靠饱和度分开。

    复现 2026-11-01 入口③：暗黄块 (105,80,56) 旁边的灰地面 (100,95,90) 在
    「通道最大差 < 45」下会被一起收进来，掩码外接从 29.5 mm 撑到 33–47 mm，
    而 `fill` 往往还判合格 —— 只能靠色相/饱和度/明度三项挡住。
    """
    fr = _synth_frame(yaw_deg=13.0, block_rgb=(105, 80, 56),
                      shadow={"rgb": (100, 95, 90), "w_mm": 12.0})
    r = m3.color_rect_yaw(fr, fr["_block"], side_mm=29.5)
    assert r["trusted"], r
    assert max(r["ext_mm"]) <= 31.5, ("灰影进了掩码", r)
    assert _yaw_err_mod90(r["yaw_deg"], 13.0) <= 1.0, r
    # 反向变异：把饱和度门放开 ⇒ 灰影必须进来、外接尺寸必须撑破。
    orig = m3.COLOR_SAT_TOL
    m3.COLOR_SAT_TOL = 2.0
    try:
        bad = m3.color_rect_yaw(fr, fr["_block"], side_mm=29.5)
    finally:
        m3.COLOR_SAT_TOL = orig
    assert max(bad["ext_mm"]) > 33.0, ("变异没让用例变红，用例是空转", bad)


@case
def test_color_rect_keeps_block_own_shaded_half():
    """块**自己**暗下去的一半必须留在掩码里 —— 阴影只改明度，不改色相/饱和度。

    这条守的是 ``COLOR_VAL_TOL``：收得太死就会把阴影面切掉、外接尺寸缩水。
    """
    fr = _synth_frame(yaw_deg=-6.0, block_rgb=(105, 80, 56),
                      shade={"rgb": (70, 53, 37)})
    r = m3.color_rect_yaw(fr, fr["_block"], side_mm=29.5)
    assert r["trusted"], r
    # 看**短边**：只留一半的掩码是个 29.5×14.7 的矩形，`fill` 照样 0.94 判合格，
    # 长边也还是 29.5 —— 只有短边会塌。
    assert min(r["ext_mm"]) >= 27.5, ("自己暗掉的一半被切掉了", r)
    # 阴影面被切掉的帧会「判假 → 走救援 → 救援把尺寸补回 33.5 mm 的框」，
    # 光看 ext 是发现不了的；没走救援才说明掩码本身完整。
    assert "rescue" not in r, ("块自己的一半被切掉、走了救援", r)
    assert _yaw_err_mod90(r["yaw_deg"], -6.0) <= 1.0, r


@case
def test_color_rect_rescue_keeps_good_frames_untouched():
    """判真的帧不许被救援碰过（判真即返回，连 rescue 字段都不该有）。"""
    fr = _synth_frame(yaw_deg=-8.0)
    r = m3.color_rect_yaw(fr, fr["_block"], side_mm=29.5)
    assert r["trusted"] and "rescue" not in r, r


# ---------------------------------------------------------------- 画幅截断闸门
# 合成一块**正对**的平面：640×480、恒深 0.4 m，相机在 base z=0.5 m 朝下
# ⇒ base z 恒为 100 mm，像素比例 0.667 mm/px。这样「顶面点离图像边几像素」
# 就只由块心位置决定，可以精确构造「贴边」与「居中」两种帧。
SYNTH_K = np.array([[600.0, 0.0, 319.5], [0.0, 600.0, 239.5],
                    [0.0, 0.0, 1.0]])
SYNTH_MM_PER_PX = 400.0 / 600.0


def _clip_frame(cx_mm, cy_mm, top_mm=100.0, block_mm=29.5):
    # 彩色图上真画一块 29.5 mm 的方块：颜色掩码要从块心生长，若整幅同色，
    # 它会一直长到 ±`COLOR_HALF_PX` 的窗口边（外接 106 mm），尺寸闸门会
    # 正当地判「这不是一块 29.5 mm 的块」—— 那测的就不是画幅闸门了。
    img = np.full((480, 640, 3), 90, dtype=np.uint8)
    u = 319.5 + float(cx_mm) / SYNTH_MM_PER_PX
    v = 239.5 - float(cy_mm) / SYNTH_MM_PER_PX
    r_px = int(round((float(block_mm) / SYNTH_MM_PER_PX) / 2.0))
    ui, vi = int(round(u)), int(round(v))
    img[max(0, vi - r_px):vi + r_px,
        max(0, ui - r_px):ui + r_px] = (60, 60, 180)
    return {"depth": np.full((480, 640), 0.4, dtype=np.float64),
            "color": img,
            "K": SYNTH_K, "R_bc": np.diag([1.0, -1.0, -1.0]),
            "t_bc": np.array([0.0, 0.0, 0.5]),
            "_block": _clip_block(cx_mm, cy_mm, top_mm)}


def _clip_block(cx_mm, cy_mm, top_mm=100.0):
    u = 319.5 + float(cx_mm) / SYNTH_MM_PER_PX
    v = 239.5 - float(cy_mm) / SYNTH_MM_PER_PX
    return {"idx": 0, "cx": float(cx_mm), "cy": float(cy_mm),
            "top": float(top_mm), "h": 29.5, "low": False,
            "uv": (u, v), "yaw_rect": 0.0, "yaw_trusted": True,
            "class": "x", "class_share": 1.0,
            "ext_rect": [29.5, 29.5], "rect_fill": 0.95}


def _measure_synth(cx_mm, cy_mm, block_mm=29.5):
    """把 ``load_frame`` / ``find_blocks`` 换成合成帧，跑真正产测量入口。"""
    import graspnet_candidates as gc
    fr = _clip_frame(cx_mm, cy_mm, block_mm=block_mm)
    ob_lf, ob_fb = gc.load_frame, gc.find_blocks
    gc.load_frame = lambda d, **kw: fr
    gc.find_blocks = lambda f, **kw: ([fr["_block"]], 0.0)
    try:
        return m3.measure_frame("/synth", window_xy_mm=[cx_mm, cy_mm])
    finally:
        gc.load_frame, gc.find_blocks = ob_lf, ob_fb


@case
def test_measure_frame_flags_truncated_face():
    """顶面贴到画幅边 ⇒ ``usable=False`` 且带原因；居中 ⇒ 可用。

    这是**生产**测量链的闸门（不是入口实验那个）：中心一旦沿「相机→塔」
    方向偏 +5~+7 mm，融合中位数就跟着偏，而且两视点会一起偏（同一批位姿），
    `pose_consistency` 查不出来。
    """
    mid = _measure_synth(0.0, 0.0)
    assert mid["found"] and mid["usable"] is True, mid
    assert mid["framing"]["clipped"] is False, mid["framing"]
    assert mid["framing"]["clear_px"] > 100.0, mid["framing"]

    # 190 mm → u≈604.5 px，块的 25 mm 圈已经出右边
    edge = _measure_synth(190.0, 0.0)
    assert edge["found"], edge
    assert edge["usable"] is False and edge["framing"]["clipped"] is True, edge
    assert edge["framing"]["reason"], "判不可用必须带原因"

    # 反向变异：闸门收到 0 ⇒ 贴边帧必须「变得可用」（说明红是闸门给的）
    orig = m3.TOP_FACE_CLEAR_PX
    m3.TOP_FACE_CLEAR_PX = 0.0
    try:
        assert _measure_synth(190.0, 0.0)["usable"] is True, \
            "变异没让用例变红，用例是空转"
    finally:
        m3.TOP_FACE_CLEAR_PX = orig


@case
def test_measure_layer_drops_truncated_frames():
    """贴边帧不进融合，且要在 `frames_dropped` 里留痕（审计用）。"""
    orig = m3.measure_frame

    def fake(d, **kw):
        bad = os.path.basename(d) == "p2"
        return {"found": True, "frame": os.path.basename(d),
                "usable": not bad, "framing": {"clipped": bad,
                                               "clear_px": 0.0 if bad else 200.0},
                "center_base_mm": [422.0, -115.0] if not bad else [429.0, -115.0],
                "top_mm": 32.0, "yaw_deg": 0.5, "yaw_color_deg": 0.5,
                "yaw_trusted": True}

    m3.measure_frame = fake
    try:
        rec = m3.measure_layer([("pose1", ["p1"]), ("pose2", ["p2"])],
                               window_xy_mm=[422.65, -114.86], layer=2)
    finally:
        m3.measure_frame = orig
    assert rec["merged"]["found"] is True, rec["merged"]
    assert rec["merged"]["center_base_mm"][0] == 422.0, \
        ("贴边帧进了融合", rec["merged"])
    assert rec["frames_dropped"] == [{"pose": "pose2", "frame": "p2",
                                      "clear_px": 0.0,
                                      "reason": "framing_clipped",
                                      "size_ratio": None}], rec["frames_dropped"]
    assert rec["poses"][1]["n_dropped"] == 1, rec["poses"][1]


@case
def test_measure_layer_all_frames_dropped_is_fail_closed():
    """一整个视点全贴边 ⇒ 该视点算「没测到」，不许拿它凑数。"""
    orig = m3.measure_frame
    m3.measure_frame = lambda d, **kw: {
        "found": True, "frame": os.path.basename(d), "usable": False,
        "framing": {"clipped": True, "clear_px": 3.0},
        "center_base_mm": [422.0, -115.0], "top_mm": 32.0,
        "yaw_deg": 0.5, "yaw_color_deg": 0.5, "yaw_trusted": True}
    try:
        rec = m3.measure_layer([("pose1", ["p1"]), ("pose2", ["p2"])],
                               window_xy_mm=[422.65, -114.86], layer=2)
    finally:
        m3.measure_frame = orig
    assert rec["merged"]["found"] is False, ("全贴边还量出了塔顶", rec["merged"])
    assert rec["consistency"]["ok"] is False, rec["consistency"]
    assert len(rec["frames_dropped"]) == 2, rec["frames_dropped"]


# ---------------------------------------------------------------- 尺寸合理性闸门
@case
def test_size_gate_rejects_half_block():
    """彩色掩码只剩半块（短边 14.75 mm）⇒ ratio≈0.50，整帧不可用。

    半块的深度中心/顶高都会被那半块带着走，而两视点会一起错，一致性查不出来。
    """
    r = _measure_synth(0.0, 0.0, block_mm=14.75)
    assert r["found"], r
    assert r["size_ratio"] is not None and r["size_ratio"] < 0.85, r
    assert r["size_ok"] is False, r
    assert r["size_reason"] == "size_implausible", r
    assert r["usable"] is False, ("半块掩码仍被当成可用帧", r)


@case
def test_size_gate_accepts_good_range():
    """好帧实测比值 0.874–1.036 必须全放行（闸门只看彩色外接短边）。"""
    orig = m3.color_rect_yaw

    def with_ext(ratio):
        def fake(frame, block, **kw):
            e = m3.BLOCK_TRUE_H_MM * float(ratio)
            return {"yaw_deg": 0.0, "ext_mm": [e, e], "fill": 1.0,
                    "n_pixels": 4000, "trusted": True, "reason": ""}
        return fake

    try:
        for ratio in (0.874, 1.0, 1.036):
            m3.color_rect_yaw = with_ext(ratio)
            r = _measure_synth(0.0, 0.0)
            assert r["size_ok"] is True, (ratio, r["size_ratio"])
            assert r["size_reason"] == "", (ratio, r)
            assert r["usable"] is True, (ratio, r)
    finally:
        m3.color_rect_yaw = orig


@case
def test_size_gate_missing_ext_keeps_usable():
    """彩色掩码没量到（``ext_mm=[0,0]``）⇒ 不判尺寸，不连累 depth 中心。"""
    orig = m3.color_rect_yaw

    def no_colour(frame, block, **kw):
        return {"yaw_deg": None, "ext_mm": [0.0, 0.0], "fill": 0.0,
                "n_pixels": 0, "trusted": False, "reason": "掩码/轮廓太小"}

    m3.color_rect_yaw = no_colour
    try:
        r = _measure_synth(0.0, 0.0)
    finally:
        m3.color_rect_yaw = orig
    assert r["size_ratio"] is None, r
    assert r["size_ok"] is None, r
    assert r["size_reason"] == "", r
    assert r["usable"] is True, ("缺彩色外接被当成了尺寸失败", r)


@case
def test_measure_layer_dropped_records_size_reason():
    """尺寸被拒的帧要在 `frames_dropped` 里留下 reason 与比值。"""
    orig = m3.measure_frame

    def fake(d, **kw):
        bad = os.path.basename(d) == "p2"
        return {"found": True, "frame": os.path.basename(d),
                "usable": not bad,
                "framing": {"clipped": False, "clear_px": 200.0},
                "size_ratio": 0.50 if bad else 0.98,
                "size_ok": (None if not bad else False),
                "center_base_mm": [422.0, -115.0] if not bad else [422.0, -115.0],
                "top_mm": 32.0, "yaw_deg": 0.5, "yaw_color_deg": 0.5,
                "yaw_trusted": True}

    m3.measure_frame = fake
    try:
        rec = m3.measure_layer([("pose1", ["p1"]), ("pose2", ["p2"])],
                               window_xy_mm=[422.65, -114.86], layer=2)
    finally:
        m3.measure_frame = orig
    assert rec["merged"]["found"] is True, rec["merged"]
    assert rec["frames_dropped"] == [{"pose": "pose2", "frame": "p2",
                                      "clear_px": 200.0,
                                      "reason": "size_implausible",
                                      "size_ratio": 0.50}], rec["frames_dropped"]


# ---------------------------------------------------------------- 一致性门（2026-11-04 收紧）
@case
def test_pose_xy_tol_is_5mm():
    """xy 门 10 → 5 mm：4.9 mm 过、5.2 mm 红。归档 85 层 p95=3.07 mm。

    round-03 L2 的 8.79 mm 假一致就是这么漏过去的 —— 5 mm 会当场停轮。
    """
    a = {"found": True, "center_base_mm": [422.0, -115.0], "top_mm": 31.0,
         "yaw_deg": 0.0, "yaw_color_deg": 1.0}
    near = dict(a, center_base_mm=[426.9, -115.0])          # 4.9 mm
    assert m3.pose_consistency([a, near])["ok"] is True
    far = dict(a, center_base_mm=[427.2, -115.0])           # 5.2 mm
    r = m3.pose_consistency([a, far])
    assert r["ok"] is False and r["reason"] == "pose_mismatch", r
    assert r["tol"]["xy_mm"] == 5.0, r["tol"]


@case
def test_pose_z_tol_is_5mm():
    """z 门同收到 5 mm（归档 85 层最大 2.48 mm）。"""
    a = {"found": True, "center_base_mm": [422.0, -115.0], "top_mm": 31.0,
         "yaw_deg": 0.0, "yaw_color_deg": 1.0}
    assert m3.pose_consistency([a, dict(a, top_mm=35.9)])["ok"] is True
    r = m3.pose_consistency([a, dict(a, top_mm=36.2)])
    assert r["ok"] is False, r
    assert r["tol"]["z_mm"] == 5.0, r["tol"]


# ---------------------------------------------------------------- 深度完整度（只记录）
def _fake_face(right_depth_m=None):
    """40×40 深度图，中央 8×8 像素是块顶（0.65 m）；``right_depth_m`` 覆盖右半。"""
    h = w = 40
    K = [[1000.0, 0.0, 20.0], [0.0, 1000.0, 20.0], [0.0, 0.0, 1.0]]
    depth = np.full((h, w), 0.65, dtype=np.float32)
    depth[:8, :] = 0.0                                  # 背景无效
    depth[32:, :] = 0.0
    depth[:, :8] = 0.0
    depth[:, 32:] = 0.0
    mask = np.zeros((h, w), dtype=np.uint8)
    mask[16:24, 16:24] = 1
    if right_depth_m is not None:
        d = np.array(depth, copy=True)
        d[16:24, 20:24] = float(right_depth_m)
        depth = d
    frame = {"depth": depth,
             "K": np.asarray(K, dtype=np.float64),
             "R_bc": np.eye(3), "t_bc": np.zeros(3)}
    return frame, mask


@case
def test_depth_face_coverage_full_and_truncated():
    """整片顶面都有深度 ⇒ cover≈1；右半深度跳到别的高度 ⇒ cover≈0.5。"""
    frame, mask = _fake_face()
    blk_ = {"cx": 0.0, "cy": 0.0, "top": 650.0}
    full = m3.depth_face_coverage(frame, blk_, mask)
    assert_close(full["cover"], 1.0, 0.01, "整片")
    assert full["n_mask_px"] == 64 and full["n_depth_px"] == 64, full
    assert_close(full["depth_color_xy_mm"], 0.0, 0.5, "彩色中心 == 深度中心")

    frame2, _ = _fake_face(right_depth_m=0.80)          # 高 150 mm，出带
    cut = m3.depth_face_coverage(frame2, blk_, mask)
    assert_close(cut["cover"], 0.5, 0.01, "右半出带")
    assert cut["cover"] < full["cover"], (cut, full)


@case
def test_depth_face_coverage_empty_mask():
    """掩码空 ⇒ cover=None（不制造一个假的 0）。"""
    frame, _ = _fake_face()
    empty = np.zeros(frame["depth"].shape, dtype=np.uint8)
    r = m3.depth_face_coverage(frame, {"cx": 0.0, "cy": 0.0, "top": 650.0},
                               empty)
    assert r["cover"] is None and r["depth_color_xy_mm"] is None, r
    assert r["n_mask_px"] == 0, r


# ------------------------------------------------ 挑坏视点（2026-11-04 补拍，选项 B）
def _pose(name, covers, found=True):
    return {"name": name, "fused": {"found": found},
            "frames": [{"usable": True, "face_cover": c} for c in covers]}


@case
def test_suspect_pose_picks_lowest_coverage():
    """round-03 L2 现场：坏视点 0.26–0.29、好视点 0.63–0.99 ⇒ 指认坏的那个。"""
    meas = {"poses": [_pose("pose1", [0.95, 0.94, 0.63]),
                      _pose("pose2", [0.26, 0.29, 0.28])]}
    assert m3.suspect_pose_index(meas) == 1, meas
    # 反过来也一样（不是「总是挑 pose2」）：换序后 pose2 落到下标 0
    meas["poses"].reverse()
    assert m3.suspect_pose_index(meas) == 0, meas


@case
def test_suspect_pose_ignores_unusable_frames():
    """被画幅/尺寸闸门剔掉的帧不进完整度均值 —— 否则剔掉的烂帧会误导补拍。"""
    meas = {"poses": [{"name": "pose1", "fused": {"found": True},
                       "frames": [{"usable": False, "face_cover": 0.10},
                                  {"usable": True, "face_cover": 0.95}]},
                      _pose("pose2", [0.90])]}
    assert m3.suspect_pose_index(meas) == 1, meas


@case
def test_suspect_pose_none_without_face_cover():
    """一帧都没记 face_cover ⇒ None（没依据就不挑，调用方停轮而不是瞎动臂）。"""
    meas = {"poses": [{"name": "pose1", "fused": {"found": True},
                       "frames": [{"usable": True}]},
                      {"name": "pose2", "fused": {"found": True},
                       "frames": [{"usable": True}]}]}
    assert m3.suspect_pose_index(meas) is None, meas
    assert m3.suspect_pose_index({"poses": []}) is None


@case
def test_suspect_pose_from_measure_layer():
    """``measure_layer`` 必须把每帧 ``face_cover`` 带进 ``poses`` —— 补拍靠它挑人。"""
    orig = m3.measure_frame

    def fake(d, **kw):
        low = "bad" in d
        return {"found": True, "frame": os.path.basename(d), "usable": True,
                "framing": {"clipped": False, "clear_px": 200.0},
                "size_ratio": 0.98, "size_ok": True,
                "face_cover": 0.28 if low else 0.95,
                "depth_color_xy_mm": 8.8 if low else 3.0,
                "center_base_mm": [422.0, -115.0], "top_mm": 32.0,
                "yaw_deg": 0.5, "yaw_color_deg": 0.5, "yaw_trusted": True}

    m3.measure_frame = fake
    try:
        rec = m3.measure_layer([("pose1", ["good1"]), ("pose2", ["bad1"])],
                               window_xy_mm=[422.65, -114.86], layer=2)
    finally:
        m3.measure_frame = orig
    assert m3.suspect_pose_index(rec) == 1, rec["poses"]
    assert_close(m3.pose_coverage(rec["poses"][1]), 0.28, 1e-9, "坏视点均值")
    assert rec["poses"][1]["frames"][0]["depth_color_xy_mm"] == 8.8


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

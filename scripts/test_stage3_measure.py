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

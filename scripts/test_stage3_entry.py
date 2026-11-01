#!/usr/bin/env python3
"""stage3_entry 的纯逻辑测试：z 标定落盘、释放重复性、遮挡分类、偏置向量。

跑法：
  python3 scripts/test_stage3_entry.py

不碰相机、不碰板子、不动臂：只喂合成数据。
真机路径（``--go``）在这里一律跑不到。
"""
import json
import os
import sys
import types

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import stage3_entry as e                                            # noqa: E402
import stage3_round as s3r                                          # noqa: E402

CASES = []


def case(fn):
    CASES.append(fn)
    return fn


def assert_close(a, b, tol=1e-6, why=""):
    assert abs(float(a) - float(b)) <= tol, \
        "%s: %r vs %r（容差 %g）" % (why or "数值", a, b, tol)


def blk(idx, cx, cy, top, h, low=False, yaw=0.0, trusted=True):
    return {"idx": idx, "cx": cx, "cy": cy, "top": top, "h": h,
            "low": low, "yaw_rect": yaw, "yaw_trusted": trusted}


# ---------------------------------------------------------------- z 标定
@case
def test_zcal_doc_sign_and_pass():
    """bias = 顶高 − 垫面 − 真高；每次残差 ≤2 mm 才算过。"""
    rows = [
        {"top_mm": 2.0 + 29.5 + 1.2},          # bias +1.2
        {"top_mm": 2.0 + 29.5 + 1.4,
         "pick_bias_mm": 1.1},
        {"top_mm": 2.0 + 29.5 + 0.9,
         "pick_bias_mm": 1.5},
    ]
    doc = e.zcal_doc(rows, pad_mm=2.0)
    assert doc["by_site"]["tower"] == 1.2, doc
    assert_close(doc["by_site"]["pick"], 1.3)     # 两个拾取侧读数的中位
    assert doc["n"] == 3
    assert doc["residual_mm"]["max"] == 0.3, doc
    assert doc["pass"] is True, doc
    assert [r["rep"] for r in doc["rows"]] == [1, 2, 3]
    assert_close(doc["rows"][2]["bias_mm"], 0.9)


@case
def test_zcal_doc_fails_when_one_rep_far():
    """中位再好看也没用：**任何一次**校正后超 2 mm 就不过。"""
    rows = [{"top_mm": 2.0 + 29.5 + 1.0},
            {"top_mm": 2.0 + 29.5 + 1.1},
            {"top_mm": 2.0 + 29.5 + 5.0}]
    doc = e.zcal_doc(rows, pad_mm=2.0)
    assert doc["pass"] is False, doc
    assert doc["residual_mm"]["max"] > 2.0


@case
def test_zcal_doc_empty_is_not_pass():
    doc = e.zcal_doc([], pad_mm=2.0)
    assert doc["n"] == 0 and doc["pass"] is False
    assert doc["by_site"]["tower"] == 0.0


@case
def test_zcal_doc_feeds_load_zcal():
    """``stage3_round.load_zcal`` 必须直接吃得下这份落盘。"""
    doc = e.zcal_doc([{"top_mm": 2.0 + 29.5 + 1.2,
                       "pick_bias_mm": 1.4}], pad_mm=2.0)
    path = os.path.join(os.environ.get("TMPDIR", "/tmp"),
                        "stage3-entry-zcal-test.json")
    with open(path, "w") as f:
        f.write(__import__("json").dumps(doc))
    try:
        zc = s3r.load_zcal(path)
    finally:
        os.remove(path)
    assert_close(zc["tower"], 1.2)
    assert_close(zc["pick"], 1.4)
    assert zc["model"] == "constant"


# ---------------------------------------------------------------- 重复性
@case
def test_group_stats():
    s = e.group_stats([1.0, 2.0, 3.0])
    assert s["n"] == 3
    assert_close(s["mean"], 2.0)
    assert_close(s["median"], 2.0)
    assert_close(s["range"], 2.0)
    assert_close(s["std"], 1.0)          # ddof=1 的样本标准差
    assert e.group_stats([]) == {"n": 0}
    assert e.group_stats([None]) == {"n": 0}


@case
def test_release_summary_orders_by_spread():
    groups = {
        "open50_dwell0.00": [{"dxy_mm": 1.0, "dyaw_deg": 0.5, "dz_mm": 0.2},
                             {"dxy_mm": 1.1, "dyaw_deg": 0.6, "dz_mm": 0.3},
                             {"dxy_mm": 1.2, "dyaw_deg": 0.4, "dz_mm": 0.1}],
        "open70_dwell0.00": [{"dxy_mm": 2.0, "dyaw_deg": 1.0, "dz_mm": 0.5},
                             {"dxy_mm": 4.0, "dyaw_deg": 1.2, "dz_mm": 0.4},
                             {"dxy_mm": 3.0, "dyaw_deg": 0.8, "dz_mm": 0.6}],
        "open95_dwell0.00": [],           # 没跑成：不参与排序
    }
    summ = e.release_summary(groups)
    assert summ["best"] == "open50_dwell0.00", summ
    assert summ["order"] == ["open50_dwell0.00", "open70_dwell0.00"], summ
    s = summ["groups"]["open70_dwell0.00"]
    assert s["n"] == 3
    assert_close(s["dxy_mm"]["range"], 2.0)
    assert_close(s["dxy_mm"]["median"], 3.0)


# ---------------------------------------------------------------- 遮挡
@case
def test_occlusion_separates_tower_from_held_block():
    """手里那块在塔顶上方 ~74.5 mm —— 绝不能被认成塔顶。"""
    top = 2.0 + 2 * 29.5              # 2 层塔顶
    held = top + 45.0 + 29.5
    blocks = [blk(1, 422.0, -115.0, top, top),          # 塔顶
              blk(2, 421.0, -116.0, held, held),        # 手里的块
              blk(3, 500.0, -50.0, top, top)]           # 窗外
    rep = e.occlusion_report(blocks, top, [422.65, -114.86],
                             held_top_mm=held)
    assert rep["tower_top_visible"] is True, rep
    assert [b["idx"] for b in rep["tower_clusters"]] == [1], rep
    assert [b["idx"] for b in rep["held_clusters"]] == [2], rep
    assert rep["n_window"] == 2, rep
    # 只有手里的块、塔顶看不见 —— 必须判「看不见」，不能拿最高簇凑数
    rep2 = e.occlusion_report([blk(2, 422.0, -115.0, held, held)], top,
                              [422.65, -114.86], held_top_mm=held)
    assert rep2["tower_top_visible"] is False, rep2
    assert rep2["n_window"] == 1


@case
def test_occlusion_place_uses_measured_center():
    """③ 放置目标必须是实测塔顶中心 —— 用格心会把块放到塔边摔下去。

    事故号：2026-11-01 第 1 层档，实测 (423.26, −104.11) vs 格心 (422.65, −114.86)，
    差 10.76 mm，块落到格 1 附近。
    反向变异：`occlusion_place_xy` 里返回 `slot_mm` ⇒ 本用例必须变红。
    """
    slot = [422.65, -114.86]
    base = {"center_base_mm": [423.26, -104.11], "top_mm": 31.99}
    xy = e.occlusion_place_xy(base, slot)
    assert_close(xy[0], 423.26)
    assert_close(xy[1], -104.11)
    # 差多少必须真是 10.76 —— 免得将来谁把两处都改成格心还自洽
    d = ((xy[0] - slot[0]) ** 2 + (xy[1] - slot[1]) ** 2) ** 0.5
    assert 10.5 < d < 11.0, d
    # 量不到中心 ⇒ 停，不许静默退回格心（那正是事故路径）
    try:
        e.occlusion_place_xy({}, slot)
    except e.EntryStop as exc:
        assert "格心" in str(exc.why), exc.why
    else:
        raise AssertionError("缺中心时没停，会退回格心 → 掉块")


@case
def test_occlusion_ignores_low_and_far():
    top = 61.0
    blocks = [blk(1, 422.0, -115.0, top, top, low=True),
              blk(2, 422.0, -115.0, top, 10.0),        # 相对台面 10 mm：低于 15
              blk(3, 422.65, -114.86 + 60.0, top, top)]  # 半径外
    rep = e.occlusion_report(blocks, top, [422.65, -114.86], radius_mm=50.0)
    assert rep["n_window"] == 0 and not rep["tower_top_visible"], rep


# ---------------------------------------------------------------- 偏置
@case
def test_tol_offset_vector():
    off = e.tol_offset_mm([422.0, -115.0], [425.0, -111.0])
    assert off["dx_mm"] == 3.0 and off["dy_mm"] == 4.0
    assert off["d_mm"] == 5.0
    assert e.tol_offset_mm([0, 0], [0, 0])["d_mm"] == 0.0


@case
def test_release_row_offsets():
    """「实测 − 预测」三项：Δxy 是模长，Δyaw 折 mod 90，Δz 为负=落稳下沉。"""
    item = {"yaw_place_deg": 0.0, "block_idx": 7}
    rec = {"predicted_place_mm": [422.0, -115.0], "predicted_top_mm": 36.5,
           "measured": {"merged": {"center_base_mm": [425.0, -111.0],
                                   "yaw_deg": 91.0, "top_mm": 35.0}}}
    row = e._release_row(item, rec, 3)
    assert row["rep"] == 3
    assert row["dxy_mm"] == 5.0
    assert row["dx_mm"] == 3.0 and row["dy_mm"] == 4.0
    assert row["dyaw_deg"] == 1.0, row
    assert row["dz_mm"] == -1.5


# ---------------------------------------------------------------- 前置闸门
@case
def test_layer_height_check():
    """人手摆的高度：差在容差内才算对，正负都要给出来。"""
    ok, d = e.layer_height_check(2.0 + 29.5 + 1.3, 2.0 + 29.5)
    assert ok is True and d == 1.3, (ok, d)
    ok, d = e.layer_height_check(2.0 + 2 * 29.5 - 0.9, 2.0 + 2 * 29.5)
    assert ok is True and d == -0.9, (ok, d)
    # 漏一层（29.5 mm）必被拦
    ok, d = e.layer_height_check(2.0 + 29.5, 2.0 + 2 * 29.5)
    assert ok is False and d == -29.5, (ok, d)
    # 半块高 14.75 也要拦（容差 12）
    assert e.layer_height_check(2.0 + 29.5 + 14.75, 2.0 + 29.5)[0] is False


@case
def test_tower_occupancy_verdict():
    """塔位空着返回 None；有块返回带格号的一句话。"""
    assert e.tower_occupancy_verdict({"found": False}) is None
    assert e.tower_occupancy_verdict({}) is None
    msg = e.tower_occupancy_verdict(
        {"found": True, "top_mm": 31.8, "center_base_mm": [422.7, -114.9]}, 4)
    assert "格 4" in msg and "31.8" in msg, msg


@case
def test_clear_tower_step_skips_first_placement():
    """第一次放置前不让人搬（塔位本来就该是空的），之后**每次**都要。"""
    import types
    calls = []
    orig_ask, orig_clear = e.ask, e.tower_clear_now
    e.ask = lambda text: calls.append(("ask", text))
    e.tower_clear_now = lambda *a, **k: calls.append(("clear", k.get("cell_idx")))
    args = types.SimpleNamespace(tower_cell_idx=4, no_self_return=True)
    try:
        e.clear_tower_step(args, None, None, None, "/tmp", "t", None, None,
                           placed=False)
        assert calls == [], calls
        e.clear_tower_step(args, None, None, None, "/tmp", "t", None, None,
                           placed=True)
        assert [c[0] for c in calls] == ["ask", "clear"], calls
        assert calls[1][1] == 4, calls
    finally:
        e.ask, e.tower_clear_now = orig_ask, orig_clear


@case
def test_clear_tower_step_robot_returns_by_default():
    """默认**不叫人**：塔上那块交给机器人自己抓回拾取区。

    反向变异：把 ``return_to_pick`` 换成原来的 ``ask`` —— 本测试必须变红
    （``ask`` 一次都不该被调到）。
    """
    import types
    calls = []
    orig = (e.ask, e.tower_clear_now, e.return_to_pick)
    e.ask = lambda text: calls.append(("ask", text))
    e.tower_clear_now = lambda *a, **k: calls.append(("clear", k.get("cell_idx")))
    # 第 10 个位置参数就是 zcal（见 return_to_pick 的签名）
    e.return_to_pick = lambda *a, **k: calls.append(("return", a[9]))
    args = types.SimpleNamespace(tower_cell_idx=4)
    merged = {"top_mm": 32.4, "center_base_mm": [425.5, -117.8]}
    try:
        e.clear_tower_step(args, None, None, None, "/tmp", "t", None, None,
                           placed=True, merged=merged, zcal={"tower": 0.9})
        assert [c[0] for c in calls] == ["return"], calls
        assert calls[0][1] == {"tower": 0.9}, calls
    finally:
        e.ask, e.tower_clear_now, e.return_to_pick = orig


@case
def test_clear_tower_step_without_merged_stops():
    """要机器人回位却没有塔顶实测 ⇒ 停（fail-closed），不静默退回人手。"""
    import types
    args = types.SimpleNamespace(tower_cell_idx=4)
    try:
        e.clear_tower_step(args, None, None, None, "/tmp", "t", None, None,
                           placed=True, merged=None)
    except e.EntryStop as exc:
        assert "没有塔顶实测" in exc.why, exc.why
    else:
        raise AssertionError("merged 缺了还往下跑")


@case
def test_free_slot_mm_keeps_gap_and_margin():
    """回位点必须离**每个**块 ≥60 mm、离区边 ≥25 mm，并尽量靠区心。"""
    zone = [0.25, 0.575, 0.17, 0.42]          # 米
    blocks = [blk(0, 300.0, 200.0, 30, 29.5), blk(1, 500.0, 200.0, 30, 29.5)]
    spot = e.free_slot_mm(blocks, zone)
    assert spot is not None, "两个块就把区占满了？"
    x, y = spot["xy_mm"]
    for b in blocks:
        d = ((x - b["cx"]) ** 2 + (y - b["cy"]) ** 2) ** 0.5
        assert d >= e.PICK_RETURN_GAP_MM - 1e-6, (b["idx"], d)
    assert x - 250.0 >= e.PICK_RETURN_MARGIN_MM - 1e-6
    assert 575.0 - x >= e.PICK_RETURN_MARGIN_MM - 1e-6
    assert y - 170.0 >= e.PICK_RETURN_MARGIN_MM - 1e-6
    assert 420.0 - y >= e.PICK_RETURN_MARGIN_MM - 1e-6
    # 靠区心：两个块左右对称，最靠心的候选点应当在中线上
    assert_close(x, (250.0 + 575.0) / 2.0, 5.0, "回位点 x 应贴区心")


def _dense_blocks(step=30):
    """把拾取区铺成 30 mm 点阵（真塞满的样子）。"""
    pts = [(x, y) for x in range(250, 576, step)
           for y in range(170, 421, step)]
    return [blk(i, float(x), float(y), 30, 29.5) for i, (x, y) in enumerate(pts)]


@case
def test_free_slot_mm_returns_none_when_crowded():
    """塞满就说塞满（调用方 fail-closed），不硬塞到块上。"""
    zone = [0.25, 0.575, 0.17, 0.42]
    blocks = _dense_blocks()
    assert e.free_slot_mm(blocks, zone) is None
    # 反向变异：把 gap 放宽到 15 mm，同样的块就「放得下」了 —— 说明拒的
    # 确实是间距判据，不是别的分支（空集/越界）提前返回 None
    assert e.free_slot_mm(blocks, zone, gap_mm=15.0) is not None


@case
def test_free_slot_mm_empty_zone_takes_center():
    """区里一块没有 ⇒ 放区心（不留死角、也不贴边）。"""
    spot = e.free_slot_mm([], [0.25, 0.575, 0.17, 0.42])
    assert spot["gap_mm"] is None, spot
    assert_close(spot["xy_mm"][0], 412.5, 5.0, "空区 x")
    assert_close(spot["xy_mm"][1], 295.0, 5.0, "空区 y")


@case
def test_tower_block_item_uses_corrected_top():
    """抓塔上那块：h = 顶高 − z 标定 ⇒ 正好是「垫面 + 一块真高」。

    反向变异：漏掉 z 标定（直接用 33.12）⇒ h 大 1.28 mm，抓取变浅、本测试变红。
    """
    scene = {"yaw_sign": 1.0, "finger_axis_delta_deg": 0.0,
             "grasp_at": "block_center"}
    merged = {"top_mm": 33.12, "center_base_mm": [419.51, -116.55],
              "yaw_deg": 12.4, "yaw_trusted": True}
    it = e.tower_block_item(merged, scene, {"tower": 1.28})
    assert_close(it["block_h_used_mm"], 31.84, 1e-6, "h")
    assert_close(it["block"]["cx"], 419.51, 1e-6, "cx")
    assert it["block_idx"] == e.TOWER_BLOCK_IDX
    assert it["cand"]["dist_mm"] == 0.0, "候选离块心必须是 0（块心抓）"
    assert_close(it["yaw_grasp_deg"], 12.4, 1e-6, "几何臂 ψ=+θ")
    assert it["yaw_source"] == "geometric"


@case
def test_tower_block_item_folds_yaw_90():
    """塔上那块的正方体对称：抓/放角都折到 (−45°, 45°]。

    反向变异：去掉 `fold_yaw_90` 那一行 ⇒ 89.75° 原样留下，本测试变红，
    而那正是 2026-11-01 入口⑤ 回位被拒（229.6° / −210.045° 两边都是死）的输入。
    """
    scene = {"yaw_sign": 1.0, "finger_axis_delta_deg": 0.0,
             "grasp_at": "block_center"}
    merged = {"top_mm": 32.99, "center_base_mm": [421.07, -119.06],
              "yaw_deg": 89.75, "yaw_trusted": True}
    it = e.tower_block_item(merged, scene, {"tower": 0.0})
    assert_close(it["yaw_grasp_deg"], -0.25, 1e-6, "89.75° 折 90°")
    assert_close(it["yaw_place_deg"], 0.0, 1e-6, "放置角")
    # 折完的抓/放角差（= 腕要转的角度）不许超过半个象限
    for merged_yaw in (12.4, 46.0, 89.75, -100.0, 137.0):
        merged2 = dict(merged, yaw_deg=merged_yaw)
        it2 = e.tower_block_item(merged2, scene, {"tower": 0.0})
        for key in ("yaw_grasp_deg", "yaw_place_deg"):
            assert -45.0 < it2[key] <= 45.0, (merged_yaw, key, it2[key])


@case
def test_tower_block_item_without_yaw():
    """量不到 yaw ⇒ 抓取角 0 且标 geometric_missing（不假装有角）。"""
    scene = {"yaw_sign": 1.0, "finger_axis_delta_deg": 0.0}
    merged = {"top_mm": 33.12, "center_base_mm": [419.51, -116.55],
              "yaw_deg": None}
    it = e.tower_block_item(merged, scene, {"tower": 1.28})
    assert it["yaw_source"] == "geometric_missing", it["yaw_source"]
    assert_close(it["yaw_grasp_deg"], 0.0, 1e-9, "yaw")


@case
def test_return_to_pick_builds_and_execs_one_plan():
    """回位 = 一条计划（塔上抓 → 拾取区空位放），收尾回核塔位。"""
    import types
    seen = {}

    class _Board:
        def joints(self, dest):
            return {"name": [], "position": []}

        def exec_plan(self, path, log, go=True, **kw):
            seen["plan_path"] = path
            return 0

    merged = {"top_mm": 32.45, "center_base_mm": [425.52, -117.8],
              "yaw_deg": 3.0, "yaw_trusted": True}
    blocks = [blk(0, 300.0, 200.0, 30, 29.5)]
    orig = (e.capture_pick, e.analyse_pick, e.tower_clear_now, s3r.layer_plan,
            s3r.write_plan, e.s2.chain_q)
    e.capture_pick = lambda *a, **k: ("/tmp/fake-pick", [])
    e.analyse_pick = lambda *a, **k: {"blocks": blocks}
    e.tower_clear_now = lambda *a, **k: seen.setdefault(
        "clear_cell", k.get("cell_idx"))
    e.s2.chain_q = lambda js: [0.0] * 6

    def _layer_plan(chain, q_start, item, bl, slot, surface, **kw):
        seen.update(item=item, slot=slot, surface=surface, blocks=bl, kw=kw)
        return {"waypoints": [], "meta": {}}

    s3r.layer_plan = _layer_plan
    s3r.write_plan = lambda path, plan: path
    args = types.SimpleNamespace(d_mm=330.0, tower_half_edge=0.020,
                                 tower_cell_idx=4)
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        try:
            rec = e.return_to_pick(args, _Board(), None,
                                   {"pick_zone": [0.25, 0.575, 0.17, 0.42]},
                                   td, "t", None, [422.65, -114.86], merged,
                                   {"tower": 1.28}, rdir=td)
        finally:
            (e.capture_pick, e.analyse_pick, e.tower_clear_now,
             s3r.layer_plan, s3r.write_plan, e.s2.chain_q) = orig
    # 放回点就是 free_slot_mm 选出来的那个，且表面是台面（不是垫面）
    assert_close(seen["slot"][0], rec["spot_mm"][0] / 1000.0, 1e-9, "slot x")
    assert_close(seen["surface"], 0.0, 1e-9, "回位铺在台面上")
    assert seen["item"]["block_idx"] == e.TOWER_BLOCK_IDX
    assert_close(seen["item"]["block_h_used_mm"], 31.17, 1e-6, "塔上块 h")
    assert rec["exec_rc"] == 0
    assert seen["clear_cell"] == 4, "放完要就地核塔位（cell_idx 传对）"


@case
def test_return_to_pick_stops_when_crowded():
    """拾取区没空位 ⇒ EntryStop，不动臂。"""
    import types

    class _Board:
        def joints(self, dest):
            raise AssertionError("没空位就不该去动臂")

    orig = (e.capture_pick, e.analyse_pick)
    e.capture_pick = lambda *a, **k: ("/tmp/fake-pick", [])
    e.analyse_pick = lambda *a, **k: {"blocks": _dense_blocks()}
    args = types.SimpleNamespace(d_mm=330.0, tower_half_edge=0.020,
                                 tower_cell_idx=4)
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        try:
            e.return_to_pick(args, _Board(), None,
                             {"pick_zone": [0.25, 0.575, 0.17, 0.42]},
                             td, "t", None, [422.65, -114.86],
                             {"top_mm": 32.0, "center_base_mm": [422.0, -115.0]},
                             {"tower": 1.0}, rdir=td)
        except e.EntryStop as exc:
            assert "放不下" in exc.why, exc.why
        else:
            raise AssertionError("没空位还往下跑")
        finally:
            e.capture_pick, e.analyse_pick = orig


@case
def test_return_to_pick_red_exec_stops():
    """回位执行判红 ⇒ 停在原地（块可能还在爪里），不续跑。"""
    import types

    class _Board:
        def joints(self, dest):
            return {"name": [], "position": []}

        def exec_plan(self, path, log, go=True, **kw):
            return 3

    orig = (e.capture_pick, e.analyse_pick, e.tower_clear_now,
            s3r.layer_plan, s3r.write_plan, e.s2.chain_q)
    e.capture_pick = lambda *a, **k: ("/tmp/fake-pick", [])
    e.analyse_pick = lambda *a, **k: {"blocks": [blk(0, 300.0, 200.0, 30, 29.5)]}
    e.s2.chain_q = lambda js: [0.0] * 6
    s3r.layer_plan = lambda *a, **k: {"waypoints": [], "meta": {}}
    s3r.write_plan = lambda path, plan: path
    called = []
    e.tower_clear_now = lambda *a, **k: called.append("clear")
    args = types.SimpleNamespace(d_mm=330.0, tower_half_edge=0.020,
                                 tower_cell_idx=4)
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        try:
            e.return_to_pick(args, _Board(), None,
                             {"pick_zone": [0.25, 0.575, 0.17, 0.42]},
                             td, "t", None, [422.65, -114.86],
                             {"top_mm": 32.0, "center_base_mm": [422.0, -115.0],
                              "yaw_deg": 1.0}, {"tower": 1.0}, rdir=td)
        except e.EntryStop as exc:
            assert exc.rc == 3, exc.rc
            assert "停在原地" in exc.why, exc.why
        else:
            raise AssertionError("判红了还往下跑")
        finally:
            (e.capture_pick, e.analyse_pick, e.tower_clear_now,
             s3r.layer_plan, s3r.write_plan, e.s2.chain_q) = orig
    assert called == [], "判红后不该去核塔位"


@case
def test_load_rows_reads_and_sorts():
    """续跑：只认 rep 是整数的 row.json，按 rep 排序。"""
    import json
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        for rep, top in ((2, 32.0), (1, 33.0)):
            d = os.path.join(td, "zcal-%02d" % rep)
            os.makedirs(d)
            json.dump({"rep": rep, "top_mm": top}, open(os.path.join(d, "row.json"), "w"))
        os.makedirs(os.path.join(td, "zcal-09"))        # 空目录
        os.makedirs(os.path.join(td, "zcal-10"))
        open(os.path.join(td, "zcal-10", "row.json"), "w").write("{坏")
        rows = e.load_rows(td)
        assert [r["rep"] for r in rows] == [1, 2], rows


@case
def test_pick_item_converts_runtime_error():
    """拾取区没块 ⇒ EntryStop，不是 traceback。"""
    import types
    orig = s3r.build_items
    s3r.build_items = lambda *a, **k: (_ for _ in ()).throw(
        RuntimeError("候选池只够 0 块（要 1）"))
    args = types.SimpleNamespace(arm="geometric")
    empty = {"blocks": [], "per_block": []}
    try:
        try:
            e.pick_item(args, empty, {"pick_zone": []}, {})
        except e.EntryStop as exc:
            assert "拾取区没有可用块" in exc.why, exc.why
            assert exc.rc == 2
        else:
            raise AssertionError("没有转成 EntryStop")
    finally:
        s3r.build_items = orig
    # 正常路径原样返回
    one = {"idx": 3, "cx": 400.0, "cy": 300.0}
    s3r.build_items = lambda *a, **k: [{"block_idx": 3, "block": one}]
    try:
        got = e.pick_item(args, {"blocks": [one], "per_block": []},
                          {"pick_zone": []}, {})
        assert got[0]["block_idx"] == 3, got
    finally:
        s3r.build_items = orig


@case
def test_pick_item_skips_crowded_block():
    """第一选被邻距护栏拒掉时换下一块；全挤在一起就干净停机。"""
    blocks = [{"idx": 0, "cx": 400.0, "cy": 300.0},
              {"idx": 1, "cx": 450.0, "cy": 300.0},     # 与 #0 只隔 50 mm
              {"idx": 2, "cx": 500.0, "cy": 400.0}]     # 离两者都 >100 mm
    analysis = {"blocks": blocks, "per_block": [
        {"block_idx": 0, "class": "a", "best": {}, "dist_mm": 1.0},
        {"block_idx": 1, "class": "b", "best": {}, "dist_mm": 2.0},
        {"block_idx": 2, "class": "c", "best": {}, "dist_mm": 3.0}]}
    scene = {"pick_zone": [0.25, 0.575, 0.17, 0.42]}
    args = types.SimpleNamespace(arm="geometric")
    orig = s3r.build_items
    try:
        s3r.build_items = lambda *a, **k: [{"block_idx": b["idx"], "block": b}
                                           for b in blocks]
        got = e.pick_item(args, analysis, scene, {})
        assert got[0]["block_idx"] == 2, got       # 跳过挤在一起的前两块
        # 全挤在一起 ⇒ EntryStop（不是把 PlanError 抛到顶）
        s3r.build_items = lambda *a, **k: [{"block_idx": 0, "block": blocks[0]},
                                           {"block_idx": 1, "block": blocks[1]}]
        try:
            e.pick_item(args, {"blocks": blocks[:2], "per_block": [
                {"block_idx": 0, "class": "a", "best": {}, "dist_mm": 1.0},
                {"block_idx": 1, "class": "b", "best": {}, "dist_mm": 2.0}]},
                scene, {})
        except e.EntryStop as exc:
            assert "挤在一起" in exc.why, exc.why
        else:
            raise AssertionError("全挤在一起却没停")
    finally:
        s3r.build_items = orig


# ---------------------------------------------------------------- 释放续跑
@case
def test_resume_tower_action_three_ways():
    merged = {"top_mm": 32.0}
    assert e.resume_tower_action("", {}, None, False) == "empty"
    assert e.resume_tower_action("塔上有块", {}, None, False) == "stop"
    # 续跑时塔上就是上次刚放下去那块：实测结构优先
    assert e.resume_tower_action("塔上有块", {"merged": merged}, None, True) == "return"
    # 实测没结构 ⇒ 退回上次行里的 merged
    assert e.resume_tower_action("塔上有块", {}, merged, True) == "return"
    # 两样都没有 ⇒ 停（不瞎摸）
    assert e.resume_tower_action("塔上有块", {}, None, True) == "stop"


def _release_env(td, reps=3, resume=True, opens="50", dwells="0"):
    args = types.SimpleNamespace(
        run_dir=td, opens=opens, dwells=dwells, reps=reps, confirm=0,
        resume=resume, tower_half_edge=0.02, tower_cell_idx=4,
        d_mm=331.0, arm="geometric", no_self_return=False)
    ctx = (None, None, {"terminal_frames": 1}, {"cells": [{"idx": 4}]},
           [422.65, -114.86], 2.344, {"tower": 1.266, "pick": 0.816})
    return args, ctx


def _seed_release_rows(td, name, reps):
    for rep in reps:
        d = os.path.join(td, "entry-release", name, "%s-%02d" % (name, rep))
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, "row.json"), "w") as fh:
            json.dump({"rep": rep, "dxy_mm": 1.0, "dyaw_deg": 0.1,
                       "dz_mm": 0.1,
                       "measured": {"top_mm": 32.0,
                                    "center_base_mm": [420.0, -114.0],
                                    "yaw_deg": 0.0}}, fh)


def _release_stubs(calls, merged_msg):
    """把臂/相机全换成记账替身；返回还原函数。"""
    def fake_measure(*a, **k):
        if merged_msg is None:
            return {}, ""
        return {"merged": {"top_mm": 32.0, "center_base_mm": [420.0, -114.0],
                           "yaw_deg": 0.0}}, merged_msg

    def fake_return(*a, **k):
        calls.append(("return", a[5]))

    def fake_clear(*a, **k):
        calls.append(("clear", a[5], bool(a[8])))   # (…, cdir, tag, grid, slot, placed, …)

    def fake_capture(*a, **k):
        return "frame", {}

    def fake_place(*a, **k):
        calls.append(("place", k["plan_kwargs"]["place_open_mm"]))
        return {"measured": {"merged": {"top_mm": 32.0,
                                        "center_base_mm": [420.0, -114.0],
                                        "yaw_deg": 0.0}},
                "predicted_place_mm": [420.0, -114.0],
                "predicted_top_mm": 31.9}

    saved = {n: getattr(e, n) for n in
             ("measure_tower", "return_to_pick", "clear_tower_step",
              "capture_pick", "analyse_pick", "pick_item", "place_one",
              "ask")}
    e.ask = lambda *a, **k: None                  # 测试里不等回车
    e.measure_tower = fake_measure
    e.return_to_pick = fake_return
    e.clear_tower_step = fake_clear
    e.capture_pick = fake_capture
    e.analyse_pick = lambda *a, **k: {}
    e.pick_item = lambda *a, **k: [{"block_idx": 0, "yaw_place_deg": 0.0,
                                    "h": 29.5}]
    e.place_one = fake_place

    def restore():
        for n, fn in saved.items():
            setattr(e, n, fn)
    return restore


@case
def test_release_resume_returns_stuck_block_then_skips_done_reps():
    import tempfile
    name = "open50_dwell0.00"
    with tempfile.TemporaryDirectory() as td:
        _seed_release_rows(td, name, (1, 2))
        args, ctx = _release_env(td, reps=3, resume=True)
        calls = []
        restore = _release_stubs(calls, "塔上还有块")
        try:
            rc = e.mode_release(args, ctx)
        finally:
            restore()
        assert rc == 0, rc
        # 开工先由机器人把上次那块拆回
        assert ("return", "resume-clear") in calls, calls
        # 已完成的 1、2 次不重跑：只放第 3 次
        assert [c for c in calls if c[0] == "place"] == [("place", 50.0)], calls
        # 第 3 次之前塔是空的（placed=False），收尾才由机器人拆回
        assert ("clear", "open50_dwell0.00-03", False) in calls, calls
        assert ("clear", "release-end", True) in calls, calls
        with open(os.path.join(td, "entry-release", "summary.json")) as fh:
            doc = json.load(fh)
        assert len(doc["groups_raw"][name]) == 3, doc["groups_raw"][name]


@case
def test_release_resume_all_done_does_no_motion():
    import tempfile
    name = "open50_dwell0.00"
    with tempfile.TemporaryDirectory() as td:
        _seed_release_rows(td, name, (1, 2, 3))
        args, ctx = _release_env(td, reps=3, resume=True)
        calls = []
        restore = _release_stubs(calls, None)     # 塔位空着
        try:
            rc = e.mode_release(args, ctx)
        finally:
            restore()
        assert rc == 0, rc
        assert calls == [], calls                  # 不拆、不放、不重复收尾
        with open(os.path.join(td, "entry-release", "summary.json")) as fh:
            doc = json.load(fh)
        assert len(doc["groups_raw"][name]) == 3, doc["groups_raw"][name]


@case
def test_release_fresh_run_refuses_non_empty_tower():
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        args, ctx = _release_env(td, reps=3, resume=False)
        calls = []
        restore = _release_stubs(calls, "塔上还有块")
        try:
            rc = e.mode_release(args, ctx)
        finally:
            restore()
        assert rc == 2, rc
        assert calls == [], calls                  # 非续跑：不动臂、不瞎拆


# ---------------------------------------------------------------- z 标定来源
@case
def test_resolve_zcal_path_fail_closed():
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        # 显式路径：原样返回（存在与否交给 load_zcal 报错）
        assert e.resolve_zcal_path(td, "/x/zcal.json", "release") == "/x/zcal.json"
        # 标定模式自己没有输入
        assert e.resolve_zcal_path(td, None, "zcal") is None
        # release 缺文件 ⇒ 停，不许按 0 偏差静默跑
        try:
            e.resolve_zcal_path(td, None, "release")
        except e.EntryStop as exc:
            assert "缺 z 标定" in exc.why, exc.why
            assert exc.rc == 2
        else:
            raise AssertionError("缺 z 标定却没停")
        # 按约定位置放进一份 ⇒ 自动用它
        d = os.path.join(td, "entry-zcal")
        os.makedirs(d)
        p = os.path.join(d, "zcal.json")
        open(p, "w").write("{}")
        assert e.resolve_zcal_path(td, None, "release") == p


# ---------------------------------------------------------------- 后矫正（⑤）
@case
def test_nudge_delta_sign_and_mod90():
    """δxy = 目标 − 实测；yaw 折 mod 90 取最短角，块应转的方向与臂符号无关。"""
    tgt = [425.0, -114.86]
    d = e.nudge_delta([422.0, -114.86], 0.0, tgt)
    assert_close(d["dx_mm"], 3.0, 1e-9)
    assert_close(d["dy_mm"], 0.0, 1e-9)
    assert_close(d["d_mm"], 3.0, 1e-9)
    assert_close(d["dyaw_deg"], 0.0, 1e-9, "yaw 已经正了")
    # 块在 +7°：tool0 要转 −7°，块跟着转 −7°
    d = e.nudge_delta([425.0, -114.86], 7.0, tgt, yaw_sign=1.0)
    assert_close(d["dyaw_deg"], -7.0, 1e-9)
    assert_close(d["dyaw_block_deg"], -7.0, 1e-9)
    # 方块按 90° 周期：87° 该转 +3°，不是 −87°
    d = e.nudge_delta([425.0, -114.86], 87.0, tgt, yaw_sign=1.0)
    assert_close(d["dyaw_deg"], 3.0, 1e-9)
    # 臂的符号约定反过来时，tool0 的角反号，但**块该转的角度不变**
    d = e.nudge_delta([425.0, -114.86], 7.0, tgt, yaw_sign=-1.0)
    assert_close(d["dyaw_deg"], 7.0, 1e-9)
    assert_close(d["dyaw_block_deg"], -7.0, 1e-9)
    # 场景里 finger_axis_delta_deg 非零时，目标 tool0 角就是它
    d = e.nudge_delta([425.0, -114.86], 0.0, tgt, yaw_sign=1.0, delta_deg=2.5)
    assert_close(d["yaw_to_deg"], 2.5, 1e-9)


@case
def test_nudge_delta_yaw_quadrant_rotates_approach_only():
    """`--nudge-yaw-quadrant` 只改**接近朝向**，不改任何 δ。

    方块对 90° 自对称 ⇒ 转 90° 夹另一对侧面仍是同一个合法抓取，所以平移与
    转角判据必须逐字节不动，只有 `yaw_from_deg`（tool0 绕 z 的朝向）跟着转。
    这条是方向帧判别实验的**前提**：指轴转了 90°，夹爪系里的侧推就该跟着转。

    反向变异：把 `+ 90.0 * quad` 从 `yaw_from` 那行删掉 ⇒ 第 1 条立刻变红；
    若误加在 `dyaw` 上 ⇒ 第 3 条变红。
    """
    tgt = [422.65, -114.86]
    meas = [421.0, -120.0]
    base = e.nudge_delta(meas, 82.0, tgt, yaw_sign=1.0)
    q1 = e.nudge_delta(meas, 82.0, tgt, yaw_sign=1.0, yaw_quadrant=1)
    q2 = e.nudge_delta(meas, 82.0, tgt, yaw_sign=1.0, yaw_quadrant=2)
    q3 = e.nudge_delta(meas, 82.0, tgt, yaw_sign=1.0, yaw_quadrant=3)

    # 1) 朝向确实转了 90°（折 180 之后，差值的绝对值是 90）
    diff = e.pp.fold_yaw_180(q1["yaw_from_deg"] - base["yaw_from_deg"])
    assert_close(abs(diff), 90.0, 1e-9, (base["yaw_from_deg"],
                                         q1["yaw_from_deg"]))
    # 82° → −8°：落在 (−90, +90] 里的那一支
    assert_close(q1["yaw_from_deg"], -8.0, 1e-9, q1["yaw_from_deg"])

    # 2) 平移一格不动
    for d in (q1, q2, q3):
        for k in ("dx_mm", "dy_mm", "d_mm"):
            assert_close(d[k], base[k], 1e-9, k)
    # 3) 转角的**判据**不动（块按 90° 周期，等价解）
    for d in (q1, q2, q3):
        assert_close(d["dyaw_deg"], base["dyaw_deg"], 1e-9)
        assert_close(d["dyaw_block_deg"], base["dyaw_block_deg"], 1e-9)

    # 4) 平行两指绕工具轴转 180° 只是两根手指互换 ⇒ 0≡2、1≡3
    assert_close(q2["yaw_from_deg"], base["yaw_from_deg"], 1e-9)
    assert_close(q3["yaw_from_deg"], q1["yaw_from_deg"], 1e-9)

    # 5) 诊断档（--nudge-zero）必须把**转过的**朝向原样发给计划：
    #    否则转了 90° 的接近会退回原朝向，实验就白做了。
    z = e.applied_nudge_command(q1, zero=True)
    assert_close(z["yaw_from_deg"], q1["yaw_from_deg"], 1e-9)
    assert_close(z["dyaw_deg"], 0.0, 1e-9)
    assert_close(z["dx_mm"], 0.0, 1e-9)


@case
def test_applied_nudge_command_zero_only_touches_motion():
    """`--nudge-zero` 只把平移/转角置零，yaw 的朝向基准与实测 δ 本身不动。

    反向变异：`zero` 分支若漏置某一项（或落到了 `cmd` 上原地改），
    下面「原样对照」和「四项全零」两条必有一条变红。
    """
    cmd = e.nudge_delta([421.0, -120.0], 84.0, [422.65, -114.86],
                        yaw_sign=1.0)
    keep = dict(cmd)

    # 默认档：原样发给计划，且**不改动传入的 cmd**
    out = e.applied_nudge_command(cmd)
    assert out == {k: float(v) for k, v in keep.items()}, out
    assert cmd == keep, "applied_nudge_command 改了传进来的 cmd"

    # 诊断档：平移与转角四项全零，朝向基准 / δ 记录不变
    z = e.applied_nudge_command(cmd, zero=True)
    for k in ("dx_mm", "dy_mm", "d_mm", "dyaw_deg", "dyaw_block_deg"):
        assert_close(z[k], 0.0, 1e-9, k)
    assert_close(z["yaw_from_deg"], keep["yaw_from_deg"], 1e-9)
    assert_close(z["yaw_to_deg"], keep["yaw_to_deg"], 1e-9)

    # 置零后 nudge_row 不许崩：没有指令方向 ⇒ gain/perp 记 None，
    # 但位移量照报 —— 诊断档要的就是那个位移。
    before = _merged(421.0, -120.0, 84.0)
    after = _merged(421.0, -125.0, 84.0)
    row = e.nudge_row(1, z, before, after, [422.65, -114.86])
    assert row["push"]["gain_xy"] is None, row["push"]
    assert_close(row["push"]["d_actual_mm"], 5.0, 0.01)
    assert e.format_nudge_line("nudge-99", row)


@case
def test_nudge_push_pred_is_jaw_axis_projection():
    """预测的侧推只取 E 沿**指轴（tool0 x）**的分量，且指轴转 180° 就翻号。

    这一条是第一百八十九轮的判据本身：指轴 +y 时推 −y、−y 时推 +y
    （实测 −4.86 vs +3.87）。反向变异：换成「E 原样返回」（那会把侧向也算进去）、
    或把指轴取成 tool0 y，下面的数和翻号关系都对不上。
    """
    E = (-2.05, -4.78)
    # 指轴 = +x：只剩 E 的 x 分量
    px, py = e.nudge_push_pred(E, 0.0)
    assert_close(px, -2.05, 1e-9, "yaw=0 沿指轴")
    assert_close(py, 0.0, 1e-9, "yaw=0 垂直指轴必须为 0")
    # 指轴 = +y：只剩 E 的 y 分量
    px, py = e.nudge_push_pred(E, 90.0)
    assert_close(px, 0.0, 1e-9)
    assert_close(py, -4.78, 1e-9, "yaw=90 沿指轴")
    # 平行两指换手不算换姿态 ⇒ â 与 −â 是同一根指轴，预测必须**不变**
    # （写成 δ·â 就会随 180° 翻号，那是错的：实测 jaw +85° 与 −85° 两次都是推 −y）
    for yaw in (84.78, -85.48, 12.0):
        a = e.nudge_push_pred(E, yaw)
        b = e.nudge_push_pred(E, yaw + 180.0)
        for u, v in zip(a, b):
            assert_close(u, v, 1e-9, "180° 不变")
    # 量级不超过 |E|
    for yaw in (0.0, 45.0, 137.0):
        px, py = e.nudge_push_pred(E, yaw)
        assert (px ** 2 + py ** 2) ** 0.5 <= 5.20 + 1e-9


@case
def test_nudge_bias_comp_cancels_predicted_push():
    """补偿后块落在「实测 + 想要的 δ」；不补偿则偏一个 Δ，符号给反则偏两个。

    合成端到端：闭合时块被推走 Δ（就用模型的预测当「真值」），
    横移按 ``applied`` 走，则终态 = 实测 + Δ + applied。
    """
    E = (-2.05, -4.78)
    meas = [421.0, -120.0]
    want = (3.0, -2.0)
    for yaw in (0.0, 84.78, -85.48):
        target = [meas[0] + want[0], meas[1] + want[1]]
        cmd = e.nudge_delta(meas, yaw, target, yaw_sign=1.0)
        push = e.nudge_push_pred(E, cmd["yaw_from_deg"])

        def final(applied):
            return (meas[0] + push[0] + applied["dx_mm"],
                    meas[1] + push[1] + applied["dy_mm"])

        # 1) 不补偿：被推走一个 Δ
        raw = final(e.applied_nudge_command(cmd))
        assert_close(raw[0] - target[0], push[0], 1e-9, "yaw %.1f 不补偿 x" % yaw)
        assert_close(raw[1] - target[1], push[1], 1e-9, "yaw %.1f 不补偿 y" % yaw)

        # 2) 补偿：落在目标上
        comp = e.applied_nudge_command(cmd, bias_xy_mm=E)
        got = final(comp)
        assert_close(got[0], target[0], 1e-9, "yaw %.1f 补偿后 x" % yaw)
        assert_close(got[1], target[1], 1e-9, "yaw %.1f 补偿后 y" % yaw)

        # 3) 符号给反：误差翻倍（这一条守住「减」而不是「加」）
        bad = e.applied_nudge_command(cmd, bias_xy_mm=(-E[0], -E[1]))
        wrong = final(bad)
        assert_close(wrong[0] - target[0], 2.0 * push[0], 1e-9, "反号 x")
        assert_close(wrong[1] - target[1], 2.0 * push[1], 1e-9, "反号 y")

    # 4) 诊断档 + 补偿：δ 置零后，横移目标应当是 −Δ（把推走的那一下拉回来）
    cmd0 = e.nudge_delta(meas, 84.78, meas, yaw_sign=1.0)
    z = e.applied_nudge_command(cmd0, zero=True, bias_xy_mm=E)
    px, py = e.nudge_push_pred(E, z["yaw_from_deg"])
    assert_close(z["dx_mm"], -px, 1e-9)
    assert_close(z["dy_mm"], -py, 1e-9)
    assert_close(z["dyaw_deg"], 0.0, 1e-9)


@case
def test_nudge_bias_comp_spec_parsing():
    """`--nudge-bias-comp` 只收两个数；空 ⇒ 不补偿（不静默套常数）。"""
    assert e.nudge_bias_comp(None) is None
    assert e.nudge_bias_comp("") is None
    assert e.nudge_bias_comp("  ") is None
    assert e.nudge_bias_comp("-2.05,-4.78") == (-2.05, -4.78)
    assert e.nudge_bias_comp("-2.05, -4.78") == (-2.05, -4.78)
    assert e.nudge_bias_comp("-2.05，-4.78") == (-2.05, -4.78)   # 中文逗号
    try:
        e.nudge_bias_comp("1,2,3")
    except SystemExit:
        pass
    else:
        raise AssertionError("三个数应当拒收")


@case
def test_nearby_blocks_keeps_only_ring():
    """离格心 [25, 90] mm 的其它块才算邻块；目标自己、过远的、low 的都不算。"""
    blocks = [blk(0, 0.0, 0.0, 31.0, 29.5),          # 目标自己（距离 0）
              blk(1, 60.0, 0.0, 31.0, 29.5),         # 邻块
              blk(2, 200.0, 0.0, 31.0, 29.5),        # 太远
              blk(3, 0.0, 70.0, 5.0, 4.0, low=True)]  # 台面噪声
    out = e.nearby_blocks(blocks, [0.0, 0.0])
    assert [r["idx"] for r in out] == [1], out
    assert_close(out[0]["dist_mm"], 60.0, 1e-9)


@case
def test_neighbour_shift_pairs_or_refuses():
    """配得上就报移动量；数量/位置变了就记 None，不猜。"""
    b = [{"idx": 1, "center_mm": [60.0, 0.0]},
         {"idx": 2, "center_mm": [0.0, 70.0]}]
    a = [{"idx": 7, "center_mm": [61.5, 0.0]},
         {"idx": 8, "center_mm": [0.0, 110.0]}]        # 第二块跑了 40 mm
    out = e.neighbour_shift(b, a, match_mm=30.0)
    assert out[0] == {"before_idx": 1, "after_idx": 7, "move_mm": 1.5}, out[0]
    assert out[1]["after_idx"] is None and out[1]["move_mm"] is None, out[1]


def _merged(cx, cy, yaw_color, yaw_depth=None, top=31.5):
    return {"found": True, "center_base_mm": [cx, cy], "top_mm": top,
            "yaw_deg": yaw_depth, "yaw_color_deg": yaw_color}


@case
def test_nudge_row_gain_and_gate():
    """push gain = 实走/指令的投影比；终态按**彩色** yaw 判 §4.1 的 5 mm / 5°。"""
    cmd = dict(e.nudge_delta([421.0, 0.0], 6.0, [425.0, 0.0]), )
    before = _merged(421.0, 0.0, 6.0, yaw_depth=20.0)
    after = _merged(424.0, 0.0, 1.0, yaw_depth=3.0)
    row = e.nudge_row(1, cmd, before, after, [425.0, 0.0])
    assert_close(row["push"]["d_actual_mm"], 3.0, 1e-9)
    assert_close(row["push"]["gain_xy"], 0.75, 1e-9, "3 mm 实走 / 4 mm 指令")
    assert_close(row["push"]["perp_mm"], 0.0, 1e-9)
    assert_close(row["yaw"]["dyaw_actual_deg"], -5.0, 1e-9)
    assert_close(row["yaw"]["gain_yaw"], round(5.0 / 6.0, 2), 1e-9,
                 "行里的数一律保留 2 位")
    assert row["before"]["yaw_err_deg"] == 6.0, row["before"]
    assert row["after"]["xy_err_mm"] == 1.0
    assert row["final_ok"] is True, row
    # 深度 yaw 差 20° 也不影响判决（契约 §4.2.5：深度只报告）
    assert row["before"]["yaw_depth_deg"] == 20.0

    # 终态 yaw 仍差 8° ⇒ 不过门
    row = e.nudge_row(2, cmd, before, _merged(425.0, 0.0, 8.0), [425.0, 0.0])
    assert row["final_ok"] is False and row["reason"] == "still_off", row

    # 没有彩色 yaw ⇒ 证明不了，ok=None（不拿深度顶替）
    row = e.nudge_row(3, cmd, before, _merged(425.0, 0.0, None, yaw_depth=0.0),
                      [425.0, 0.0])
    assert row["final_ok"] is None and row["reason"] == "no_colour_yaw", row


@case
def test_nudge_pose_gate_enforces_contract_4_2_5():
    """契约 §4.2.5：两视点差 ≤10 mm / ≤6°，超了或证明不了 ⇒ 转人评，不许记成 ok。

    现实原型（2026-11-01 实测）：零位移诊断档第 1 次两视点差 **11.17 mm**，原来
    照样记成 ``status: ok`` 混进 ``n_final_ok``。这条测试把那个缺口钉住。
    """
    ok_b = dict(_merged(421.0, 0.0, 6.0), center_span_mm=2.5,
                yaw_color_span_deg=0.5)
    ok_a = dict(_merged(422.0, 0.0, 1.0), center_span_mm=1.8,
                yaw_color_span_deg=0.4)
    assert e.nudge_pose_gate(ok_b, ok_a) is None, "过门时不许报理由"

    # 中心两视点差 11.17 mm > 10 mm（真实那一行的数）
    bad_x = dict(ok_b, center_span_mm=11.17)
    why = e.nudge_pose_gate(bad_x, ok_a)
    assert why and "11.2 mm" in why and "矫正前" in why, why
    # 后半程坏一样拦，且理由点名「矫正后」
    why = e.nudge_pose_gate(ok_b, dict(ok_a, center_span_mm=12.0))
    assert why and "矫正后" in why, why
    # 彩色 yaw 两视点差 7° > 6°
    why = e.nudge_pose_gate(dict(ok_b, yaw_color_span_deg=7.0), ok_a)
    assert why and "yaw" in why, why
    # 只有一个视点能测到 ⇒ 证明不了一致 ⇒ 同样转人评
    lone = {k: v for k, v in ok_b.items() if k != "center_span_mm"}
    why = e.nudge_pose_gate(lone, ok_a)
    assert why and "只有一个视点" in why, why

    # 只查矫正前（``after`` 省略）时不许凭空判「矫正后」缺测
    assert e.nudge_pose_gate(ok_b) is None
    assert e.nudge_pose_gate(bad_x) is not None

    # 反向变异：把门拆掉（阈值放到无穷远）必须让上面那几条都放行，
    # 否则说明是别的分支在拦，测试根本没测到门。
    old = (e.m3.POSE_XY_TOL_MM, e.m3.POSE_YAW_TOL_DEG)
    try:
        e.m3.POSE_XY_TOL_MM = e.m3.POSE_YAW_TOL_DEG = float("inf")
        assert e.nudge_pose_gate(bad_x, ok_a) is None, "阈值拆除后仍报红 ⇒ 测试没测到门"
        assert e.nudge_pose_gate(dict(ok_b, yaw_color_span_deg=7.0)) is None
    finally:
        e.m3.POSE_XY_TOL_MM, e.m3.POSE_YAW_TOL_DEG = old


@case
def test_nudge_summary_splits_skipped_from_failed():
    """「没敢矫正」与「矫正了没修回来」分开计数。"""
    ok = {"status": "ok", "final_ok": True,
          "before": {"xy_err_mm": 6.0, "yaw_err_deg": 8.0},
          "after": {"xy_err_mm": 1.0, "yaw_err_deg": 1.0},
          "push": {"gain_xy": 0.9, "perp_mm": 0.2},
          "yaw": {"gain_yaw": 0.8}}
    bad = {"status": "ok", "final_ok": False,
           "before": {"xy_err_mm": 4.0, "yaw_err_deg": 2.0},
           "after": {"xy_err_mm": 4.5, "yaw_err_deg": 6.0},
           "push": {"gain_xy": 0.1, "perp_mm": 3.0},
           "yaw": {"gain_yaw": 1.5}}
    skipped = {"status": "out_of_range", "final_ok": None}
    s = e.nudge_summary([ok, bad, skipped])
    assert s["n"] == 3 and s["n_nudged"] == 2, s
    assert s["n_final_ok"] == 1 and s["n_skipped"] == 1, s
    assert s["gain_xy"]["n"] == 2 and s["gain_xy"]["median"] == 0.5, s
    assert s["xy_err_before"]["n"] == 2, s


@case
def test_format_nudge_line_never_raises_on_missing_values():
    """2026-11-01 真机事故回归：成功分支的打印曾是崩溃点。

    原版把 ``_fmt(...)`` 的**字符串**塞进 ``%.2f``，第 2 次一走到成功分支就
    ``TypeError``，把整轮 10 次打断。这条测试要求：正常行打得出、缺值（None）
    的 yaw 也能打、没矫正的行带原因。
    """
    ok = {"status": "ok", "final_ok": True,
          "before": {"xy_err_mm": 2.98, "yaw_err_deg": -5.22},
          "after": {"xy_err_mm": 4.77, "yaw_err_deg": -0.37},
          "push": {"gain_xy": 0.72},
          "yaw": {"dyaw_actual_deg": 4.85, "dyaw_expected_deg": 5.22}}
    line = e.format_nudge_line("nudge-02", ok)
    assert "2.98" in line and "4.77" in line and "过了" in line, line

    # 终态没有彩色 yaw（no_colour_yaw）⇒ 显示「—」，不抛异常
    blank = dict(ok, after={"xy_err_mm": 1.0, "yaw_err_deg": None},
                 yaw={"dyaw_actual_deg": None, "dyaw_expected_deg": 5.0})
    line = e.format_nudge_line("nudge-03", blank)
    assert "—" in line, line

    over = dict(ok, final_ok=False,
                after={"xy_err_mm": 9.0, "yaw_err_deg": 8.0})
    assert "仍超门" in e.format_nudge_line("nudge-04", over)

    skipped = {"status": "out_of_range", "reason": "δ 超上限"}
    line = e.format_nudge_line("nudge-05", skipped)
    assert "没矫正" in line and "δ 超上限" in line, line


# ---------------------------------------------------------------- 协议
@case
def test_protocol_non_empty_for_all_modes():
    for m in e.MODES:
        assert e.protocol(m).strip(), m


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

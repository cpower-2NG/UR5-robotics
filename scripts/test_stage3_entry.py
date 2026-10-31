#!/usr/bin/env python3
"""stage3_entry 的纯逻辑测试：z 标定落盘、释放重复性、遮挡分类、偏置向量。

跑法：
  python3 scripts/test_stage3_entry.py

不碰相机、不碰板子、不动臂：只喂合成数据。
真机路径（``--go``）在这里一律跑不到。
"""
import os
import sys

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
    args = types.SimpleNamespace(tower_cell_idx=4)
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
def test_pick_item_converts_runtime_error():
    """拾取区没块 ⇒ EntryStop，不是 traceback。"""
    import types
    orig = s3r.build_items
    s3r.build_items = lambda *a, **k: (_ for _ in ()).throw(
        RuntimeError("候选池只够 0 块（要 1）"))
    args = types.SimpleNamespace(arm="geometric")
    try:
        try:
            e.pick_item(args, {}, {"pick_zone": []}, {})
        except e.EntryStop as exc:
            assert "拾取区没有可用块" in exc.why, exc.why
            assert exc.rc == 2
        else:
            raise AssertionError("没有转成 EntryStop")
    finally:
        s3r.build_items = orig
    # 正常路径原样返回
    s3r.build_items = lambda *a, **k: [{"block_idx": 3}]
    try:
        assert e.pick_item(args, {}, {}, {})[0]["block_idx"] == 3
    finally:
        s3r.build_items = orig


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

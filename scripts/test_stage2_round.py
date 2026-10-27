#!/usr/bin/env python3
"""stage2_round 的纯逻辑自检（不启 ROS、不连板子、不动臂）。

钉住的事
────────
  · 场景：默认值、命令行快捷方式、9 格校验；``grid_order`` 与 grid.json 不一致
    必须**报错**（否则一半色格判错且现场看不出来）；
  · 选块：块顺序按类消耗、同色取候选最近的；没颜色的块单独列出、不静默抓；
  · 三臂：轮内轮流 + **轮次偏移**（三轮下来每臂 6 块）；
  · 两段 yaw：几何臂 ψ=−θ+δ、zero 臂 0/0、GraspNet 臂 候选角+δ、放置=δ；
    不可信的几何 yaw 要**标出来**而不是悄悄当 0；
  · 选格：同色优先、备用格通配、没槽就停（不硬塞）；
  · 跨姿态配对：同一块按位置合并；多出来的块发新 id（不冒充旧块）；
  · 判分配对（**关键**）：按颜色 + 离预期格最近做一对一，不看执行结果 ——
    拿掉颜色门槛就必须配对出错（变异测试），证明门槛真的在参与；
  · 计划接线：两段 yaw / 半开 / q-start 这些参数真的进了 precision_plan 的命令行；
  · ``place2`` 巡检位存在、与 ``place`` 拉开 ≥30 mm、且能解 IK（有 URDF 时）。

跑法：
    python3 scripts/test_stage2_round.py
"""

import json
import math
import os
import shutil
import sys
import tempfile

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np                                                # noqa: E402

import precision_plan as pp                                       # noqa: E402
import stage2_mat as sm                                           # noqa: E402
import stage2_pad as sp                                           # noqa: E402
import stage2_round as sr                                         # noqa: E402

CASES = []
TMP = tempfile.mkdtemp(prefix="stage2-round-test-")


def case(fn):
    CASES.append(fn)
    return fn


# ---------------------------------------------------------------- 夹具
def make_grid(order=None, yaw_deg=5.0, center_mm=(500.0, -300.0)):
    """合成 grid.json：垫子在 base (500,−300) mm、绕 z 转 ``yaw_deg``。"""
    meta = sm.build_meta(order or sm.DEFAULT_ORDER, 60.0)
    T = np.eye(4)
    a = math.radians(yaw_deg)
    T[:2, :2] = [[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]]
    T[:3, 3] = [center_mm[0] / 1000.0, center_mm[1] / 1000.0, 0.0]
    checks = {"page_a": {"n_markers": 10}, "page_b": {"n_markers": 10}}
    return sp.build_grid(meta, T, checks, T_A_B=np.eye(4).tolist())


def entry(idx, cls, x_mm, y_mm, dist=3.0, angle=10.0, yaw_pix=None,
          trusted=True):
    block = {"idx": idx, "cx": x_mm, "cy": y_mm, "top": 32.0, "h": 32.0,
             "yaw_pix": yaw_pix, "yaw_pix_trusted": trusted}
    best = {"angle_deg": angle, "width_mm": 26.0}
    p = {"block_idx": idx, "class": cls, "dist_mm": dist, "best": best}
    return {"p": p, "block": block, "best": best}


def scene(**kw):
    s = json.loads(json.dumps(sr.DEFAULT_SCENE))
    s.update(kw)
    return s


# ---------------------------------------------------------------- 场景
@case
def test_load_scene_defaults_and_shortcuts():
    s = sr.load_scene()
    assert s["arms_order"] == ["geometric", "zero", "graspnet"]
    assert s["survey_poses"] == ["place", "place2"]
    assert s["finger_axis_delta_deg"] == 0.0
    assert s["yaw_sign"] == -1.0
    assert s["blocks_per_round"] == 6

    ok = ",".join(sm.DEFAULT_ORDER)
    s = sr.load_scene(grid_order=ok, block_order=" red, cyan ,,")
    assert s["grid_order"] == sm.DEFAULT_ORDER
    assert s["blocks_order"] == ["red", "cyan"]

    try:
        sr.load_scene(grid_order="red,orange")
    except ValueError as exc:
        assert "9" in str(exc), exc
    else:
        raise AssertionError("只给 2 个格色竟然放行 —— 后 7 格会被悄悄写空")


@case
def test_scene_file_overrides_and_ignores_null():
    p = os.path.join(TMP, "scene.json")
    json.dump({"finger_axis_delta_deg": 2.5, "yaw_sign": 1.0,
               "blocks_order": None, "grid_json": "/tmp/g.json"}, open(p, "w"))
    s = sr.load_scene(p)
    assert s["finger_axis_delta_deg"] == 2.5 and s["yaw_sign"] == 1.0
    assert s["blocks_order"] == []            # None 不覆盖默认（默认就是空）
    assert s["grid_json"] == "/tmp/g.json"


@case
def test_grid_order_validation():
    grid = make_grid()
    sr.check_grid_order(scene(), grid)                    # 没配置 = 不校验
    sr.check_grid_order(scene(grid_order=list(sm.DEFAULT_ORDER)), grid)

    swapped = list(sm.DEFAULT_ORDER)
    swapped[0], swapped[1] = swapped[1], swapped[0]
    try:
        sr.check_grid_order(scene(grid_order=swapped), grid)
    except ValueError as exc:
        assert "不一致" in str(exc), exc
    else:
        raise AssertionError("格色顺序与垫子不同还放行 —— 现场一半色格会判错")

    bad = json.loads(json.dumps(grid))
    bad["cells"][2]["color"] = "purple"
    try:
        sr.check_grid_order(scene(grid_order=list(sm.DEFAULT_ORDER)), bad)
    except ValueError as exc:
        assert "格 2" in str(exc), exc
    else:
        raise AssertionError("grid.json 自己的格色被改过也没抓出来")


@case
def test_load_grid_requires_pad_fields():
    grid = make_grid()
    p = os.path.join(TMP, "grid-ok.json")
    json.dump(grid, open(p, "w"))
    assert sr.load_grid(p)["grid_order"] == sm.DEFAULT_ORDER

    bad = json.loads(json.dumps(grid))
    del bad["T_base_pad"]
    p2 = os.path.join(TMP, "grid-bad.json")
    json.dump(bad, open(p2, "w"))
    try:
        sr.load_grid(p2)
    except ValueError as exc:
        assert "T_base_pad" in str(exc), exc
    else:
        raise AssertionError("没有 T_base_pad 的 grid.json 被当成合格输入")


# ---------------------------------------------------------------- 选块
@case
def test_candidate_pool_filters():
    analysis = {"blocks": [{"idx": 0, "cx": 500.0, "cy": 300.0},
                           {"idx": 1, "cx": 300.0, "cy": 300.0},   # 出区
                           {"idx": 2, "cx": 500.0, "cy": 300.0}],  # 无候选
            "per_block": [{"block_idx": 0, "class": "red", "dist_mm": 3.0,
                           "best": {"angle_deg": 1.0, "width_mm": 26.0}},
                          {"block_idx": 1, "class": "cyan", "dist_mm": 2.0,
                           "best": {"angle_deg": 2.0, "width_mm": 26.0}},
                          {"block_idx": 2, "class": "purple", "dist_mm": 15.0,
                           "best": {"angle_deg": 3.0, "width_mm": 26.0}}]}
    pool = sr.candidate_pool(analysis, scene()["pick_zone"])
    assert [x["p"]["block_idx"] for x in pool] == [0], pool


@case
def test_choose_blocks_order_and_auto():
    pool = [entry(0, "red", 500, 300, dist=4.0), entry(1, "red", 560, 300, dist=1.0),
            entry(2, "cyan", 440, 340, dist=2.5), entry(3, "unknown", 420, 300)]
    chosen, missing, skipped = sr.choose_blocks(pool, order=["red", "purple"])
    assert [x["p"]["block_idx"] for x in chosen] == [1], chosen
    assert missing == ["purple"], missing
    assert [x["p"]["block_idx"] for x in skipped] == [3]

    chosen, missing, _ = sr.choose_blocks(pool, order=None, n=2)
    assert [x["p"]["block_idx"] for x in chosen] == [1, 2]
    assert missing == []

    chosen, _, _ = sr.choose_blocks(pool, order=["red", "red", "red"], n=9)
    assert len(chosen) == 2, "同色两块只该被消耗两次：%s" % chosen


@case
def test_assign_arm_and_round_offset():
    assert [sr.assign_arm(i, sr.ARMS) for i in range(4)] == \
        ["geometric", "zero", "graspnet", "geometric"]
    assert [sr.assign_arm(i, sr.ARMS, 1) for i in range(3)] == \
        ["zero", "graspnet", "geometric"]
    try:
        sr.assign_arm(0, [])
    except ValueError:
        pass
    else:
        raise AssertionError("空 arms_order 应报错")


# ---------------------------------------------------------------- yaw
@case
def test_yaw_for_arm_math():
    b = {"yaw_pix": 20.0, "yaw_pix_trusted": True}
    cand = {"angle_deg": 12.0}
    s = scene()
    g, p, src, note = sr.yaw_for_arm("geometric", b, cand, s)
    assert abs(g - 340.0) < 1e-9 and p == 0.0 and src == "geometric", (g, p, src)
    s = scene(finger_axis_delta_deg=5.0, yaw_sign=1.0)
    g, p, src, _ = sr.yaw_for_arm("geometric", b, cand, s)
    assert abs(g - 25.0) < 1e-9 and p == 5.0, (g, p)
    g, p, src, _ = sr.yaw_for_arm("zero", b, cand, s)
    assert (g, p, src) == (0.0, 0.0, "zero")
    g, p, src, _ = sr.yaw_for_arm("graspnet", b, cand, scene(finger_axis_delta_deg=3.0))
    assert abs(g - 15.0) < 1e-9 and p == 3.0 and src == "graspnet", (g, p)
    g, p, src, note = sr.yaw_for_arm("geometric", {"yaw_pix": None}, cand, s)
    assert (g, p, src) == (0.0, 5.0, "geometric_missing"), (g, p, src)
    assert note
    g, _, src, note = sr.yaw_for_arm(
        "geometric", {"yaw_pix": 20.0, "yaw_pix_trusted": False}, cand, s)
    assert src == "geometric_untrusted" and note, src
    assert abs(g - 25.0) < 1e-9, "不可信的 yaw 仍应给出算出来的值，只是要标源"
    try:
        sr.yaw_for_arm("magic", b, cand, s)
    except ValueError:
        pass
    else:
        raise AssertionError("不认识的臂应报错")


@case
def test_yaw_wrap_and_moduli():
    assert abs(sr.wrap_deg(-10.0) - 350.0) < 1e-9
    assert abs(sr.wrap_deg(370.0) - 10.0) < 1e-9
    g, _, _, _ = sr.yaw_for_arm("geometric", {"yaw_pix": 350.0}, {}, scene())
    assert abs(g - 10.0) < 1e-9, g


# ---------------------------------------------------------------- 分配
@case
def test_build_assignments_slots_and_order():
    grid = make_grid()
    pool = [entry(0, "cyan", 500, 300, dist=3.0, yaw_pix=10.0),
            entry(1, "red", 540, 320, dist=1.0, yaw_pix=350.0)]
    res = sr.build_assignments(pool, scene(blocks_order=["red", "cyan"]),
                               round_no=1, grid=grid)
    items = res["items"]
    assert [it["class"] for it in items] == ["red", "cyan"]
    assert items[0]["slot_color"] == "red" and items[0]["slot_idx"] == 0
    assert items[1]["slot_color"] == "cyan" and items[1]["slot_idx"] == 5
    assert items[0]["arm"] == "geometric" and items[1]["arm"] == "zero"
    # 世界 yaw：red 的像素 yaw 350° → ψ = −350° = 10°
    assert abs(items[0]["yaw_grasp_deg"] - 10.0) < 1e-9, items[0]

    res2 = sr.build_assignments(pool, scene(blocks_order=["red", "cyan"]),
                                round_no=2, grid=grid)
    assert res2["items"][0]["arm"] == "zero", "第 2 轮没整体偏移"


@case
def test_build_assignments_stops_when_no_slot():
    grid = make_grid()
    # 青色格只有 1 个（idx 5）+ 2 个备用格（7、8）= 最多 3 块。第 4 块必须停，
    # 不许硬塞进别人的格（那正是「安静地放错格」）。
    pool = [entry(i, "cyan", 500 + 5 * i, 300, dist=1.0 + i) for i in range(4)]
    res = sr.build_assignments(pool, scene(), round_no=1, grid=grid)
    assert [it["slot_idx"] for it in res["items"]] == [5, 7, 8], res["items"]
    assert "没有空槽" in (res["stop_reason"] or ""), res["stop_reason"]


@case
def test_build_assignments_uses_spare_wildcard():
    order = list(sm.DEFAULT_ORDER)
    order[7] = "spare"
    grid = make_grid(order)
    cells = {c["idx"]: c for c in grid["cells"]}
    assert cells[7]["color"] == "spare"
    pool = [entry(0, "purple", 500, 300, dist=1.0),
            entry(1, "purple", 540, 300, dist=2.0)]
    res = sr.build_assignments(pool, scene(blocks_per_round=2), round_no=1,
                               grid=grid)
    slots = [it["slot_idx"] for it in res["items"]]
    assert slots == [6, 7], "同色先用自己格、再用备用格：%s" % slots


@case
def test_build_assignments_skips_unknown_color():
    grid = make_grid()
    pool = [entry(0, "unknown", 500, 300), entry(1, "red", 540, 300)]
    res = sr.build_assignments(pool, scene(), round_no=1, grid=grid)
    assert [it["class"] for it in res["items"]] == ["red"]
    assert [x["p"]["block_idx"] for x in res["skipped"]] == [0]


@case
def test_cluster_warnings():
    items = [{"block_idx": 0, "block_xy_mm": [500.0, 300.0]},
             {"block_idx": 1, "block_xy_mm": [550.0, 300.0]},
             {"block_idx": 2, "block_xy_mm": [640.0, 300.0]}]
    w = sr.cluster_warnings(items)
    assert w == [(0, 1, 50.0)], w


# ---------------------------------------------------------------- 终态配对
@case
def test_match_block_ids_across_poses():
    p1 = [{"center_pad_mm": [0.0, 0.0]}, {"center_pad_mm": [60.0, 0.0]}]
    p2 = [{"center_pad_mm": [1.0, 1.0]}, {"center_pad_mm": [61.0, -1.0]},
          {"center_pad_mm": [120.0, 0.0]}]
    m = sr.match_block_ids([p1, p2])
    assert [b["block_id"] for b in m[0]] == [0, 1]
    assert [b["block_id"] for b in m[1]] == [0, 1, 2], "多出来的块必须发新 id"

    p3 = [{"center_pad_mm": [0.0, 0.0]}, {"center_pad_mm": [60.0, 0.0]}]
    p4 = [{"center_pad_mm": [25.0, 0.0]}, {"center_pad_mm": [60.0, 0.0]}]
    m = sr.match_block_ids([p3, p4])
    assert [b["block_id"] for b in m[1]] == [2, 1], \
        "挪了 25 mm（>10 mm 门限）的块被当成了同一块"


def _items_for_link():
    return [{"class": "red", "slot_idx": 0},
            {"class": "cyan", "slot_idx": 5}]


@case
def test_link_poses_by_color_not_position():
    grid = make_grid()
    cells = {c["idx"]: c for c in grid["cells"]}
    # 红块**放在了青格里**、青块放在了红格里 —— 判分必须报错，不能被配对抹平。
    red_at_cyan = [{"block_id": 0, "class": "red", "class_status": "ok",
                    "center_pad_mm": list(cells[5]["center_pad_mm"]),
                    "yaw_deg": 0.0},
                   {"block_id": 1, "class": "cyan", "class_status": "ok",
                    "center_pad_mm": list(cells[0]["center_pad_mm"]),
                    "yaw_deg": 0.0}]
    linked, unmatched = sr.link_poses([red_at_cyan], _items_for_link(), grid)
    got = {b["block_id"]: b["class"] for b in linked[0]}
    assert got == {0: "red", 1: "cyan"}, got
    assert not unmatched


@case
def test_link_poses_reports_unmatched_and_missing():
    grid = make_grid()
    cells = {c["idx"]: c for c in grid["cells"]}
    obs = [{"block_id": 0, "class": "red", "class_status": "ok",
            "center_pad_mm": list(cells[0]["center_pad_mm"]), "yaw_deg": 0.0},
           {"block_id": 1, "class": "purple", "class_status": "ok",
            "center_pad_mm": list(cells[2]["center_pad_mm"]), "yaw_deg": 0.0}]
    linked, unmatched = sr.link_poses([obs], _items_for_link(), grid)
    assert [b["class"] for b in linked[0]] == ["red"], linked
    assert [b["class"] for b in unmatched] == ["purple"], unmatched


@case
def test_link_gate_is_not_vacuous():
    """拿掉颜色门槛，同一份输入必须配出一个「颜色不对」的对 —— 证明门槛在参与。"""
    grid = make_grid()
    cells = {c["idx"]: c for c in grid["cells"]}
    obs = [{"block_id": 0, "class": "purple", "class_status": "ok",
            "center_pad_mm": list(cells[0]["center_pad_mm"]), "yaw_deg": 0.0}]
    items = _items_for_link()
    linked, unmatched = sr.link_poses([obs], items, grid)
    assert linked[0] == [] and len(unmatched) == 1
    linked2, unmatched2 = sr.link_poses([obs], items, grid, class_gate=False)
    assert [b["block_id"] for b in linked2[0]] == [0], linked2
    assert not unmatched2


@case
def test_block_means_averages_poses():
    matched = [[{"block_id": 0, "class": "red", "class_status": "ok",
                 "center_pad_mm": [0.0, 0.0], "yaw_deg": 1.0}],
               [{"block_id": 0, "class": None, "class_status": None,
                 "center_pad_mm": [2.0, 4.0], "yaw_deg": 3.0}]]
    m = sr.block_means(matched)[0]
    assert m["center_pad_mm"] == [1.0, 2.0]
    assert abs(m["yaw_deg"] - 2.0) < 1e-9
    assert m["class"] == "red" and m["n_poses"] == 2


# ---------------------------------------------------------------- 计划接线
@case
def test_pick_plan_passes_two_stage_yaw_and_half_open():
    calls = []

    class R:
        returncode = 0
        stderr = ""

    def fake_run(args, **kw):
        calls.append(args)
        return R()

    orig = sr.subprocess.run
    sr.subprocess.run = fake_run
    try:
        it = {"yaw_grasp_deg": 30.0, "yaw_place_deg": -15.0,
              "yaw_source": "geometric", "pick_open_mm": 50.0,
              "place_open_mm": 50.0}
        sr.pick_plan(os.path.join(TMP, "p.json"), "/tmp/c.json", 3,
                     [450.0, -250.0], it, "/tmp/r.urdf", "/tmp/frame")
        sr.survey_plan(os.path.join(TMP, "s.json"), "place2",
                       [0.0, -1.2, 1.4, -1.6, -1.5, 0.0], "/tmp/r.urdf")
    finally:
        sr.subprocess.run = orig
    assert len(calls) == 2, calls
    a, a2 = calls
    assert "--yaw-grasp-deg" in a and a[a.index("--yaw-grasp-deg") + 1] == "30.0000"
    assert a[a.index("--yaw-place-deg") + 1] == "-15.0000"
    assert a[a.index("--yaw-source") + 1] == "geometric"
    assert a[a.index("--pick-open-mm") + 1] == "50.000"
    assert a[a.index("--place-open-mm") + 1] == "50.000"
    assert a[a.index("--slot") + 1] == "0.450000,-0.250000"
    assert "--survey" in a2 and a2[a2.index("--survey") + 1] == "place2"
    assert a2[a2.index("--q-start") + 1].count(",") == 5


# ---------------------------------------------------------------- 指轴标定
@case
def test_signed_err_mod90():
    assert abs(sr.signed_err_mod90(1.0) - 1.0) < 1e-9
    assert abs(sr.signed_err_mod90(89.0) + 1.0) < 1e-9
    assert abs(sr.signed_err_mod90(46.0) + 44.0) < 1e-9
    assert abs(sr.signed_err_mod90(44.0) - 44.0) < 1e-9


@case
def test_calib_report_constant_offset():
    items = [{"arm": "geometric"}] * 3
    linked = [[{"block_id": 0, "class": "red", "class_status": "ok",
                "center_pad_mm": [0.0, 0.0], "yaw_deg": 2.0},
               {"block_id": 1, "class": "cyan", "class_status": "ok",
                "center_pad_mm": [60.0, 0.0], "yaw_deg": 2.5},
               {"block_id": 2, "class": "purple", "class_status": "ok",
                "center_pad_mm": [120.0, 0.0], "yaw_deg": 1.8}]]
    cal = sr.calib_report(linked, items)
    assert cal["verdict"] == "zero", cal
    assert abs(cal["delta_deg_estimate"] - 2.0) < 1e-9, cal
    assert cal["spread_deg"] < 3.0, cal
    assert cal["ok"], cal


@case
def test_calib_report_constant_4deg_is_offset_not_zero():
    """一致的 4° 偏置：δ 应给出 4°（不是 0），并要求重验。"""
    items = [{"arm": "geometric"}] * 3
    linked = [[{"block_id": i, "class": "red", "class_status": "ok",
                "center_pad_mm": [float(60 * i), 0.0], "yaw_deg": 4.0 + 0.2 * i}
               for i in range(3)]]
    cal = sr.calib_report(linked, items)
    assert cal["verdict"] == "constant_offset", cal
    assert 4.0 <= cal["delta_deg_estimate"] <= 4.5, cal
    assert cal["ok"]


@case
def test_calib_report_rejects_large_offset_and_spread():
    items = [{"arm": "geometric"}] * 3
    big = [[{"block_id": i, "class": "red", "class_status": "ok",
             "center_pad_mm": [float(60 * i), 0.0], "yaw_deg": 6.0}
            for i in range(3)]]
    cal = sr.calib_report(big, items)
    assert cal["verdict"] != "zero", "6° 常数偏置被当成 0 —— δ 会永远不生效"
    mixed = [[{"block_id": i, "class": "red", "class_status": "ok",
               "center_pad_mm": [float(60 * i), 0.0], "yaw_deg": v}
              for i, v in enumerate([10.0, -10.0, 6.0])]]
    cal = sr.calib_report(mixed, items)
    assert cal["verdict"] == "inconsistent" and not cal["ok"], cal
    assert "极差" in cal["reason"], cal


@case
def test_calib_report_too_few_blocks():
    cal = sr.calib_report([[]], [{"arm": "geometric"}])
    assert not cal["ok"] and cal["verdict"] == "insufficient"
    assert "没有量到" in cal["reason"]


# ---------------------------------------------------------------- 只抓帧（B1）
@case
def test_do_capture_sequence_and_gate():
    calls = []

    class B:
        rc = 0

        def push(self):
            calls.append("push")

        def joints(self, d):
            calls.append(("joints", os.path.basename(d)))
            return {"position": [0.0] * 6}

        def exec_plan(self, plan, log, go=False):
            calls.append(("exec", os.path.basename(plan), go))
            return B.rc

        def capture(self, d, tag):
            calls.append(("capture", tag))

    class R:
        returncode = 0
        stderr = ""

    orig = sr.subprocess.run
    sr.subprocess.run = lambda *a, **k: R()
    out = os.path.join(TMP, "cap-frame")
    try:
        rc, fdir = sr.do_capture(B(), "place", out, "/tmp/r.urdf", TMP, "tag-1")
        assert rc == 0 and fdir == out, (rc, fdir)
        assert calls == [("joints", "joints-place"), ("exec", "survey-place.json", True),
                         ("capture", "tag-1")], calls
        calls.clear()
        rc, fdir = sr.do_capture(B(), "place2", out, "/tmp/r.urdf", TMP, "tag-2",
                                 go=False)
        assert rc == 0 and fdir is None, "预演不应抓帧"
        assert ("capture", "tag-2") not in calls, calls
        B.rc = 3
        rc, fdir = sr.do_capture(B(), "pick", out, "/tmp/r.urdf", TMP, "tag-3")
        assert rc == 3 and fdir is None, "执行判红后不许抓帧"
        assert not [c for c in calls if c[0] == "capture" and c[1] == "tag-3"]
    finally:
        sr.subprocess.run = orig


@case
def test_capture_cli_gate_requires_go_and_out():
    p = sr.main(["--run-dir", TMP, "--urdf", "/tmp/r.urdf",
                 "--capture-pose", "place", "--capture-out", TMP + "/f"])
    assert p == 2, "没有 --go 就动臂了？"
    p = sr.main(["--run-dir", TMP, "--urdf", "/tmp/r.urdf",
                 "--capture-pose", "place"])
    assert p == 2, "没有 --capture-out 也放行"


# ---------------------------------------------------------------- place2
@case
def test_place2_survey_exists_and_is_separate():
    assert "place2" in pp.SURVEY_TOOL0, pp.SURVEY_TOOL0.keys()
    a, b = pp.SURVEY_TOOL0["place"], pp.SURVEY_TOOL0["place2"]
    d = math.dist(a, b) * 1000.0
    assert d >= 30.0, "两个终态视点只差 %.1f mm —— 多姿态一致性会变成共模误差" % d
    assert abs(a[2] - b[2]) < 1e-9, "高度不同会改变视场，先别动 z"


@case
def test_place2_survey_solves_ik(chain):
    import test_precision_plan as tp
    _blocks, _cand, q0 = tp.scene(chain)      # q0 = 拾取巡检位
    for which in ("place", "place2"):
        wps, checks, meta = pp.build_survey(chain, q0, which)
        assert wps and all(c.get("ok", True) for c in checks), checks
        assert meta["which"] == which


@case
def test_cli_exit_codes_before_touching_board():
    bad_scene = sr.main(["--run-dir", TMP, "--urdf", "/tmp/r.urdf",
                         "--grid-order", "red,orange"])
    assert bad_scene == 2, bad_scene
    missing_grid = sr.main(["--run-dir", TMP, "--urdf", "/tmp/r.urdf"])
    assert missing_grid == 2, missing_grid
    p = os.path.join(TMP, "grid-cli.json")
    json.dump(make_grid(), open(p, "w"))
    no_frame = sr.main(["--run-dir", TMP, "--urdf", "/tmp/r.urdf", "--grid", p])
    assert no_frame == 2, no_frame
    bad = json.loads(json.dumps(make_grid()))
    del bad["T_base_pad"]
    p2 = os.path.join(TMP, "grid-cli-bad.json")
    json.dump(bad, open(p2, "w"))
    bad_grid = sr.main(["--run-dir", TMP, "--urdf", "/tmp/r.urdf", "--grid", p2])
    assert bad_grid == 2, bad_grid


# ---------------------------------------------------------------- runner
def main():
    try:
        import test_precision_plan as tp
        urdf = tp.find_urdf()
        chain = tp.urkin.Chain(urdf) if urdf else None
    except Exception:                                            # noqa: BLE001
        chain = None
    if chain is None:
        print("!! 找不到 robot.urdf：place2 的 IK 用例会跳过 —— 跳过不等于通过")

    npass = nfail = nskip = 0
    for fn in CASES:
        needs_kin = fn.__code__.co_argcount > 0
        if needs_kin and chain is None:
            print("SKIP %s" % fn.__name__)
            nskip += 1
            continue
        try:
            fn(chain) if needs_kin else fn()
        except AssertionError as e:
            print("FAIL %s: %s" % (fn.__name__, e))
            nfail += 1
        except Exception as e:                                   # noqa: BLE001
            print("ERROR %s: %s: %s" % (fn.__name__, type(e).__name__, e))
            nfail += 1
        else:
            print("PASS %s" % fn.__name__)
            npass += 1
    print("\n%d PASS, %d FAIL, %d SKIP" % (npass, nfail, nskip))
    shutil.rmtree(TMP, ignore_errors=True)
    return 1 if nfail else 0


if __name__ == "__main__":
    sys.exit(main())

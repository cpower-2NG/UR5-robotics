#!/usr/bin/env python3
"""stage3_score 的纯逻辑测试：逐层状态机、阈值、稳定性、终态缺失、退出码。

跑法：
  python3 scripts/test_stage3_score.py

所有观测都是合成 dict —— 不读图、不碰真机。

反向变异（这几条必须能变红，否则判分器等于自我认证）
────────────────────────────────────────────────────
· 第 1 层必须判 ``baseline`` 且没有 z 增量（把它当达标层会被这条抓住）；
· z 增量 >2 mm 必须判 fail；
· 终态 ``measured`` 显式为 null 时不能崩，且要判 ``fail``（塔完整却量不到）；
· 未做人眼复核时任何 ``ok`` 都要降成 ``person_review``。
"""
import os
import sys

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import stage3_score as s3                                         # noqa: E402

CASES = []


def case(fn):
    CASES.append(fn)
    return fn


def layer_rec(k, top, cx=422.65, cy=-114.86, yaw=0.0, exec_rc=0,
              cons_ok=True, trusted=True, found=True, pred=None,
              pred_yaw=None, pred_top=None):
    merged = {"found": found, "center_base_mm": [cx, cy], "top_mm": top,
              "yaw_deg": yaw, "yaw_trusted": trusted}
    cons = {"ok": cons_ok,
            "reason": ("ok" if cons_ok is True else
                       "single_pose" if cons_ok is None else "pose_mismatch")}
    rec = {"layer": k, "block_idx": k, "class": "cyan", "exec_rc": exec_rc,
           "measured": {"merged": merged, "consistency": cons}}
    if pred is not None or pred_yaw is not None or pred_top is not None:
        rec["planned"] = {"predicted_center_base_mm": pred,
                          "yaw_place_deg": pred_yaw,
                          "predicted_top_mm": pred_top}
    return rec


def terminal(cx=422.65, cy=-114.86, top=90.5, yaw=0.0, wait_s=10.0):
    return {"wait_s": wait_s,
            "measured": {"merged": {"found": True,
                                    "center_base_mm": [cx, cy],
                                    "top_mm": top, "yaw_deg": yaw,
                                    "yaw_trusted": True},
                         "consistency": {"ok": True}}}


_UNSET = object()


def obs3(layers=None, term=_UNSET, **kw):
    """3 层全过的默认轮；kw 覆盖顶层字段。``term=None`` 表示**没有**终态。"""
    if layers is None:
        layers = [layer_rec(1, 31.5), layer_rec(2, 61.0), layer_rec(3, 90.5)]
    if term is _UNSET:
        term = terminal()
    o = {"version": 2, "kind": "stage3-round", "round": 1, "group": "open",
         "n_layers": 3, "block_h_mm": 29.5,
         "grid_center_base_mm": [422.65, -114.86],
         "grid_yaw_base_deg": 179.809,
         "layers": layers, "terminal": term,
         "human_review": {"approved": True, "note": ""}}
    o.update(kw)
    return o


# ---------------------------------------------------------------- 全过
@case
def test_all_pass_with_review():
    res = s3.score_round(obs3())
    st = [r["status"] for r in res["layers"]]
    assert st == ["baseline", "ok", "ok"], st
    assert res["layers"][0]["z_increment_mm"] is None, "第 1 层没有下层可比"
    assert res["layers"][1]["z_increment_mm"] == 29.5
    assert res["verdict"] == "ok" and s3.exit_code(res) == 0, res


@case
def test_first_layer_is_baseline_not_pass():
    """变异哨：第 1 层改成「达标层」会让这条红。"""
    res = s3.score_round(obs3())
    r1 = res["layers"][0]
    assert r1["status"] == "baseline", r1["status"]
    assert r1["z_increment_mm"] is None and r1["xy_rel_mm"] is None


@case
def test_absolute_offsets_are_report_only():
    lay = [layer_rec(1, 31.5, cx=425.0), layer_rec(2, 61.0, cx=430.0),
           layer_rec(3, 90.5, cx=435.0)]
    res = s3.score_round(obs3(layers=lay))
    assert [r["status"] for r in res["layers"]] == ["baseline", "ok", "ok"], \
        "绝对格心偏差只报告，不该把层判红"
    assert res["layers"][2]["xy_abs_mm"] > 10.0
    assert abs(res["layers"][2]["yaw_abs_deg"] - 0.191) < 0.01, \
        res["layers"][2]["yaw_abs_deg"]


# ---------------------------------------------------------------- 阈值
@case
def test_z_increment_threshold():
    lay = [layer_rec(1, 31.5), layer_rec(2, 63.5), layer_rec(3, 93.0)]
    res = s3.score_round(obs3(layers=lay))
    r2 = res["layers"][1]
    assert r2["z_err_mm"] == 2.5 and r2["status"] == "fail", r2
    assert res["verdict"] == "fail" and res["reason"] == "layer_threshold"


@case
def test_relative_xy_threshold():
    lay = [layer_rec(1, 31.5), layer_rec(2, 61.0),
           layer_rec(3, 90.5, cx=429.5)]
    res = s3.score_round(obs3(layers=lay))
    assert res["layers"][2]["xy_rel_mm"] == 6.85
    assert res["layers"][2]["status"] == "fail"


@case
def test_relative_yaw_threshold():
    lay = [layer_rec(1, 31.5), layer_rec(2, 61.0, yaw=2.0),
           layer_rec(3, 90.5, yaw=8.0)]
    res = s3.score_round(obs3(layers=lay))
    assert res["layers"][2]["yaw_rel_deg"] == 6.0
    assert res["layers"][2]["status"] == "fail"
    # mod 90：差 1° 不能因为绕了 90° 就算成 89°
    lay2 = [layer_rec(1, 31.5, yaw=89.5), layer_rec(2, 61.0, yaw=0.5)]
    res2 = s3.score_round(obs3(layers=lay2, n_layers=2, term=terminal(top=61.0)))
    assert res2["layers"][1]["yaw_rel_deg"] == 1.0, res2["layers"][1]


# ---------------------------------------------------------------- 人评
@case
def test_consistency_not_ok_is_person_review():
    lay = [layer_rec(1, 31.5), layer_rec(2, 61.0, cons_ok=None)]
    res = s3.score_round(obs3(layers=lay, n_layers=2, term=terminal(top=61.0)))
    assert res["layers"][1]["status"] == "person_review"
    assert res["verdict"] == "person_review"


@case
def test_untrusted_yaw_is_person_review():
    lay = [layer_rec(1, 31.5, trusted=False), layer_rec(2, 61.0)]
    res = s3.score_round(obs3(layers=lay, n_layers=2, term=terminal(top=61.0)))
    assert res["layers"][0]["status"] == "person_review"
    assert res["verdict"] == "person_review"


@case
def test_missing_human_review_blocks_ok():
    res = s3.score_round(obs3(human_review=None))
    assert all(r["status"] == "person_review" for r in res["layers"]), \
        [r["status"] for r in res["layers"]]
    assert res["verdict"] == "person_review" and s3.exit_code(res) == 1
    res2 = s3.score_round(obs3(human_review={"approved": False}))
    assert res2["verdict"] == "person_review"
    res3 = s3.score_round(obs3(human_review={"approved": True,
                                             "layers": {"2": False}}))
    assert res3["layers"][1]["status"] == "person_review"
    assert res3["layers"][2]["status"] == "ok", "只降被判否的那一层"


# ---------------------------------------------------------------- 缺数据
@case
def test_exec_failure_is_fail_not_missing():
    lay = [layer_rec(1, 31.5), layer_rec(2, 61.0),
           layer_rec(3, 90.5, exec_rc=4)]
    res = s3.score_round(obs3(layers=lay))
    assert res["layers"][2]["status"] == "fail"
    assert res["verdict"] == "fail"


@case
def test_missing_layer_counts_as_missing_data():
    lay = [layer_rec(1, 31.5), layer_rec(2, 61.0)]
    res = s3.score_round(obs3(layers=lay, n_layers=3))
    assert res["verdict"] == "missing_data" and s3.exit_code(res) == 2


@case
def test_measured_missing_is_missing():
    lay = [layer_rec(1, 31.5), layer_rec(2, 61.0, found=False)]
    res = s3.score_round(obs3(layers=lay, n_layers=2, term=terminal(top=61.0)))
    assert res["layers"][1]["status"] == "missing"
    assert res["verdict"] == "missing_data"


@case
def test_terminal_missing_after_complete_tower_is_fail():
    res = s3.score_round(obs3(term={"wait_s": 10.0,
                                    "measured": {"merged": {"found": False}}}))
    assert res["verdict"] == "fail"
    assert res["reason"] == "terminal_missing_after_complete_tower"


@case
def test_terminal_measured_is_null_does_not_crash():
    """回归：``measured`` 显式 null 曾让整条 elif 链 AttributeError。"""
    res = s3.score_round(obs3(term={"wait_s": 10.0, "measured": None}))
    assert res["verdict"] == "fail"
    assert res["stability"]["status"] == "missing"


@case
def test_terminal_absent_is_fail():
    res = s3.score_round(obs3(term=None))
    assert res["verdict"] == "fail"
    assert res["reason"] == "terminal_missing_after_complete_tower"


# ---------------------------------------------------------------- 稳定性
@case
def test_stability_ok():
    res = s3.score_round(obs3())
    assert res["stability"]["status"] == "ok", res["stability"]
    assert res["stability"]["dz_mm"] == 0.0


@case
def test_stability_drift_fail():
    res = s3.score_round(obs3(term=terminal(cx=426.0)))
    assert res["stability"]["status"] == "fail"
    assert res["verdict"] == "fail" and res["reason"] == "stability"


@case
def test_stability_collapse_is_fail():
    res = s3.score_round(obs3(term=terminal(top=79.0)))
    assert res["stability"]["status"] == "fail"
    assert any("collapse" in r for r in res["stability"]["reasons"]), \
        res["stability"]


@case
def test_stability_missing_when_terminal_yaw_absent():
    t = terminal()
    t["measured"]["merged"]["yaw_deg"] = None
    res = s3.score_round(obs3(term=t))
    assert res["stability"]["status"] == "missing"
    assert res["verdict"] == "missing_data"


# ---------------------------------------------------------------- 其他
@case
def test_release_report():
    lay = [layer_rec(1, 31.5),
           layer_rec(2, 61.0, pred=[422.0, -114.0], pred_yaw=0.0,
                     pred_top=61.5)]
    res = s3.score_round(obs3(layers=lay, n_layers=2, term=terminal(top=61.0)))
    rel = res["layers"][1]["release"]
    assert rel["dxy_mm"] is not None and rel["dyaw_deg"] is not None
    assert rel["dz_mm"] == -0.5, rel


@case
def test_grid_center_fallback_from_grid_doc():
    o = obs3()
    del o["grid_center_base_mm"]
    o["grid"] = {"grid_center_base_mm": [422.65, -114.86]}
    res = s3.score_round(o)
    assert res["grid_center_base_mm"] == [422.65, -114.86]
    assert res["layers"][2]["xy_abs_mm"] is not None


@case
def test_exit_codes():
    assert s3.exit_code({"verdict": "ok"}) == 0
    assert s3.exit_code({"verdict": "fail"}) == 1
    assert s3.exit_code({"verdict": "person_review"}) == 1
    assert s3.exit_code({"verdict": "missing_data"}) == 2


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

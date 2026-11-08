#!/usr/bin/env python3
"""demo_stage3 的纯逻辑测试：参数拼装、场景覆盖、b̂ 先验、失败即拒。

跑法：
  python3 scripts/test_demo_stage3.py

不连板、不动臂、不读图。反向变异要能抓住的几条：

  · ``--bias-seed`` 必须用 ``=`` 形式发出去（负值分开写会被 argparse 当选项）；
  · 真跑必须带 ``--go``、离线必须不带 —— 反过来就是「预演动臂」或「真跑不动」；
  · 融合帧数只改 scene 的两个键，别的一个字不能动；
  · stdin 不是交互终端时 ``ask_yes`` 必须返回 False（红线 6 的 fail-closed）。
"""
import argparse
import json
import os
import sys
import tempfile

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import demo_stage3 as d                                          # noqa: E402

CASES = []


def case(fn):
    CASES.append(fn)
    return fn


def ns(**kw):
    base = dict(n_layers=3, frames=3, dry_run=False,
                urdf="/u.urdf", zcal="/z.json",
                frame="/f", joints="/j.json")
    base.update(kw)
    return argparse.Namespace(**base)


def tmpdir():
    return tempfile.mkdtemp(prefix="demo3-")


# ---------------------------------------------------------------- 参数
@case
def test_round_argv_real_has_go():
    argv = d.round_argv(ns(), "/run", "/g.json", "/s.json", None)
    assert "--go" in argv
    assert "--frame" not in argv and "--joints" not in argv
    for flag in ("--n-layers", "--group", "--arm", "--place-target",
                 "--bias-comp", "--nudge-axes", "--place-clearance-mm",
                 "--d-mm", "--terminal-wait-s"):
        assert flag in argv, flag


@case
def test_round_argv_dry_has_no_go():
    argv = d.round_argv(ns(dry_run=True), "/run", "/g.json", "/s.json", None)
    assert "--go" not in argv
    assert argv[argv.index("--frame") + 1] == "/f"
    assert argv[argv.index("--joints") + 1] == "/j.json"


@case
def test_bias_seed_single_equals_arg():
    argv = d.round_argv(ns(), "/run", "/g.json", "/s.json", "-2.791,-0.248")
    seeds = [a for a in argv if a.startswith("--bias-seed")]
    assert seeds == ["--bias-seed=-2.791,-0.248"], seeds


@case
def test_teardown_argv_has_go_and_flag():
    """拆塔是在判分之后单独调的：必须带 --teardown-only，且仍带 --go。"""
    argv = d.teardown_argv(ns(), "/run", "/g.json", "/s.json", None)
    assert "--teardown-only" in argv
    assert "--go" in argv
    assert "--frame" not in argv and "--joints" not in argv
    for flag in ("--scene", "--grid", "--zcal", "--urdf", "--d-mm",
                 "--tower-half-edge"):
        assert flag in argv, flag


@case
def test_teardown_decision_ignores_build_rc():
    """person_review 的 rc=1 不是失败：复核后重判 ok 必须仍然拆塔。

    这里只验纯函数；主流程里 rc 只是打印用，不进闸门。
    """
    assert d.teardown_decision("ok", False) == "teardown"
    assert d.teardown_decision("ok", True) == "keep"
    for bad in ("person_review", "fail", "missing_data", None):
        assert d.teardown_decision(bad, False) == "leave", bad
        assert d.teardown_decision(bad, True) == "leave", bad


# ---------------------------------------------------------------- 场景
@case
def test_scene_only_overrides_frames():
    raw = {"pick_zone": [0.25, 0.575, 0.17, 0.42], "pick_open_mm": 50.0,
           "finger_axis_delta_deg": 0.0}
    p = os.path.join(tmpdir(), "scene.json")
    json.dump(raw, open(p, "w"))
    out = d.make_scene(ns(scene=p, frames=2), os.path.dirname(p))
    got = json.load(open(out))
    assert got["pick_frames"] == 2 and got["terminal_frames"] == 2
    assert got["pick_zone"] == raw["pick_zone"]
    assert got["pick_open_mm"] == 50.0
    assert got["finger_axis_delta_deg"] == 0.0


# ---------------------------------------------------------------- b̂
@case
def test_bias_seed_requires_v3():
    p = os.path.join(tmpdir(), "bias.json")
    json.dump({"version": 2, "bias_mm": [-1.0, 2.0]}, open(p, "w"))
    assert d.bias_seed(p) is None
    json.dump({"version": 3, "bias_mm": [-2.791, -0.248]}, open(p, "w"))
    assert d.bias_seed(p) == "-2.791,-0.248"
    json.dump({"version": 3, "bias_mm": [1]}, open(p, "w"))
    assert d.bias_seed(p) is None


@case
def test_bias_seed_missing_file():
    assert d.bias_seed(os.path.join(tmpdir(), "nope.json")) is None


# ---------------------------------------------------------------- 人在环
@case
def test_ask_yes_rejects_non_tty():
    class FakeStdin:
        def isatty(self):
            return False

    old = sys.stdin
    sys.stdin = FakeStdin()
    try:
        log = d.Log(os.path.join(tmpdir(), "demo.log"))
        assert d.ask_yes(log, "? ") is False
        assert d.authorized(log, 3) is False
        log.close()
    finally:
        sys.stdin = old


# ---------------------------------------------------------------- 杂项
@case
def test_round_dirs_filters():
    p = tmpdir()
    for name in ("round-01", "round-2", "round-x", "notes", "score.json"):
        os.makedirs(os.path.join(p, name), exist_ok=True)
    assert d.round_dirs(p) == {"round-01", "round-2"}


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

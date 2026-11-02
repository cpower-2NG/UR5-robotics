#!/usr/bin/env python3
"""stage3_plan：载运高度、表面语义、1–5 层堆叠计划、塔体护栏的变异测试。

跑法：
  ~/robotics/venvs/robot-pose/bin/python scripts/test_stage3_plan.py

运动学部分要一份机器人 URDF（**不进仓库**，从板上取的那一份）：
  $PRECISION_URDF，或 ~/robotics/notes/ur5-vision-grasp/runs/*/robot.urdf 里最新的一份。
找不到就跳过并**大声说明跳过了**（跳过不算通过）。

四条必须变红的变异
──────────────────
1. 载运高度降回 ``lift_z``（第 5 层会塔侧横移）→ ``test_carry_height_*`` 红；
2. 塔体护栏关掉 / ``below_tip`` 漏掉裸指尖 204.2 mm → ``test_mutation_*`` 红；
3. 下一层表面用「本层脚下表面」而不是「校正顶高」→ ``test_layer_surface_*`` 红；
4. 塔体 box 不接进计划（只算不查）→ 同一个护栏用例红。
"""
import glob
import os
import sys

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np                                                # noqa: E402

import precision_plan as pp                                       # noqa: E402
import stage3_round as s3r                                        # noqa: E402
import urkin                                                      # noqa: E402

CASES = []


def case(fn):
    CASES.append(fn)
    return fn


def assert_close(a, b, tol=1e-9, why=""):
    assert abs(float(a) - float(b)) <= tol, \
        "%s: %r vs %r（容差 %g）" % (why or "数值", a, b, tol)


def find_urdf():
    env = os.environ.get("PRECISION_URDF")
    if env:
        return env if os.path.exists(env) else None
    pat = os.path.expanduser("~/robotics/notes/ur5-vision-grasp/runs/*/robot.urdf")
    hits = sorted(glob.glob(pat), key=os.path.getmtime)
    return hits[-1] if hits else None


H_BLK = 0.0295                       # 真高 29.5 mm（2026-10 用户多次实测）
PAD_Z = 0.002                        # grid.json T_base_pad[2,3]
SLOT_MM = (422.65, -114.86)          # 中心格 idx4


def surface_for(k, bias=0.0):
    """第 k 层的**理想**放置表面（无偏差）。"""
    return PAD_Z if k == 1 else PAD_Z + (k - 1) * H_BLK


def expected_place_z(surface):
    """29.5 mm 块：place_z = surface + 0.2202（推导见契约 §3.4）。"""
    grasp_z = H_BLK + pp.TOOL0_TO_FINGERTIP + pp.GRASP_CLEARANCE
    grasp_depth = H_BLK - (grasp_z - pp.TOOL0_TO_BARE_FINGERTIP)
    below_tip = H_BLK - grasp_depth
    return pp.TOOL0_TO_BARE_FINGERTIP + surface + below_tip + pp.PLACE_CLEARANCE


# ---------------------------------------------------------------- 纯公式
@case
def test_carry_height_clears_place_approach():
    """载运高度 = max(lift, place_approach + 50 mm)；第 5 层必须由后者接管。"""
    for k in range(1, 6):
        surf = surface_for(k)
        carry = pp.stage3_carry_z(H_BLK, surf)
        place_z = expected_place_z(surf)
        approach = place_z + pp.PLACE_APPROACH_GAP
        lift = H_BLK + pp.TOOL0_TO_FINGERTIP + pp.LIFT_HEIGHT
        assert carry >= lift - 1e-12, "载运高度低于 lift_z（k=%d）" % k
        assert carry >= approach + 0.05 - 1e-12, \
            "第 %d 层横移时块底只高过接近点 %.1f mm" % (
                k, (carry - approach) * 1000)
    # 低层由 lift_z 决定，高层由塔顶决定
    assert_close(pp.stage3_carry_z(H_BLK, 0.0),
                 H_BLK + pp.TOOL0_TO_FINGERTIP + pp.LIFT_HEIGHT)
    c5 = pp.stage3_carry_z(H_BLK, surface_for(5))
    a5 = expected_place_z(surface_for(5)) + pp.PLACE_APPROACH_GAP
    assert_close(c5, a5 + 0.05, 1e-9, "第 5 层载运高度")


@case
def test_layer_surface_uses_corrected_top_not_below_surface():
    """变异哨：拿 ``surface_est_mm``（本层脚下）当下一层表面会差一整块。"""
    prev = {"surface_est_mm": 31.5, "next_surface_est_mm": 61.0}
    assert_close(s3r.layer_surface_mm(2, prev, PAD_Z * 1000.0), 61.0)
    assert_close(s3r.layer_surface_mm(1, prev, PAD_Z * 1000.0), 2.0)
    try:
        s3r.layer_surface_mm(2, {"surface_est_mm": 31.5}, 2.0)
    except RuntimeError:
        pass
    else:
        raise AssertionError("没有校正顶高时必须报错，不能退回脚下表面")
    wrong = s3r.layer_surface_mm(2, prev, 2.0) - prev["surface_est_mm"]
    assert_close(wrong, H_BLK * 1000.0, 1e-9, "两个量的差必须是一整块")


@case
def test_closed_loop_yaw_uses_colour_not_depth():
    """闭环目标必须和判分同源（彩色掩码，契约 §4.2.5）。

    这一对数是入口③ 5 层的真值：同一座**静止**的塔，基准帧深度 89.334 /
    彩色 1.652，持块帧深度 14.5 / 彩色 1.6。闭环若拿深度，第二层就要把块
    拧 15° 去「对齐」一个并不存在的朝向。
    """
    m = {"found": True, "yaw_deg": 14.5, "yaw_trusted": True,
         "yaw_color_deg": 1.652, "yaw_color_trusted": True}
    yaw, src = s3r.closed_loop_yaw(m)
    assert_close(yaw, 1.652, 1e-9, "闭环 yaw 必须是彩色那一路")
    assert src == "color", src


@case
def test_closed_loop_yaw_refuses_without_colour():
    """彩色判不了就返回 None（调用方停轮），**不拿深度顶替**。"""
    for m, want in (({"found": True, "yaw_deg": 14.5, "yaw_trusted": True,
                      "yaw_color_deg": None, "yaw_color_trusted": None},
                     "no_color_yaw"),
                    ({"found": True, "yaw_deg": 14.5, "yaw_trusted": True,
                      "yaw_color_deg": 1.6, "yaw_color_trusted": False},
                     "color_yaw_untrusted")):
        yaw, src = s3r.closed_loop_yaw(m)
        assert yaw is None, "深度值 %r 不许顶替" % m["yaw_deg"]
        assert src == want, (src, want)


@case
def test_eih_group_refuses_instead_of_silently_running_open():
    """``--group eih`` 只占了个名字：选它必须被拒，不能跑成开环还记成 eih。"""
    assert s3r.unsupported_group("open") is None
    assert s3r.unsupported_group("closed") is None
    why = s3r.unsupported_group("eih")
    assert why and "eih" in why, "eih 必须给出明确的拒绝原因"
    # 真入口：要在**任何动臂动作之前**返回 2。scene 故意给个不存在的路径——
    # 若闸门排在 load_scene 之后，这里就会因别的原因失败而暴露出次序错了。
    import argparse
    ns = argparse.Namespace(group="eih", scene="/nonexistent/scene.json")
    rc = s3r.run_round(ns)
    assert rc == 2, "run_round 对 eih 应返回 2（fail-closed），实得 %r" % (rc,)


@case
def test_bias_from_layer_is_measured_minus_commanded():
    """b̂ = 实测 − **命令**（不是期望）。两者分开记，否则补偿会被算两遍。"""
    cmd = (422.65, -114.86)
    bias, why = s3r.bias_from_layer((419.0, -118.0), cmd)
    assert why is None, why
    assert_close(bias[0], 419.0 - 422.65, 1e-9, "bx")
    assert_close(bias[1], -118.0 + 114.86, 1e-9, "by")
    # 没有测量 ⇒ 判不了，不能默认 0（默认 0 = 悄悄退回开环）。
    b2, why2 = s3r.bias_from_layer(None, cmd)
    assert b2 is None and why2 == "measured_missing", (b2, why2)


@case
def test_bias_over_limit_is_a_failure_not_a_bias():
    """30 mm 的「偏置」是块掉了 / 没抓住 —— 停轮，不许拿去推下一层。"""
    cmd = (422.65, -114.86)
    bias, why = s3r.bias_from_layer((422.65 + 30.0, -114.86), cmd)
    assert bias is None, "超上限必须拒绝，实得 %r" % (bias,)
    assert why and why.startswith("bias_too_large"), why
    # 上限之内照收：刚好 15.0 mm 不拒（判据是 >）。
    ok, why_ok = s3r.bias_from_layer((422.65 + 15.0, -114.86), cmd)
    assert why_ok is None and ok[0] == 15.0, (ok, why_ok)


@case
def test_target_with_bias_cancels_a_repeating_offset():
    """轮内自适应：命令 = 期望 − 历史均值；下一层若偏得一样多，就落在期望上。"""
    desired = (422.65, -114.86)
    # 第 1 层没有历史 ⇒ 不猜，命令 = 期望。
    assert s3r.target_with_bias(desired, []) == [422.65, -114.86]
    # mode=none ⇒ 旧的纯开环语义，逐字节一样。
    hist = [[-3.0, -2.0]]
    assert s3r.target_with_bias(desired, hist, "none") == [422.65, -114.86]
    # 历史均值 (−3, −2) ⇒ 命令右移 3、下移 2，抵消掉那个固定偏置。
    t = s3r.target_with_bias(desired, hist)
    assert_close(t[0], 422.65 + 3.0, 1e-9, "命令 x")
    assert_close(t[1], -114.86 + 2.0, 1e-9, "命令 y")
    # 性质：若这一层仍偏 (−3, −2)，落点 = 命令 + b̂ = 期望。
    bias, _ = s3r.bias_from_layer((t[0] - 3.0, t[1] - 2.0),
                                  (t[0], t[1]))
    assert_close(bias[0], -3.0, 1e-9, "b̂x")
    assert_close(t[0] + bias[0], desired[0], 1e-9, "落点 x 回到期望")
    # 历史取均值，不是最后一条：两层 (−3,−2)、(−1,−4) ⇒ (−2,−3)。
    t2 = s3r.target_with_bias(desired, [[-3.0, -2.0], [-1.0, -4.0]])
    assert_close(t2[0], 422.65 + 2.0, 1e-9, "均值 x")
    assert_close(t2[1], -114.86 + 3.0, 1e-9, "均值 y")


@case
def test_commanded_target_converts_mm_bias_into_metres():
    """接线那一行：期望是米、b̂ 是毫米 —— 换算错了塔会偏 1000 倍。"""
    desired_m = (0.42265, -0.11486)
    t = s3r.commanded_target_m(desired_m, [[-3.0, -2.0]])
    assert_close(t[0], 0.42565, 1e-9, "命令 x（m）")
    assert_close(t[1], -0.11286, 1e-9, "命令 y（m）")
    assert s3r.commanded_target_m(desired_m, []) == [0.42265, -0.11486]
    assert (s3r.commanded_target_m(desired_m, [[-3.0, -2.0]], "none")
            == [0.42265, -0.11486])


@case
def test_bias_seed_compensates_layer_one():
    """**种子是必需的**：没有它，第 1 层不补、第 2 层补满 ⇒ 1↔2 层一个 |b̂| 折角。

    判分看相对对中，4 mm 折角直接吃掉「≤5 mm」。种子让第 1 层也补偿。
    """
    desired = (422.65, -114.86)
    seed = [-3.5, -2.5]
    # 第 1 层：历史空，但种子在 ⇒ 命令 = 期望 − 种子。
    t1 = s3r.target_with_bias(desired, [], seed=seed)
    assert_close(t1[0], 426.15, 1e-9, "第 1 层命令 x")
    assert_close(t1[1], -112.36, 1e-9, "第 1 层命令 y")
    # 第 2 层：种子与实测一起平均（不是丢掉种子）。
    t2 = s3r.target_with_bias(desired, [[-2.5, -1.5]], seed=seed)
    assert_close(t2[0], 422.65 + 3.0, 1e-9, "均值 x")
    assert_close(t2[1], -114.86 + 2.0, 1e-9, "均值 y")
    # mode=none 时种子也不许生效。
    assert s3r.target_with_bias(desired, [], "none", seed=seed) == [422.65, -114.86]

    # 性质检验：种子 = 真实偏置 b 时，每层的落点都落在期望上、层间无折角。
    b = [-3.0, -2.0]
    land = [s3r.target_with_bias(desired, hist, seed=b) for hist in
            ([], [b], [b, b])]
    for t in land:
        assert_close(t[0] + b[0], desired[0], 1e-9, "落点 x")
        assert_close(t[1] + b[1], desired[1], 1e-9, "落点 y")
    for i in range(1, len(land)):
        assert_close(land[i][0] - land[i - 1][0], 0.0, 1e-9, "层间无折角")


@case
def test_bias_seed_parsing_and_round_estimate():
    """``--bias-seed`` 的解析与轮间估计的合并（下一轮第 1 层的先验从这来）。"""
    assert s3r.parse_bias_seed(None) is None
    assert s3r.parse_bias_seed("") is None
    assert s3r.parse_bias_seed("  ") is None
    assert s3r.parse_bias_seed("-3.56,-2.56") == [-3.56, -2.56]
    assert s3r.parse_bias_seed("-3.56, -2.56") == [-3.56, -2.56]
    assert s3r.parse_bias_seed("-3.56，-2.56") == [-3.56, -2.56]   # 中文逗号
    for bad in ("1", "1,2,3", "a,b"):
        try:
            s3r.parse_bias_seed(bad)
        except ValueError:
            continue
        raise AssertionError("坏种子 %r 必须被拒" % (bad,))

    # 第 1 轮：两层 b̂ → 估计 = 它们的均值，n=2。
    e1 = s3r.merge_bias_estimate(None, [[-3.0, -2.0], [-1.0, -4.0]])
    assert e1["n"] == 2 and e1["bias_mm"] == [-2.0, -3.0], e1
    # 第 2 轮：再加一层 ⇒ n 加权，不是只留最后一轮。
    e2 = s3r.merge_bias_estimate(e1, [[0.0, 0.0]])
    assert e2["n"] == 3, e2
    assert_close(e2["bias_mm"][0], -4.0 / 3.0, 1e-3, "合并 x")   # 落盘保留 3 位
    assert_close(e2["bias_mm"][1], -2.0, 1e-9, "合并 y")

    # 落盘→读回：下一轮的种子就是这么来的；坏文件当没有，不炸整轮。
    import json
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, s3r.BIAS_ESTIMATE_FILE)
        with open(p, "w") as fh:
            json.dump(e2, fh)
        assert s3r.load_bias_estimate(p) == e2["bias_mm"]
        with open(p, "w") as fh:
            fh.write("{not json")
        assert s3r.load_bias_estimate(p) is None
        with open(p, "w") as fh:
            json.dump({"bias_mm": [1.0]}, fh)
        assert s3r.load_bias_estimate(p) is None


@case
def test_bias_comp_flag_reaches_the_round():
    """``--bias-comp`` 必须是真开关：拼错的取值要在解析期就拒，不能静默补/不补。"""
    base = ["--run-dir", "/tmp/x", "--urdf", "/tmp/u.urdf", "--n-layers", "3"]
    rc = None
    try:
        s3r.main(base + ["--bias-comp", "bogus"])
    except SystemExit as exc:          # argparse 对非法取值 exit(2)
        rc = exc.code
    assert rc == 2, "非法 --bias-comp 应被 argparse 拒，实得 %r" % (rc,)


@case
def test_survey_pose_tracks_tower_top():
    """相机—塔顶距离恒定；塔越高 tool0 越高。"""
    d = pp.STAGE3_SURVEY_DEFAULT_D
    dx, dy, dz = pp.STAGE3_CAMERA_OFFSET
    for top in (0.0315, 0.1475):
        p0, p1 = pp.stage3_survey_tool0(top, (0.42265, -0.11486), distance=d)
        assert_close(p0[2], top - dz + d, 1e-12)
        assert_close(p0[0], 0.42265 - dx, 1e-12)
        assert_close(p1[0] - p0[0], pp.STAGE3_SURVEY_SECOND_OFFSET[0], 1e-12)
        assert_close(p1[1] - p0[1], pp.STAGE3_SURVEY_SECOND_OFFSET[1], 1e-12)
        assert_close(p1[2], p0[2], 1e-12, "两个视点必须同高")


# ---------------------------------------------------------------- 运动学
def scene(chain):
    """合成拾取场景：一块 29.5 mm，邻块 78 mm 外。"""
    blocks = [
        {"idx": 2, "class": "purple", "cx": 520.0, "cy": 330.0,
         "top": 29.5, "h": 29.5},
        {"idx": 3, "class": "cyan", "cx": 460.0, "cy": 380.0,
         "top": 29.5, "h": 29.5},
    ]
    cand = {"score": 0.31, "width_mm": 26.2, "depth": 0.011, "dist_mm": 5.0,
            "angle_deg": 12.0, "seed": 0, "tilt_deg": 4.0,
            "xyz_base": [0.523, 0.331, 0.0295],
            "approach_base": [0.0, 0.0, -1.0]}
    q0, pe, re, _ = chain.ik(pp.target_matrix(pp.SURVEY_TOOL0["pick"]),
                             [0.0, -1.2, 1.4, -1.6, -1.5, 0.0])
    assert pe < 0.1 and re < 0.05, "巡检位自己就解不出来：%.3f mm" % pe
    return blocks, cand, q0


def one_layer(chain, q_start, blocks, cand, k, tower_top_mm=None,
              tower_xy_mm=None):
    surf = surface_for(k)
    slot = (SLOT_MM[0] / 1000.0, SLOT_MM[1] / 1000.0)
    top_after = surf + H_BLK
    pair = pp.stage3_survey_tool0(top_after, slot)
    carry = pp.stage3_carry_z(H_BLK, surf)
    return pp.build_stack(chain, q_start, cand, dict(blocks[0]), blocks, slot,
                          surf, tower_xy_mm=tower_xy_mm,
                          tower_top_mm=tower_top_mm, survey_tool0=pair[0],
                          carry_z=carry, yaw_grasp_deg=0.0,
                          yaw_place_deg=0.0, yaw_source="zero",
                          place_open_mm=50.0, release_dwell_s=0.5)


@case
def test_one_to_five_layers_plan(chain):
    """1–5 层都能解出来：软限位、腕奇异、护栏、IK 残差全过。"""
    blocks, cand, q0 = scene(chain)
    q = q0
    for k in range(1, 6):
        # 本层放置前塔顶 = 本层表面（第 1 层没有塔）。
        surface_top_mm = surface_for(k) * 1000.0 if k > 1 else PAD_Z * 1000.0
        tower_xy = SLOT_MM if k > 1 else None
        wps, checks, meta = one_layer(
            chain, q, blocks, cand, k,
            tower_top_mm=surface_top_mm, tower_xy_mm=tower_xy)
        assert [w["name"] for w in wps] == [
            "approach", "grasp", "lift", "to_slot", "place_approach",
            "place", "retreat", "survey_place"], [w["name"] for w in wps]
        assert len(checks) == len(wps)
        assert meta["kind"] == "stack"
        assert_close(meta["surface_z_mm"], surface_for(k) * 1000.0, 0.01,
                     "第 %d 层表面" % k)
        assert_close(meta["place_z"], expected_place_z(surface_for(k)),
                     1e-9, "第 %d 层放置高度" % k)
        assert_close(meta["place_approach_z"] - meta["place_z"],
                     pp.PLACE_APPROACH_GAP, 1e-12)
        # 载运高度必须高过放置接近点（**先升后横移**）
        assert meta["carry_z"] >= meta["place_approach_z"] + 0.05 - 1e-12, \
            "第 %d 层 to_slot 会在塔侧横移" % k
        assert meta["carry_z"] >= meta["lift_z"] - 1e-12
        if k > 1:
            assert meta["tower_box"] is not None, "第 %d 层没有塔体 box" % k
            # 箱顶 = 本层放置前的塔顶 = 本层表面
            assert_close(meta["tower_box"][5], surface_for(k), 1e-4,
                         "box 顶（m）")
        # 每一条航点都落到目标上（残差用同一套判据）
        by = {w["name"]: w for w in wps}
        gx, gy = meta["grasp_point"]
        sx, sy = [v / 1000.0 for v in meta["place_target_mm"]]
        for n, xyz in (("approach", (gx, gy, meta["approach_z"])),
                       ("grasp", (gx, gy, meta["grasp_z"])),
                       ("lift", (gx, gy, meta["carry_z"])),
                       ("to_slot", (sx, sy, meta["carry_z"])),
                       ("place_approach", (sx, sy, meta["place_approach_z"])),
                       ("place", (sx, sy, meta["place_z"])),
                       ("retreat", (sx, sy, meta["place_approach_z"]))):
            pe, re = pp.pose_residual(chain, by[n]["q"], xyz, 0.0)
            assert pe < 0.1 and re < 0.05, \
                "第 %d 层 %s 残差 %.3f mm / %.4f°" % (k, n, pe, re)
        q = np.asarray(wps[-1]["q"], float)          # 下一层从 survey 位起
    # 第 5 层箱顶 = 4 块高度（m）
    assert_close(meta["tower_box"][5], PAD_Z + 4 * H_BLK, 1e-4)


@case
def test_return_to_pick_plan_solves(chain):
    """**拆塔**那条计划能解：塔上抓（h = 垫 + 块高）→ 拾取区台面放。

    与叠塔是镜像：抓取 ``wz`` 多一个「垫 + 一块」，放置表面回到台面。
    这里管的是**运动学可解 + 高度算式**；``h`` 扣没扣 z 标定由
    ``test_stage3_entry.test_tower_block_item_uses_corrected_top`` 管。
    """
    import stage3_entry as se
    merged = {"top_mm": (PAD_Z + H_BLK) * 1000.0 + 1.28,
              "center_base_mm": list(SLOT_MM), "yaw_deg": 8.0,
              "yaw_trusted": True}
    item = se.tower_block_item(merged, {"yaw_sign": 1.0,
                                        "finger_axis_delta_deg": 0.0,
                                        "grasp_at": "block_center"},
                               {"tower": 1.28})
    wz = PAD_Z + H_BLK
    assert_close(item["block_h_used_mm"], wz * 1000.0, 1e-6, "塔上块 h")

    blocks, _cand, q0 = scene(chain)
    spot = (0.34, 0.28)
    pair = pp.stage3_survey_tool0(merged["top_mm"] / 1000.0,
                                  (SLOT_MM[0] / 1000.0, SLOT_MM[1] / 1000.0))
    carry = pp.stage3_carry_z(item["block_h_used_mm"] / 1000.0,
                              pp.TABLE_PLANE_Z)
    wps, checks, meta = pp.build_stack(
        chain, q0, item["cand"], dict(item["block"]), blocks, spot,
        pp.TABLE_PLANE_Z, survey_tool0=pair[0], carry_z=carry,
        yaw_grasp_deg=item["yaw_grasp_deg"],
        yaw_place_deg=item["yaw_place_deg"],
        yaw_source=item["yaw_source"],
        pick_open_mm=50.0, place_open_mm=50.0)
    assert [w["name"] for w in wps] == [
        "approach", "grasp", "lift", "to_slot", "place_approach",
        "place", "retreat", "survey_place"], [w["name"] for w in wps]
    assert len(checks) == len(wps)
    assert meta["tower_box"] is None, "抓塔上那块时不该把塔当障碍（它就是目标）"

    # 抓取高度：wz 整块抬高，不触发 GRASP_Z_MAX 的夹逼
    grasp_z = wz + pp.TOOL0_TO_FINGERTIP + pp.GRASP_CLEARANCE
    assert grasp_z < pp.GRASP_Z_MAX, "抓取高度被夹逼了（%.4f）" % grasp_z
    assert_close(meta["grasp_z"], grasp_z, 1e-9, "grasp_z")
    grasp_depth = wz - (grasp_z - pp.TOOL0_TO_BARE_FINGERTIP)
    place_z = (pp.TOOL0_TO_BARE_FINGERTIP + pp.TABLE_PLANE_Z
               + (wz - grasp_depth) + pp.PLACE_CLEARANCE)
    assert_close(meta["place_z"], place_z, 1e-9, "place_z")
    assert_close(meta["place_z"] - meta["place_approach_z"],
                 -pp.PLACE_APPROACH_GAP, 1e-12)

    by = {w["name"]: w for w in wps}
    gx, gy = meta["grasp_point"]
    sx, sy = [v / 1000.0 for v in meta["place_target_mm"]]
    assert_close(sx, spot[0], 1e-4, "放回点 x")
    # 抓取段用折过 180° 的抓取角（build_pick 里 fold_yaw_180），放置段用放置角
    yg_f = pp.fold_yaw_180(item["yaw_grasp_deg"])
    yp = float(item["yaw_place_deg"])
    for n, xyz, yw in (
            ("approach", (gx, gy, meta["approach_z"]), yg_f),
            ("grasp", (gx, gy, meta["grasp_z"]), yg_f),
            ("lift", (gx, gy, meta["carry_z"]), yg_f),
            ("to_slot", (sx, sy, meta["carry_z"]), yp),
            ("place_approach", (sx, sy, meta["place_approach_z"]), yp),
            ("place", (sx, sy, meta["place_z"]), yp),
            ("retreat", (sx, sy, meta["place_approach_z"]), yp)):
        pe, re = pp.pose_residual(chain, by[n]["q"], xyz, yw)
        assert pe < 0.1 and re < 0.05, \
            "回位 %s 残差 %.3f mm / %.4f°" % (n, pe, re)
    # 收尾巡检位在塔上方（= 回核塔位那一帧）
    pe, re = pp.pose_residual(chain, by["survey_place"]["q"], pair[0], 0.0)
    assert pe < 0.1 and re < 0.05, "回位收尾巡检位残差 %.3f mm" % pe


@case
def test_carry_height_mutation_would_be_caught(chain):
    """变异哨：把载运高度降回 lift_z，to_slot 就会低于放置接近点。"""
    blocks, cand, q0 = scene(chain)
    surf = surface_for(5)
    lift_z = H_BLK + pp.TOOL0_TO_FINGERTIP + pp.LIFT_HEIGHT
    _, approach = pp.place_heights(H_BLK, H_BLK - (H_BLK + 0.1757 + 0.01
                                                   - pp.TOOL0_TO_BARE_FINGERTIP),
                                   table_z=surf)
    assert lift_z < approach, \
        "第 5 层 lift_z(%.3f) 应低于 place_approach(%.3f) —— 这条前提变了" \
        % (lift_z, approach)
    carry = pp.stage3_carry_z(H_BLK, surf)
    assert carry > lift_z, "stage3_carry_z 没有接管高层"


@case
def test_mutation_tower_box_is_enforced(chain):
    """把塔体 box 顶抬到块必须穿过的位置 ⇒ **必须拒绝**（guard 真的接着）。"""
    blocks, cand, q0 = scene(chain)
    surf = surface_for(2)
    # 箱顶 = 本层表面 + 20 mm：place 时块底只到 surface+5 mm，会进箱。
    bad_top_mm = (surf + 0.020) * 1000.0
    try:
        one_layer(chain, q0, blocks, cand, 2, tower_top_mm=bad_top_mm,
                  tower_xy_mm=SLOT_MM)
    except pp.PlanError:
        pass
    else:
        raise AssertionError(
            "块穿进塔体 box 却没被拦 —— below_tip 是不是漏了裸指尖 204.2 mm？"
            "（漏掉时块被当成贴在法兰下，护栏形同关闭）")
    # 同样的路径、正常的箱顶（= 上一层顶）必须能规划
    one_layer(chain, q0, blocks, cand, 2,
              tower_top_mm=surf * 1000.0, tower_xy_mm=SLOT_MM)


@case
def test_check_held_block_direct(chain):
    """直接量一次持块包络：块底在箱内 ⇒ False；箱在块下方 ⇒ True。"""
    q = np.asarray(scene(chain)[2], float)
    T = chain.fk(dict(zip(chain.move_joints, q)))
    below = H_BLK - (H_BLK + 0.1757 + 0.01 - pp.TOOL0_TO_BARE_FINGERTIP) \
        + pp.TOOL0_TO_BARE_FINGERTIP
    zb = float(T[2, 3]) - below
    cx, cy = float(T[0, 3]), float(T[1, 3])
    inside = (cx - 0.05, cx + 0.05, cy - 0.05, cy + 0.05,
              zb - 0.01, zb + 0.01)
    ok, worst = pp.check_held_block(chain, q, q, 0.020, H_BLK, below, inside)
    assert ok is False and worst[2] < 0, (ok, worst)
    below_box = (cx - 0.05, cx + 0.05, cy - 0.05, cy + 0.05,
                 0.0, zb - 0.02)
    ok2, _ = pp.check_held_block(chain, q, q, 0.020, H_BLK, below, below_box)
    assert ok2 is True, "箱在块下方却判撞"
    # 漏掉裸指尖项时块底会抬高 ~21 cm —— 同一个箱就会漏判
    bad_below = H_BLK - (H_BLK + 0.1757 + 0.01 - pp.TOOL0_TO_BARE_FINGERTIP)
    ok3, _ = pp.check_held_block(chain, q, q, 0.020, H_BLK, bad_below, inside)
    assert ok3 is True, \
        "这个变异应当漏判 —— 若这里也判撞，说明用例构造没测到 below_tip"


# ---------------------------------------------------------------- 跑
@case
def test_place_clearance_knob(chain):
    """放置间隙可调：3 mm 的 place_z 比默认 5 mm 低 2 mm，meta 记的是**实际值**。

    两个断言各堵一个老坑：
    · 参数没传进 `place_heights` ⇒ 差值断言红；
    · meta 写死模块常量 `PLACE_CLEARANCE`（改之前就是这样）⇒ 记录值断言红。
    """
    blocks, cand, q0 = scene(chain)
    surf = surface_for(1)
    slot = (SLOT_MM[0] / 1000.0, SLOT_MM[1] / 1000.0)
    pair = pp.stage3_survey_tool0(surf + H_BLK, slot)
    kw = dict(surface_z=surf, survey_tool0=pair[0],
              carry_z=pp.stage3_carry_z(H_BLK, surf),
              yaw_grasp_deg=0.0, yaw_place_deg=0.0, yaw_source="zero",
              place_open_mm=50.0, release_dwell_s=0.0)
    _, _, m5 = pp.build_stack(chain, q0, cand, dict(blocks[0]), blocks, slot,
                              **kw)
    # 走**真入口那条路**（stage3_entry.place_one → layer_plan → build_stack）：
    # 只测 build_stack 会漏掉「中间层没透传」这种坑，2026-10-31 真机上就踩过。
    item = {"block": dict(blocks[0]), "block_h_used_mm": 29.5,
            "cand": cand, "yaw_grasp_deg": 0.0, "yaw_place_deg": 0.0,
            "yaw_source": "zero", "grasp_at": "block_center"}
    plan3 = s3r.layer_plan(chain, q0, item, blocks, slot, surf,
                           survey_tool0=pair[0],
                           carry_z=kw["carry_z"], place_open_mm=50.0,
                           release_dwell_s=0.0, place_clearance=0.003)
    m3 = plan3["meta"]
    assert_close(m5["place_clearance_mm"], 5.0, why="不给参数 = 老行为 5 mm")
    assert_close(m3["place_clearance_mm"], 3.0, why="meta 要记实际间隙")
    assert_close(m5["place_z"] - m3["place_z"], 0.002, tol=1e-4,
                 why="间隙少 2 mm，place_z 正好低 2 mm")


# ---------------------------------------------------------------- 后矫正 nudge
NUDGE_NAMES = ["nudge_approach", "nudge_grip", "nudge_shift", "nudge_twist",
               "nudge_release", "nudge_retreat", "nudge_survey"]


def nudge_block():
    """台面上孤零零一块 29.5 mm，中心 422.65 / −114.86 mm。"""
    return {"idx": 0, "class": "purple", "cx": 422.65, "cy": -114.86,
            "top": 29.5, "h": 29.5}


@case
def test_nudge_plan_geometry(chain):
    """矫正计划：先对齐块心、不抬升、平移到 δ、原地转到 δyaw。"""
    blocks, cand, q0 = scene(chain)
    bx = nudge_block()
    dx, dy, dyaw = 3.0, -2.0, 4.0
    wps, checks, meta = pp.build_nudge(
        chain, q0, bx, shift_xy_mm=(dx, dy), dyaw_deg=dyaw,
        surface_z=PAD_Z, yaw_block_deg=1.5)
    assert [w["name"] for w in wps] == NUDGE_NAMES, [w["name"] for w in wps]
    assert len(checks) == len(wps)
    assert meta["kind"] == "nudge"

    by = {w["name"]: w for w in wps}
    x0, y0 = bx["cx"] / 1000.0, bx["cy"] / 1000.0
    # 1) 进/抓都在**实测块心**上；平移后落在 δ 的终点
    za, zg = meta["approach_z"], meta["grip_z"]
    for n, z in (("nudge_approach", za), ("nudge_grip", zg)):
        pe, _ = pp.pose_residual(chain, by[n]["q"], (x0, y0, z), 1.5)
        assert pe < 0.1, "%s 没对准块心：%.3f mm" % (n, pe)
    for n, z in (("nudge_shift", zg), ("nudge_twist", zg),
                 ("nudge_release", zg), ("nudge_retreat", za)):
        pe, _ = pp.pose_residual(chain, by[n]["q"],
                                 (x0 + dx / 1000.0, y0 + dy / 1000.0, z), 1.5)
        assert pe < 0.1, "%s 没落在 δ 终点：%.3f mm" % (n, pe)
    # 2) **不抬**：抓住到松开这一段所有航点同高（这是「只夹不抬」的定义）
    def z_of(n):
        return float(chain.fk(by[n]["q"])[2, 3])

    zs = [z_of(n) for n in ("nudge_grip", "nudge_shift",
                            "nudge_twist", "nudge_release")]
    assert max(zs) - min(zs) < 1e-6, \
        "抓/移/转/松不在同一高度：%s（会变成抬起来）" % zs
    assert z_of("nudge_approach") > z_of("nudge_grip") + 0.03, \
        "进给点必须高过夹持点至少 30 mm"
    # 3) 转角只发生在 twist 这一步：shift 前仍是原 yaw，之后是 yaw + δ
    _, re_twist = pp.pose_residual(chain, by["nudge_twist"]["q"],
                                   chain.fk(by["nudge_twist"]["q"])[:3, 3],
                                   5.5)
    assert re_twist < 0.05, "twist 没转到 yaw+δ：%.4f°" % re_twist
    assert_close(meta["yaw_from_deg"], 1.5, 1e-9)
    assert_close(meta["yaw_to_deg"], 5.5, 1e-9)
    assert_close(meta["shift_norm_mm"], np.hypot(dx, dy), 1e-3,
                 "meta 里是四舍五入到 0.001 mm")
    # 4) 夹持高度按「块顶 + 裸指尖 + 夹持余量」算，不是拍脑袋的常数
    assert_close(meta["grip_z"],
                 PAD_Z + np.clip(H_BLK + pp.TOOL0_TO_FINGERTIP
                                 + pp.GRASP_CLEARANCE,
                                 pp.GRASP_Z_MIN, pp.GRASP_Z_MAX),
                 1e-9, "夹持高度")


@case
def test_nudge_rejects_out_of_range(chain):
    """超上限不是「放歪」而是「没放对」：必须拒，不许硬拽。"""
    _, _, q0 = scene(chain)
    bx = nudge_block()
    for kw in ({"shift_xy_mm": (pp.NUDGE_MAX_SHIFT_MM + 1.0, 0.0)},
               {"shift_xy_mm": (0.0, -(pp.NUDGE_MAX_SHIFT_MM + 1.0))},
               {"dyaw_deg": pp.NUDGE_MAX_YAW_DEG + 1.0},
               {"dyaw_deg": -(pp.NUDGE_MAX_YAW_DEG + 1.0)}):
        try:
            pp.build_nudge(chain, q0, bx, surface_z=PAD_Z, **kw)
        except pp.PlanError:
            pass
        else:
            raise AssertionError("超上限没有拒绝：%r" % (kw,))
    # 正好到上限要放行（边界不是「大于等于」）
    wps, _, _ = pp.build_nudge(chain, q0, bx,
                               shift_xy_mm=(pp.NUDGE_MAX_SHIFT_MM, 0.0),
                               surface_z=PAD_Z)
    assert [w["name"] for w in wps] == NUDGE_NAMES


@case
def test_nudge_refuses_crowded_neighbour(chain):
    """邻块 40 mm < MIN_BLOCK_GAP：张开的两指会碰到它，计划期就拒。"""
    _, _, q0 = scene(chain)
    bx = nudge_block()
    near = dict(bx, idx=1, cx=bx["cx"] + 40.0)
    try:
        pp.build_nudge(chain, q0, bx, surface_z=PAD_Z, blocks=[bx, near])
    except pp.PlanError:
        pass
    else:
        raise AssertionError("邻块太近仍然写出了计划")


@case
def test_nudge_plan_json_is_executable_shape(chain):
    """走**真入口那条路**（stage3_entry._nudge_once → nudge_plan → build_nudge）。

    执行器只认 ``version`` / ``meta`` / ``q_start`` / ``waypoints`` 这一种形状；
    这里把它钉死，免得中间层漏透传（2026-10-31 place_clearance 就踩过）。
    """
    _, _, q0 = scene(chain)
    bx = nudge_block()
    slot = (SLOT_MM[0] / 1000.0, SLOT_MM[1] / 1000.0)
    pair = pp.stage3_survey_tool0(PAD_Z + H_BLK, slot)
    plan = s3r.nudge_plan(chain, q0, bx, [bx],
                          shift_xy_mm=(3.0, -2.0), dyaw_deg=4.0,
                          surface_m=PAD_Z, yaw_block_deg=1.5, open_mm=50.0,
                          survey_tool0=pair[0])
    assert plan["version"] == 2, plan["version"]
    assert plan["meta"]["kind"] == "nudge", plan["meta"]["kind"]
    assert len(plan["q_start"]) == 6
    assert [w["name"].split("_split")[0] for w in plan["waypoints"]] \
        == NUDGE_NAMES, [w["name"] for w in plan["waypoints"]]
    # 夹爪动作要在计划里看得见（执行器靠 before/after 认出「这一次要开合」）
    acts = {w.get("after", {}).get("gripper") for w in plan["waypoints"]
            if w.get("after")}
    assert "close" in acts and "open_to" in acts, acts
    # 收尾巡检位 = 测量用的那一对里的第一个（复测不用再走一趟）
    assert_close(plan["meta"]["survey_tool0"][0], pair[0][0], 1e-4,
                 "survey 收尾位（meta 保留 4 位）")
    assert_close(plan["meta"]["shift_norm_mm"], np.hypot(3.0, -2.0), 1e-3)


def main():
    urdf = find_urdf()
    failed = skipped = 0
    for fn in CASES:
        if "chain" in fn.__code__.co_varnames[:fn.__code__.co_argcount]:
            if not urdf:
                skipped += 1
                print("SKIP  %s（没有 robot.urdf）" % fn.__name__)
                continue
            try:
                fn(urkin.Chain(urdf))
                print("PASS  %s" % fn.__name__)
            except Exception as exc:                              # noqa: BLE001
                failed += 1
                print("FAIL  %s\n      %s: %s"
                      % (fn.__name__, type(exc).__name__, exc))
            continue
        try:
            fn()
            print("PASS  %s" % fn.__name__)
        except Exception as exc:                                  # noqa: BLE001
            failed += 1
            print("FAIL  %s\n      %s: %s"
                  % (fn.__name__, type(exc).__name__, exc))
    if not urdf:
        print("\n!! 没找到 robot.urdf（$PRECISION_URDF 或 runs/*/robot.urdf）——"
              " %d 项运动学测试被**跳过**，不算通过" % skipped)
    total = len(CASES)
    print("\n%d/%d PASS（%d 跳过）" % (total - failed - skipped, total, skipped))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())

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
def test_group_gate_refuses_unknown_names_before_moving():
    """没实现的组名必须在**任何动臂动作之前**被拒 —— 不静默降级成开环。

    ``eih`` 2026-11-02 起已实现（条目在 `IMPLEMENTED_GROUPS`），所以这里用
    一个**不在 GROUPS 里**的名字验闸门本身还活着。scene 故意给个不存在的
    路径：闸门若排在 `load_scene` 之后，就会因别的原因失败而暴露次序错了。
    """
    assert s3r.unsupported_group("open") is None
    assert s3r.unsupported_group("closed") is None
    assert s3r.unsupported_group("eih") is None, "eih 已实现，不该再被拒"
    assert s3r.unsupported_group("nudge") is None, "nudge 已实现，不该再被拒"
    why = s3r.unsupported_group("nope")
    assert why and "nope" in why, "未知组名必须给出明确的拒绝原因"
    import argparse
    ns = argparse.Namespace(group="nope", scene="/nonexistent/scene.json")
    rc = s3r.run_round(ns)
    assert rc == 2, "run_round 对未知组应返回 2（fail-closed），实得 %r" % (rc,)


@case
def test_nudge_plan_is_executable_plan_json(chain):
    """`s3r.nudge_plan` 是主轮 nudge 组真正调的入口（不是 `pp.build_nudge`）。

    它多了一层 `_finish_plan`（拆跳变段 + 执行器要的 version/q_start/meta.checks）。
    只测 `pp.build_nudge` 会漏掉这一层：航点数被 `split_jumps` 改动过，
    `meta["checks"]` 与航点必须仍一一对应，否则执行器闸门在真机上才报错。
    """
    blocks, _cand, q0 = scene(chain)
    bx = nudge_block()
    sur = PAD_Z
    plan = s3r.nudge_plan(chain, q0, bx, blocks,
                          shift_xy_mm=(3.0, -2.0), dyaw_deg=4.0,
                          surface_m=sur, yaw_block_deg=1.5,
                          tower_box=pp.tower_box_from((bx["cx"], bx["cy"]),
                                                      sur * 1000.0))
    assert plan["version"] == 2, plan["version"]
    assert len(plan["q_start"]) == 6
    assert plan["waypoints"], "空计划"
    assert len(plan["meta"]["checks"]) == len(plan["waypoints"]), \
        "checks 与航点数对不上（执行器会拒）"
    names = [w["name"] for w in plan["waypoints"]]
    for want in NUDGE_NAMES:
        assert want in names, "缺航点 %s：%s" % (want, names)
    assert names[-1] == "nudge_survey", "最后一条必须是 survey（轮内接着抓帧）"
    # 每一步都要有 q 与预检，缺一个执行器就不认。
    for w in plan["waypoints"]:
        assert len(w["q"]) == 6 and "name" in w
    # 超限必须在**写计划之前**就拒（`PlanError`），不能生成一张坏计划。
    try:
        s3r.nudge_plan(chain, q0, bx, blocks,
                       shift_xy_mm=(40.0, 0.0), surface_m=sur,
                       yaw_block_deg=1.5)
    except pp.PlanError:
        pass
    else:
        raise AssertionError("40 mm 平移必须拒（上限 15 mm）")


@case
def test_nudge_plan_retries_equivalent_yaw_on_joint_swing(chain):
    """关节跳变超限 → 换一个 ±90° 的等价朝向重试；别的拒绝**不许**重试。

    2026-11-02 round-04 第 3 层真机：块实测 yaw 84.155°（方块 90° 对称 ⇒
    等价于 −5.845°），`fold_yaw_180` 折完仍是 84.155°，`nudge_approach` 要让
    wrist_3 转 199.1° 被 `MAX_JOINT_SWING` 拦下，整轮停住。这里把「第一次必拒」
    注入进去，钉住两件事：① 会换写法重试并成功；② `dyaw` 逐字不变——
    换的是朝向的写法，不是扭转量。

    反向变异：删掉重试循环 → 本测试抛 `JointSwingError` 变红；
    把 `except pp.JointSwingError` 放宽成 `except pp.PlanError` →
    下面「别的拒绝只调一次」变红。
    """
    blocks, _cand, q0 = scene(chain)
    bx = nudge_block()
    sur = PAD_Z
    real = pp.build_nudge
    calls = []

    def fake_once(*a, **kw):
        calls.append(kw.get("yaw_shift_deg", 0.0))
        if len(calls) == 1:
            raise pp.JointSwingError(
                "起点 → 航点 nudge_approach 有单个关节要转 199.1°（上限 180°）")
        return real(*a, **kw)

    try:
        pp.build_nudge = fake_once
        plan = s3r.nudge_plan(chain, q0, bx, blocks,
                              shift_xy_mm=(0.0, 0.0), dyaw_deg=5.845,
                              surface_m=sur, yaw_block_deg=84.155,
                              tower_box=pp.tower_box_from(
                                  (bx["cx"], bx["cy"]), sur * 1000.0))
    finally:
        pp.build_nudge = real
    assert len(calls) >= 2, "超限后没重试：%r" % (calls,)
    assert plan["meta"]["yaw_shift_deg"] != calls[0], \
        "重试换了朝向却没记进 meta：%r" % (plan["meta"],)
    assert abs(plan["meta"]["yaw_shift_deg"]) in (90.0, 180.0), plan["meta"]
    assert_close(plan["meta"]["dyaw_deg"], 5.845, 1e-9)

    # 非跳变类的拒绝必须原样抛出：换朝向救不回「平移超限 / 邻块太近 / 撞箱」，
    # 重试只会把真原因掩盖成别的报错。
    calls2 = []

    def fake_other(*a, **kw):
        calls2.append(kw.get("yaw_shift_deg", 0.0))
        raise pp.PlanError("矫正平移 40.0 mm 超过 15 mm 上限")

    try:
        pp.build_nudge = fake_other
        try:
            s3r.nudge_plan(chain, q0, bx, blocks, shift_xy_mm=(3.0, 0.0),
                           surface_m=sur, yaw_block_deg=84.155)
        except pp.JointSwingError:
            raise AssertionError("普通 PlanError 被当成可重试的跳变错误")
        except pp.PlanError:
            pass
        else:
            raise AssertionError("普通 PlanError 居然被吞掉")
    finally:
        pp.build_nudge = real
    assert len(calls2) == 1, "普通 PlanError 被重试了 %d 次" % len(calls2)


@case
def test_build_nudge_yaw_shift_is_same_grasp(chain):
    """`yaw_shift_deg` 只改朝向的写法：起始角 +k·90°、末端角同步、dyaw 不变。"""
    blocks, _cand, q0 = scene(chain)
    bx = nudge_block()
    w0, _c0, m0 = pp.build_nudge(chain, q0, bx, dyaw_deg=5.845,
                                 surface_z=PAD_Z, yaw_block_deg=5.845)
    w1, _c1, m1 = pp.build_nudge(chain, q0, bx, dyaw_deg=5.845,
                                 surface_z=PAD_Z, yaw_block_deg=5.845,
                                 yaw_shift_deg=-90.0)
    assert_close(m0["yaw_shift_deg"], 0.0, 1e-9)
    assert_close(m1["yaw_shift_deg"], -90.0, 1e-9)
    assert_close(m1["yaw_from_deg"], m0["yaw_from_deg"] - 90.0, 1e-9)
    assert_close(m1["yaw_to_deg"], m0["yaw_to_deg"] - 90.0, 1e-9)
    assert_close(m1["dyaw_deg"], m0["dyaw_deg"], 1e-9)
    assert [w["name"] for w in w1] == [w["name"] for w in w0]


@case
def test_nudge_block_ref_matches_measured_centre_to_candidate():
    """nudge 要的 ``block`` 必须来自**当次候选表**，不能自己编一个 idx。

    换位置后邻块也在候选里；认错块 = 拿邻块的位置去算张开与碰撞。
    """
    cands = [{"idx": 3, "cx": 422.0, "cy": -114.0},
             {"idx": 7, "cx": 500.0, "cy": -114.0}]
    meas = {"poses": [{"frames": [{"diag": {"candidates": cands}}]}]}
    merged = {"center_base_mm": [420.0, -116.0], "top_mm": 148.2}
    block, blocks = s3r.nudge_block_ref(merged, meas)
    assert block["idx"] == 3, block
    assert_close(block["cx"], 420.0, 1e-9, "block.cx 用实测中心")
    assert_close(block["h"], s3r.m3.BLOCK_TRUE_H_MM, 1e-9, "h 相对脚下那层")
    assert_close(block["top"], 148.2, 1e-9, "top 是绝对顶高")
    assert [c["idx"] for c in blocks] == [3, 7], blocks
    # 实测中心离**所有**候选都超过 tol ⇒ 判不了是哪块：返回 None 让调用方停轮。
    far = {"center_base_mm": [470.0, -116.0], "top_mm": 148.2}
    assert s3r.nudge_block_ref(far, meas) is None, "认不出块必须返回 None"
    # 候选表都拿不到（测量坏了）：退回实测中心，不假装有 idx。
    b2, bl2 = s3r.nudge_block_ref(merged, {})
    assert b2["idx"] == -1 and len(bl2) == 1, (b2, bl2)


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
def test_applied_bias_accepts_round_json_history_entries():
    """真机的 `bias_history` 是 {"layer", "bias_mm"} 字典，不是 [bx, by]。

    2026-11-04 首轮真机第 1 层放完、nudge 过门后，算第 2 层补偿时
    `applied_bias` 抛 KeyError: 0 —— 离线预检没触发是因为那段没有实测、
    `bias_history` 一直是空的。这条测试把两种形状都钉住。
    """
    hist = [{"layer": 1, "bias_mm": [-3.0, -2.0]},
            {"layer": 2, "bias_mm": [-1.0, -4.0]}]
    b = s3r.applied_bias(hist)
    assert_close(b[0], -2.0, 1e-9, "字典形状均值 x")
    assert_close(b[1], -3.0, 1e-9, "字典形状均值 y")
    # 种子 + 字典混用：均值分母要把种子算进去
    b2 = s3r.applied_bias(hist[:1], seed=(-2.7, -0.5))
    assert_close(b2[0], (-2.7 - 3.0) / 2.0, 1e-9, "种子+字典 x")
    assert_close(b2[1], (-0.5 - 2.0) / 2.0, 1e-9, "种子+字典 y")
    # 与纯列表形状结果一致
    b3 = s3r.applied_bias([[-3.0, -2.0], [-1.0, -4.0]])
    assert_close(b3[0], b[0], 1e-12, "两种形状 x 一致")
    assert_close(b3[1], b[1], 1e-12, "两种形状 y 一致")


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
def test_bias_fixed_ignores_in_round_history():
    """``--bias-comp fixed``：b̂ 只用先验，本轮实测只记账不改命令。

    2026-11-05 第 6 轮后加：轮内自适应每层换一次 b̂，命令跟着摆
    （round-06 实测跨层摆幅 x 1.18 / y 1.27 mm，与放置散差同量级）。
    钉住 b̂ 后，层间误差只剩 ``b̂_k − b̂_{k−1}`` 里真实的那部分。
    """
    desired = (422.65, -114.86)
    seed = [-4.53, -0.91]
    # 历史再长也不许影响命令 —— 三种长度给出同一个目标。
    outs = [s3r.target_with_bias(desired, h, "fixed", seed=seed)
            for h in ([], [[-9.0, 3.0]], [[-9.0, 3.0], [1.0, -8.0]])]
    for t in outs:
        assert_close(t[0], desired[0] + 4.53, 1e-9, "fixed x")
        assert_close(t[1], desired[1] + 0.91, 1e-9, "fixed y")
    # 没有先验 ⇒ 退回不补（并打印告警），不能悄悄用历史。
    assert s3r.applied_bias([[-9.0, 3.0]], "fixed") == [0.0, 0.0]
    # 记账不受影响：历史仍能算出跨轮估计要的值。
    assert s3r.applied_bias([], "adaptive", seed=seed) == [float(seed[0]),
                                                          float(seed[1])]


@case
def test_place_target_previous_uses_prev_measured_xy_only():
    """``--place-target previous`` = xy 跟上一层实测、yaw 仍走计划绝对朝向。

    源码级护栏（跑 run_round 要真机/大件）：必须有一条 ``place_target ==
    "previous"`` 的分支取 ``prev["merged"]["center_base_mm"]``，且那条分支里
    **不许** 出现 ``closed_loop_yaw`` —— 那是 closed 组的语义，会引入 86–90° 腕角异常。
    """
    import inspect
    src = inspect.getsource(s3r.run_round)
    assert '"previous"' in inspect.getsource(s3r.main), "缺 --place-target"
    i = src.index('args.place_target == "previous"')
    branch = src[i:i + 600]
    assert 'center_base_mm' in branch, "previous 目标要取上一层实测中心"
    assert "closed_loop_yaw" not in branch, \
        "previous 只跟 xy；跟 yaw 是 closed 组的语义"


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
        # 旧语义版本（矫正前口径，version 1）不能当种子 —— 两种样本不能相加。
        with open(p, "w") as fh:
            json.dump({"version": 1, "model": "constant",
                       "bias_mm": [-2.83, -1.19], "n": 7}, fh)
        assert s3r.load_bias_estimate(p) is None, \
            "version 1 的旧估计必须被拒（口径已改成矫正后）"
        assert e2["version"] == s3r.BIAS_ESTIMATE_VERSION, e2


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


# ---------------------------------------------------------------- 最终版策略
@case
def test_nudge_target_open_uses_plan_yaw_every_layer():
    """最终版主组：每一层的 nudge 朝向目标都是**计划绝对朝向**。

    open 组的 ``yaw_place`` 是 0.0（geometric 臂），所以目标恒为 0；即便上一层
    实测 84°，也不许跟 —— 那是 closed 组 86–90° 腕部异常的来源。
    """
    prev84 = {"merged": {"found": True, "yaw_color_deg": 84.0}}
    for k in (1, 2, 3, 5):
        y, src = s3r.nudge_target_for_group("open", k, 0.0, prev84)
        assert y == 0.0 and src == "plan", (k, y, src)
    # 计划朝向不是 0 时也要原样用计划值（不折、不跟上一层）。
    y, src = s3r.nudge_target_for_group("open", 3, 6.5, prev84)
    assert y == 6.5 and src == "plan", (y, src)


@case
def test_nudge_target_closed_still_follows_previous():
    """closed 组语义没被改坏：第 1 层计划，第 2 层起跟上一层实测。"""
    prev = {"merged": {"found": True, "yaw_color_deg": 84.0,
                       "yaw_color_trusted": True}}
    assert s3r.nudge_target_for_group("closed", 1, 0.0, prev) == (0.0, "plan")
    y, src = s3r.nudge_target_for_group("closed", 2, 0.0, prev)
    assert y is not None and src == "color", (y, src)
    # 缺上一层 ⇒ 判不了（停轮），不拿计划值偷偷顶替。
    y, src = s3r.nudge_target_for_group("closed", 2, 0.0, {"merged": {}})
    assert y is None and src == "no_prev_layer", (y, src)


def _meas(cx, cy, yaw=None):
    return {"found": True, "center_base_mm": [cx, cy], "yaw_color_deg": yaw}


@case
def test_stop_gate_l1_uses_grid_center():
    """第 1 层参考**格心**：≤10 mm 过，>10 mm 停（放第 2 层之前的预检）。"""
    slot = (422.65, -114.86)
    g = s3r.stop_gate_xy(_meas(slot[0] + 9.9, slot[1]), None, 1, slot)
    assert g["pass"] is True and g["ref"] == "grid_center", g
    assert_close(g["xy_rel_mm"], 9.9, 0.01, "xy_rel")
    g = s3r.stop_gate_xy(_meas(slot[0] + 10.1, slot[1]), None, 1, slot)
    assert g["pass"] is False and g["ref"] == "grid_center", g


@case
def test_stop_gate_l2_uses_previous_measured_center():
    """第 2 层起参考**上一层矫正后实测中心**，不是格心。"""
    slot = (422.65, -114.86)
    prev = _meas(430.0, -110.0)
    g = s3r.stop_gate_xy(_meas(430.0, -110.0 + 9.9), prev, 2, slot)
    assert g["pass"] is True and g["ref"] == "previous_layer", g
    g = s3r.stop_gate_xy(_meas(430.0, -110.0 + 10.1), prev, 2, slot)
    assert g["pass"] is False and g["ref"] == "previous_layer", g
    # 上一层没量到 ⇒ 判不了，不制造新失败（前面的门已经停了）。
    g = s3r.stop_gate_xy(_meas(430.0, -110.0), {"found": False}, 2, slot)
    assert g["pass"] is None and g["reason"] == "previous_missing", g


@case
def test_stop_gate_ignores_yaw():
    """停轮门**只看 xy**：yaw 差 20°、xy 2 mm ⇒ 过（yaw 仍按 5° 判分）。"""
    slot = (422.65, -114.86)
    g = s3r.stop_gate_xy(_meas(slot[0] + 2.0, slot[1], yaw=20.0),
                         None, 1, slot)
    assert g["pass"] is True, g
    assert_close(g["xy_rel_mm"], 2.0, 0.01, "xy_rel")


@case
def test_bias_uses_post_nudge_measurement():
    """b̂ 取**矫正后**的 ``merged`` —— 回归护栏（源码级）。

    2026-11-05 用户拍板改口径：round-04 实测 nudge 拖拽 5/5 沿 −x、1.2–4.6 mm，
    是系统性位移；只记「矫正前」的放置偏置等于假设拖拽不存在，补偿只对一半。
    现在 b̂ = 放置偏置 + 拖拽，``bias_from_layer`` 必须直接吃矫正后的 ``merged``。
    这条用源码顺序把守：``nudge_after_place(`` 必须早于 ``bias_from_layer(``，
    且实参是 ``merged``（不是别的别名），也不许再有 ``bias_merged`` 这个旧别名。
    """
    import inspect
    import re
    src = inspect.getsource(s3r.run_round)
    i_nudge = src.index("nudge_after_place(")
    i_call = src.index("bias_from_layer(")
    assert i_nudge < i_call, (
        "bias_from_layer 必须在 nudge 之后（要吃矫正后的 merged）",
        i_nudge, i_call)
    assert "bias_merged" not in src, "旧口径的 bias_merged 别名必须删掉"
    assert re.search(r"bias_from_layer\(\s*merged\.get", src), \
        "bias_from_layer 的实测值必须来自矫正后的 merged"


@case
def test_place_clearance_cli_wires_through_to_both_plans():
    """``--place-clearance-mm`` 要真的影响计划 —— 回归护栏（源码级）。

    2026-11-05 用户拍板第 6 轮降放置间隙 5→3 mm（入口②实测 3 mm 组 Δxy σ 0.80 mm，
    比 5 mm/停留 0 的 3.40 mm 小 4 倍）。这条堵三处「改了不生效」：
    参数没定义 / 没换成米 / 只透传给首层计划而漏掉 carry 处重规划那条路。
    """
    import inspect
    import re
    assert '"--place-clearance-mm"' in inspect.getsource(s3r.main), \
        "缺 --place-clearance-mm 参数"
    src = inspect.getsource(s3r.run_round)
    assert re.search(r"args\.place_clearance_mm\s*/\s*1000", src), \
        "命令行给的是 mm，计划器吃 m，必须 /1000"
    assert src.count("place_clearance=place_clearance_m") == 2, \
        "layer_plan 与 place_plan 两条路都要透传"
    # 缺省仍是 precision_plan 的 5 mm（不传参数 = 老行为）
    assert "pp.PLACE_CLEARANCE" in src


@case
def test_final_run_seed_shifts_command_positive():
    """最终版首轮种子 (−2.7, −0.5)：命令 = 期望 − b̂ ⇒ 命令朝 **+x/+y** 偏。

    台账 n=21 干净子集的先验；符号写反会把块推得更歪。
    """
    desired = (422.65, -114.86)
    t = s3r.target_with_bias(desired, [], seed=[-2.7, -0.5])
    assert_close(t[0], 425.35, 1e-9, "命令 x")
    assert_close(t[1], -114.36, 1e-9, "命令 y")


@case
def test_nudge_row_records_drag_vector():
    """nudge 拖拽只记账：``drag_xy_mm = after.center − before.center``。"""
    before = {"center_base_mm": [420.0, -115.0], "yaw_color_deg": 2.0,
              "yaw_deg": 2.0, "top_mm": 100.0}
    after = {"center_base_mm": [422.0, -116.5], "yaw_color_deg": 0.5,
             "yaw_deg": 0.5, "top_mm": 100.0}
    cmd = {"dx_mm": 3.0, "dy_mm": -1.0, "dyaw_block_deg": -1.5}
    row = s3r.nudge_row(1, cmd, before, after, [420.0, -115.0],
                        axes=("yaw",))
    assert row["drag_xy_mm"] == [2.0, -1.5], row.get("drag_xy_mm")


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


@case
def test_retry_pose_is_perpendicular_to_baseline():
    """补拍位：只挪水平、距离恒为 ``STAGE3_RETRY_SHIFT_MM``、垂直于原两视点基线。

    垂直于基线是为了换开那个视角的病根；沿基线挪只会靠近另一个视点。
    两个视点各自补拍时方向必须**一致**（都由 pair[0]→pair[1] 定），
    否则换谁坏会得到镜像的补拍位、不可复现。
    """
    pair = pp.stage3_survey_tool0(0.1475, (0.42265, -0.11486))
    base = (pair[1][0] - pair[0][0], pair[1][1] - pair[0][1])
    dirs = []
    for idx in (0, 1):
        p = pp.stage3_survey_retry_tool0(pair[idx], pair)
        assert_close(p[2], pair[idx][2], 1e-12, "补拍不改高度")
        d = (p[0] - pair[idx][0], p[1] - pair[idx][1])
        assert_close(np.hypot(*d), pp.STAGE3_RETRY_SHIFT_MM / 1000.0, 1e-12,
                     "水平位移")
        assert_close(d[0] * base[0] + d[1] * base[1], 0.0, 1e-12,
                     "必须垂直于基线")
        dirs.append(d)
    assert_close(dirs[0][0] * dirs[1][1] - dirs[0][1] * dirs[1][0], 0.0,
                 1e-15, "两个方向不得镜像")
    assert dirs[0][0] * dirs[1][0] + dirs[0][1] * dirs[1][1] > 0, dirs


def _meas_two_poses(cover1, cover2, reason="pose_mismatch"):
    """两视点的最小 meas 记录（只填挑坏视点要用的那几项）。"""
    def pose(name, cov):
        return {"name": name, "fused": {"found": True},
                "frames": [{"usable": True, "face_cover": cov}]}
    return {"poses": [pose("pose1", cover1), pose("pose2", cover2)],
            "consistency": {"ok": False, "reason": reason}}


@case
def test_retry_bad_pose_replaces_only_the_bad_view():
    """补拍只换**坏的那个视点**的帧组，好的原样留着（一致性仍比两个独立视点）。"""
    pair = pp.stage3_survey_tool0(0.1475, (0.42265, -0.11486))
    meas = _meas_two_poses(0.95, 0.27)
    frames = [("pose1", ["g1", "g2"]), ("pose2", ["b1", "b2"])]
    seen = {}
    o_move, o_cap, o_ml = s3r.go_move, s3r.capture_frames, s3r.m3.measure_layer

    def fake_move(board, chain, cdir, tag, xyz, urdf):
        seen["move"] = (tag, xyz)
        return 0

    def fake_cap(board, prefix, tag, n):
        seen["cap"] = (prefix, n)
        return ["nr1", "nr2"]

    def fake_ml(pose_frames, **kw):
        seen["frames"] = pose_frames
        return {"poses": [], "consistency": {"ok": True, "reason": "ok"},
                "merged": {"found": True}}

    s3r.go_move, s3r.capture_frames, s3r.m3.measure_layer = (
        fake_move, fake_cap, fake_ml)
    try:
        meas2, row = s3r.retry_bad_pose(
            None, None, "/tmp/x", 4, 2, pair, frames, meas, {}, "r.urdf",
            20.0, 3)
    finally:
        s3r.go_move, s3r.capture_frames, s3r.m3.measure_layer = o_move, o_cap, o_ml

    assert row["pose"] == "pose2" and row["index"] == 1, row
    assert_close(seen["move"][1][0],
                 pp.stage3_survey_retry_tool0(pair[1], pair)[0], 1e-12)
    assert seen["frames"] == [("pose1", ["g1", "g2"]),
                              ("pose2", ["nr1", "nr2"])], seen["frames"]
    assert seen["cap"][1] == 3, seen
    assert meas2["consistency"]["ok"] is True, meas2
    assert row["consistency_before"]["reason"] == "pose_mismatch", row
    assert row["cover_before"] == 0.27, row


@case
def test_retry_bad_pose_skips_without_coverage():
    """没有 face_cover 依据就不动臂、不假装补拍过（返回原 meas + skipped）。"""
    pair = pp.stage3_survey_tool0(0.1475, (0.42265, -0.11486))
    meas = {"poses": [{"name": "pose1", "fused": {"found": True},
                       "frames": [{"usable": True}]},
                      {"name": "pose2", "fused": {"found": True},
                       "frames": [{"usable": True}]}],
            "consistency": {"ok": False, "reason": "pose_mismatch"}}
    o_move = s3r.go_move

    def boom(*a, **kw):
        raise AssertionError("没有依据时不许动臂")

    s3r.go_move = boom
    try:
        meas2, row = s3r.retry_bad_pose(
            None, None, "/tmp/x", 4, 2, pair, [("pose1", []), ("pose2", [])],
            meas, {}, "r.urdf", 20.0, 3)
    finally:
        s3r.go_move = o_move
    assert meas2 is meas, "没补拍就得原样返回"
    assert row == {"skipped": "no_face_cover"}, row


@case
def test_retry_pose_shift_is_configurable():
    """位移是参数（真机若发现 20 mm 仍坏，可只改参数不改几何）。"""
    pair = pp.stage3_survey_tool0(0.1475, (0.42265, -0.11486))
    p = pp.stage3_survey_retry_tool0(pair[0], pair, shift_mm=35.0)
    assert_close(np.hypot(p[0] - pair[0][0], p[1] - pair[0][1]), 0.035, 1e-12)


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


# ---------------------------------------------------------------- 拆塔（回位）
@case
def test_layer_plan_teardown_item_grabs_top_layer(chain):
    """拆塔项走**真入口那条路**（layer_plan → build_stack → build_pick）。

    2026-11-02 事故回归：``block_h_used_mm``（块顶 91 mm）被当成块高，
    绝对 277 mm 被 `GRASP_Z_MAX` 截到 224.2 mm，指尖落到最底层 20 mm，
    一抓把三层全带起来。这里断言指尖落在**顶层块内部**、放置高度按 29.5 mm。
    """
    blocks, cand, q0 = scene(chain)
    top_mm, bh_mm = 91.34, 29.5
    slot = (0.45, -0.25)                     # 拾取区回位空位
    tower = dict(blocks[0], cx=SLOT_MM[0], cy=SLOT_MM[1],
                 top=top_mm, h=top_mm)
    cand_t = dict(cand, xyz_base=[SLOT_MM[0] / 1000.0, SLOT_MM[1] / 1000.0,
                                  top_mm / 1000.0])
    item = {"block": tower, "cand": cand_t, "block_idx": 99, "class": "cyan",
            "block_h_raw_mm": top_mm, "block_h_used_mm": top_mm,
            "block_true_h_mm": bh_mm,
            "yaw_grasp_deg": 0.0, "yaw_place_deg": 0.0, "yaw_source": "zero",
            "grasp_at": "block_center"}
    plan = s3r.layer_plan(chain, q0, item, [tower], slot, pp.TABLE_PLANE_Z,
                          carry_z=pp.stage3_carry_z(bh_mm / 1000.0,
                                                    pp.TABLE_PLANE_Z),
                          place_open_mm=50.0, release_dwell_s=0.0, scene={})
    m = plan["meta"]
    assert_close(m["block_true_h_mm"], bh_mm, why="meta 要记物理块高")
    assert_close(m["block_bottom_mm"], top_mm - bh_mm, why="块底 = 块顶 − 块高")
    tip_mm = m["grasp_z"] * 1000.0 - pp.TOOL0_TO_BARE_FINGERTIP * 1000.0
    assert top_mm - bh_mm + 5.0 < tip_mm < top_mm, \
        "指尖 %.2f mm 不在顶层块内（块底 %.2f / 块顶 %.2f）" % (
            tip_mm, top_mm - bh_mm, top_mm)
    assert m["place_z"] < 0.23, \
        "放置高度 %.4f 仍按块顶 91 mm 算 —— 会凭空抬高一个塔高" % m["place_z"]

    # 漏给 block_true_h_mm：块顶 91 mm 不像台面块 ⇒ 计划期就拒（不是抓歪了才发现）
    bad = dict(item)
    bad.pop("block_true_h_mm")
    try:
        s3r.layer_plan(chain, q0, bad, [tower], slot, pp.TABLE_PLANE_Z,
                       carry_z=0.42, place_open_mm=50.0, scene={})
    except pp.PlanError as exc:
        assert "block_h_m" in str(exc), exc
    else:
        raise AssertionError("拆塔项漏了 block_true_h_mm 居然能出计划")


# ---------------------------------------------------------------- 后矫正 nudge
@case
def test_build_place_matches_pick_place_geometry(chain):
    """后半段与整条 pick 的放置几何**逐值相同** —— 换目标重规划不能顺手改高度。

    eye-in-hand 把一层切成两段：段 A 抓，段 B 放。两段必须用同一套
    `place_heights` / 夹持深度；差一点就是「改目标时把块按错高度放下」。
    """
    blocks, cand, q0 = scene(chain)
    block = dict(blocks[0], h=29.5)
    slot = (SLOT_MM[0] / 1000.0, SLOT_MM[1] / 1000.0)
    wps1, _, m1 = pp.build_pick(chain, q0, cand, block, blocks, slot,
                                surface_z=PAD_Z, grasp_at="block_center")
    q_carry = next(np.asarray(w["q"], dtype=float)
                   for w in wps1 if w["name"] == "to_slot")
    wps2, checks2, m2 = pp.build_place(
        chain, q_carry, block, slot, PAD_Z, m1["carry_z"],
        open_mm=40.0, survey_tool0=m1["survey_tool0"])
    assert [w["name"] for w in wps2] == ["to_slot", "place_approach",
                                         "place", "retreat", "survey_place"]
    assert len(checks2) == len(wps2)
    assert m2["kind"] == "place"
    for key in ("place_z", "place_approach_z", "block_below_tip_mm",
                "grasp_depth_mm", "carry_z"):
        assert_close(m2[key], m1[key], 1e-9, key)
    # 第一条航点在 carry 高度、**先横移**：place_approach 才下降
    by = {w["name"]: w for w in wps2}
    z_carry = float(chain.fk(by["to_slot"]["q"])[2, 3])
    z_ap = float(chain.fk(by["place_approach"]["q"])[2, 3])
    assert_close(z_carry, m1["carry_z"], 1e-3, "to_slot 高度")
    assert z_carry > z_ap, "改目标后必须先在 carry 高度横移，再下降"
    pe, _ = pp.pose_residual(chain, by["place"]["q"],
                             (slot[0], slot[1], m2["place_z"]), 0.0)
    assert pe < 0.1, "place 没落到目标：%.3f mm" % pe
    # carry 低于 place_approach ⇒ 拒绝（斜切下去会撞塔）
    try:
        pp.build_place(chain, q_carry, block, slot, PAD_Z,
                       m2["place_approach_z"] - 0.01)
    except pp.PlanError:
        pass
    else:
        raise AssertionError("carry_z 低于 place_approach 时必须拒")


@case
def test_tower_box_from_is_the_single_source():
    """两个规划入口共用同一个塔体 box —— 差一个余量就是一个多判、一个漏判。"""
    b = pp.tower_box_from((422.0, -114.0), 100.0, 0.020)
    assert abs(b[5] - 0.100) < 1e-12 and b[4] == 0.0
    half = 0.020 + pp.TOWER_BOX_MARGIN
    assert_close(b[0], 0.422 - half, 1e-12, "x0")
    assert_close(b[1], 0.422 + half, 1e-12, "x1")
    assert pp.tower_box_from(None, 100.0) is None
    assert pp.tower_box_from((0.0, 0.0), None) is None


NUDGE_NAMES = ["nudge_approach", "nudge_grip", "nudge_twist", "nudge_shift",
               "nudge_release", "nudge_retreat", "nudge_survey"]
#: 抬最小一步（lift_mm > 0）时多出来的两段
NUDGE_LIFT_NAMES = ["nudge_approach", "nudge_grip", "nudge_lift",
                    "nudge_twist", "nudge_shift", "nudge_lower",
                    "nudge_release", "nudge_retreat", "nudge_survey"]


def nudge_block():
    """台面上孤零零一块 29.5 mm，中心 422.65 / −114.86 mm。"""
    return {"idx": 0, "class": "purple", "cx": 422.65, "cy": -114.86,
            "top": 29.5, "h": 29.5}


@case
def test_nudge_plan_geometry(chain):
    """矫正计划：对齐块心 → 先绕块心扭转 → 平移到 δ；lift=0 时不抬。"""
    blocks, cand, q0 = scene(chain)
    bx = nudge_block()
    dx, dy, dyaw = 3.0, -2.0, 4.0
    wps, checks, meta = pp.build_nudge(
        chain, q0, bx, shift_xy_mm=(dx, dy), dyaw_deg=dyaw,
        surface_z=PAD_Z, yaw_block_deg=1.5)
    assert [w["name"] for w in wps] == NUDGE_NAMES, [w["name"] for w in wps]
    assert len(checks) == len(wps)
    assert meta["kind"] == "nudge"
    assert meta["lift_mm"] == 0.0, meta["lift_mm"]

    by = {w["name"]: w for w in wps}
    x0, y0 = bx["cx"] / 1000.0, bx["cy"] / 1000.0
    # 1) 进/抓都在**实测块心**上；**扭转也在块心**上（先扭后移，
    #    旧序列把扭转放在平移之后、绕的是移动前的块心）
    za, zg = meta["approach_z"], meta["grip_z"]
    for n, z in (("nudge_approach", za), ("nudge_grip", zg)):
        pe, _ = pp.pose_residual(chain, by[n]["q"], (x0, y0, z), 1.5)
        assert pe < 0.1, "%s 没对准块心：%.3f mm" % (n, pe)
    pe, _ = pp.pose_residual(chain, by["nudge_twist"]["q"], (x0, y0, zg), 5.5)
    assert pe < 0.1, "twist 不在块心上（先扭后移要求绕块心）：%.3f mm" % pe
    for n, z in (("nudge_shift", zg), ("nudge_release", zg),
                 ("nudge_retreat", za)):
        pe, _ = pp.pose_residual(chain, by[n]["q"],
                                 (x0 + dx / 1000.0, y0 + dy / 1000.0, z), 5.5)
        assert pe < 0.1, "%s 没落在 δ 终点：%.3f mm" % (n, pe)
    # 2) lift=0 ⇒ **不抬**：抓住到松开这一段所有航点同高（旧行为可原样退回）
    def z_of(n):
        return float(chain.fk(by[n]["q"])[2, 3])

    zs = [z_of(n) for n in ("nudge_grip", "nudge_shift",
                            "nudge_twist", "nudge_release")]
    assert max(zs) - min(zs) < 1e-6, \
        "lift=0 时抓/转/移/松必须同高：%s" % zs
    assert z_of("nudge_approach") > z_of("nudge_grip") + 0.03, \
        "进给点必须高过夹持点至少 30 mm"
    # 3) 转角只发生在 twist 这一步：shift 已是 yaw + δ
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
def test_nudge_plan_lifting_order_and_height(chain):
    """`lift_mm > 0`：夹住后抬这一小段再扭/移，最后放回原高度松开。

    2026-11-03 用户拍板（台账第 224 轮）：抬最小一步让块脱开下层摩擦，
    push gain 才可能稳定。这里钉三件事：航点顺序、抬升量、以及**落下**
    回到同一个 grip_z（否则松手高度变了，等于改了放置条件）。
    """
    blocks, cand, q0 = scene(chain)
    bx = nudge_block()
    wps, checks, meta = pp.build_nudge(
        chain, q0, bx, shift_xy_mm=(3.0, -2.0), dyaw_deg=4.0,
        surface_z=PAD_Z, yaw_block_deg=1.5, lift_mm=2.0)
    assert [w["name"] for w in wps] == NUDGE_LIFT_NAMES, \
        [w["name"] for w in wps]
    by = {w["name"]: w for w in wps}

    def z_of(n):
        return float(chain.fk(by[n]["q"])[2, 3])

    zg = meta["grip_z"]
    assert_close(meta["lift_mm"], 2.0, 1e-9, "meta.lift_mm")
    for n in ("nudge_lift", "nudge_twist", "nudge_shift"):
        assert_close(z_of(n), zg + 0.002, 1e-6,
                     "%s 应在 grip_z + 2 mm" % n)
    # 落下回到原高度，松开高度与不抬档一致
    assert_close(z_of("nudge_lower"), zg, 1e-6, "nudge_lower")
    assert_close(z_of("nudge_release"), zg, 1e-6, "nudge_release")
    # 抬起后扭转仍在块心上
    x0, y0 = bx["cx"] / 1000.0, bx["cy"] / 1000.0
    pe, _ = pp.pose_residual(chain, by["nudge_twist"]["q"],
                             (x0, y0, zg + 0.002), 5.5)
    assert pe < 0.1, "lift 档 twist 不在块心：%.3f mm" % pe


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


@case
def test_nudge_plan_twist_only_still_solves(chain):
    """`--nudge-axes yaw`（只扭转）这一档：shift 全零、dyaw 非零，计划照样可执行。

    主流程从 round-04 起就走这一档（2026-11-02 用户拍板）。反向变异：把
    ``shift_xy_mm`` 换成非零值不会红（这是合法输入），真正的钉子在下一条
    ``shift_norm_mm`` 必须是 0 —— 若某天有人把「只扭转」实现成「顺手也推一点」，
    这里立刻红。
    """
    _, _, q0 = scene(chain)
    bx = nudge_block()
    slot = (SLOT_MM[0] / 1000.0, SLOT_MM[1] / 1000.0)
    pair = pp.stage3_survey_tool0(PAD_Z + H_BLK, slot)
    plan = s3r.nudge_plan(chain, q0, bx, [bx],
                          shift_xy_mm=(0.0, 0.0), dyaw_deg=6.0,
                          surface_m=PAD_Z, yaw_block_deg=1.5, open_mm=50.0,
                          survey_tool0=pair[0])
    assert_close(plan["meta"]["shift_norm_mm"], 0.0, 1e-9,
                 "只扭转档不许带平移")
    assert_close(plan["meta"]["dyaw_deg"], 6.0, 1e-6)
    assert [w["name"].split("_split")[0] for w in plan["waypoints"]] \
        == NUDGE_NAMES, [w["name"] for w in plan["waypoints"]]
    acts = {w.get("after", {}).get("gripper") for w in plan["waypoints"]
            if w.get("after")}
    assert "close" in acts and "open_to" in acts, acts


@case
def test_nudge_after_a_layer_solves_on_the_near_branch(chain):
    """第 2 层及以上的 nudge 必须**接在当前那一支上**，不能整链换支。

    2026-11-02 离线复现的两个坑（都在这一条里钉死）：

    1. `build_nudge` 把 ``off_xy=None`` 传给 `check_held_block` ⇒ 一给塔体 box
       就 ``TypeError``（第 1 层没有塔，所以入口⑤ 一直没暴露）。
    2. 箱顶 = 被矫正块的底面，块底由 FK 推、IK 残差 ≤0.1 mm ⇒ 判定在「贴住」
       与「侵入一丝毫」之间摇摆，不同 IK 支给出相反结果，DP 被迫整链换到
       w1=−180° 那一支（单跳 208.9°）。`NUDGE_BOX_TOP_TOL` 就是为这个加的。

    判据：能生成计划、每段单跳 ≤180°（不然 `split_jumps` 就拒了）、
    末位回到起点（survey 就是起点，不用再走一趟）。
    """
    blocks, cand, q0 = scene(chain)
    q = q0
    for k in (1, 2):                     # 先叠到第 2 层，臂停在它的 survey 位
        surf_k = surface_for(k)
        wps, _checks, _meta = one_layer(
            chain, q, blocks, cand, k,
            tower_top_mm=(surf_k * 1000.0 if k > 1 else None),
            tower_xy_mm=(SLOT_MM if k > 1 else None))
        q = np.asarray(wps[-1]["q"], float)

    surf = surface_for(2)
    slot = (SLOT_MM[0] / 1000.0, SLOT_MM[1] / 1000.0)
    top_mm = (surf + H_BLK) * 1000.0
    pair = pp.stage3_survey_tool0(surf + H_BLK, slot)
    bx = {"idx": 0, "cx": SLOT_MM[0], "cy": SLOT_MM[1],
          "top": top_mm, "h": H_BLK * 1000.0}
    plan = s3r.nudge_plan(              # 传了 tower_box ⇒ 两条老路都会崩/换支
        chain, q, bx, [{"idx": 0, "cx": SLOT_MM[0], "cy": SLOT_MM[1]}],
        shift_xy_mm=(3.0, -2.0), dyaw_deg=4.0, surface_m=surf,
        yaw_block_deg=1.5, open_mm=40.0, survey_tool0=pair[0],
        tower_box=pp.tower_box_from(SLOT_MM, surf * 1000.0, 0.020),
        held_half_edge=0.020)

    prev = np.asarray(plan["q_start"], float)
    for w in plan["waypoints"]:
        qq = np.asarray(w["q"], float)
        jump = float(np.abs(np.degrees(qq - prev)).max())
        assert jump < 180.0, "%s 单跳 %.1f° —— 换支了" % (w["name"], jump)
        prev = qq
    assert [w["name"] for w in plan["waypoints"]][-1] == "nudge_survey"
    assert_close(float(np.abs(prev - np.asarray(plan["q_start"])).max()), 0.0,
                 1e-9, "survey 必须回到起点（同一趟测量位）")


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

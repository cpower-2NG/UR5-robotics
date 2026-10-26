#!/usr/bin/env python3
"""precision_plan 的纯逻辑测试：公式边界、钳制、插点，以及**护栏的变异测试**。

跑法：
  ~/robotics/venvs/robot-pose/bin/python scripts/test_precision_plan.py

为什么要有变异测试
------------------
「预检过了」这句话本身不证明预检有用 —— 一个永远返回 True 的函数也会说过了。
所以这里**故意把航点压低 50 mm**，要求它必须判红；如果哪天有人把护栏删了、
或者把它改成自我认证，这一项会立刻变红。

运动学测试要一份机器人 URDF（**不进仓库**，从板上取的那一份）：
  $PRECISION_URDF，或 ~/robotics/notes/ur5-vision-grasp/runs/*/robot.urdf 里最新的一份。
找不到就跳过这几项并**大声说明跳过了**（跳过不算通过）。
"""
import glob
import json
import os
import sys

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np                                                # noqa: E402
from scipy.spatial.transform import Rotation as Rot               # noqa: E402

import precision_plan as pp                                       # noqa: E402
import urkin                                                      # noqa: E402

CASES = []


def case(fn):
    CASES.append(fn)
    return fn


def find_urdf():
    env = os.environ.get("PRECISION_URDF")
    if env:
        return env if os.path.exists(env) else None
    pat = os.path.expanduser("~/robotics/notes/ur5-vision-grasp/runs/*/robot.urdf")
    hits = sorted(glob.glob(pat), key=os.path.getmtime)
    return hits[-1] if hits else None


# ---------------------------------------------------------------- 纯公式
@case
def test_duration_bounds():
    assert pp.duration_for([0] * 6, [0] * 6) == 2.0, "同一姿态也应给 2 s 底"
    assert pp.duration_for([0] * 6, [0.1] * 6) == 2.0, "小动作不该短于 2 s"
    assert abs(pp.duration_for([0] * 6, [1.0] + [0] * 5) - 4.5) < 1e-9, "1 rad → 4.5 s"
    big = pp.duration_for([0] * 6, [4.0] + [0] * 5)
    assert big == 12.0, "超大动作要封顶 12 s，实际 %.2f" % big
    a = pp.duration_for([0] * 6, [0.5] + [0] * 5)
    b = pp.duration_for([0] * 6, [1.5] + [0] * 5)
    assert a < b, "时长应随关节角单调增"


@case
def test_down_rotation():
    R = pp.down_rotation(0.0)
    assert np.allclose(R, np.diag([1.0, -1.0, -1.0])), R
    # 竖直朝下：tool0 的 z 轴指向 −z
    assert np.allclose(R[:, 2], [0, 0, -1]), R[:, 2]
    for yaw in (-180, -90, 0, 45, 90, 180):
        R = pp.down_rotation(yaw)
        assert np.allclose(R[:, 2], [0, 0, -1]), "yaw=%s 时 z 轴不朝下了" % yaw
        assert abs(np.linalg.det(R) - 1) < 1e-9
        T = pp.target_matrix((0.5, 0.3, 0.2), yaw)
        assert np.allclose(T[:3, 3], [0.5, 0.3, 0.2])


# ---------------------------------------------------------------- 运动学
def scene(chain):
    """合成场景：一块在 (520, 330) mm 高 32 mm，邻块 78 mm 外。"""
    blocks = [
        {"idx": 2, "class": "purple", "cx": 520.0, "cy": 330.0, "top": 32.0, "h": 32.0},
        {"idx": 3, "class": "cyan", "cx": 460.0, "cy": 380.0, "top": 30.0, "h": 30.0},
    ]
    cand = {"score": 0.31, "width_mm": 26.2, "depth": 0.011, "dist_mm": 5.0,
            "angle_deg": 12.0, "seed": 0, "tilt_deg": 4.0,
            "xyz_base": [0.523, 0.331, 0.032],
            "approach_base": [0.0, 0.0, -1.0]}
    q0, pe, re, _ = chain.ik(pp.target_matrix(pp.SURVEY_TOOL0["pick"]),
                            [0.0, -1.2, 1.4, -1.6, -1.5, 0.0])
    assert pe < 0.1 and re < 0.05, "巡检位自己就解不出来：%.3f mm" % pe
    return blocks, cand, q0


@case
def test_limits_reject_raw_overflow(chain):
    q_good = [6.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert pp.check_limits(chain, q_good) == [], "6.0 rad 在软限位内，不该判红"
    bad = pp.check_limits(chain, [6.2, 0.0, 0.0, 0.0, 0.0, 0.0])
    assert bad and bad[0][0] == "shoulder_pan_joint", bad
    # 这条是防「先折算到 ±π 再判」的：6.5 rad 折回来是 −0.78 rad，看着完全合法。
    assert pp.check_limits(chain, [6.5, 0, 0, 0, 0, 0]), \
        "6.5 rad 被放过 —— 限位检查是不是先做了 ±π 折算？"
    assert pp.check_limits(chain, [0, 0, -3.1, 0, 0, 0]), "肘 −3.1 rad 超过 −2.992"


@case
def test_grasp_formula_and_clamp(chain):
    blocks, cand, q0 = scene(chain)
    wps, checks, meta = pp.build_pick(chain, q0, cand, blocks[0], blocks, (0.45, -0.25))
    top = blocks[0]["top"] / 1000.0
    assert abs(meta["approach_z"] - (top + pp.TOOL0_TO_FINGERTIP + pp.APPROACH_CLEARANCE)) < 1e-9
    assert abs(meta["lift_z"] - (top + pp.TOOL0_TO_FINGERTIP + pp.LIFT_HEIGHT)) < 1e-9
    # 顶面 32 mm：32+175.7+10 = 217.7 mm，落在钳制区间内，不钳
    assert abs(meta["grasp_z"] - 0.2177) < 1e-9, meta["grasp_z"]
    assert pp.GRASP_Z_MIN <= meta["grasp_z"] <= pp.GRASP_Z_MAX

    # 台面读数比几何零点高 6 mm（2026-10-25 实测）：wz 必须用**相对高度 h**，
    # 不是绝对 top。用 top 的话整条抓取链会凭空抬高 6 mm，而现场看不出来。
    rel = dict(blocks[0], top=38.0, h=32.0)
    _, _, m_rel = pp.build_pick(chain, q0, cand, rel, [rel], (0.45, -0.25))
    assert abs(m_rel["wz_used_mm"] - 32.0) < 1e-9, m_rel["wz_used_mm"]
    assert abs(m_rel["wz_measured_mm"] - 38.0) < 1e-9, m_rel["wz_measured_mm"]
    assert abs(m_rel["grasp_z"] - 0.2177) < 1e-9, "h 没被用上：%s" % m_rel["grasp_z"]
    # 名义抓取深度 ≈18.5 mm（与块高无关，这是参数 0.1757 的含义）
    assert abs(m_rel["grasp_depth_mm"] - 18.5) < 1e-6, m_rel["grasp_depth_mm"]

    # 矮块：算出来低于下限 ⇒ 抬到 0.2142
    low = dict(blocks[0], top=2.0, h=2.0)
    _, _, m_low = pp.build_pick(chain, q0, cand, low, [low], (0.45, -0.25))
    assert abs(m_low["grasp_z"] - pp.GRASP_Z_MIN) < 1e-9, m_low["grasp_z"]
    # 高块：算出来高于上限 ⇒ 压到 0.2242
    tall = dict(blocks[0], top=200.0, h=200.0)
    _, _, m_tall = pp.build_pick(chain, q0, cand, tall, [tall], (0.45, -0.25))
    assert abs(m_tall["grasp_z"] - pp.GRASP_Z_MAX) < 1e-9, m_tall["grasp_z"]
    # 抓取下限 = **裸指尖**离台面 10 mm，不再借用「放置下降高度」当理由
    assert abs(pp.GRASP_Z_MIN - (pp.GRASP_TIP_MIN_Z
                                 + pp.TOOL0_TO_BARE_FINGERTIP)) < 1e-12
    assert abs(pp.GRASP_Z_MIN - 0.2142) < 1e-9, pp.GRASP_Z_MIN


@case
def test_place_height_follows_block(chain):
    """放置下降必须按**这一块**挂多长算：块底落在台面上方，不能被压进台面。

    2026-10-26 现场缺陷：放置下降写死 0.2142（= 0.0285 + 0.1757 + 0.01），
    其中 0.0285 正好是 A1 的块高 —— 28.5 mm 的块恰好落到台面，缺陷被块高掩盖。
    换成 29.6 mm 的块，块底压进台面 1.1 mm，现场看到的就是「下死力压住桌面」。
    """
    blocks, cand, q0 = scene(chain)
    blk = dict(blocks[0], top=29.6, h=29.6)          # 本轮 #7 cyan 的实测高度
    _, _, meta = pp.build_pick(chain, q0, cand, blk, [blk], (0.45, -0.25))

    below = meta["block_below_tip_mm"] / 1000.0
    tip_z = meta["place_z"] - pp.TOOL0_TO_BARE_FINGERTIP
    block_bottom = tip_z - below
    assert abs(block_bottom - pp.PLACE_CLEARANCE) < 1e-6, \
        "块底应在台面上方 %.3f m，实际 %.4f m" % (pp.PLACE_CLEARANCE, block_bottom)
    assert block_bottom > 0, "块底 %.1f mm 在台面**以下** —— 放置会把块压进桌面" \
        % (block_bottom * 1000)

    # 旧公式（写死 0.2142）在同一块上算出什么：这条是防常量回家的回归哨
    old_place_z = 0.0285 + pp.TOOL0_TO_FINGERTIP + 0.01
    old_bottom = (old_place_z - pp.TOOL0_TO_BARE_FINGERTIP) - below
    assert old_bottom < 0, "旧公式对 29.6 mm 的块应算出 1~1.2 mm 的压入量，实际 %.4f" \
        % old_bottom

    # 块越高，放置点越高（因为挂得更长）；矮块不会低于抓取下限对应的几何
    tall = dict(blocks[0], top=35.0, h=35.0)
    _, _, m_tall = pp.build_pick(chain, q0, cand, tall, [tall], (0.45, -0.25))
    assert m_tall["place_z"] > meta["place_z"], \
        "35 mm 的块放置点反而更低：%.4f <= %.4f" % (m_tall["place_z"], meta["place_z"])
    for m in (meta, m_tall):
        assert m["place_approach_z"] - m["place_z"] > 0.03, m


@case
def test_pick_waypoint_order_and_actions(chain):
    blocks, cand, q0 = scene(chain)
    wps, checks, meta = pp.build_pick(chain, q0, cand, blocks[0], blocks, (0.45, -0.25))
    names = [w["name"] for w in wps]
    assert names == ["approach", "grasp", "lift", "to_slot", "place_approach",
                     "place", "retreat", "survey_place"], names
    by = {w["name"]: w for w in wps}
    assert by["approach"]["before"]["gripper"] == "open", "接近前必须张开"
    assert by["grasp"]["after"]["gripper"] == "close", "抓取后必须闭合"
    assert by["lift"]["after"]["gripper"] == "readback"
    assert by["place"]["after"]["gripper"] == "open", "放置后必须张开"
    for w in wps:
        assert len(w["q"]) == 6 and 2.0 <= w["duration"] <= 12.0, w
    assert len(checks) == len(wps), "预检记录数必须等于航点数"


@case
def test_pick_ik_residuals(chain):
    blocks, cand, q0 = scene(chain)
    _, checks, _ = pp.build_pick(chain, q0, cand, blocks[0], blocks, (0.45, -0.25))
    for c in checks:
        assert c["ik_pos_mm"] < 0.1, "%s IK 位置残差 %.3f mm" % (c["name"], c["ik_pos_mm"])
        assert c["ik_rot_deg"] < 0.05, "%s IK 姿态残差 %.4f°" % (c["name"], c["ik_rot_deg"])
    assert checks[0]["elbow_min_z_mm"] > 0, "段内最低点应在台面之上"


@case
def test_block_gap_guard(chain):
    blocks, cand, q0 = scene(chain)
    close = [blocks[0], dict(blocks[1], cx=560.0, cy=350.0)]   # 44.7 mm
    assert pp.check_block_gap(blocks[0], close) * 1000 < 50
    try:
        pp.build_pick(chain, q0, cand, blocks[0], close, (0.45, -0.25))
    except pp.PlanError as e:
        assert "邻块" in str(e), e
    else:
        raise AssertionError("邻块只隔 45 mm，预检必须判红（夹爪全开 95 mm）")


@case
def test_candidate_anchor_guard(chain):
    blocks, cand, q0 = scene(chain)
    far = dict(cand, dist_mm=12.5)
    try:
        pp.build_pick(chain, q0, far, blocks[0], blocks, (0.45, -0.25))
    except pp.PlanError as e:
        assert "锚定" in str(e), e
    else:
        raise AssertionError("候选离块心 12.5 mm，超过 12 mm 锚定半径，必须判红")


@case
def test_split_jumps(chain):
    # 两条航点之间一个 2.5 rad 的跳变 ⇒ 至少要插 2 个点，且插完每段 ≤ 1.0 rad
    # 基准姿态用 0.5 rad 而不是全零：全零正好是腕 2 = 0 的奇异位形，
    # 腕护栏会（正确地）把它拦下 —— 那样测到的是护栏，不是插点。
    base = [0.5] * 6
    wps = [{"name": "a", "q": list(base), "duration": 2.0},
           {"name": "b", "q": [base[0] + 2.5] + base[1:], "duration": 2.0}]
    out, added = pp.split_jumps(wps, chain, max_jump=1.0)
    assert added == 2, "2.5 rad / 1.0 ⇒ 应插 2 个点，实际 %d" % added
    assert [w["name"] for w in out] == ["a", "b_split1", "b_split2", "b"], \
        [w["name"] for w in out]
    jumps = [abs(np.asarray(out[i + 1]["q"]) - np.asarray(out[i]["q"])).max()
             for i in range(len(out) - 1)]
    assert max(jumps) <= 1.0 + 1e-9, jumps
    # 插出来的点必须落在原来的关节直线上（JTC 本来就是直线，插点不改轨迹形状）
    assert np.allclose(out[1]["q"], [base[0] + 2.5 / 3] + base[1:])
    # 不超阈值时不插
    out2, added2 = pp.split_jumps(wps, chain, max_jump=3.0)
    assert added2 == 0 and out2 == wps


@case
def test_wrist_guard(chain):
    """腕 2 过 0（奇异位形）必须判红 —— 包括**插点插出来**的过 0。"""
    i = chain.move_joints.index("wrist_2_joint")
    bad = [0.5] * 6
    bad[i] = 0.05
    hit = pp.check_wrist(chain, bad)
    assert hit is not None and abs(hit[0] - np.degrees(0.05)) < 0.01, hit
    good = [0.5] * 6
    good[i] = 0.6
    assert pp.check_wrist(chain, good) is None, "0.6 rad 不该判红"
    # 从 +0.6 走到 −0.6：两端都合法，但直线插值会穿过 0 —— 必须拦
    wps = [{"name": "a", "q": good, "duration": 2.0},
           {"name": "b", "q": [0.5] * i + [-0.6] + [0.5] * (5 - i), "duration": 2.0}]
    try:
        pp.split_jumps(wps, chain, max_jump=1.0)
    except pp.PlanError:
        pass
    else:
        raise AssertionError("插值穿过腕 2 = 0，护栏没拦")


# ---------------------------------------------------------------- 变异测试
@case
def test_mutation_lowered_waypoint_is_rejected(chain):
    """**判红的证明**：把抓取/放置航点压低 50 mm，预检必须拦住。

    压低 50 mm 之后**裸指尖**已经到台面以下（抓取 −40 mm / 放置 −34 mm），
    连杆还在高处 —— 所以这一项同时证明「指尖点」这条判据真的在起作用，
    而不是只看肘和腕 1，也不是拿 0.1757 那个参数冒充指尖。
    """
    blocks, cand, q0 = scene(chain)
    _, _, meta = pp.build_pick(chain, q0, cand, blocks[0], blocks, (0.45, -0.25))
    gx, gy = meta["grasp_point"]
    for label, z in (("抓取", meta["grasp_z"] - 0.050),
                     ("放置", meta["place_z"] - 0.050)):
        try:
            pp.add_move([], chain, "mut_" + label, (gx, gy, z), q0, 0.0, 0.0)
        except pp.PlanError as e:
            assert "护栏" in str(e), "拦是拦住了，但理由不对：%s" % e
            assert "fingertip" in str(e), "拦住的原因不是指尖：%s" % e
        else:
            raise AssertionError("把%s航点压低 50 mm 竟然通过了预检 —— 护栏是摆设"
                                 % label)


@case
def test_mutation_deep_grasp_reaches_floor(chain):
    """再压深 100 mm 也必须红；顺带证明 5 mm 以内不算红（护栏不是一碰就响）。

    注意这里的余量是 5 mm 不是 20 mm：指尖下限改成**裸指尖** 0.2042 + 8 mm 之后，
    32 mm 的块抓取时指尖只在台面上方 13.5 mm —— 再压 20 mm 是真会到台面以下，
    那时候判红是**对的**，不该为让测试变绿放宽护栏。
    """
    blocks, cand, q0 = scene(chain)
    _, _, meta = pp.build_pick(chain, q0, cand, blocks[0], blocks, (0.45, -0.25))
    gx, gy = meta["grasp_point"]
    pp.add_move([], chain, "ok5", (gx, gy, meta["grasp_z"] - 0.005), q0, 0.0, 0.0)
    # 20 mm 更深：拿参数 0.1757 冒充指尖时这条会**放过**（那点 z=22 mm > 20 mm 下限），
    # 只有用裸指尖 0.2042 才拦得住 —— 这一项就是「指尖用的是哪个数」的证据。
    try:
        pp.add_move([], chain, "mut20", (gx, gy, meta["grasp_z"] - 0.020), q0, 0.0, 0.0)
    except pp.PlanError as e:
        assert "fingertip" in str(e), e
    else:
        raise AssertionError("压深 20 mm 时**裸指尖**已在台面以下，护栏却放过 —— "
                             "是不是又拿 0.1757 冒充指尖了？")
    try:
        pp.add_move([], chain, "mut100", (gx, gy, meta["grasp_z"] - 0.100), q0, 0.0, 0.0)
    except pp.PlanError:
        pass
    else:
        raise AssertionError("压低 100 mm 竟然通过了预检")


@case
def test_fingertip_guard_direct(chain):
    """不经过 add_move，直接测 check_guard 本身：同一姿态下压 60 mm。"""
    blocks, cand, q0 = scene(chain)
    _, _, meta = pp.build_pick(chain, q0, cand, blocks[0], blocks, (0.45, -0.25))
    gx, gy = meta["grasp_point"]
    T = pp.target_matrix((gx, gy, meta["grasp_z"]))
    q_hi, pe, re, _ = chain.ik(T, q0)
    assert pe < 0.1
    ok, worst = pp.check_guard(chain, q_hi, q_hi)
    assert ok, "抓取点自身不该被判红：%s" % (worst,)
    T2 = pp.target_matrix((gx, gy, meta["grasp_z"] - 0.060))
    q_lo, pe2, re2, _ = chain.ik(T2, q_hi)
    assert pe2 < 0.1, "压低后的姿态应该解得出来（否则测的就不是护栏）"
    ok2, worst2 = pp.check_guard(chain, q_hi, q_lo)
    assert not ok2, "压低 60 mm 却没判红：%s" % (worst2,)
    assert worst2[1] == "fingertip", "判红的应该是指尖，实际 %s" % (worst2[1],)


# ---------------------------------------------------------------- runner
def main():
    urdf = find_urdf()
    if urdf:
        chain = urkin.Chain(urdf)
        print("URDF: %s" % urdf)
    else:
        chain = None
        print("!! 找不到 robot.urdf（$PRECISION_URDF 或 runs/*/robot.urdf）")
        print("!! 需要运动学的 %d 项会**跳过** —— 跳过不等于通过。" % (len(CASES) - 2))

    npass = nskip = nfail = 0
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
        except Exception as e:                               # noqa: BLE001
            print("ERROR %s: %s: %s" % (fn.__name__, type(e).__name__, e))
            nfail += 1
        else:
            print("PASS %s" % fn.__name__)
            npass += 1
    print("\n%d PASS, %d FAIL, %d SKIP" % (npass, nfail, nskip))
    return 1 if nfail else 0


if __name__ == "__main__":
    sys.exit(main())

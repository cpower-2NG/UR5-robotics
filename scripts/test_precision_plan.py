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
def test_place_target_does_not_trust_grasp_eccentricity(chain):
    """放置目标默认**就是槽位**，不拿抓取偏心去补 —— 但偏心要记下来。

    曾经补过（把 tool0 放在 slot−偏心）。2026-10-26 第 6 轮实测判决：
    抓取偏心预测 6.44 mm，抓着块实际只偏 2.76 mm，补了反而多偏 4.3 mm。
    三轮的夹持偏移 9.7 / 14.5 / 2.8 mm 跟抓取偏心 2.4 / 5.5 / 6.4 mm 不对应
    —— 抓取前的偏心预测不了夹持后的偏移。

    （这里曾经写「块在爪里会转、会移」当解释，2026-10-26 复核**作废**：那个
    「转了 24°」来自 36.01 × 36.05 的近方形顶面（长短差 0.04 mm），yaw 在那里
    没有定义；块在爪里到底转没转、握在哪，目前既无证据也无反证。）

    所以：默认不补（这条钉住默认值），但 meta 里必须留着 grasp_ecc_mm，
    等有相机能看见夹着的块再拿真值来判。
    """
    blocks, cand, q0 = scene(chain)
    blk = blocks[0]
    slot = (0.45, -0.25)
    wps, _, meta = pp.build_pick(chain, q0, cand, blk, blocks, slot)
    by = {w["name"]: w for w in wps}

    ex = blk["cx"] / 1000.0 - cand["xyz_base"][0]       # 测试场景里是 (−3, −1) mm
    ey = blk["cy"] / 1000.0 - cand["xyz_base"][1]
    assert abs(ex) > 1e-3 and abs(ey) > 1e-3, "测试场景本身没有偏心，测不出东西"
    assert pp.PLACE_COMPENSATE_ECC is False, \
        "补偿又被打开了 —— 先确认有过新的真机证据（见 PLACE_COMPENSATE_ECC）"

    T = chain.fk(by["place"]["q"])
    assert abs(T[0, 3] - slot[0]) < 1e-4, \
        "place x 应在槽位 %.4f，实际 %.4f" % (slot[0], T[0, 3])
    assert abs(T[1, 3] - slot[1]) < 1e-4, \
        "place y 应在槽位 %.4f，实际 %.4f" % (slot[1], T[1, 3])
    # 偏心量仍然要记下来（误差归因就靠它），只是不拿来挪目标
    assert meta["grasp_ecc_mm"] == [round(ex * 1000, 2), round(ey * 1000, 2)], \
        "meta 里的偏心与实际不符：%r" % (meta["grasp_ecc_mm"],)
    assert meta["place_target_mm"] == [round(slot[0] * 1000, 2),
                                       round(slot[1] * 1000, 2)], \
        "meta 里的放置目标应与槽位一致：%r" % (meta["place_target_mm"],)

    # 开关打开时的行为也要是对的（不然将来想用反而用不上）：slot−偏心
    old = pp.PLACE_COMPENSATE_ECC
    try:
        pp.PLACE_COMPENSATE_ECC = True
        wps2, _, meta2 = pp.build_pick(chain, q0, cand, blk, blocks, slot)
        T2 = chain.fk([w for w in wps2 if w["name"] == "place"][0]["q"])
        assert abs(T2[0, 3] - (slot[0] - ex)) < 1e-4, \
            "开关打开时 place x 应在 %.4f，实际 %.4f" % (slot[0] - ex, T2[0, 3])
        assert meta2["place_target_mm"] == [round((slot[0] - ex) * 1000, 2),
                                            round((slot[1] - ey) * 1000, 2)]
    finally:
        pp.PLACE_COMPENSATE_ECC = old


@case
def test_ik_branch_selection_keeps_forearm_above_table(chain):
    """槽 2（y=−350 mm）那类目标：单种子 IK 会钻进台面下，整链选支才不会。

    2026-10-26 第 6 轮的实数：块 #2 red 高 31.9 mm、抓点 (500.2, 409.0)、
    槽 (450, −350)。逐点用上一个航点的解当种子时，`to_slot` 落进
    pan=−206.6° 那一支（前臂 117 mm），`place` 前臂掉到 **−9.8 mm**，
    被高度护栏拒绝 —— 而同一目标另一支解的前臂在 **498 mm**。
    这就是「计划生成失败、一轮白停」的来路：不是够不到，是没换支。
    """
    blocks = [{"idx": 2, "class": "red", "cx": 504.8, "cy": 404.5,
               "top": 31.9, "h": 31.9}]
    cand = {"score": 0.2417, "width_mm": 48.57, "depth": 0.01, "dist_mm": 6.4,
            "angle_deg": -80.3, "seed": 0, "tilt_deg": 25.0,
            "xyz_base": [0.5002, 0.4090, 0.0310],
            "approach_base": [0.0, 0.0, -1.0]}
    q_start = [0.293028, -1.320825, 1.459162, -1.708434, -1.568054, -1.278646]
    slot = (0.45, -0.35)

    # ① 复现旧做法（逐点用上一点的解当种子）：必须在 place 判红 ——
    #    否则这条测试就不再证明「换支」是必要的了。
    wz = blocks[0]["h"] / 1000.0
    gx, gy = cand["xyz_base"][:2]
    gz = float(np.clip(wz + pp.TOOL0_TO_FINGERTIP + pp.GRASP_CLEARANCE,
                       pp.GRASP_Z_MIN, pp.GRASP_Z_MAX))
    depth = wz - (gz - pp.TOOL0_TO_BARE_FINGERTIP)
    pz, paz = pp.place_heights(wz, depth)
    ex, ey = blocks[0]["cx"] / 1000 - gx, blocks[0]["cy"] / 1000 - gy
    sx, sy = slot[0] - ex, slot[1] - ey
    targets = [
        ("approach", (gx, gy, wz + pp.TOOL0_TO_FINGERTIP + pp.APPROACH_CLEARANCE)),
        ("grasp", (gx, gy, gz)),
        ("lift", (gx, gy, wz + pp.TOOL0_TO_FINGERTIP + pp.LIFT_HEIGHT)),
        ("to_slot", (sx, sy, wz + pp.TOOL0_TO_FINGERTIP + pp.LIFT_HEIGHT)),
        ("place_approach", (sx, sy, paz)),
        ("place", (sx, sy, pz)),
    ]
    q = list(q_start)
    naive_bad = None
    for name, xyz in targets:
        prev = q
        q, pe, re, _ = pp.solve_waypoint(chain, xyz, 0.0, q)
        assert pe < 0.1 and re < 0.05, "%s 单种子都解不出：%.3f mm" % (name, pe)
        ok, worst = pp.check_guard(chain, prev, q, pp.TABLE_PLANE_Z)
        if not ok:
            naive_bad = (name, worst[1], worst[2] * 1000)
            break
    assert naive_bad is not None, \
        "旧做法居然过了 —— 这组数不再复现那个 bug，测试要重写"
    assert naive_bad[0] == "place" and naive_bad[1] == "forearm_link", naive_bad
    assert naive_bad[2] < 0, "旧做法应当让前臂钻到台面下，实际 %.1f mm" % naive_bad[2]

    # ② 新做法：整条计划必须生成得出来，且每个航点的前臂都在台面上方
    wps, checks, meta = pp.build_pick(chain, q_start, cand, blocks[0], blocks,
                                      slot)
    assert meta["place_target_mm"] == [round(slot[0] * 1000, 2),
                                       round(slot[1] * 1000, 2)], meta
    for w in wps:
        if w["name"].startswith("to_slot_split") or w["name"] == "survey_place":
            continue
        fz = chain.fk_link("forearm_link", w["q"])[2, 3]
        assert fz > pp.GUARD_MARGIN, \
            "航点 %s 的前臂只有 %.1f mm（下限 %.0f）" % (
                w["name"], fz * 1000, pp.GUARD_MARGIN * 1000)
    # 每一段都要真的过护栏（不只是端点高）
    for a, b in zip(wps, wps[1:]):
        ok, worst = pp.check_guard(chain, a["q"], b["q"], pp.TABLE_PLANE_Z)
        assert ok, "%s→%s 段内护栏不过：%s z=%.1f" % (
            a["name"], b["name"], worst[1], worst[2] * 1000)


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


# ---------------------------------------------------------------- 编排的日志拼接
@case
def test_split_exec_log_keeps_first_half():
    """把一条计划切成两段跑时，本地日志必须留下**完整的一份**。

    第二段跑完`板上`的 JSONL 是「第一段 + 第二段」。拉回来如果直接覆盖，
    第一段的闭合/抬升回读就没了 —— 而滑移判据正是拿那两个数算的，
    症状是「本轮 slipped=None」，看起来像没滑，实际是**没读到**。
    所以这里同时钉住两件事：追加模式不能多也不能少，非追加模式必须覆盖。
    """
    import subprocess
    import tempfile
    import precision_round as pr

    d = tempfile.mkdtemp()
    local = os.path.join(d, "exec.jsonl")
    remote = os.path.join(d, "remote.jsonl")
    with open(local, "w") as f:
        f.write("a\nb\n")                      # 第一段已经拉回来
    with open(remote, "w") as f:
        f.write("a\nb\nc\nd\n")                # 板上：第一段 + 第二段

    b = pr.Board.__new__(pr.Board)             # 不走 __init__，不需要真的板子
    b.alias = "board"
    orig = pr.sh
    pr.sh = lambda cmd, **kw: subprocess.run(["cp", remote, cmd[-1]], check=True)
    try:
        b._pull_log("remote.jsonl", local, append=True)
        got = open(local).read()
        assert got == "a\nb\nc\nd\n", "追加后应恰好多出二段两行，实际 %r" % got

        with open(remote, "w") as f:
            f.write("c\nd\n")                  # 新一轮：板上只有这一次的
        b._pull_log("remote.jsonl", local, append=False)
        got = open(local).read()
        assert got == "c\nd\n", "非追加必须整体覆盖，实际 %r" % got

        # 续跑：板上被清过，只有第二段；本地留着第一段。按行号切会吞掉
        # 第二段自己的前几行 —— 这里钉住「全部追加」。
        with open(local, "w") as f:
            f.write("a\nb\n")
        with open(remote, "w") as f:
            f.write("e\nf\ng\n")
        b._pull_log("remote.jsonl", local, append=True, remote_has_prefix=False)
        got = open(local).read()
        assert got == "a\nb\ne\nf\ng\n", "续跑应整段追加，实际 %r" % got
    finally:
        pr.sh = orig


@case
def test_resume_run_command_shape():
    """续跑的命令行要写对 —— 写错了不是报错，是**臂会重跑一遍前段**。

    危险具体在哪：漏了 ``--from``，执行器会从 approach 开始再来一次，
    夹着块的爪会被重新下探到台面、重新闭合。板上日志**必须**先清掉：里面留着
    上一次被闸门拒掉的那几行，不清就会把续跑判成「闸门没过」；本地那份留着，
    第一段的闭合/抬升回读只能从它里面读。
    """
    import subprocess
    import tempfile
    import types
    import precision_round as pr

    d = tempfile.mkdtemp()
    plan = os.path.join(d, "pick.json")
    open(plan, "w").write("{}")
    log = os.path.join(d, "exec.jsonl")
    with open(log, "w") as f:                       # 第一段已经拉回来了
        f.write('{"event": "arrive", "waypoint": "lift"}\n')
    remote = os.path.join(d, "remote.jsonl")
    with open(remote, "w") as f:                    # 板上的第二段
        f.write('{"event": "traj", "waypoint": "to_slot_split1", "ok": true}\n')

    b = pr.Board.__new__(pr.Board)
    b.alias = "board"
    seen = []

    def fake_sh(cmd, **kw):
        """只把「拉日志」这条 scp 真的执行掉；推计划那条是发去板上的，跳过。

        板上日志路径是 `<plan>.jsonl`（不是任意 .jsonl），所以按「源是 .jsonl、
        目的地不是 board:」来认。
        """
        if (any(str(c).endswith(".jsonl") for c in cmd)
                and not str(cmd[-1]).startswith("board:")):
            return subprocess.run(["cp", remote, cmd[-1]], check=True)
        return subprocess.run(["true"], capture_output=True)

    orig_sh, orig_ssh = pr.sh, pr.Board.ssh
    pr.sh = fake_sh
    pr.Board.ssh = lambda self, cmd, **kw: (
        seen.append(cmd),
        types.SimpleNamespace(returncode=0, stdout=""))[1]
    try:
        b.exec_plan(plan, log, True, from_="to_slot_split1", append=True,
                    reset_remote=True)
    finally:
        pr.sh, pr.Board.ssh = orig_sh, orig_ssh

    run = [c for c in seen if "precision_exec.py" in c]
    assert len(run) == 1, "续跑只发一条执行命令，实际 %r" % (seen,)
    assert "--from to_slot_split1" in run[0], "续跑没带上 --from：%r" % run[0]
    assert "--upto" not in run[0], "续跑不该再带 --upto：%r" % run[0]
    assert any("rm -f" in c for c in seen), "续跑必须清板上日志（里面是被拒的那几行）"
    got = open(log).read()
    assert got.count("\n") == 2, "续跑后本地应有两行（第一段 + 第二段），实际 %r" % got
    assert "to_slot_split1" in got, "第二段没并进本地日志：%r" % got


@case
def test_last_arrived_reads_the_log():
    """续跑从**日志里最后一条 arrive** 接着跑，不写死航点名。

    `traj ok` 只说明控制器收下了轨迹；`arrive` 才是实测到位的记录。
    半截行（scp 断掉）不能被当成航点，否则续跑起点会是个不存在的名字。
    """
    import tempfile
    import precision_round as pr

    d = tempfile.mkdtemp()
    log = os.path.join(d, "exec.jsonl")
    assert pr.last_arrived(log) is None, "日志不存在时应返回 None"
    with open(log, "w") as f:
        f.write('{"event": "gate", "ok": true}\n')
        f.write('{"event": "arrive", "waypoint": "approach"}\n')
        f.write('{"event": "traj", "waypoint": "grasp", "ok": true}\n')
        f.write('{"event": "arrive", "waypoint": "grasp"}\n')
        f.write('{"event": "arrive", "waypoint": "lift"}\n')
        f.write('{"event": "gate", "ok": false}\n')
        f.write('{"event": "arrive", "waypoint": "to_sl')      # 半截行
    assert pr.last_arrived(log) == "lift", \
        "应取最后一条完整 arrive（lift），实际 %r" % (pr.last_arrived(log),)


@case
def test_failed_gate_then_traj_is_a_new_segment():
    """人工续跑之后，前一段那次「闸门被拒」不能把这一轮判红。

    第 5 轮的真日志就是这样：被拒的 gate（34.08°）后面紧跟续跑的轨迹，
    全都成功、块也放好了。按「所有 gate 都 ok」判就会把这一轮记成红字 ——
    但那次拒绝是**历史**，不是这一轮的判决。
    反过来，被拒的 gate 之后什么都没有（正常在闸门处停住）必须照常判红。
    """
    import json
    import tempfile
    import precision_verify as pv

    d = tempfile.mkdtemp()
    log = os.path.join(d, "exec.jsonl")

    def write(recs):
        with open(log, "w") as f:
            for r in recs:
                f.write(json.dumps(r) + "\n")

    base = [{"event": "gripper", "where": "grasp.after", "action": "close",
             "position_mm": 24.89},
            {"event": "grasp_check", "lifted_mm": 24.89, "slipped": False}]
    resumed = base + [
        {"event": "gate", "ok": False, "drift_deg": 34.08},   # 第一段被拒
        {"event": "gate", "ok": True},
        {"event": "traj", "waypoint": "to_slot_split1", "ok": True},
        {"event": "gate", "ok": True},
        {"event": "traj", "waypoint": "place", "ok": True},
        {"event": "arrive", "waypoint": "survey_place", "err_deg": 0.02}]
    write(resumed)
    ls = pv.summarize_exec_log(log)
    assert ls["gate_all_ok"], "续跑成功却判成闸门没过：%r" % (ls,)
    assert ls["traj_ok"], "续跑成功却判成轨迹失败：%r" % (ls,)
    assert ls["close_mm"] == 24.89, "按段切之后丢了第一段的闭合回读：%r" % (ls,)
    assert ls["lift_mm"] == 24.89, "按段切之后丢了 grasp_check：%r" % (ls,)
    assert ls["slipped"] is False, "按段切之后丢了滑移判据：%r" % (ls,)

    # 正常在闸门处停住：拒绝之后没有任何轨迹 ⇒ 必须判红
    write(base + [{"event": "gate", "ok": False, "drift_deg": 34.08}])
    ls = pv.summarize_exec_log(log)
    assert not ls["gate_all_ok"], "停在闸门上却没判红：%r" % (ls,)


@case
def test_next_waypoint_reads_the_plan():
    """lift 之后的那一条**从计划里读**，不是写死的名字。

    第 5 轮的真计划是 approach/grasp/lift/**to_slot_split1**/to_slot/…：
    肩关节 33.3° → −34.9° 超过 1 rad，`split_jumps` 插了中间点。
    写死 `to_slot` 就跳过它，第二段的闸门基准差 34.08° ⇒ 被拒 ⇒ 一轮抓放白停
    （臂夹着块悬在 lift）。所以这里就按真计划的形状钉住。
    """
    import precision_round as pr

    plan = {"waypoints": [{"name": n} for n in
                          ("approach", "grasp", "lift", "to_slot_split1",
                           "to_slot", "place_approach", "place", "retreat",
                           "survey_place")]}
    assert pr.next_waypoint(plan, "lift") == "to_slot_split1", \
        "必须取插入点，不能取写死的 to_slot"
    assert pr.next_waypoint(plan, "grasp") == "lift"
    for bad in ("nope", "survey_place"):
        try:
            pr.next_waypoint(plan, bad)
        except SystemExit:
            pass
        else:
            raise AssertionError("%r 应当直接报错（不存在 / 已是最后一条）" % bad)


@case
def test_finish_pose_guard(chain):
    """`--finish` 必须在计划的 survey_place 位姿上收尾，否则量出来的放置偏是假的。

    这条守的是「人工补账」这个口子：臂要是停在半路（比如夹着块停在 lift），
    抓回来的帧里根本没有这一轮的落点，槽位和块不在同一套坐标里 ——
    宁可报错让人把臂开回巡检位，也不能悄悄写一个漂亮的错数。
    """
    import json
    import tempfile
    import precision_round as pr

    d = tempfile.mkdtemp()
    names = chain.move_joints
    q = [0.1] * len(names)
    plan = os.path.join(d, "pick.json")
    json.dump({"waypoints": [{"name": "survey_place", "q": q}]}, open(plan, "w"))

    def frame(tag, qq):
        p = os.path.join(d, tag)
        os.makedirs(p, exist_ok=True)
        json.dump({"name": names, "position": qq},
                  open(os.path.join(p, "joint_states.json"), "w"))
        return p

    ok = frame("place", q)
    assert pr.check_survey_pose(ok, plan, chain.path) < 0.01, \
        "同一个位姿却判成不一样"

    other = list(q)
    other[0] += 0.2                                    # 11.5°
    bad = frame("held", other)
    try:
        pr.check_survey_pose(bad, plan, chain.path)
    except SystemExit:
        pass
    else:
        raise AssertionError("位姿差了 11.5° 还放行 —— 会写出假的放置误差")


# ---------------------------------------------------------------- runner
def main():
    urdf = find_urdf()
    if urdf:
        chain = urkin.Chain(urdf)
        chain.path = urdf          # 给需要 URDF 路径的用例用
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

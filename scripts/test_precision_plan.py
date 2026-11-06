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
    # 高块：算出来高于上限 ⇒ 压到 0.2242。块真的 200 mm 高、底面在台面，
    # 所以必须显式给 block_h_m（漏给会被新护栏拒，见下一条测试）。
    tall = dict(blocks[0], top=200.0, h=200.0)
    _, _, m_tall = pp.build_pick(chain, q0, cand, tall, [tall], (0.45, -0.25),
                                 block_h_m=0.200)
    assert abs(m_tall["grasp_z"] - pp.GRASP_Z_MAX) < 1e-9, m_tall["grasp_z"]
    # 抓取下限 = **裸指尖**离台面 10 mm，不再借用「放置下降高度」当理由
    assert abs(pp.GRASP_Z_MIN - (pp.GRASP_TIP_MIN_Z
                                 + pp.TOOL0_TO_BARE_FINGERTIP)) < 1e-12
    assert abs(pp.GRASP_Z_MIN - 0.2142) < 1e-9, pp.GRASP_Z_MIN


@case
def test_tower_block_grasp_anchors_to_block_bottom(chain):
    """拆塔（2026-11-02 事故回归）：块顶 91.34 mm、块高 29.5 mm。

    旧代码把块顶当块高 ⇒ 绝对 277 mm 被 `GRASP_Z_MAX` 截到 224.2 mm ⇒ 指尖
    落在台面上 20 mm（最底层），一抓把三层全带起来。现在几何锚在块底：
    指尖应落在**顶层那块内部**、离块底 ≈11 mm，抓取深度仍是名义 18.5 mm。
    """
    blocks, cand, q0 = scene(chain)
    top_mm, bh_mm = 91.34, 29.5
    tower = dict(blocks[0], cx=520.0, cy=330.0, top=top_mm, h=top_mm)
    _, _, meta = pp.build_pick(chain, q0, cand, tower, [tower], (0.45, -0.25),
                               block_h_m=bh_mm / 1000.0)
    bottom_m = (top_mm - bh_mm) / 1000.0
    assert abs(meta["block_bottom_mm"] - bottom_m * 1000) < 1e-6, meta
    assert abs(meta["block_true_h_mm"] - bh_mm) < 1e-6, meta
    tip_m = meta["grasp_z"] - pp.TOOL0_TO_BARE_FINGERTIP
    # 指尖在顶层块内，离块底 ≈11 mm（与台面块同一条式子），绝不是台面 20 mm
    assert bottom_m + 0.005 < tip_m < top_mm / 1000.0, tip_m * 1000
    # meta 的 z 值只留到 0.1 mm，比较容差按这个分辨率给
    assert abs(tip_m - bottom_m - (bh_mm / 1000.0
                                   - meta["grasp_depth_mm"] / 1000.0)) < 1e-4, \
        tip_m * 1000
    assert abs(meta["grasp_depth_mm"] - 18.5) < 1e-6, meta["grasp_depth_mm"]
    # 放置高度按**块高 29.5** 算 ⇒ 与台面块同级（≈0.2209 m），不是抬高 62 mm
    assert meta["place_z"] < 0.23, meta["place_z"]
    assert meta["approach_z"] > top_mm / 1000.0, "接近点必须在块顶之上"


@case
def test_tower_item_without_block_h_is_rejected(chain):
    """漏给 block_h_m 而块顶又高得不像台面块 ⇒ 计划期就拒（事故形态的闸门）。"""
    blocks, cand, q0 = scene(chain)
    tower = dict(blocks[0], top=91.34, h=91.34)
    try:
        pp.build_pick(chain, q0, cand, tower, [tower], (0.45, -0.25))
    except pp.PlanError as exc:
        assert "block_h_m" in str(exc), exc
    else:
        raise AssertionError("块顶 91 mm 却没给 block_h_m，居然过了")


@case
def test_block_bottom_noise_at_table_is_tolerated(chain):
    """塔底最后一块直接坐垫纸上：块顶实测比名义块高矮一点点 ⇒ 块底算出小负数。

    那是顶面测量噪声（单帧 σ 1.2–2.6 mm），不是「参数给反了」——
    2026-11-06 拆第 5 块就因块底 −0.004 mm 被判红。容差内夹到 0，
    抓取深度仍按名义块高；参数真给反（大出几十毫米）照样要拒。
    """
    blocks, cand, q0 = scene(chain)
    bh_mm = 29.5
    for top_mm in (29.5 - 0.004, 29.5 - 1.8):
        flat = dict(blocks[0], top=top_mm, h=top_mm)
        _, _, meta = pp.build_pick(chain, q0, cand, flat, [flat], (0.45, -0.25),
                                   block_h_m=bh_mm / 1000.0)
        assert abs(meta["block_bottom_mm"]) < 1e-9, \
            "块底 %.4f mm 应夹到 0" % meta["block_bottom_mm"]
        # 指尖锚在块底(0)+名义块高上，不跟块顶噪声走 —— 这是修护栏时唯一
        # 要求不变的量；抓取深度 = 实测块顶 − 指尖，当然随块顶变。
        assert abs(meta["fingertip_z_mm"] - 11.0) < 1e-9, meta
        assert abs(meta["grasp_depth_mm"] - (top_mm - 11.0)) < 0.051, meta
    tower = dict(blocks[0], top=29.5, h=29.5)
    try:
        pp.build_pick(chain, q0, cand, tower, [tower], (0.45, -0.25),
                      block_h_m=0.091)
    except pp.PlanError as exc:
        assert "块底算出" in str(exc), exc
    else:
        raise AssertionError("块高 91 mm 比块顶 29.5 mm 大 61 mm，居然过了")


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
def test_wrist_winding_is_avoided(chain):
    """腕 1 / 腕 3 不许停在「绕死了」的那一圈 —— 2026-10-27 线缆拧死事故。

    回位那条轨迹的终点是 wrist_1=−262.6°、wrist_3=−253.4°：位姿完全正确，
    但夹爪线缆比出事前多拧了约 285°，人只能手掰回来。判据：腕 1/腕 3 的
    **字面角**离开 0 超过 210° 就算拧死 —— 此前跑通的计划最狠 −204.1° 要放行。
    """
    i1 = chain.move_joints.index("wrist_1_joint")
    i3 = chain.move_joints.index("wrist_3_joint")
    ok = [0.0] * 6
    ok[i1], ok[i3] = np.radians(-204.1), np.radians(-120.0)
    assert pp.check_winding(chain, ok) is None, "−204.1° 是跑通过的角度，不该拦"

    bad = [0.0] * 6
    bad[i1], bad[i3] = np.radians(-262.6), np.radians(-253.4)
    hit = pp.check_winding(chain, bad)
    assert hit is not None and hit[0] == "wrist_1_joint", hit

    # 折角：离 ref 最近的那一圈是拧死的那圈 ⇒ 必须换一圈（+97.4° 同一位姿）
    ref = [0.0] * 6
    ref[i1] = np.radians(-117.0)
    got = pp.unwrap_near(chain, bad, ref)
    assert abs(np.degrees(got[i1]) - 97.4) < 0.2, np.degrees(got[i1])
    assert pp.check_winding(chain, got) is None
    # 位姿一个字节都不变
    pe, re = pp.pose_residual(chain, got, tuple(chain.fk(bad)[:3, 3]), 0.0)
    assert pe < 0.01, pe
    # 不拧的情形仍按「离 ref 最近」走，别乱换圈
    mild = [0.0] * 6
    mild[i1] = np.radians(-150.0)
    assert abs(np.degrees(pp.unwrap_near(chain, mild, ref)[i1]) + 150.0) < 0.2


@case
def test_no_full_circle_wrist_flip(chain):
    """腕整圈翻转必须从根上消失 —— 2026-10-27 B2 第 5 轮块 #2 的真机事故。

    实数（板上 `pick-02.json`）：抓取角 148.583°、块心 (398.1, 406.14)、
    块高 29.26 mm、槽 (362.65, −114.66)、起点 q = [−45.52, −67.59, 72.74,
    −95.1, −89.82, −135.56]°。同一末端位姿的一支 IK 给的是 wrist_2=+270.1°，
    与 −89.9° 按 2π 折算距离为 0 ⇒ 整链选支以为腕不用动，实际整圈翻转；
    臂走到第 3 个插点（wrist_2=+64.4°）时夹爪朝天，人按了示教器 STOP。

    ① 把 `unwrap_near` 关掉（= 旧行为）⇒ 必须被 `MAX_JOINT_SWING` 拦下；
    ② 正身 ⇒ 计划生成得出来，且**没有任何一段关节超过 180°**。
    """
    block = {"idx": 2, "class": "cyan", "cx": 398.1, "cy": 406.14,
             "top": 29.26, "h": 29.26}
    cand = {"score": 0.3605, "width_mm": 58.54, "depth": 0.01, "dist_mm": 1.32,
            "angle_deg": -117.9, "seed": 1, "tilt_deg": 20.0,
            "xyz_base": [0.3981, 0.40614, 0.0293],
            "approach_base": [0.0, 0.0, -1.0]}
    q_start = [np.radians(v) for v in
               (-45.52, -67.59, 72.74, -95.1, -89.82, -135.56)]
    slot = (0.36265, -0.11466)
    kw = dict(yaw_grasp_deg=148.583, yaw_place_deg=0.0, yaw_source="geometric",
              pick_open_mm=50.0, place_open_mm=50.0, grasp_at="block_center",
              release_dwell_s=0.5)

    def build():
        wps, _, _ = pp.build_pick(chain, q_start, cand, block, [block], slot, **kw)
        return pp.split_jumps(wps, chain)[0]

    def swings(wps):
        return [float(np.abs(np.asarray(b["q"]) - np.asarray(a["q"])).max())
                for a, b in zip(wps, wps[1:])]

    # ① 旧行为（抓取角不折 180° + 行程按 2π 折算 + 不折角）⇒ 必须被
    #    `MAX_JOINT_SWING` 拦下，否则这条测试证明不了护栏在起作用。
    #    `fold_yaw_180` 也要一起关掉：148.583° 折成 −31.417° 之后，腕根本不用
    #    绕远路，单独关 unwrap/travel_cost 已经复现不出事故（2026-10-27 实测）。
    orig = (pp.unwrap_near, pp.travel_cost, pp.fold_yaw_180)
    pp.unwrap_near = lambda ch, q, ref: np.asarray(q, float)
    pp.travel_cost = pp.joint_dist
    pp.fold_yaw_180 = lambda d: float(d)
    try:
        try:
            build()
        except pp.PlanError as exc:
            assert "整圈翻转" in str(exc), exc
        else:
            raise AssertionError("腕整圈翻转的计划居然生成出来了")
    finally:
        pp.unwrap_near, pp.travel_cost = orig[0], orig[1]
        pp.fold_yaw_180 = orig[2]

    # ② 正身
    wps = build()
    assert max(swings(wps)) <= pp.MAX_JOINT_SWING, \
        "最大关节行程 %.1f°（上限 180°）" % np.degrees(max(swings(wps)))
    i = chain.move_joints.index("wrist_2_joint")
    w2 = [np.degrees(w["q"][i]) for w in wps]
    assert max(w2) - min(w2) < 180.0, \
        "wrist_2 跨 %.1f°：%s" % (max(w2) - min(w2), [round(v, 1) for v in w2])
    # 插点从 18 个降到 ≤2 个：翻转那条路上原本每个航点都被切成 6 段，
    # 插点只是消化跳变，不是解法 —— 真跳变（to_slot 转 68.7°）仍该插。
    assert len(wps) - 8 <= 2, [w["name"] for w in wps]


@case
def test_grasp_yaw_folds_mod_180(chain):
    """抓取角必须折到 (−90°, +90°] —— 2026-10-27 round-06 块 #4 的实数。

    实数（`round-06/`）：几何 yaw 182.864°、块心 (462.38, 197.23)、块高
    31.21 mm、槽 (362.85, −54.66)、起点 q = [10.5, −82.0, 82.4, −90.3,
    −89.9, −79.6]°。不折角时腕 3 要先转到 +97.6° 抓（177.2°）、再转回
    −116.0° 放（213.6°）⇒ 被 `MAX_JOINT_SWING` 拦下、整轮只剩 1 块、
    标定判红「样本不足」。

    平行两指夹爪转 180° = 两指互换、抓的仍是同一对侧面（矩形块对 180°
    自对称），所以这是**物理等价**。折成 +2.864° 后同一块的最大单关节
    跳变 213.6° → 55.0°，不产生插点。
    """
    # 折算本身：等价写法互相折到同一值
    assert abs(pp.fold_yaw_180(182.864) - 2.864) < 1e-9
    assert abs(pp.fold_yaw_180(-177.136) - 2.864) < 1e-9
    assert abs(pp.fold_yaw_180(179.0) + 1.0) < 1e-9
    assert abs(pp.fold_yaw_180(-90.0) - 90.0) < 1e-9
    assert pp.fold_yaw_180(0.0) == 0.0

    block = {"idx": 4, "class": "orange", "cx": 462.377, "cy": 197.230,
             "top": 31.214, "h": 31.214}
    cand = {"score": 0.2286, "width_mm": 48.48, "depth": None, "dist_mm": 1.36,
            "angle_deg": 129.8, "seed": 1, "tilt_deg": 27.64,
            "xyz_base": [0.4611, 0.1977, 0.0302],
            "approach_base": [-0.359, -0.294, -0.886]}
    q_start = [np.radians(v) for v in (10.5, -82.0, 82.4, -90.3, -89.9, -79.6)]
    kw = dict(yaw_grasp_deg=182.864, yaw_place_deg=0.0,
              yaw_source="geometric", pick_open_mm=50.0, place_open_mm=50.0,
              grasp_at="block_center", release_dwell_s=0.5)
    slot = (0.36285, -0.05466)

    wps, _, meta = pp.build_pick(chain, q_start, cand, block, [block], slot, **kw)
    wps = pp.split_jumps(wps, chain)[0]
    assert abs(meta["yaw_grasp_deg"] - 2.864) < 1e-6, meta["yaw_grasp_deg"]
    assert abs(meta["yaw_grasp_raw_deg"] - 182.864) < 1e-6, \
        meta["yaw_grasp_raw_deg"]
    swings = [float(np.abs(np.asarray(b["q"]) - np.asarray(a["q"])).max())
              for a, b in zip([{"q": q_start}] + wps, wps)]
    assert max(swings) <= pp.MAX_JOINT_SWING, \
        "未折角：最大关节行程 %.1f°" % np.degrees(max(swings))
    assert len(wps) == 8, [w["name"] for w in wps]   # 不该再有插点

    # 变异：关掉折算 ⇒ 必须被护栏拦下（否则这条测试证明不了折算在起作用）
    orig = pp.fold_yaw_180
    pp.fold_yaw_180 = lambda d: float(d)
    try:
        try:
            wps0, _, _ = pp.build_pick(chain, q_start, cand, block, [block],
                                       slot, **kw)
            pp.split_jumps(wps0, chain)
        except pp.PlanError as exc:
            assert "整圈翻转" in str(exc), exc
        else:
            raise AssertionError("未折角的计划居然通过了")
    finally:
        pp.fold_yaw_180 = orig


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
def test_pick_two_stage_yaw(chain):
    """阶段 2：抓取段按块面 yaw、放置段按格线 yaw —— 两段各自落到目标上。

    两段 yaw 不同（30 / −15）才证明是真的逐航点给角，而不是一个全局值。
    """
    blocks, cand, q0 = scene(chain)
    wps, _, meta = pp.build_pick(chain, q0, cand, blocks[0], blocks, (0.45, -0.25),
                                 yaw_grasp_deg=30.0, yaw_place_deg=-15.0,
                                 yaw_source="face", pick_open_mm=50.0,
                                 place_open_mm=50.0)
    by = {w["name"]: w for w in wps}
    gx, gy = meta["grasp_point"]
    sx, sy = [v / 1000.0 for v in meta["place_target_mm"]]
    for n, xyz, yaw in (("approach", (gx, gy, meta["approach_z"]), 30.0),
                        ("grasp", (gx, gy, meta["grasp_z"]), 30.0),
                        ("lift", (gx, gy, meta["lift_z"]), 30.0),
                        ("to_slot", (sx, sy, meta["lift_z"]), -15.0),
                        ("place", (sx, sy, meta["place_z"]), -15.0),
                        ("retreat", (sx, sy, meta["place_approach_z"]), -15.0)):
        pe, re = pp.pose_residual(chain, by[n]["q"], xyz, yaw)
        assert pe < 0.1 and re < 0.05, \
            "%s 没落到 yaw %.0f°：%.3f mm / %.4f°" % (n, yaw, pe, re)
    assert meta["yaw_grasp_deg"] == 30.0 and meta["yaw_place_deg"] == -15.0
    assert meta["yaw_source"] == "face", meta["yaw_source"]
    assert meta["pick_open_mm"] == 50.0 and meta["place_open_mm"] == 50.0
    assert by["approach"]["before"] == {"gripper": "open_to", "mm": 50.0}, \
        by["approach"]["before"]
    assert by["place"]["after"] == {"gripper": "open_to", "mm": 50.0}, \
        by["place"]["after"]
    assert by["grasp"]["after"]["gripper"] == "close", "闭合不能变成半开"


@case
def test_mutation_yaw_ignored_is_caught(chain):
    """**判红的证明**：yaw=30° 的抓取点拿去按 0° 量，残差必须很大。

    这条挡住「参数接了但没传进 IK」—— 那种情况下上面那条会照样绿。
    """
    blocks, cand, q0 = scene(chain)
    wps, _, meta = pp.build_pick(chain, q0, cand, blocks[0], blocks, (0.45, -0.25),
                                 yaw_grasp_deg=30.0)
    q_grasp = {w["name"]: w for w in wps}["grasp"]["q"]
    gx, gy = meta["grasp_point"]
    pe, re = pp.pose_residual(chain, q_grasp, (gx, gy, meta["grasp_z"]), 0.0)
    assert re > 20.0, "yaw 30° 的抓取点按 0° 量只差 %.2f° —— 这条判据失灵了" % re
    # yaw_place 缺省 = yaw_grasp（A3 单角语义），不是 0。
    assert meta["yaw_place_deg"] == 30.0, meta["yaw_place_deg"]


@case
def test_exec_accepts_dict_gripper_actions():
    """计划里的夹爪动作全链：字符串与 ``{"gripper":"open_to","mm":..}`` 都认。"""
    import precision_exec as pe
    assert pe._action_name("close") == "close"
    assert pe._action_name({"gripper": "open_to", "mm": 50}) == "open_to"
    assert pe._action_str("readback") == "readback"
    assert pe._action_str({"gripper": "open_to", "mm": 50}) == "open_to(50.0)"
    assert pe._action_str({"gripper": "open_to"}) == "open_to"
    # 带速度的动作要在日志/控制台上分得出来（否则现场看不出这轮到底快还是慢）。
    assert pe._action_str({"gripper": "open_to", "mm": 50,
                           "speed": 25}) == "open_to(50.0) @25%"


@case
def test_grasp_at_block_center_uses_the_centroid(chain):
    """抓块心：抓取段三个航点落在**块心**上，不是候选锚点。

    2026-10-27 用户决议（松手弹滑四项处置之一）：块心更对称，合拢时两侧受力
    均匀，块在指间转得少。默认仍是 candidate（A3 逐字节不变）。
    """
    blocks, cand, q0 = scene(chain)
    blk = blocks[0]
    wps, _, meta = pp.build_pick(chain, q0, cand, blk, blocks, (0.45, -0.25),
                                 grasp_at="block_center")
    assert meta["grasp_at"] == "block_center", meta["grasp_at"]
    gx, gy = meta["grasp_point"]
    assert abs(gx - blk["cx"] / 1000.0) < 1e-9, meta["grasp_point"]
    assert abs(gy - blk["cy"] / 1000.0) < 1e-9, meta["grasp_point"]
    # 抓取却拿块心 ⇒ 偏心按定义是 0（这条同时证明 ex/ey 不是抄来的常数）
    assert meta["grasp_ecc_mm"] == [0.0, 0.0], meta["grasp_ecc_mm"]
    by = {w["name"]: w for w in wps}
    for n, z in (("approach", meta["approach_z"]), ("grasp", meta["grasp_z"]),
                 ("lift", meta["lift_z"])):
        pe, re = pp.pose_residual(chain, by[n]["q"], (gx, gy, z), 0.0)
        assert pe < 0.1 and re < 0.05, "%s 没落在块心：%.3f mm" % (n, pe)
    # 变异测试：默认仍是 candidate —— 块心不是默认值。
    _, _, m_def = pp.build_pick(chain, q0, cand, blk, blocks, (0.45, -0.25))
    assert m_def["grasp_at"] == "candidate", m_def["grasp_at"]
    assert abs(m_def["grasp_point"][0] - cand["xyz_base"][0]) < 1e-12
    assert abs(m_def["grasp_ecc_mm"][0] - (blk["cx"] - cand["xyz_base"][0] * 1000)) < 0.01


@case
def test_speed_and_dwell_land_on_the_motion_they_describe(chain):
    """速度写成**寄存器**要挂在正确的那一下动作上；dwell 只在 place 上。

    速度挂错地方不会报错、只会「看起来快了」——所以逐条断言哪个航点带它、
    哪个航点**不带**。默认三个参数都是 None/0，A3 计划必须逐字节不变。
    """
    blocks, cand, q0 = scene(chain)
    wps, _, meta = pp.build_pick(chain, q0, cand, blocks[0], blocks, (0.45, -0.25),
                                 place_open_mm=50.0, grip_speed_pct=40,
                                 release_speed_pct=25, release_dwell_s=0.5)
    by = {w["name"]: w for w in wps}
    assert by["grasp"]["after"] == {"gripper": "close", "readback": True,
                                    "speed": 40}, by["grasp"]["after"]
    assert by["place"]["after"] == {"gripper": "open_to", "mm": 50.0,
                                    "speed": 25}, by["place"]["after"]
    assert by["place"]["dwell_s"] == 0.5, by["place"]
    # 挂错的三种典型：合拢速度串到松手、松手速度串到接近、dwell 串到所有点。
    assert by["place"]["after"]["speed"] == 25, "松手用了合拢的速度"
    assert "speed" not in by["approach"]["before"], by["approach"]["before"]
    assert "speed" not in by["lift"]["after"], by["lift"]["after"]
    for w in wps:
        if w["name"] != "place":
            assert "dwell_s" not in w, "%s 不该有 dwell" % w["name"]
    assert meta["grip_speed_pct"] == 40 and meta["release_speed_pct"] == 25
    assert meta["release_dwell_s"] == 0.5

    # 不给 ⇒ 一个 speed / dwell 都不许出现（A3 兼容）
    wps2, _, m2 = pp.build_pick(chain, q0, cand, blocks[0], blocks, (0.45, -0.25),
                                place_open_mm=50.0)
    for w in wps2:
        assert "dwell_s" not in w, "%s 凭空多出 dwell" % w["name"]
        for key in ("before", "after"):
            if isinstance(w.get(key), dict):
                assert "speed" not in w[key], "%s.%s 凭空多出 speed" % (w["name"], key)
    assert m2["grip_speed_pct"] is None and m2["release_speed_pct"] is None
    assert m2["release_dwell_s"] == 0.0


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
def test_fold_yaw_90():
    """正方体四面对称：抓取角折到 (−45°, +45°]（90° 换面，矩形块不成立）。

    2026-11-01 入口⑤ 回位的真数：塔上那块实测 yaw 89.75°，只按 180° 折就留在
    89.75° ⇒ 腕 3 要么 229.6°、要么 −210.045°（超缠绕帽 0.045°），两边都是死。
    按 90° 折成 −0.25° 后，同一趟回位最大单跳 42.4°、腕 3 峰值 135.6°。
    """
    assert abs(pp.fold_yaw_90(89.75) - (-0.25)) < 1e-9
    assert abs(pp.fold_yaw_90(-0.25) - (-0.25)) < 1e-9
    assert abs(pp.fold_yaw_90(45.0) - 45.0) < 1e-9
    assert abs(pp.fold_yaw_90(45.01) - (45.01 - 90.0)) < 1e-9
    assert abs(pp.fold_yaw_90(100.0) - 10.0) < 1e-9
    assert abs(pp.fold_yaw_90(-100.0) - (-10.0)) < 1e-9
    assert pp.fold_yaw_90(0.0) == 0.0
    assert pp.fold_yaw_90(180.0) == 0.0
    # 折完必须落在半个象限内：整圈任意角都不许超 ±45°
    for deg in range(-360, 361, 7):
        v = pp.fold_yaw_90(deg)
        assert -45.0 < v <= 45.0, (deg, v)
    # 与 180° 折不是一回事：89.75° 在 180° 折下不动，90° 折才动
    assert abs(pp.fold_yaw_180(89.75) - 89.75) < 1e-9


@case
def test_split_jumps_checks_first_hop(chain):
    """起点 → 第 1 个航点这一跳也必须判 `MAX_JOINT_SWING`。

    2026-11-01 入口⑤ 第 6 次回位（台账第一百八十七轮 §7）：wrist_3 从 −79.6°
    直接要 +150.0°（229.6°），而相邻航点的检查要从第 2 个点才开始 —— 那一跳
    整条链上没人看。侥幸的是后面 to_slot 也超限，计划才被拦下。
    """
    i = chain.move_joints.index("wrist_3_joint")
    base = [0.5] * 6
    q0 = list(base)
    far = list(base)
    far[i] = base[i] + 4.0                       # 4.0 rad > π，字面整圈翻
    wps = [{"name": "approach", "q": far, "duration": 2.0}]
    # 不给 q_start ⇒ 旧行为：第一跳没人看（这就是那个洞本身）
    out, added = pp.split_jumps(wps, chain, max_jump=1.0)
    assert out == wps and added == 0
    # 给上 q_start ⇒ 必须拦，且要说清是「起点 → 第 1 个点」
    try:
        pp.split_jumps(wps, chain, max_jump=1.0, q_start=q0)
    except pp.PlanError as e:
        assert "起点 → 航点 approach" in str(e), e
    else:
        raise AssertionError("起点→首个航点转 %.0f°，护栏没拦"
                             % np.degrees(4.0))
    # 第一跳只判不插点：1.5 rad 超 max_jump 但没超 π ⇒ 一条都不插，
    # 既有计划（阶段 2 的产物）的 JSON 不因此改变
    near = list(base)
    near[i] = base[i] + 1.5
    wps2 = [{"name": "approach", "q": near, "duration": 2.0}]
    out2, added2 = pp.split_jumps(wps2, chain, max_jump=1.0, q_start=q0)
    assert added2 == 0 and out2 == wps2, (added2, out2)


@case
def test_split_jumps_checks_first_hop_interpolation(chain):
    """第一跳「只判不插点」不能顺手把**插值点**的奇异检查一起跳过。

    两头都远离奇异，直线中间照样能穿过腕 2 的 0°：−89° → +89° 只有 178°
    （未超 `MAX_JOINT_SWING`，不报整圈翻转），但 JTC 照字面走下去，中点就
    落在奇异上 —— 2026-11-01 修首跳时真出过这个漏网（台账第一百八十七轮）。
    """
    i = chain.move_joints.index("wrist_2_joint")
    base = [0.5] * 6
    q0 = list(base)
    q0[i] = np.radians(-89.0)
    cross = list(base)
    cross[i] = np.radians(89.0)
    wps = [{"name": "approach", "q": cross, "duration": 2.0}]
    try:
        pp.split_jumps(wps, chain, max_jump=1.0, q_start=q0)
    except pp.PlanError as e:
        assert "腕 2" in str(e), e
    else:
        raise AssertionError("首跳插值穿过腕 2 奇异，护栏没拦")
    # 反向对照：同一跳但中间不穿奇异 ⇒ 不许误伤
    ok = list(base)
    ok[i] = np.radians(-30.0)
    wps2 = [{"name": "approach", "q": ok, "duration": 2.0}]
    out2, added2 = pp.split_jumps(wps2, chain, max_jump=1.0, q_start=q0)
    assert added2 == 0 and out2 == wps2, (added2, out2)


@case
def test_finish_plan_passes_q_start_to_split_jumps(chain):
    """接线：`stage3_round._finish_plan` 必须把 q_start 交下去。

    否则 `split_jumps` 的第一跳检查形同虚设（默认 q_start=None）。
    """
    import stage3_round as s3r
    i = chain.move_joints.index("wrist_3_joint")
    q0 = [0.5] * 6
    far = [0.5] * 6
    far[i] = 0.5 + 4.0
    wps = [{"name": "approach", "q": far, "duration": 2.0}]
    checks = [{"name": "approach"}]
    try:
        s3r._finish_plan(wps, checks, {}, q0, chain)
    except pp.PlanError as e:
        assert "起点 → 航点 approach" in str(e), e
    else:
        raise AssertionError("_finish_plan 没把 q_start 交给 split_jumps")


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

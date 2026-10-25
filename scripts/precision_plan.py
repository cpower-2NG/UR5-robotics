#!/usr/bin/env python3
"""把「抓哪一块、放哪个槽」变成一份**可执行、且预检过**的关节计划（计划 JSON v2）。

这一层不碰硬件、不发轨迹：输入是候选 JSON + URDF + 当前关节角，输出是文件。
执行是 ``precision_exec.py`` 的事；两者分开，是因为**计划可以离线重放**
（今天 live-04 那一帧就能走完整条链），而执行不能重来。

为什么不走 MoveIt
-----------------
现在板上的栈是 ``start_moveit:=false``（见 ``docs/待执行命令.md`` 的启动参数）。
今天 A/B 两次真机抓取用的就是这套数值 IK + 两点式 JTC，一次到位、误差 <0.01°。
再起一套 move_group 只会多一个会漂的组件。

几何约定（**全部来自规划器，别在这里另立一套**）
------------------------------------------------
    tool0 指尖偏置        0.1757 m   (= 裸指尖 204.2 − A1 方块高 28.5)
                                    ⚠️ 这是**参数**不是尺寸：式子 wz + 0.1757 + c 里的
                                    「指尖」在顶面以下，夹持深度与块高无关。
                                    裸指尖（闭合）是 0.2042 m，报深度时用它。
    wz 用**相对台面的高度** `h`，不用绝对顶面 z —— 腕部深度实测把台面读高
    ~6 mm（本轮 table_z=+6.0），而几何锚在 table_plane_z=0。
    抓取抬升              0.01 m
    接近余量              0.05 m
    搬运高度              0.15 m
    放置面 z              0.0285 m
    放置释放余量          0.01 m   ⇒ 放置下降 z = 0.2142

姿态固定为「竖直朝下」rpy=(180°,0,ψ)，ψ 默认 0：AG-95 的手指轴相对 tool0
的方向没有文档也没实测过，本轮不跟 GraspNet 的平面内角（它只记录在 meta 里）。
A/B 把腕转 180° 仍然抓成功，说明当前方块尺度下朝向不敏感。

预检（任一不过就**不写计划**，退出码 2）
----------------------------------------
1. IK 残差 < 0.1 mm / 0.05°
2. 软限位（URDF 的 safety_controller，不是硬限位）
3. 高度护栏：肘 / 腕 1 与**指尖点**（tool0 + R·[0,0,0.1757]）的 z ≥ 台面 + 20 mm，
   关节空间逐段采样 8 点。指尖单独查，是因为夹着块横移时先碰台面的是它，
   而肘、腕 1 都还在高处 —— 只查连杆等于没查。
4. 段内最大关节跳变 ≤ 1.0 rad，超过就插中间航点
5. 腕 2 离 0 至少 0.15 rad（0 是腕奇异；插点也要查 —— 端点远离不等于插值点远离）
6. 高度钳制：抓取 z ∈ [0.2142, 0.2242]（不低于已验证的放置深度）
7. 邻块间距 ≥ 60 mm（夹爪全开 95 mm；A/B 实测 77 mm 不碰）

用法：
  # 走到抓取巡检位 / 放置巡检位
  python3 precision_plan.py --survey pick  --urdf robot.urdf --from-frame f0 --out p.json
  # 抓一块、放到某个槽
  python3 precision_plan.py --pick --candidates cand.json --block 2 \\
      --slot 0.45,-0.25 --urdf robot.urdf --from-frame f0 --out pick.json
  # 到任意 tool0 位置
  python3 precision_plan.py --move 0.53,0.27,0.36 --urdf robot.urdf --from-frame f0 --out m.json
"""
import argparse
import json
import os
import sys

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np                                                # noqa: E402
from scipy.spatial.transform import Rotation as Rot               # noqa: E402

import urkin                                                      # noqa: E402

# ---- 几何常量（出处见模块 docstring）
# `TOOL0_TO_FINGERTIP` 这个**参数** = 裸指尖（闭合，tool0→指尖 204.2 mm）减去
# A1 那批方块的高度 28.5 mm。所以 `wz + TOOL0_TO_FINGERTIP + c` 这个式子里的
# 「指尖」落在**顶面以下 (204.2 − 28.5 − ...) **，夹持深度与块高无关。
# 要报「指尖在顶面下多深」时必须用**裸指尖**这个数，不能用参数。
TOOL0_TO_BARE_FINGERTIP = 0.2042
TOOL0_TO_FINGERTIP = 0.1757
APPROACH_CLEARANCE = 0.05
GRASP_CLEARANCE = 0.01
LIFT_HEIGHT = 0.15
PLACE_Z = 0.0285
PLACE_RELEASE_LIFT = 0.01
PLACE_DESCEND_Z = PLACE_Z + TOOL0_TO_FINGERTIP + PLACE_RELEASE_LIFT   # 0.2142
PLACE_APPROACH_Z = PLACE_Z + TOOL0_TO_FINGERTIP + APPROACH_CLEARANCE  # 0.2542
GRASP_Z_MIN = PLACE_DESCEND_Z          # 别比已验证过的放置深度更低
GRASP_Z_MAX = GRASP_Z_MIN + 0.010      # 上限给 10 mm：块高 30~35 mm，实测够用
# 腕 2（第 5 关节）离 0 至少这么远。0 就是腕奇异（1、3 轴重合），
# A/B 两次实测腕 2 = −89.8°，余量极大；这个数只是防止 IK 挑到另一支解。
WRIST_MIN_RAD = 0.15

# 巡检位（tool0，m）。相机相对 tool0 的偏置是 (−31.9, +85.3, −24.2) mm，
# 由 live-04 的 URDF FK 与帧内 TF 逐位对上（差 0.0 mm）。
# 这两个位置各解一次 IK，残差 0.000 mm / 0.000°。
SURVEY_TOOL0 = {
    "pick": (0.5319, 0.2747, 0.3643),     # 相机 ~(500, 360, 340) mm，三块都在画面中部
    "place": (0.5319, -0.3853, 0.3643),   # 相机 ~(500, -300, 340) mm，四个槽都在画面内
}

GUARD_LINKS = ("forearm_link", "wrist_1_link")
GUARD_MARGIN = 0.02
MAX_JOINT_JUMP = 1.0
GUARD_SAMPLES = 8
TABLE_PLANE_Z = 0.0
MIN_BLOCK_GAP = 0.060


class PlanError(RuntimeError):
    """预检不过。抛这个而不是 sys.exit，测试里能直接断言。"""


def down_rotation(yaw_deg=0.0):
    """竖直朝下：R = Rz(ψ) · Rx(π)。"""
    return Rot.from_euler("z", yaw_deg, degrees=True).as_matrix() @ \
        Rot.from_euler("x", 180.0, degrees=True).as_matrix()


def target_matrix(xyz, yaw_deg=0.0):
    T = np.eye(4)
    T[:3, :3] = down_rotation(yaw_deg)
    T[:3, 3] = xyz
    return T


def duration_for(q0, q1, seconds_per_rad=3.0, base=1.5, lo=2.0, hi=12.0):
    """运动时长：按最大关节角走 3 s/rad，再加 1.5 s 底 —— 宁可慢。"""
    d = float(np.max(np.abs(np.asarray(q1) - np.asarray(q0))))
    return float(min(hi, max(lo, base + seconds_per_rad * d)))


def check_wrist(chain, q, tol=WRIST_MIN_RAD):
    """腕 2（第 5 关节）过零 ⇒ 腕奇异：第 1 与第 3 关节轴重合，关节速度发散。
    JTC 为了保持末端走直线会把腕甩起来，夹着块横移时尤其危险。
    返回 None 或 (实际角度, 下限角度)，单位都是度。
    """
    try:
        i = chain.move_joints.index("wrist_2_joint")
    except (ValueError, AttributeError):
        i = 4
    v = float(q[i])
    if abs(v) >= tol:
        return None
    return round(float(np.degrees(v)), 2), round(float(np.degrees(tol)), 2)


def check_limits(chain, q):
    """关节角按**原值**判软限位。

    ⚠️ 不要先折算到 ±π 再判：UR 报的关节角不绕圈（可以到 6.2 rad），
    折算会把一个越界的 6.5 rad 变成 −0.78 rad 放过去。软限位本身就在 ±6.133，
    所以原值直接比才对。
    """
    bad = []
    for name, val in zip(chain.move_joints, q):
        lo, hi, _ = chain.limits.get(name, (-np.inf, np.inf, 0.0))
        if not (lo <= float(val) <= hi):
            bad.append((name, round(float(np.degrees(val)), 2)))
    return bad


def check_guard(chain, q0, q1, table_z=TABLE_PLANE_Z, margin=GUARD_MARGIN,
                links=GUARD_LINKS, samples=GUARD_SAMPLES):
    """关节空间直线上逐点 FK，检查两件事：

    · 肘 / 腕 1 这些**大件**离台面的高度（擦台的典型是它们，不是指尖）；
    · **指尖点**的高度 —— 指尖 = tool0 + R·[0,0,0.1757]（不是简单往下减，
      姿态不是竖直朝下时要按 R 算）。夹着块横移时真正会碰到台面/别的块的
      是它，只查连杆查不出来。

    返回 (ok, worst)，worst = (t, 名字, z)。
    """
    q0 = np.asarray(q0, float)
    q1 = np.asarray(q1, float)
    worst = None
    for i in range(samples + 1):
        q = q0 + (q1 - q0) * (i / samples)
        qd = dict(zip(chain.move_joints, q))
        for link in links:
            z = float(chain.fk_link(link, qd)[2, 3])
            if worst is None or z < worst[2]:
                worst = (i / samples, link, z)
        T = chain.fk(qd)
        tip_z = float(T[2, 3] + T[:3, 2] @ np.array([0.0, 0.0, TOOL0_TO_FINGERTIP]))
        if worst is None or tip_z < worst[2]:
            worst = (i / samples, "fingertip", tip_z)
    if worst is not None and worst[2] < table_z + margin:
        return False, worst
    return True, worst


def solve_waypoint(chain, xyz, yaw_deg, seed, max_iter=200):
    T = target_matrix(xyz, yaw_deg)
    q, pe, re, nfev = chain.ik(T, seed, max_iter=max_iter)
    return q, pe, re, nfev


def add_move(waypoints, chain, name, xyz, q_seed, yaw_deg, table_z,
             duration=None, before=None, after=None):
    """解一个航点并跑单点预检，追加到 waypoints。返回该点的 q。"""
    q, pe, re, nfev = solve_waypoint(chain, xyz, yaw_deg, q_seed)
    if pe > 0.1 or re > 0.05:
        raise PlanError("航点 %s IK 没解出来：位置残差 %.3f mm、姿态残差 %.4f° "
                        "(xyz=%s)" % (name, pe, re, xyz))
    bad = check_limits(chain, q)
    if bad:
        raise PlanError("航点 %s 超出软限位：%s" % (name, bad))
    ws = check_wrist(chain, q)
    if ws:
        raise PlanError("航点 %s 的腕 2 只有 %.2f°，离奇异位形不足 %.2f°（下限）"
                        % (name, ws[0], ws[1]))
    ok, worst = check_guard(chain, q_seed, q, table_z)
    if not ok:
        raise PlanError("航点 %s 的段内高度护栏不过：t=%.2f 处 %s z=%.1f mm"
                        "（下限 %.1f mm）"
                        % (name, worst[0], worst[1], worst[2] * 1000,
                           (table_z + GUARD_MARGIN) * 1000))
    wp = {"name": name, "q": [float(v) for v in q],
          "duration": float(duration or duration_for(q_seed, q))}
    if before:
        wp["before"] = before
    if after:
        wp["after"] = after
    waypoints.append(wp)
    return q, {"name": name, "ik_pos_mm": round(float(pe), 4),
               "ik_rot_deg": round(float(re), 5), "nfev": int(nfev),
               "elbow_min_z_mm": round(worst[2] * 1000, 1) if worst else None}


def split_jumps(waypoints, chain, max_jump=MAX_JOINT_JUMP):
    """相邻航点间关节跳变超过 max_jump 时，在关节空间插中间点。

    注意：JTC 本来就是在关节空间做直线插值，所以插点**不改变轨迹形状** ——
    它改变的是「这一条指令有多大」，便于日志与失败定位，也让 max_joint_jump
    这条约定在执行器的逐航点闸门里仍然有意义。
    """
    out = []
    added = 0
    prev_q = None
    for w in waypoints:
        q = np.asarray(w["q"], float)
        if prev_q is not None:
            jump = float(np.max(np.abs(q - prev_q)))
            if jump > max_jump:
                n = int(np.ceil(jump / max_jump))
                for i in range(1, n):
                    qi = prev_q + (q - prev_q) * (i / n)
                    # 端点都远离奇异，**插值点**不一定：q5 一过零就落在奇异上
                    ws = check_wrist(chain, qi)
                    if ws:
                        raise PlanError("插点 %s_split%d 的腕 2 只有 %.2f°，"
                                        "离奇异位形不足 %.2f°（下限）"
                                        % (w["name"], i, ws[0], ws[1]))
                    out.append({"name": "%s_split%d" % (w["name"], i),
                                "q": [float(v) for v in qi],
                                "duration": float(duration_for(prev_q, qi))})
                    added += 1
        out.append(w)
        prev_q = q
    return out, added


def check_block_gap(block, blocks, min_gap=MIN_BLOCK_GAP):
    """抓一块时，最近的邻块中心距 —— 夹爪全开时手指要落得下。"""
    others = [b for b in blocks if b["idx"] != block["idx"]]
    if not others:
        return None
    return min(float(np.hypot(b["cx"] - block["cx"], b["cy"] - block["cy"]))
               for b in others) / 1000.0


# ---------------------------------------------------------------- 计划的三种形态
def build_survey(chain, q_start, which):
    xyz = SURVEY_TOOL0[which]
    wps, checks = [], []
    q, c = add_move(wps, chain, "survey_" + which, xyz, q_start, 0.0,
                    TABLE_PLANE_Z)
    checks.append(c)
    return wps, checks, {"kind": "survey", "which": which, "tool0": list(xyz)}


def build_move(chain, q_start, xyz, name="move"):
    wps, checks = [], []
    q, c = add_move(wps, chain, name, xyz, q_start, 0.0, TABLE_PLANE_Z)
    checks.append(c)
    return wps, checks, {"kind": "move", "tool0": list(xyz)}


def build_pick(chain, q_start, cand, block, blocks, slot):
    """完整拾放循环：approach → grasp(闭合) → lift → to_slot → place_approach
    → place(张开) → retreat → survey_place。"""
    gx, gy = cand["xyz_base"][0], cand["xyz_base"][1]
    # 用**相对台面的高度** `h`，不用绝对顶面 z：
    # 腕部深度实测把台面读高 ~6 mm（本轮帧 table_z=+6.0 mm，真块高 = top − table_z
    # = 24.3~26.0 mm），而本规划器的整个几何都锚在 table_plane_z=0（放置高度、
    # PLACE_DESCEND_Z 都按台面建）。拿绝对 top 当 wz 会把那 6 mm 原样带进抓取高度，
    # 指尖就少抓 6 mm —— 正好是用户说的「抓得更浅」那一类误差。
    wz = (block["h"] if block.get("h") is not None else block["top"]) / 1000.0
    approach_z = wz + TOOL0_TO_FINGERTIP + APPROACH_CLEARANCE
    grasp_z = float(np.clip(wz + TOOL0_TO_FINGERTIP + GRASP_CLEARANCE,
                            GRASP_Z_MIN, GRASP_Z_MAX))
    lift_z = wz + TOOL0_TO_FINGERTIP + LIFT_HEIGHT
    sx, sy = slot

    gap = check_block_gap(block, blocks)
    if gap is not None and gap < MIN_BLOCK_GAP:
        raise PlanError("块 #%d 离最近邻块只有 %.1f mm（下限 %.0f mm）—— "
                        "夹爪全开 95 mm 会碰邻块" % (block["idx"], gap * 1000,
                                                   MIN_BLOCK_GAP * 1000))
    if cand["dist_mm"] > 12.0:
        raise PlanError("候选离块心 %.1f mm，超过 12 mm 锚定半径" % cand["dist_mm"])

    wps, checks = [], []
    q = q_start
    for name, xyz, after, before in [
        ("approach", (gx, gy, approach_z), None,
         {"gripper": "open", "readback": True}),
        ("grasp", (gx, gy, grasp_z), {"gripper": "close", "readback": True}, None),
        ("lift", (gx, gy, lift_z), {"gripper": "readback"}, None),
        ("to_slot", (sx, sy, lift_z), None, None),
        ("place_approach", (sx, sy, PLACE_APPROACH_Z), None, None),
        ("place", (sx, sy, PLACE_DESCEND_Z), {"gripper": "open", "readback": True}, None),
        ("retreat", (sx, sy, PLACE_APPROACH_Z), None, None),
        ("survey_place", SURVEY_TOOL0["place"], None, None),
    ]:
        q, c = add_move(wps, chain, name, xyz, q, 0.0, TABLE_PLANE_Z,
                        after=after, before=before)
        checks.append(c)

    meta = {"kind": "pick", "block_idx": block["idx"], "class": block["class"],
            "block_top_mm": round(block["top"], 2),
            "block_h_mm": round(wz * 1000, 2),
            "wz_measured_mm": round(block["top"], 2),
            "wz_used_mm": round(wz * 1000, 2),
            "table_z_mm": round(block["top"] - wz * 1000, 2),
            "grasp_depth_mm": round(wz * 1000 - (grasp_z * 1000
                                                 - TOOL0_TO_BARE_FINGERTIP * 1000), 2),
            "block_xy_mm": [round(block["cx"], 2), round(block["cy"], 2)],
            "grasp_point": [gx, gy], "grasp_z": round(grasp_z, 4),
            "approach_z": round(approach_z, 4), "lift_z": round(lift_z, 4),
            "cand_score": round(cand["score"], 4),
            "cand_width_mm": round(cand["width_mm"], 2),
            "cand_dist_mm": round(cand["dist_mm"], 2),
            "cand_angle_deg": round(cand["angle_deg"], 2),
            "cand_seed": cand.get("seed"),
            "slot": list(slot), "block_gap_mm": round(gap * 1000, 1) if gap else None}
    return wps, checks, meta


# ---------------------------------------------------------------- CLI
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--from-frame", default=None,
                    help="从帧目录的 joint_states.json 取 q_start")
    ap.add_argument("--q-start", default=None, help="逗号分隔的 6 个关节角（弧度）")
    ap.add_argument("--out", required=True)
    grp = ap.add_mutually_exclusive_group(required=True)
    grp.add_argument("--survey", choices=["pick", "place"])
    grp.add_argument("--move", help="tool0 目标 x,y,z（米）")
    grp.add_argument("--pick", action="store_true")
    ap.add_argument("--candidates", default=None)
    ap.add_argument("--block", type=int, default=None)
    ap.add_argument("--slot", default=None, help="x,y（米）")
    args = ap.parse_args()

    if args.from_frame:
        js = json.load(open(os.path.join(args.from_frame, "joint_states.json")))
        chain = urkin.Chain(args.urdf)
        q_start = [float(js["position"][js["name"].index(n)]) for n in chain.move_joints]
    elif args.q_start:
        chain = urkin.Chain(args.urdf)
        q_start = [float(v) for v in args.q_start.split(",")]
    else:
        ap.error("要给 --from-frame 或 --q-start")

    try:
        if args.survey:
            wps, checks, meta = build_survey(chain, q_start, args.survey)
        elif args.move:
            xyz = tuple(float(v) for v in args.move.split(","))
            wps, checks, meta = build_move(chain, q_start, xyz)
        else:
            if not (args.candidates and args.slot) or args.block is None:
                ap.error("--pick 要同时给 --candidates / --block / --slot")
            cand = json.load(open(args.candidates))
            per = [p for p in cand["per_block"] if p["block_idx"] == args.block]
            if not per or not per[0]["best"]:
                raise PlanError("块 #%d 没有 ≤%.0f mm 的候选"
                                % (args.block, cand["anchor_mm"]))
            block = next(b for b in cand["blocks"] if b["idx"] == args.block)
            slot = tuple(float(v) for v in args.slot.split(","))
            wps, checks, meta = build_pick(chain, q_start, per[0]["best"],
                                           block, cand["blocks"], slot)
        n_before_split = len(wps)
        wps, added = split_jumps(wps, chain)
        # 每个**原始**航点都必须有一条预检记录（插出来的点继承相邻两点的检查）。
        if len(checks) != n_before_split:
            raise PlanError("预检记录不完整：%d 条航点只有 %d 条记录"
                            % (n_before_split, len(checks)))
    except PlanError as e:
        print("!! 预检不过：%s" % e, file=sys.stderr)
        print("   计划**没有**写出 —— 不要执行。", file=sys.stderr)
        return 2

    plan = {"version": 2, "meta": dict(meta, checks=checks, splits_added=added,
                                       q_start_deg=[round(float(np.degrees(v)), 2)
                                                    for v in q_start]),
            "q_start": [float(v) for v in q_start], "waypoints": wps}
    json.dump(plan, open(args.out, "w"), ensure_ascii=False, indent=1)

    print("计划 → %s（%d 条航点，插点 %d）" % (args.out, len(wps), added))
    print("  %-14s %-38s %6s %9s %9s" % ("航点", "q (deg)", "时长s", "IK mm", "肘高mm"))
    by_name = {c["name"]: c for c in checks}
    for w in wps:
        c = by_name.get(w["name"], {})
        print("  %-14s %-38s %6.1f %9.3f %9.1f"
              % (w["name"],
                 "[" + " ".join("%6.1f" % np.degrees(v) for v in w["q"]) + "]",
                 w["duration"], c.get("ik_pos_mm", float("nan")),
                 c.get("elbow_min_z_mm") or float("nan")))
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""E9 腕部巡检位：让**腕部相机**正对拾取区中心，并把 tool0 摆到对应位置。

为什么要这个工具
----------------
2026-11-09 用户定：**规划感知走腕部几何链**，俯视相机只留作跑后判分器
（``--verify --colour-burst``）。俯视的绝对 xy 有 20–45 mm 系统偏差，只能当
视场引导；腕部链由手眼外参定。

腕部相机光轴与 tool0 +z 平行，但本体横偏 91 mm（tool0 系
``(−31.92, −85.32, +24.26) mm``，见 ``runs/2026-11-07-stage4-entry/e4a-b/
bcase-plan.json``）。所以**不能把 tool0 停在拾取区中心** —— 要按手眼把它推到
「相机落在中心上方」：

    cam_base = tool0 + Rz(ψ)·Rx(180°)·t_tc

``ψ=0``、相机高 500 mm、相机在拾取区中心 ``(0.4125, 0.2950)`` ⇒
tool0 = ``(0.4444, 0.2097, 0.5243) m``。

覆盖核验（``--check``，offline）：契约拾取区 0.325×0.250 m 的八个角点
（z=0 与 z=60 mm）四边余量 ≥100 px；2026-11-09 实算 s5 三根塔投影到
u/v = 744/426、908/611、939/296，全部在 1280×720 画幅内。

为什么不用既有的 ``E9_SURVEY_TOOL0=(0.520, 0.300, 0.400)``：它不是感知位，
实算拾取区左/下角落在画幅外，s5 的黄塔投到 v=968（画幅高 720）。

用法（开发机；只出计划，不动臂）
--------------------------------
    python3 scripts/stage4_wrist_survey.py \\
        --urdf ~/robotics/notes/ur5-vision-grasp/runs/2026-10-25-precision/robot.urdf \\
        --from-frame <live joints 所在目录> \\
        --out runs/<run>/e9/survey-plan.json

退出码：0 = 计划写出且预检通过；2 = 预检不过/输入缺失（不写计划）。
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

import precision_plan as pp                                       # noqa: E402
import urkin                                                      # noqa: E402

#: 手眼：相机光心在 tool0 系的位置（米）。出处见模块 docstring。
T_TOOL0_CAM_M = np.array([-0.031924, -0.085321, 0.024256])

#: 契约拾取区（base 系，米）：x0, x1, y0, y1。
PICK_ZONE_M = (0.25, 0.575, 0.17, 0.42)

#: 巡检位相机高度（台面上方，米）。可用 ``--height`` 覆盖。
SURVEY_H_M = 0.500

#: D435i 彩色流 1280×720 的内参（2026-11-07 两次独立 burst 的 meta 逐位一致）。
K_1280x720 = (908.152, 908.375, 639.435, 363.314)
SIZE = (1280, 720)


def survey_tool0(zone=PICK_ZONE_M, height=SURVEY_H_M, yaw_deg=0.0):
    """拾取区中心上方 ``height`` 处的相机 → 对应的 tool0 位置（米）。"""
    R = pp.down_rotation(yaw_deg)
    cam = np.array([0.5 * (zone[0] + zone[1]), 0.5 * (zone[2] + zone[3]),
                    float(height)])
    return cam - R @ T_TOOL0_CAM_M, cam, R


def coverage_pose(zone, cam, R, block_h_m=0.060):
    """任意腕部位姿（相机光心 + 朝向）下，区域八角的四边余量（像素）。

    这个函数是 ``coverage`` 的泛化：偏移/倾斜视点（V1 同区多机位、正式轮
    多视点观察）不能再用「区域中心正上方的竖直朝下」那一条公式核验。
    ``R`` 是**相机**在 base 系的朝向；optical 系与 tool0 系只差 0.57°
    （R_tc 非对角元 ≤0.0097）⇒ 两者可互换。
    """
    fx, fy, cx, cy = K_1280x720
    w, h = SIZE
    us, vs = [], []
    for x in (zone[0], zone[1]):
        for y in (zone[2], zone[3]):
            for z in (0.0, block_h_m):
                pc = R.T @ (np.array([x, y, z]) - np.asarray(cam, float))
                us.append(fx * pc[0] / pc[2] + cx)
                vs.append(fy * pc[1] / pc[2] + cy)
    return (min(us), w - max(us), min(vs), h - max(vs))


def coverage(zone=PICK_ZONE_M, height=SURVEY_H_M, yaw_deg=0.0,
             block_h_m=0.060):
    """拾取区八个角点投到腕部画面的四边余量（像素）；负 = 出画幅。"""
    tool0, cam, R = survey_tool0(zone, height, yaw_deg)
    return coverage_pose(zone, cam, R, block_h_m), tool0, cam


def project_block(cam, R, block_xy, z_top):
    fx, fy, cx, cy = K_1280x720
    pc = (R).T @ (np.array([block_xy[0], block_xy[1], z_top]) - cam)
    return fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy


def survey_plan(chain, q_start, zone=PICK_ZONE_M, height=SURVEY_H_M,
                yaw_deg=0.0, transit_dz=0.10, qs=None):
    """从 ``q_start`` 平动到巡检位，产出可直接执行的计划 dict。

    返回 ``(plan, checks, cam, margins)``。预检不过抛 ``pp.PlanError``
    （调用方据此整份不写、不执行，fail-closed）。

    这是**唯一**构造巡检位计划的地方：E9 的 ``run.sh`` 收尾段（跑后判分帧）
    和手动出计划都走它，两处的位姿因此逐位一致 —— 跑前跑后必须是**同一个
    位姿**，否则两帧真值的块位置不能直接比。

    ``qs``：已经解好的两点关节角 ``[transit, pose]``（来自跨机位的全局 DP，
    见 ``stage4_clear_run.plan_views``）。给定时不再逐点局部 IK —— 一整串
    机位必须共用一条分支链，逐点就近会换支并逼出 180° 以上的单关节翻转。
    """
    marg, tool0, cam = coverage(zone, height, yaw_deg)
    if min(marg) <= 0:
        raise pp.PlanError("巡检位有角点出画幅（四边余量 %s px）"
                           % (tuple(int(round(m)) for m in marg),))
    wps, checks, meta = pp.build_move(
        chain, q_start,
        tuple(float(v) for v in tool0), name="survey",
        transit_dz=float(transit_dz), yaw_deg=float(yaw_deg), qs=qs)
    wps, added = pp.split_jumps(wps, chain, q_start=q_start)
    if len(checks) != len(wps) - added:
        raise pp.PlanError("巡检位预检记录不完整")
    plan = {"version": 2,
            "meta": dict(meta, checks=checks, splits_added=added,
                         q_start_deg=[round(float(np.degrees(v)), 2)
                                      for v in q_start],
                         purpose="e9-wrist-survey",
                         survey_cam_base_m=[round(float(v), 6) for v in cam],
                         coverage_px=[round(float(v), 1) for v in marg]),
            "q_start": [float(v) for v in q_start], "waypoints": wps}
    return plan, checks, cam, marg


def main(argv=None):
    ap = argparse.ArgumentParser(description="E9 腕部巡检位 → 平动计划")
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--out", default=None,
                    help="计划写到哪（不给就只算 --check，不写文件）")
    ap.add_argument("--from-frame", default=None,
                    help="含 joint_states.json 的目录（= q_start 起点）")
    ap.add_argument("--q-start", default=None, help="逗号分隔 6 个关节角（弧度）")
    ap.add_argument("--height", type=float, default=SURVEY_H_M,
                    help="相机离台面高度（米），默认 0.500")
    ap.add_argument("--yaw", type=float, default=0.0, help="tool0 的 ψ（度）")
    ap.add_argument("--zone", default=None, metavar="X0,X1,Y0,Y1",
                    help="要用哪块区域居中，默认契约拾取区")
    ap.add_argument("--transit-dz", type=float, default=0.10,
                    help="先在同一 xy 上方 dz 停一航点再下去（米，默认 0.10）")
    ap.add_argument("--check", default=None, metavar="X,Y,...",
                    help="额外核验这些块心（逗号分隔的 x,y 对，米）是否在画幅内")
    args = ap.parse_args(argv)

    zone = PICK_ZONE_M
    if args.zone:
        vals = [float(v) for v in args.zone.split(",")]
        if len(vals) != 4:
            print("!! --zone 要 X0,X1,Y0,Y1", file=sys.stderr)
            return 2
        zone = tuple(vals)

    marg, tool0, cam = coverage(zone, args.height, args.yaw)
    print("巡检位：相机 base=(%.4f, %.4f, %.4f) m（离台面 %.0f mm）"
          % (cam[0], cam[1], cam[2], cam[2] * 1000))
    print("        tool0 base=(%.4f, %.4f, %.4f) m  ψ=%.1f°"
          % (tool0[0], tool0[1], tool0[2], args.yaw))
    print("        区域四边余量(左,右,上,下) = %s px"
          % (tuple(int(round(m)) for m in marg),))
    if min(marg) <= 0:
        print("!! 区域有角点出画幅 —— 换 --height / --yaw，不要带着盲区跑",
              file=sys.stderr)
        return 2

    if args.check:
        R = pp.down_rotation(args.yaw)
        vals = [float(v) for v in args.check.split(",")]
        for i in range(0, len(vals) - 1, 2):
            u, v = project_block(cam, R, (vals[i], vals[i + 1]), 0.060)
            ok = 0 <= u < SIZE[0] and 0 <= v < SIZE[1]
            print("        块 (%.3f, %.3f) -> u=%7.1f v=%7.1f %s"
                  % (vals[i], vals[i + 1], u, v, "★" if ok else "画幅外"))
            if not ok:
                print("!! 有块出画幅", file=sys.stderr)
                return 2

    if not args.out:
        return 0

    chain = urkin.Chain(args.urdf)
    if args.from_frame:
        js = json.load(open(os.path.join(args.from_frame, "joint_states.json")))
        q_start = [float(js["position"][js["name"].index(n)])
                   for n in chain.move_joints]
    elif args.q_start:
        q_start = [float(v) for v in args.q_start.split(",")]
    else:
        print("!! 要给 --from-frame 或 --q-start", file=sys.stderr)
        return 2

    try:
        plan, checks, cam, marg = survey_plan(
            chain, q_start, zone=zone, height=args.height,
            yaw_deg=args.yaw, transit_dz=args.transit_dz)
    except pp.PlanError as e:
        print("!! 预检不过：%s" % e, file=sys.stderr)
        print("   计划**没有**写出 —— 不要执行。", file=sys.stderr)
        return 2

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(plan, f, ensure_ascii=False, indent=1)
    print("计划 → %s（%d 条航点，插点 %d）"
          % (args.out, len(plan["waypoints"]), plan["meta"]["splits_added"]))
    for c in checks:
        print("  %-14s IK %7.3f mm %7.3f°  肘高 %s mm"
              % (c["name"], c["ik_pos_mm"], c["ik_rot_deg"],
                 c.get("elbow_min_z_mm", "?")))
    return 0


if __name__ == "__main__":
    sys.exit(main())

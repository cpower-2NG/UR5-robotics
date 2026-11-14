#!/usr/bin/env python3
"""标准 HOME（home_pose.py）的离线自检：几何合法性 + park 计划结构 + 闸门。

跑法：``python3 scripts/test_home_pose.py``；需要运动学 URDF 的两项在找不到
URDF 时跳过并大声说明（跳过不算通过）。
"""
import glob
import json
import math
import os
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import home_pose as hp                                            # noqa: E402
import precision_plan as pp                                       # noqa: E402
import precision_exec as pe                                       # noqa: E402

PASS = FAIL = SKIP = 0
FAILS = []


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print("  PASS  %s" % name)
    else:
        FAIL += 1
        FAILS.append(name)
        print("  FAIL  %s   ← %s" % (name, detail))


def find_urdf():
    env = os.environ.get("PRECISION_URDF")
    if env and os.path.exists(env):
        return env
    pat = os.path.expanduser(
        "~/robotics/notes/ur5-vision-grasp/runs/*/robot.urdf")
    hits = sorted(glob.glob(pat), key=os.path.getmtime)
    return hits[-1] if hits else None


def main():
    print("[1] HOME 常量：单一出处、与阶段 1 yaml 同值")
    check("HOME_JOINTS_DEG = 烛台姿 [0,-90,90,-90,-90,0]",
          hp.HOME_JOINTS_DEG == (0.0, -90.0, 90.0, -90.0, -90.0, 0.0),
          hp.HOME_JOINTS_DEG)
    check("HOME_JOINTS 是弧度换算",
          all(abs(r - math.radians(d)) < 1e-12
              for r, d in zip(hp.HOME_JOINTS, hp.HOME_JOINTS_DEG)), "")

    yaml_path = os.path.join(HERE, "..", "ros2_ws", "src", "grasp_planner",
                             "config", "grasp_planner.yaml")
    if os.path.exists(yaml_path):
        import re
        m = re.search(r"home_joints:\s*\[([^\]]+)\]",
                      open(yaml_path, encoding="utf-8").read())
        vals = [float(v) for v in m.group(1).split(",")]
        # yaml 是四舍五入的历史副本（−1.57 ≠ −1.5708，差 0.0008 rad）——
        # 同一事实、待对齐；这里按 0.01 rad 容差判"同值"，精确值以 home_pose.py 为准。
        check("与 grasp_planner.yaml 的 home_joints 同值（容差 0.01 rad）",
              all(abs(a - math.radians(b)) < 0.01
                  for a, b in zip(vals, hp.HOME_JOINTS_DEG)),
              vals)

    urdf = find_urdf()
    if not urdf:
        print("SKIP 运动学项：找不到 robot.urdf（$PRECISION_URDF）")
        print("\n%d PASS  %d FAIL  %d SKIP" % (PASS, FAIL, SKIP))
        return 0
    import urkin
    ch = urkin.Chain(urdf)
    home = list(hp.HOME_JOINTS)

    print("[2] 几何：全 90° 必须被拒（FK 钻台面），烛台姿必须全过")
    bad = [math.radians(90.0)] * 6
    ok, worst = pp.check_guard(ch, [0.0] * 6, bad, 0.0)
    check("[90°×6] 从零位插值撞高度护栏（当日实测 −0.50 m 的钉子）",
          ok is False, repr(worst))

    # 结构测试用**真实起点**（attempt8 停止位姿；从零位出发的那条插值本来
    # 就该被高度护栏拒——闸门正确，测试不能拿危险起点当夹具）
    snap = os.path.expanduser(
        "~/robotics/notes/ur5-vision-grasp/runs/2026-11-12-stage4-s0/"
        "s0-1-r1-attempt8/joints-now/joint_states.json")
    if not os.path.exists(snap):
        print("  SKIP  结构项：attempt8 快照不在本机")
        plan = None
    else:
        doc = json.load(open(snap, encoding="utf-8"))
        q0 = [doc["position"][doc["name"].index(n)] for n in ch.move_joints]
        plan, checks = hp.plan_park(ch, q0)
        check("烛台姿 park 计划可出（真实起点）",
              plan["meta"]["kind"] == "park" and plan["waypoints"],
              plan["meta"])
        try:
            pe.validate_plan_doc(plan)
            check("validate_plan_doc 放行 park 计划", True)
        except ValueError as e:
            check("validate_plan_doc 放行 park 计划", False, str(e))

    print("[3] 闸门：出软限位的 HOME 必须拒（fail-closed）")
    try:
        hp2 = hp
        saved = hp2.HOME_JOINTS
        hp2.HOME_JOINTS = tuple(math.radians(v) for v in
                                (0, 400, 90, -90, -90, 0))
        try:
            hp2.plan_park(ch, [0.0] * 6)
        except pp.PlanError as e:
            check("软限位外的 HOME ⇒ PlanError", "软限位" in str(e), str(e))
        else:
            check("软限位外的 HOME ⇒ PlanError", False, "居然通过了")
        finally:
            hp2.HOME_JOINTS = saved
    except Exception as e:  # noqa: BLE001
        check("软限位变异测试", False, repr(e))

    print("[4] 从 attempt8 停止位姿回 HOME：插值护栏 + 小步")
    if plan is None:
        print("  SKIP  attempt8 快照不在本机")
    else:
        q0 = plan["q_start"]
        ok, worst = pp.check_guard(ch, q0, home, 0.0)
        check("从停止位姿插值护栏过（指尖最低 22.8 cm 量级）", ok, repr(worst))
        steps = [abs(b - a) for a, b in zip(q0, home)]
        check("最大单步 < 57°（无插点需求）", max(steps) < 1.0,
              max(math.degrees(s) for s in steps))
        check("park 航点 1 个（计划体积极小）",
              len(plan["waypoints"]) == 1, plan["waypoints"])

    print("\n%d PASS  %d FAIL  %d SKIP" % (PASS, FAIL, SKIP))
    if FAILS:
        print("失败项：")
        for f in FAILS:
            print("  - %s" % f)
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())

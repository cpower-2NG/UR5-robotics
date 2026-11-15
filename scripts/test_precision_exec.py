#!/usr/bin/env python3
"""precision_exec 的离线自检：计划 JSON v2 结构校验 + traj_util（D3）。

2026-11-14 审计补的两道线都在这里钉住：
1. ``validate_plan_doc`` —— 此前不查 version / q 长度，v1 残留或截断的计划
   会静默进 ``send()``，靠 JTC 拒 goal 兜底（运行时才失败）；
2. ``traj_util.start_point`` —— t=0 起点（缺它 JTC 全速冲刺，2026-11-07
   打飞块）现在 exec 与 sim_bridge 同源。
ROS msg 相关的断言在没 ROS 环境时降级 SKIP（明确报出，不假装跑过）。
"""
import os
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import precision_exec as pe                                         # noqa: E402

PASS = FAIL = SKIP = 0
FAILS = []


def check(name, ok, detail=""):
    global PASS, FAIL
    if ok:
        PASS += 1
        print("  PASS  %s" % name)
    else:
        FAIL += 1
        FAILS.append(name)
        print("  FAIL  %s   ← %s" % (name, detail))


def skip(name, detail=""):
    global SKIP
    SKIP += 1
    print("  SKIP  %s   ← %s" % (name, detail))


def good_plan():
    return {"version": 2,
            "meta": {"kind": "pick"},
            "q_start": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "waypoints": [
                {"name": "approach", "q": [0.1] * 6, "duration": 2.0},
                {"name": "grasp", "q": [0.2] * 6, "duration": 1.5},
            ]}


def expect_reject(name, mutate):
    plan = good_plan()
    mutate(plan)
    try:
        pe.validate_plan_doc(plan)
    except ValueError as e:
        check(name, True)
        return str(e)
    check(name, False, "竟然通过了")
    return ""


def main():
    print("=" * 68)
    print("[0] 起手自检护闸：开口不全开拒发（round-17 教训的自动形态）")
    err = pe.selftest_opening_error(25.2)
    check("中断轮开口 25.2 mm ⇒ 拒发且点名 --holding-block",
          err is not None and "--holding-block" in err, str(err))
    check("正常收尾轮开口 95 mm ⇒ 放行",
          pe.selftest_opening_error(95.0) is None)
    check("读不到开口（None）⇒ 不拦（自检自己证明链路）",
          pe.selftest_opening_error(None) is None)
    check("阈值取自 SELFTEST_OPEN_MIN_MM（80）",
          pe.selftest_opening_error(80.0) is None
          and pe.selftest_opening_error(79.9) is not None)

    print("=" * 68)
    print("[1] validate_plan_doc：合法计划放行、坏计划点名")
    try:
        pe.validate_plan_doc(good_plan())
        check("合法 v2 计划通过", True)
    except ValueError as e:
        check("合法 v2 计划通过", False, str(e))

    expect_reject("v1 残留计划 ⇒ 拒（点名 version）",
                  lambda p: p.update(version=1))
    expect_reject("缺 version ⇒ 拒", lambda p: p.pop("version", None))
    expect_reject("q_start 缺失 ⇒ 拒", lambda p: p.pop("q_start"))
    expect_reject("q_start 长度不齐 ⇒ 拒",
                  lambda p: p.update(q_start=[0.0] * 5))
    expect_reject("waypoints 为空 ⇒ 拒", lambda p: p.update(waypoints=[]))
    expect_reject("waypoints 缺失 ⇒ 拒", lambda p: p.pop("waypoints"))
    expect_reject("航点名重复 ⇒ 拒",
                  lambda p: p["waypoints"][1].update(name="approach"))
    expect_reject("航点 q 长度截断 ⇒ 拒",
                  lambda p: p["waypoints"][1].update(q=[0.2] * 4))
    expect_reject("duration 非法（0/负/缺）⇒ 拒",
                  lambda p: p["waypoints"][0].update(duration=0.0))

    print("[2] traj_util：missing_joints 纯函数")
    import traj_util
    got = traj_util.missing_joints(
        ["a", "b", "c"], {"a": 0.1, "b": None, "c": 0.3})
    check("current_map 里 None/缺失的关节都报出", got == ["b"], got)
    check("全在 ⇒ 空表",
          traj_util.missing_joints(["a"], {"a": 1.0}) == [], "")
    check("空快照 ⇒ 全报",
          traj_util.missing_joints(["a", "b"], {}) == ["a", "b"], "")

    print("[3] traj_util.start_point：真起点（要 ROS msg，缺环境就 SKIP）")
    try:
        from trajectory_msgs.msg import JointTrajectoryPoint
    except ImportError as e:
        skip("start_point 需要 ROS msg（source /opt/ros/humble/setup.bash）",
             str(e))
    else:
        pt = traj_util.start_point([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        check("positions 是 float 列表、长度保持",
              isinstance(pt, JointTrajectoryPoint)
              and list(pt.positions) == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
              pt.positions)
        check("t=0、零速（缺起点 JTC 全速冲刺的事故路径钉死在这）",
              pt.time_from_start.sec == 0 and pt.time_from_start.nanosec == 0
              and all(v == 0.0 for v in pt.velocities),
              (pt.time_from_start, pt.velocities))

    print("\n%d PASS  %d FAIL  %d SKIP" % (PASS, FAIL, SKIP))
    if FAILS:
        print("失败项：")
        for f in FAILS:
            print("  - %s" % f)
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""标准 HOME（停靠位姿）—— **全仓唯一出处**（2026-11-15 用户定：所有任务跑完都回这里）。

位姿：``[0°, -90°, +90°, -90°, -90°, 0°]``（"烛台姿"）——大臂水平、小臂竖直
朝上、工具竖直朝下，悬在作业区上方。2026-11-15 几何验证（robot.urdf 实算）：

* 软限位/腕奇异/缠绕三门全过；tool0 z=+0.432 m（台面上方），工具朝下；
* 从 attempt8 停止位姿直线插值：指尖最低 +0.228 m（护栏 8 mm 下限的 28 倍余量）；
* 各关节离 0 近，线缆不缠（缠绕帽 ±210° 的零头）。

**为什么不是"各关节都 90°"**（用户的初始直觉，当日实测否决）：UR5 的连杆
方向不是对称的——[90°×6] 的 FK 让 tool0 钻到**台面下方 −0.253 m**，插值中途
wrist_1_link 最低 −0.502 m（高度护栏正确拒绝）。"好看"的其实是烛台姿。

与其他"home"的关系（一个事实一个家）：
* `grasp_planner.yaml` 的 ``home_joints``（[0,−90,90,−90,−90,0]，与这里的值
  **相同**）是阶段 1 规划器节点的内部事实；fake_joint_states 的 HOME 与它同值。
  本模块是**阶段 4 任务收尾停靠**的唯一出处：轮尾 park 计划、park_arm.py 都读这里。
"""
import math

HOME_JOINTS_DEG = (0.0, -90.0, 90.0, -90.0, -90.0, 0.0)
HOME_JOINTS = tuple(math.radians(v) for v in HOME_JOINTS_DEG)


def plan_park(chain, q_start, name="home", table_z=0.0):
    """关节空间 park 计划（计划 JSON v2）：回 HOME，全部闸门同口径。

    与 IK 计划的区别：HOME 是**固定关节值**，不解 IK；但闸门一个不少——
    软限位 / 腕奇异（终点）/ 缠绕帽（终点）/ 段内高度护栏（q_start→HOME
    直线采样，与 check_guard 同口径）/ 大跳插点（split_jumps，含中点跨零
    检查）。任何一门不过 ⇒ ``PlanError``（fail-closed，不写计划）。

    返回 ``(计划 dict, checks)``；计划可直接进 ``precision_exec`` 与
    ``run.sh``（编排器把它挂在轮尾，授权随整轮）。
    """
    import precision_plan as pp

    q_start = [float(v) for v in q_start]
    home = [float(v) for v in HOME_JOINTS]
    if len(home) != len(chain.move_joints):
        raise pp.PlanError("HOME 关节数 %d ≠ 链 %d"
                           % (len(home), len(chain.move_joints)))
    if pp.check_limits(chain, home):
        raise pp.PlanError("HOME 超出软限位：%s" % pp.check_limits(chain, home))
    if pp.check_wrist(chain, home):
        raise pp.PlanError("HOME 在腕奇异带：%s" % (pp.check_wrist(chain, home),))
    if pp.check_winding(chain, home):
        raise pp.PlanError("HOME 缠绕超帽：%s" % (pp.check_winding(chain, home),))
    ok, worst = pp.check_guard(chain, q_start, home, table_z)
    if not ok:
        raise pp.PlanError("回 HOME 的直线插值撞高度护栏：%s z=%.3f（下限 %.3f）"
                           % (worst[1], worst[2], worst[3]))
    wps = [{"name": name, "q": home,
            "duration": float(pp.duration_for(q_start, home))}]
    wps, n_split = pp.split_jumps(wps, chain, q_start=q_start)
    plan = {"version": 2,
            "meta": {"kind": "park", "home_deg": list(HOME_JOINTS_DEG),
                     "n_split": n_split,
                     "collision_validator": False},
            "q_start": q_start,
            "waypoints": wps}
    checks = [{"name": name, "n_split": n_split}]
    return plan, checks

#!/usr/bin/env python3
"""两点式轨迹的 **t=0 起点**（D3，2026-11-14 把四处拷贝收敛成一处）。

为什么必须补起点：JTC 收到单点轨迹时把整段时间压成一次**全速冲刺**
（2026-11-07 实测把块打飞，0/2；补起点后 16/16）。这条修法此前在
`precision_exec.send` / `sim_bridge._send` / `stage4_v1_sim.Arm.goto` /
`real_first_motion` 各写一遍 —— D3 的判据是「双路径单测同源」。

只放两个纯函数；ROS msg 的 import 放在函数体内，本模块在**没有 ROS
环境**的开发机上也要能 import（离线测试用）。
"""


def start_point(positions):
    """起点 JointTrajectoryPoint：t=0、零速。``positions`` 按轨迹关节序。"""
    from trajectory_msgs.msg import JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    pt = JointTrajectoryPoint()
    pt.positions = [float(v) for v in positions]
    pt.velocities = [0.0] * len(positions)
    pt.time_from_start = Duration(sec=0, nanosec=0)
    return pt


def missing_joints(names, current_map):
    """``names`` 里不在 ``current_map``（/joint_states 快照，dict）里的关节。

    调用方拿到非空返回值必须**响亮拒绝**发轨迹，不许静默跳过补起点 ——
    那等于把「缺起点 ⇒ 全速冲刺」这条 2026-11-07 的事故路径重新接回系统
    （2026-11-14 审计：sim_bridge 原来正是这么静默降级的）。
    """
    return [str(n) for n in names if (current_map or {}).get(n) is None]

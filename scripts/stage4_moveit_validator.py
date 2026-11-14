#!/usr/bin/env python3
"""段级碰撞裁决（A2）：关节直线采样过 MoveIt ``/check_state_validity``。

为什么必须有它：attempt6（2026-11-12）执行首块 approach 时，计划给的关节大重构
让夹爪撞上臂自身（PROTECTIVE_STOP C153A3）—— 步长/限位/腕翻转/高度护栏全是
「关节角合法」口径的门，拦不住**合法关节角的自碰撞**。本模块把它变成计划落盘
前的门：`precision_plan.solve_path` 对每条「进入航点的转移」调 ``validator(qa, qb)``，
任一采样点无效 ⇒ 该转移剪枝（与高度护栏同语义，DP 里 fail-closed）。

⚠️ 前提（A1，**未完成前本模块不得上正式轮**）：真机 MoveIt 模型要带 AG-95
碰撞几何（本体 + 两指）—— 现真机模型 0 个 ag95 连杆，检查会漏夹爪本身
（E8 实测）。A1 的判据见 `docs/阶段/阶段4-杂乱指定抓取/板上运行时改造方案.md` 包 A。

为什么是段级：attempt6 撞的是**段内**构型（起点与终点都可能干净、中间拧过臂），
只查航点等于没查 —— 与 `check_guard` 的「插值点也要查」同一教训。

用法（板上，与 move_group 同机同图）::

    from stage4_moveit_validator import make_segment_validator
    import stage4_clear_run as cr
    validator = make_segment_validator(chain.move_joints, group='ur_manipulator')
    plans = cr.plan_sequence(..., validator=validator)

开发机离线排练不给 validator（开发机不在 DDS 图里——这正是「规划上板」的
结构性理由之一）。 ``require=True`` 时 move_group 不可达 ⇒ RuntimeError
（正式轮 fail-closed）；采样点查询失败 ⇒ 该转移判无效（宁拒不放过）。

只读：``/check_state_validity`` 不发轨迹、不动臂、不写场景。
"""
import sys

sys.dont_write_bytecode = True

#: 采样点由**纯函数**生成（可离线自测）：转移两端点 + 段内均分点。
DEFAULT_SAMPLES = 8


def sample_line(qa, qb, samples=DEFAULT_SAMPLES):
    """关节空间直线 ``qa→qb`` 的采样序列（含两端点，共 ``samples+1`` 个）。

    与 `precision_plan.check_guard` 的采样口径一致：JTC 在关节空间直线插值，
    采出来的就是执行会经过的构型。
    """
    import numpy as np
    qa = np.asarray(qa, float)
    qb = np.asarray(qb, float)
    if qa.shape != qb.shape or qa.size == 0:
        raise ValueError("采样两端必须是同长的非空关节数组：%r vs %r"
                         % (qa.shape, qb.shape))
    return [qa + (qb - qa) * (i / samples) for i in range(samples + 1)]


_STATE = {}


def make_segment_validator(joint_names, group="ur_manipulator",
                           service="/check_state_validity",
                           samples=DEFAULT_SAMPLES, timeout_s=5.0,
                           require=True):
    """构造 ``solve_path`` 用的段级裁决闭包 ``(qa, qb) -> (ok, why)``。

    ``joint_names``：按调用方关节序（``chain.move_joints``）的名字表 —— 请求里
    的 ``JointState`` 按它构造，**只发臂关节**（真机模型不含 ag95，混进去会
    打死 move_group，见 `stage4_moveit_joints` 的教训）。
    """
    try:
        import rclpy
    except ImportError as exc:                                # noqa: PERF203
        if require:
            raise RuntimeError(
                "碰撞裁决不可用：本机没有 rclpy（%s）—— 正式轮的规划必须在"
                "板上跑（与 move_group 同图），离线排练请显式 validator=None"
                % exc)
        return None
    from moveit_msgs.srv import GetStateValidity
    from sensor_msgs.msg import JointState

    if "node" not in _STATE:
        # 2026-11-14 审计：裸 rclpy.init() 在宿主进程已 init 时（例如编排器
        # 同进程先起过别的 ROS 客户端）直接 RuntimeError。幂等：已 init 就用。
        if not rclpy.ok():
            rclpy.init()
        _STATE["node"] = rclpy.create_node("stage4_moveit_validator")
        _STATE["client"] = _STATE["node"].create_client(GetStateValidity,
                                                        service)
    node, client = _STATE["node"], _STATE["client"]
    if not client.wait_for_service(timeout_sec=timeout_s):
        if require:
            raise RuntimeError(
                "碰撞裁决不可用：/check_state_validity %.0f s 没出现 —— "
                "move_group 起了吗？（起栈 runbook 第三步）" % timeout_s)
        return None

    def validate(qa, qb):
        for q in sample_line(qa, qb, samples):
            js = JointState()
            js.name = list(joint_names)
            js.position = [float(v) for v in q]
            req = GetStateValidity.Request()
            req.group_name = group
            req.robot_state.joint_state = js
            fut = client.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout_s)
            resp = fut.result()
            if resp is None:
                return False, "check_state_validity 不应答（fail-closed）"
            if not bool(resp.valid):
                contacts = ["%s×%s" % (c.contact_body_1, c.contact_body_2)
                            for c in resp.contacts][:4]
                return False, "碰撞：%s" % (contacts or "invalid")
        return True, ""

    return validate

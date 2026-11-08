"""MoveIt 关节状态的分界过滤：只把**规划模型认识的关节**发进服务。

为什么值得单独一个模块（2026-11-08 真机事故）：真机上 ``/joint_states`` 里带
``ag95_left_finger_joint`` / ``ag95_right_finger_joint``（夹爪驱动单独发的），而真机
MoveIt 的 ``robot_description`` **只有 6 个臂关节**（模型名 ``ur``，无任何 ag95
连杆）。把整条 ``/joint_states`` 当 IK seed 发进 ``/compute_ik``，move_group 在
``robotStateMsgToRobotState → setVariablePositions`` 里抛

    moveit::Exception: Variable 'ag95_left_finger_joint' is not known to model 'ur'

**没人接这个 C++ 异常** ⇒ ``std::terminate`` ⇒ ``SIGABRT``，move_group 整进程死掉
（不是返回错误码）。之后所有 IK/FK/规划请求都变成 ``no_response``，看起来像
「全都够不着」—— 假象比崩溃本身更危险。

所以过滤要发生在**构造请求的边界**上：``with_joints`` 遇到模型不认识的关节必须
**丢弃**，绝不能像以前那样「追加」。仿真模型的 URDF 带 ag95 关节，``known`` 里
有它们，行为不变；真机 ``known`` 只有 6 个臂关节，两指被丢掉。

纯函数，无 ROS 依赖，可离线自测（``test_stage4_moveit_joints.py``）。
"""

ARM_JOINTS = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
              "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")


def keep_known(names, positions, known):
    """按 ``known`` 过滤（保持原顺序），返回 ``(names, positions)``。

    ``known is None`` 表示不过滤（原样复制）。``positions`` 里的值一律转 float。
    """
    if known is None:
        return list(names), [float(p) for p in positions]
    k = set(known)
    out_n, out_p = [], []
    for name, pos in zip(names, positions):
        if name in k:
            out_n.append(name)
            out_p.append(float(pos))
    return out_n, out_p


def merge_known(base_names, base_positions, updates, known=None):
    """以 base 为底并入 ``updates``；**不在 known 里的更新被丢弃**（不追加）。

    返回值是 ``(names, positions)``。``known is None`` 时退回旧行为（追加未知
    关节）—— 只有「拿不到模型关节名」的降级路径会走到，调用方必须已在日志里
    响亮告警。
    """
    if known is None:
        m = dict(zip(base_names, (float(p) for p in base_positions)))
        m.update({k: float(v) for k, v in updates.items()})
        names = list(base_names) + [k for k in updates if k not in base_names]
        return names, [m[k] for k in names]
    k = set(known)
    m = {n: float(p) for n, p in zip(base_names, base_positions) if n in k}
    m.update({n: float(v) for n, v in updates.items() if n in k})
    # 保持 base 的顺序在前，更新里**模型认识的新关节**追加在后
    names = [n for n in base_names if n in m]
    names += [n for n in updates if n in k and n not in names]
    return names, [m[n] for n in names]


def unknown(names, known):
    """``names`` 里不在 ``known`` 中的那些（调用方用于告警/断言）。"""
    if known is None:
        return []
    k = set(known)
    return [n for n in names if n not in k]

#!/usr/bin/env python3
"""``stage4_moveit_joints`` 自检：分界过滤不得放过模型不认识的关节。

**这是安全攸关的一条**：2026-11-08 真机上，把带 ``ag95_*`` 的 ``/joint_states``
原样发进 ``/compute_ik``，move_group 抛 C++ 异常直接 SIGABRT。所以这里钉三件事：

1. ``keep_known`` 丢掉未知关节、保持顺序、位置转 float；
2. ``merge_known`` **丢弃**未知更新（老实现是追加 —— 那正是崩溃的写法）；
3. ``known=None`` 的降级路径保留旧行为，且 ``unknown()`` 能把未知名字报出来。

跑法：
    python3 scripts/test_stage4_moveit_joints.py
"""
import os
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import stage4_moveit_joints as mj                             # noqa: E402

N_PASS = N_FAIL = 0
FAILS = []

ARM = list(mj.ARM_JOINTS)
FINGERS = ["ag95_left_finger_joint", "ag95_right_finger_joint"]


def check(name, cond, detail=''):
    global N_PASS, N_FAIL
    if cond:
        N_PASS += 1
        print('  PASS  %s' % name)
    else:
        N_FAIL += 1
        FAILS.append(name)
        print('  FAIL  %s%s'
              % (name, '' if detail == '' else '   ← %r' % (detail,)))


def section_keep():
    print('\n[1] keep_known：丢未知、保顺序、float')
    n, p = mj.keep_known(ARM + FINGERS, [0, 1, 2, 3, 4, 5, 0.5, -0.5], ARM)
    check("只留 6 个臂关节", n == ARM, n)
    check("两指被丢掉", not any(x.startswith("ag95") for x in n), n)
    check("位置顺序跟着名字走", p == [0.0, 1.0, 2.0, 3.0, 4.0, 5.0], p)
    check("位置被转成 float", all(isinstance(v, float) for v in p), p)
    n2, p2 = mj.keep_known(ARM, [1] * 6, None)
    check("known=None ⇒ 原样（含两指之外的都留）", n2 == ARM and len(p2) == 6,
          (n2, p2))


def section_merge():
    print('\n[2] merge_known：未知更新被**丢弃**，不是追加（崩溃的就是追加）')
    n, p = mj.merge_known(ARM, [0] * 6, {FINGERS[0]: 0.0475}, ARM)
    check("追加两指不得进 names", FINGERS[0] not in n, n)
    check("长度不变", len(n) == 6, n)
    n2, p2 = mj.merge_known(ARM, [0] * 6, {"wrist_3_joint": 1.25}, ARM)
    check("认识的关节能覆盖", dict(zip(n2, p2))["wrist_3_joint"] == 1.25, p2)
    check("覆盖不改变顺序", n2 == ARM, n2)
    # 仿真里 known 含两指 ⇒ 两指**必须**能加进去（行为不变）
    sim_known = ARM + FINGERS
    n3, p3 = mj.merge_known(ARM, [0] * 6, {FINGERS[0]: 0.0475,
                                          FINGERS[1]: -0.0475}, sim_known)
    check("仿真（known 含两指）⇒ 两指加得进去", n3 == ARM + FINGERS, n3)
    check("仿真两指位置正确", p3[-2:] == [0.0475, -0.0475], p3)
    # 既有 base 里的未知关节（比如过滤前读到的）也要被丢掉
    n4, p4 = mj.merge_known(ARM + FINGERS, [0] * 8, {}, ARM)
    check("base 里的未知关节也被清掉", n4 == ARM, n4)


def section_unknown():
    print('\n[3] unknown()：把越界名字报出来（调用方用于告警）')
    check("报出两指", mj.unknown(ARM + FINGERS, ARM) == FINGERS,
          mj.unknown(ARM + FINGERS, ARM))
    check("全认识 ⇒ 空", mj.unknown(ARM, ARM) == [])
    check("known=None ⇒ 不报", mj.unknown(ARM, None) == [])


def main():
    section_keep()
    section_merge()
    section_unknown()
    print('\n%d PASS / %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('失败项：' + ', '.join(FAILS))
    return 0 if N_FAIL == 0 else 1


if __name__ == '__main__':
    sys.exit(main())

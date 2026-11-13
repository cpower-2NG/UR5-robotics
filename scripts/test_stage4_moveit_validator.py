#!/usr/bin/env python3
"""A2 段级碰撞裁决：纯逻辑自测（不碰 ROS）+ 无 ROS 环境的 fail-closed 行为。"""
import math
import os
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np

import precision_plan as pp
import stage4_moveit_validator as mv
import urkin

PASS = FAIL = 0


def check(name, ok, detail=''):
    global PASS, FAIL
    if ok:
        PASS += 1
        print('  PASS  %s' % name)
    else:
        FAIL += 1
        print('  FAIL  %s  ← %s' % (name, detail))


def test_sample_line():
    qa = [0.0, 0.0]
    qb = [1.0, -2.0]
    pts = mv.sample_line(qa, qb, samples=4)
    check('采样数 = samples+1（含两端）', len(pts) == 5, len(pts))
    check('起点在列', np.allclose(pts[0], qa))
    check('终点在列', np.allclose(pts[-1], qb))
    check('中点线性', np.allclose(pts[2], [0.5, -1.0]))
    try:
        mv.sample_line([0.0], [1.0, 2.0])
    except ValueError:
        check('异长输入 ⇒ ValueError', True)
    else:
        check('异长输入 ⇒ ValueError', False)
    # 与 check_guard 同口径：JTC 直线插值，采的就是执行的构型
    chain = urkin.Chain(os.path.expanduser(
        '~/robotics/notes/ur5-vision-grasp/runs/'
        '2026-10-25-precision/robot.urdf'))
    q0 = [0.1, -1.2, 1.4, -1.6, -1.5, 0.3]
    q1 = [0.3, -1.0, 1.2, -1.4, -1.3, 0.1]
    ok, worst = pp.check_guard(chain, q0, q1, 0.0)
    pts = mv.sample_line(q0, q1, samples=pp.GUARD_SAMPLES)
    check('采样口径 = check_guard 的 8 段 9 点',
          len(pts) == pp.GUARD_SAMPLES + 1, len(pts))
    check('check_guard 在同一直线上自洽（对照组）', isinstance(ok, bool), worst)


def test_validator_fail_closed_without_ros():
    """开发机没有 rclpy / move_group：require=True 必须 RuntimeError（正式轮
    拿不到裁决就不许规划），require=False 返回 None（离线排练显式降级）。"""
    try:
        mv.make_segment_validator(['a', 'b'], require=True, timeout_s=1.0)
    except RuntimeError as e:
        check('无 ROS ⇒ RuntimeError（fail-closed）',
              '碰撞裁决不可用' in str(e), str(e))
    except Exception as e:                                    # noqa: BLE001
        check('无 ROS ⇒ RuntimeError（fail-closed）', False, repr(e))
    else:
        check('无 ROS ⇒ RuntimeError（fail-closed）', False, '竟然成功了')
    got = mv.make_segment_validator(['a', 'b'], require=False, timeout_s=1.0)
    check('require=False ⇒ 返回 None（离线排练显式降级）', got is None, got)


def test_validator_in_solve_path():
    """validator 注入点真的参与 DP 剪枝：一段被裁决拒掉 ⇒ 整链换支或拒。"""
    chain = urkin.Chain(os.path.expanduser(
        '~/robotics/notes/ur5-vision-grasp/runs/'
        '2026-10-25-precision/robot.urdf'))
    xyz0 = pp.SURVEY_TOOL0['pick']
    xyz1 = pp.SURVEY_TOOL0['place']
    q0 = [0.1, -1.2, 1.4, -1.6, -1.5, 0.3]
    targets = [('a', xyz0), ('b', xyz1)]

    def reject_all(qa, qb):
        return False, '测试注入：全部转移都拒'

    try:
        pp.solve_path(chain, targets, q0, validator=reject_all)
    except pp.BranchChainError as e:
        check('validator 全拒 ⇒ BranchChainError（fail-closed）', True)
    except Exception as e:                                    # noqa: BLE001
        check('validator 全拒 ⇒ BranchChainError（fail-closed）', False, repr(e))
    else:
        check('validator 全拒 ⇒ BranchChainError（fail-closed）', False,
              '竟然出了计划')

    # 只拒一半 ⇒ 换支仍可解（存在性判定不受影响时不应误杀）
    calls = {'n': 0}

    def reject_first_call(qa, qb):
        calls['n'] += 1
        return calls['n'] > 1, '第一次之后都放行'

    qpath = pp.solve_path(chain, targets, q0, validator=reject_first_call)
    check('部分拒绝 ⇒ 仍出计划（两航点）', len(qpath) == 2, len(qpath))
    pe, re_ = pp.pose_residual(chain, qpath[0], xyz0)
    check('出计划的航点仍满足位姿残差', pe < 0.1 and re_ < 0.05, (pe, re_))


if __name__ == '__main__':
    print('[A2] 段级碰撞裁决自检')
    test_sample_line()
    test_validator_fail_closed_without_ros()
    test_validator_in_solve_path()
    print('\n%d PASS, %d FAIL' % (PASS, FAIL))
    sys.exit(1 if FAIL else 0)

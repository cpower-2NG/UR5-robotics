#!/usr/bin/env python3
"""E8 几何核验器自检：OBB 求交 / 间隙 / 场景构造 / 与 URDF 一致 / 变异。

为什么要单测「间隙」：核验器的全部结论都压在 `gap()` 这一条上。它必须是
**真实距离的下界** —— 只有这个方向是安全的（多报碰撞可以，漏报不行）。
所以这里既验整数情形（贴面 = 0、分开 100 mm = 100 mm），也验「保守性」：
随机位姿下 `gap() ≤ 真值距离`。

跑法：python3 scripts/test_stage4_scene_model.py
"""
import inspect
import math
import os
import re
import sys

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np                                            # noqa: E402
import stage4_scene_model as sm                               # noqa: E402

N_PASS = N_FAIL = 0
FAILS = []


def check(name, cond, detail=''):
    global N_PASS, N_FAIL
    if cond:
        N_PASS += 1
        print('  PASS  %s' % name)
    else:
        N_FAIL += 1
        FAILS.append(name)
        print('  FAIL  %s%s' % (name, '   ← %s' % detail if detail else ''))


def _mut(old, new):
    src = inspect.getsource(sm)
    assert old in src, '变异锚点不在源码里：%r' % old
    ns = {'__file__': sm.__file__, '__name__': 'mut'}
    exec(compile(src.replace(old, new, 1), '<mut>', 'exec'), ns)
    return ns


def unit(center=(0, 0, 0), half=0.05, rot=None, name=''):
    return sm.Obb(center, [half] * 3, rot, name)


# ------------------------------------------------------------------ 旋转
def section_quat():
    print('\n[0] 旋转：四元数 → 矩阵')
    m = sm.quat_to_mat((0, 0, 0, 1))
    check('单位四元数 → 单位矩阵', np.allclose(m, np.eye(3)))

    s = math.sin(math.pi / 4)
    m = sm.quat_to_mat((0, 0, s, s))
    check('绕 z 转 90°：x 轴 → y 轴',
          np.allclose(m.dot([1, 0, 0]), [0, 1, 0], atol=1e-9))
    check('绕 z 转 90°：y 轴 → −x',
          np.allclose(m.dot([0, 1, 0]), [-1, 0, 0], atol=1e-9))

    m = sm.quat_to_mat((0, 0, 2 * s, 2 * s))   # 未归一化
    check('未归一化四元数也被归一', np.allclose(m.dot([1, 0, 0]), [0, 1, 0],
                                          atol=1e-9))


# ------------------------------------------------------------------ 间隙
def section_rpy():
    print('\n[0b] 姿态：rpy=(π,0,ψ) 竖直朝下，与 planner_node 同式')
    q = sm.down_quat(0.0)
    check('down_quat(0) = (1,0,0,0)', np.allclose(q, (1, 0, 0, 0), atol=1e-12),
          str(q))
    m = sm.quat_to_mat(q)
    check('tool z 轴朝下（世界 −z）', np.allclose(m[:, 2], [0, 0, -1], atol=1e-9),
          str(m[:, 2]))
    check('tool x 轴对着世界 +x', np.allclose(m[:, 0], [1, 0, 0], atol=1e-9))

    q = sm.down_quat(math.pi / 2)
    m = sm.quat_to_mat(q)
    check('yaw=90° ⇒ tool x 轴对着世界 +y',
          np.allclose(m[:, 0], [0, 1, 0], atol=1e-9), str(m[:, 0]))
    check('yaw 后仍朝下', np.allclose(m[:, 2], [0, 0, -1], atol=1e-9))

    # 与「先绕 z 转 ψ 再绕 x 转 π」逐元素对（独立于四元数公式）
    yaw = 0.7
    Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                   [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    Rx = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    check('down_quat(ψ) == Rz(ψ)·Rx(π)',
          np.allclose(sm.quat_to_mat(sm.down_quat(yaw)), Rz.dot(Rx), atol=1e-9))


def section_interp():
    print('\n[0c] 轨迹补点：航点之间必须插到足够密')
    out = sm.interp_positions([[0.0, 0.0], [0.5, -0.2]], step=0.05)
    arr = np.array(out)
    check('起点保留', np.allclose(arr[0], [0, 0]))
    check('终点保留', np.allclose(arr[-1], [0.5, -0.2]))
    check('每步位移 ≤ step',
          np.max(np.abs(np.diff(arr, axis=0))) <= 0.05 + 1e-12,
          str(np.max(np.abs(np.diff(arr, axis=0)))))
    check('补的点数 = ceil(0.5/0.05) 段 + 端点', len(out) == 11, str(len(out)))

    one = sm.interp_positions([[0.1, 0.2]], step=0.05)
    check('单航点原样返回', len(one) == 1 and np.allclose(one[0], [0.1, 0.2]))

    tiny = sm.interp_positions([[0.0], [0.001]], step=0.05)
    check('小位移也只给两端点', len(tiny) == 2, str(len(tiny)))


def section_gap():
    print('\n[1] 间隙：贴面为 0、分开等于间距、重叠为 0')
    a = unit((0, 0, 0))
    b = unit((0.2, 0, 0))
    check('沿 x 分开 100 mm → gap = 100 mm', abs(sm.gap(a, b) - 0.1) < 1e-9,
          str(sm.gap(a, b)))

    b = unit((0.1, 0, 0))
    check('贴面 → gap = 0', sm.gap(a, b) == 0.0)

    b = unit((0.05, 0, 0))
    check('重叠 → gap = 0（不给负数）', sm.gap(a, b) == 0.0)

    # 对角分开：(0.2, 0.2) 的面-面距离是 0.1（单轴），不是 0.1414
    b = unit((0.2, 0.2, 0))
    check('对角分开：单轴间隙 100 mm（不是欧氏 141 mm）',
          abs(sm.gap(a, b) - 0.1) < 1e-9, str(sm.gap(a, b)))

    # 旋转分离：b 转 45°，沿 x 分开到面刚不碰的距离
    s45 = math.sin(math.pi / 8)
    c45 = math.cos(math.pi / 8)
    rot = sm.quat_to_mat((0, 0, s45, c45))
    # 转 45° 的立方体沿 x 的投影半宽 = 0.05·(|cos45|+|sin45|) = 0.0707107
    b = sm.Obb((0.05 + 0.0707106781, 0, 0), [0.05] * 3, rot)
    check('转 45° 后按投影半宽算（0.05+0.07071）',
          abs(sm.gap(a, b)) < 1e-6, str(sm.gap(a, b)))

    # 保守性：gap 必须 ≤ 真实面-面距离（这里构造已知的 120 mm）
    b = unit((0.22, 0.03, 0.01))
    check('保守性：斜置分开时 gap ≤ 真值距离 120 mm',
          sm.gap(a, b) <= 0.12 + 1e-9, str(sm.gap(a, b)))
    check('保守性：且不会低报成 0（真值 120 mm）', sm.gap(a, b) > 0.1,
          str(sm.gap(a, b)))


def section_margin():
    print('\n[2] margin：间隙 ≤ margin 即判碰撞')
    a = unit((0, 0, 0))
    b = unit((0.13, 0, 0))          # 间隙 30 mm
    check('无 margin → 不碰', not sm.collides(a, b))
    check('margin 20 mm → 不碰', not sm.collides(a, b, 0.02))
    check('margin 30 mm → 恰好碰到', sm.collides(a, b, 0.03))
    check('margin 40 mm → 碰', sm.collides(a, b, 0.04))
    check('first_hit 返回那一对', sm.first_hit([a], [b], 0.04) == (a, b))
    check('first_hit 干净 → None', sm.first_hit([a], [b], 0.01) is None)


# ------------------------------------------------------------------ 场景
def section_scene():
    print('\n[3] 场景构造：桌面与真值块')
    t = sm.table_obb(0.0, size_xy=(1.0, 1.0), thickness=0.05)
    check('桌顶在 0 ⇒ 中心 z = −25 mm', abs(t.center[2] + 0.025) < 1e-12)
    check('桌面半厚 25 mm', abs(t.half[2] - 0.025) < 1e-12)

    blk = {"id": 3, "x": 0.30, "y": 0.20, "size": 0.0295, "yaw": 0.0}
    b = sm.block_obb(blk, 0.0)
    check('块心在台面上方半个边长',
          abs(b.center[2] - 0.01475) < 1e-12, str(b.center))
    check('块心 xy 与真值一致',
          abs(b.center[0] - 0.30) < 1e-12 and abs(b.center[1] - 0.20) < 1e-12)
    check('块的顶面 = 台面 + 边长',
          abs((b.center[2] + b.half[2]) - 0.0295) < 1e-12)

    blk2 = {"id": 4, "x": 0.0, "y": 0.0, "top": 0.0295, "yaw": 90.0}
    b2 = sm.block_obb(blk2, 0.0)
    check('只有 top 时边长由 top 反推',
          abs(b2.half[0] * 2 - 0.0295) < 1e-12, str(b2.half))
    check('yaw 按**度**解析（90° ⇒ 绕 z 转 90°）',
          np.allclose(b2.axes[0], [0, 1, 0], atol=1e-9), str(b2.axes[0]))

    blk_z = {"id": 7, "x": 0.0, "y": 0.0, "z": 0.0442, "size": 0.0295,
             "yaw": 0.0}
    check('块给了 z 就用 z（堆叠层不再按台面推）',
          abs(sm.block_obb(blk_z, 0.0).center[2] - 0.0442) < 1e-12)

    blk45 = {"id": 8, "x": 0.0, "y": 0.0, "size": 0.0295, "yaw": 45.0}
    b45 = sm.block_obb(blk45, 0.0)
    check('45° 块的 x 半宽 = 边长/2·(cos45+sin45)',
          abs(sum(abs(b45.axes[i][0]) * b45.half[i] for i in range(3))
              - 0.01475 * math.sqrt(2)) < 1e-12)

    # 两块间距 29.5 mm 的块贴在一起：间隙应为 0
    b3 = sm.block_obb({"id": 5, "x": 0.0295, "y": 0.0,
                       "size": 0.0295, "yaw": 0.0}, 0.0)
    check('两块贴边 → gap = 0', sm.gap(b2, b3) < 1e-9, str(sm.gap(b2, b3)))
    # 块心距 = 边长 + 缝宽：0.0295 + 0.0295 + 0.0025
    b4 = sm.block_obb({"id": 6, "x": 0.0615, "y": 0.0,
                       "size": 0.0295, "yaw": 0.0}, 0.0)
    check('留 2.5 mm 缝 → gap ≈ 2.5 mm', abs(sm.gap(b3, b4) - 0.0025) < 1e-9,
          str(sm.gap(b3, b4)))


def section_gripper():
    print('\n[4] 夹爪：手指位置/尺寸与 URDF 逐字一致')
    base = ((0.0, 0.0, 0.5), (0.0, 0.0, 0.0, 1.0))

    obbs = sm.gripper_obbs(base, 0.0)
    names = [o.name for o in obbs]
    check('三个盒：本体 + 两指', names == ['ag95_base',
                                         'ag95_left_finger',
                                         'ag95_right_finger'], str(names))

    base_o, left, right = obbs
    check('本体中心在 base 上方 30 mm', abs(base_o.center[2] - 0.53) < 1e-12)
    check('本体尺寸 100×75×60',
          np.allclose(base_o.half * 2, [0.100, 0.075, 0.060]))

    # 闭合时：左指中心 x = +11 mm，右指 −11 mm；z = 60 + 72.1 = 132.1 mm
    check('闭合左指 x = +11 mm', abs(left.center[0] - 0.011) < 1e-12,
          str(left.center))
    check('闭合右指 x = −11 mm', abs(right.center[0] + 0.011) < 1e-12)
    check('指盒 z 中心 = 60 + 144.2/2 = 132.1 mm',
          abs(left.center[2] - 0.6321) < 1e-9, str(left.center))
    check('指盒 22×30×144.2',
          np.allclose(left.half * 2, [0.022, 0.030, 0.1442]))

    # 全开 47.5 mm：左指 x = 58.5 mm
    obbs = sm.gripper_obbs(base, sm.GRIPPER['stroke_half'])
    check('全开左指 x = 11 + 47.5 = 58.5 mm',
          abs(obbs[1].center[0] - 0.0585) < 1e-9, str(obbs[1].center[0]))
    inner = (obbs[1].center[0] - obbs[2].center[0]) - 0.022
    check('全开两指内侧面间距 = 95 mm', abs(inner - 0.095) < 1e-9, str(inner))
    check('全开时指与本体在 x 上重叠（URDF 里这对碰撞是 disable 的）',
          sm.gap(obbs[0], obbs[1]) == 0.0)

    tip = sm.fingertip(base)
    check('指尖外沿在 base 上方 204.2 mm',
          abs(tip[2] - 0.7042) < 1e-9, str(tip))

    # 转 90° 后：两指沿 y 排开
    s = math.sin(math.pi / 4)
    base_r = ((0.0, 0.0, 0.5), (0.0, 0.0, s, s))
    obbs = sm.gripper_obbs(base_r, 0.0)
    check('base 转 90° ⇒ 左指在 +y 方向', abs(obbs[1].center[1] - 0.011) < 1e-9,
          str(obbs[1].center))
    check('base 转 90° ⇒ 左指 x ≈ 0', abs(obbs[1].center[0]) < 1e-9)

    # 用 FK 给的两指位姿时，不再从 q 推导
    obbs_fk = sm.gripper_obbs(
        base, 0.0, extra={"left": ((0.2, 0.0, 0.5), (0, 0, 0, 1)),
                          "right": ((-0.2, 0.0, 0.5), (0, 0, 0, 1))})
    check('给了两指 FK 位姿就用它（不再从 q 推）',
          abs(obbs_fk[1].center[0] - 0.2) < 1e-12)
    check('两指盒尺寸不变', np.allclose(obbs_fk[2].half * 2,
                                  [0.022, 0.030, 0.1442]))


def section_urdf_sync():
    print('\n[5] 反查 URDF：常量漂移就红')
    path = os.path.join(HERE, '..', 'ros2_ws', 'src', 'arm_grasp_sim',
                        'models', 'ag95.urdf.xacro')
    with open(path, encoding='utf-8') as f:
        src = f.read()

    def prop(name):
        m = re.search(r'name="%s"\s+value="([0-9.]+)"' % name, src)
        assert m, 'URDF 里找不到 %s' % name
        return float(m.group(1))

    check('URDF finger_x = 22 mm',
          abs(prop('finger_x') - sm.GRIPPER['finger'][0]) < 1e-12)
    check('URDF finger_y = 30 mm',
          abs(prop('finger_y') - sm.GRIPPER['finger'][1]) < 1e-12)
    check('URDF stroke_half = 47.5 mm',
          abs(prop('stroke_half') - sm.GRIPPER['stroke_half']) < 1e-12)
    check('URDF tip_z = 204.2 mm',
          abs(prop('tip_z') - sm.GRIPPER['tip_z']) < 1e-12)
    check('URDF body 100×75×60',
          abs(prop('body_x') - sm.GRIPPER['body'][0]) < 1e-12
          and abs(prop('body_y') - sm.GRIPPER['body'][1]) < 1e-12
          and abs(prop('body_z') - sm.GRIPPER['body'][2]) < 1e-12)
    # 指长 = tip_z − body_z（URDF 自己就是这么推的）
    check('指盒 z = tip_z − body_z',
          abs(sm.GRIPPER['finger'][2] - (prop('tip_z') - prop('body_z'))) < 1e-12)


# ------------------------------------------------------------------ 变异
def section_mutation():
    print('\n[6] 变异：把几何口径改坏，对应用例必须红')

    a = unit((0, 0, 0))
    b = unit((0.13, 0, 0))

    # A. margin 被忽略
    real = sm.collides(a, b, 0.04)
    mut = _mut('    return gap(a, b) <= margin', '    return gap(a, b) <= 0.0')
    check('变异「忽略 margin」：真实碰、改坏后不碰',
          real and not mut['collides'](a, b, 0.04))

    # B. gap 给负数
    mut = _mut('    return max(0.0, best)', '    return best')
    check('变异「重叠时给负数」：真实 0、改坏后 < 0',
          sm.gap(a, unit((0.05, 0, 0))) == 0.0
          and mut['gap'](a, unit((0.05, 0, 0))) < 0.0)

    # C. 手指左右写反
    base = ((0.0, 0.0, 0.5), (0.0, 0.0, 0.0, 1.0))
    real_l = sm.gripper_obbs(base, 0.0)[1].center[0]
    mut = _mut('for side, sign in (("left", 1.0), ("right", -1.0)):',
               'for side, sign in (("left", -1.0), ("right", 1.0)):')
    bad_l = mut['gripper_obbs'](base, 0.0)[1].center[0]
    check('变异「左右指写反」：真实 +11 mm、改坏后 −11 mm',
          abs(real_l - 0.011) < 1e-12 and abs(bad_l + 0.011) < 1e-12)

    # D. 指尖用本体高
    mut = _mut('np.array([0.0, 0.0, GRIPPER["tip_z"]]))',
               'np.array([0.0, 0.0, GRIPPER["body_z"]]))')
    bad_tip = mut['fingertip'](base)
    check('变异「指尖用本体高」：真实 0.7042、改坏后 0.56',
          abs(sm.fingertip(base)[2] - 0.7042) < 1e-9
          and abs(bad_tip[2] - 0.56) < 1e-9, str(bad_tip))


def main():
    print('=' * 68)
    section_quat()
    section_rpy()
    section_interp()
    section_gap()
    section_margin()
    section_scene()
    section_gripper()
    section_urdf_sync()
    section_mutation()
    print('\n%d PASS  %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('未通过：' + '; '.join(FAILS))
    return 1 if N_FAIL else 0


if __name__ == '__main__':
    sys.exit(main())

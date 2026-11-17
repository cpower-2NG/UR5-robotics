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


def section_separation():
    print('\n[1b] 带符号间隙：分离 > 0、接触 ≈ 0、穿插 < 0（S4 终态要区分接触与穿插）')
    a = unit((0, 0, 0))
    check('沿 x 分开 100 mm → +100 mm',
          abs(sm.separation(a, unit((0.2, 0, 0))) - 0.1) < 1e-9,
          str(sm.separation(a, unit((0.2, 0, 0)))))
    check('贴面 → 0（不是负数）',
          abs(sm.separation(a, unit((0.1, 0, 0)))) < 1e-12,
          str(sm.separation(a, unit((0.1, 0, 0)))))
    check('沿 x 穿插 50 mm → −50 mm',
          abs(sm.separation(a, unit((0.05, 0, 0))) + 0.05) < 1e-9,
          str(sm.separation(a, unit((0.05, 0, 0)))))
    check('完全重合 → −100 mm（半边长之和）',
          abs(sm.separation(a, unit((0, 0, 0))) + 0.1) < 1e-9,
          str(sm.separation(a, unit((0, 0, 0)))))
    # 同一对 OBB：gap 会把接触与穿插都压成 0，separation 必须分得开
    touch, pen = unit((0.1, 0, 0)), unit((0.05, 0, 0))
    check('gap 对「接触」与「穿插」都给 0（所以不能用它判终态）',
          sm.gap(a, touch) == 0.0 and sm.gap(a, pen) == 0.0)
    check('separation 对两者给不同符号',
          sm.separation(a, touch) >= 0.0 and sm.separation(a, pen) < 0.0)
    # 斜向穿透也一样（重叠深度不会跑到正数）
    check('斜向穿插仍是负数',
          sm.separation(a, unit((0.04, 0.04, 0.04))) < 0.0,
          str(sm.separation(a, unit((0.04, 0.04, 0.04)))))


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


def section_mat_quat():
    print('\n[5b] 旋转矩阵 → 四元数：四个分支都走一遍 + 往返')
    for name, q in (('单位', (0.0, 0.0, 0.0, 1.0)),
                    ('绕 x 180°', (1.0, 0.0, 0.0, 0.0)),
                    ('绕 y 180°', (0.0, 1.0, 0.0, 0.0)),
                    ('绕 z 180°', (0.0, 0.0, 1.0, 0.0)),
                    ('绕 z 90°', (0.0, 0.0, math.sin(math.pi / 4),
                                 math.sin(math.pi / 4)))):
        m = sm.quat_to_mat(q)
        back = sm.quat_to_mat(sm.mat_to_quat(m))
        check('往返 %s' % name, np.allclose(back, m, atol=1e-12),
              str(back))

    rng = np.random.RandomState(7)
    worst = 0.0
    for _ in range(50):
        rpy = rng.uniform(-math.pi, math.pi, 3)
        m = sm.quat_to_mat(sm.rpy_to_quat(rpy))
        worst = max(worst, float(np.max(np.abs(
            sm.quat_to_mat(sm.mat_to_quat(m)) - m))))
    check('50 个随机姿态往返误差 < 1e-12', worst < 1e-12, '%.3g' % worst)


def _block():
    # S1 风格：29.5 mm 立方、yaw=30°、块心离台面 14.75 mm
    return {'id': 3, 'x': 0.40, 'y': 0.30, 'z': 0.01475,
            'size': 0.0295, 'yaw': 30.0}


def section_grasp():
    print('\n[5c] 抓取候选：12 个摆位 × 2 个腕部朝向；指尖恰好越过面 18.5 mm')
    b = _block()
    cands = sm.grasp_candidates(b)
    keys = {(c['axis'], c['sign'], c['closing']) for c in cands}
    check('12 个摆位（3 轴 × 2 方向 × 2 合拢轴）', len(keys) == 12,
          str(sorted(keys)))
    check('每个摆位 2 个腕部朝向 ⇒ 24 个位姿', len(cands) == 24)

    vert = [c for c in cands if c['vertical']]
    check('B 档（a=(0,0,−1)）每块 4 个位姿 = 2 摆位 × 2 朝向', len(vert) == 4,
          str(len(vert)))
    check('B 档的摆位只来自块 z 轴向下',
          {(c['axis'], c['sign']) for c in vert} == {(2, -1)},
          str({(c['axis'], c['sign']) for c in vert}))
    check('B 档的合拢轴覆盖另外两轴',
          {c['closing'] for c in vert} == {0, 1})

    top_z = b['z'] + b['size'] / 2.0
    for c in vert:
        rot = sm.quat_to_mat(c['quat'])
        tip = sm.fingertip(c['pose'])
        check('B 档位姿#%d：tool z = (0,0,−1)' % c['variant'],
              np.allclose(rot[:, 2], [0, 0, -1], atol=1e-12))
        check('B 档位姿#%d：tool x = ±合拢轴（块轴 %d）' % (c['variant'], c['closing']),
              np.allclose(rot[:, 0],
                          sm.block_axes(b)[c['closing']] * (1 - 2 * c['variant']),
                          atol=1e-12))
        check('B 档位姿#%d：指尖在块顶下 18.5 mm、水平对准块心' % c['variant'],
              abs(tip[2] - (top_z - sm.GRASP_DEPTH)) < 1e-12
              and abs(tip[0] - b['x']) < 1e-12 and abs(tip[1] - b['y']) < 1e-12,
              str(tip))

    # 侧抓：接近轴 = 块 x 轴；指尖越过该面 18.5 mm
    side = [c for c in cands
            if (c['axis'], c['sign'], c['closing'], c['variant']) == (0, 1, 1, 0)]
    check('侧抓候选存在', len(side) == 1)
    if side:
        c = side[0]
        tip = sm.fingertip(c['pose'])
        expect = np.array([b['x'], b['y'], b['z']]) - sm.block_axes(b)[0] * (
            b['size'] / 2.0 - sm.GRASP_DEPTH)
        check('侧抓指尖 = 被接近面 + 18.5 mm（沿 a 深入）',
              np.allclose(tip, expect, atol=1e-12), '%s vs %s' % (tip, expect))
        check('侧抓 tool z = 块 x 轴（朝向块心）',
              np.allclose(sm.quat_to_mat(c['quat'])[:, 2],
                          sm.block_axes(b)[0], atol=1e-12))

    # 位姿是真的抓得住：全开夹爪在竖直抓取位姿上不得碰块，横向余量 32.75 mm
    v0 = [c for c in cands if c['vertical']][0]
    obbs = sm.gripper_obbs(v0['pose'], sm.GRIPPER['stroke_half'])
    blk = sm.block_obb(b, 0.0)
    g = min(sm.gap(o, blk) for o in obbs)
    check('全开夹爪在抓取位姿上离块 ≥ 32.75 mm（两侧各 32.75）',
          abs(g - 0.03275) < 1e-9, '%.6f' % g)
    # 而指尖确实进了块的范围（否则不算抓）：指尖点在块体内
    tip = sm.fingertip(v0['pose'])
    check('指尖点落在块内（越过顶面 18.5 mm）',
          blk.center[2] - blk.half[2] < tip[2] < blk.center[2] + blk.half[2])

    # 两个腕部朝向是同一个几何（180° 绕 a 自映射）—— 只是 IK 分支不同
    def _corner_set(obbs):
        pts = [np.round(c, 9) for o in obbs for c in o.corners()]
        return sorted(tuple(p) for p in pts)

    o0 = sm.gripper_obbs([c for c in cands if c['vertical']][0]['pose'],
                         sm.GRIPPER['stroke_half'])
    o1 = sm.gripper_obbs([c for c in cands if c['vertical']][1]['pose'],
                         sm.GRIPPER['stroke_half'])
    check('variant 0/1 的夹爪占据同一空间（180° 绕 tool z 自映射）',
          _corner_set(o0) == _corner_set(o1))


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
    mut = _mut('    return max(0.0, separation(a, b))', '    return separation(a, b)')
    check('变异「重叠时给负数」：真实 0、改坏后 < 0',
          sm.gap(a, unit((0.05, 0, 0))) == 0.0
          and mut['gap'](a, unit((0.05, 0, 0))) < 0.0)

    # H. separation 被截成非负（退回 gap 的语义）：穿插就判不出来了
    mut = _mut('    return best', '    return max(0.0, best)')
    check('变异「separation 截掉负号」：真实穿插为负、改坏后接触与穿插都是 0',
          sm.separation(a, unit((0.05, 0, 0))) < 0.0
          and mut['separation'](a, unit((0.05, 0, 0))) == 0.0
          and mut['separation'](a, unit((0.1, 0, 0))) == 0.0)

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

    # E. 抓取深度符号写反：指尖会跑到块上方之外，位姿就抓空
    b = _block()
    top_z = b['z'] + b['size'] / 2.0
    v_real = [c for c in sm.grasp_candidates(b) if c['vertical']][0]
    real_tip = sm.fingertip(v_real['pose'])[2]
    mut = _mut('pos = face - a * (tip_z - grasp_depth)',
               'pos = face + a * (tip_z - grasp_depth)')
    v_bad = [c for c in mut['grasp_candidates'](b) if c['vertical']][0]
    bad_tip = mut['fingertip'](v_bad['pose'])[2]
    check('变异「抓取深度符号写反」：真实越面 18.5 mm、改坏后掉到台面下',
          abs(real_tip - (top_z - sm.GRASP_DEPTH)) < 1e-12
          and bad_tip < 0.0, str(bad_tip))

    # F. 「竖直向下」的判据写反：选出来的「B 档」位姿全跑到桌面底下
    mut = _mut('np.allclose(a, (0.0, 0.0, -1.0),',
               'np.allclose(a, (0.0, 0.0, 1.0),')
    real_v = [c for c in sm.grasp_candidates(b) if c['vertical']]
    bad_v = [c for c in mut['grasp_candidates'](b) if c['vertical']]
    check('变异「竖直判据写反」：真实 B 档 tool0 在块顶上方、改坏后全在台面下',
          len(real_v) == 4 and len(bad_v) == 4
          and all(c['pos'][2] > top_z + 0.15 for c in real_v)
          and all(c['pos'][2] < -0.1 for c in bad_v),
          'real=%s bad=%s' % (round(real_v[0]['pos'][2], 3) if real_v else None,
                              round(bad_v[0]['pos'][2], 3) if bad_v else None))

    # G. y 轴叉乘写反：旋转矩阵成镜像，四元数还原出的 tool z 不再指向块心
    mut = _mut('yv = np.cross(a, c)', 'yv = np.cross(c, a)')
    real_z = sm.quat_to_mat(v_real['quat'])[:, 2]
    bad_z = sm.quat_to_mat(
        [c for c in mut['grasp_candidates'](b) if c['vertical']][0]['quat'])[:, 2]
    check('变异「y 轴叉乘写反」：真实 tool z=(0,0,−1)、改坏后不再指向块心',
          np.allclose(real_z, [0, 0, -1], atol=1e-12)
          and not np.allclose(bad_z, [0, 0, -1], atol=1e-6),
          'bad_z=%s' % np.round(bad_z, 3))


def section_best_hit():
    print('\n-- 抓取候选选择：竖直 4 候选按离邻块间隙取最大（2026-11-17）')
    # 贴合对真机实证：closing 0 的 partner 间隙 1.6 mm（空夹/别住），
    # closing 1 的 20 mm —— 必须选后者。
    near = {"axis": 2, "sign": -1, "closing": 0, "variant": 0,
            "per_other": {"1": 0.0016, "2": 0.05}}
    far = {"axis": 2, "sign": -1, "closing": 1, "variant": 0,
           "per_other": {"1": 0.020, "2": 0.05}}
    check('间隙大的候选胜出（不再选贴 partner 的 closing）',
          sm.pick_best_hit([near, far], vertical_only=True) is far)
    check('单候选原样返回',
          sm.pick_best_hit([near], vertical_only=True) is near)
    check('空表 ⇒ None', sm.pick_best_hit([]) is None)
    a = {"closing": 0, "gap_others_mm": 0.002}
    b = {"closing": 1, "gap_others_mm": 0.030}
    check('无 per_other ⇒ 用 gap_others_mm',
          sm.pick_best_hit([a, b], vertical_only=True) is b)
    check('vertical_only=False（E7 24 候选）⇒ 保持原序',
          sm.pick_best_hit([near, far], vertical_only=False) is near)
    t1 = {"closing": 0, "per_other": {"1": 0.02}}
    t2 = {"closing": 1, "per_other": {"1": 0.02}}
    check('间隙平手 ⇒ 回落到传入顺序（prefer_closing 仍生效）',
          sm.pick_best_hit([t1, t2], vertical_only=True) is t1)


def main():
    print('=' * 68)
    section_quat()
    section_rpy()
    section_interp()
    section_gap()
    section_separation()
    section_margin()
    section_scene()
    section_gripper()
    section_mat_quat()
    section_grasp()
    section_best_hit()
    section_urdf_sync()
    section_mutation()
    print('\n%d PASS  %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('未通过：' + '; '.join(FAILS))
    return 1 if N_FAIL else 0


if __name__ == '__main__':
    sys.exit(main())

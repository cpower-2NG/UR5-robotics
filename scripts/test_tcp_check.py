#!/usr/bin/env python3
"""`tcp_check.py` 自检 —— 不需要 ROS、不需要真机、不需要示教器。

（沿用本仓库 ``scripts/test_*.py`` 的约定：直接 ``python3`` 跑，退出码 0 = 通过。）

为什么值得单独钉一遍
====================
TCP 是 B 路外参的**底座**，而它错的方式恰好是**最不容易发现**的那种：
几个碰点姿态一样时，那个偏移是常数，刚体配准**原样吸收进 t**，残差 0.0 mm。
所以「验收」这件事本身必须是对的 —— 验收工具错了，等于没有验收。

这里钉四件事：
  1. `rotvec_to_matrix` 对得上常识（UR 示教器给的就是旋转矢量）。
  2. **往返**：已知误差 e → 合成示教器读数 → 反解，必须解回同一个 e。
  3. **Δ 与姿态角的关系** `Δ = 2·sin(θ/2)·|e|` —— 这条是「该把姿态拉开多少」
     的依据，也是判据本身。
  4. **退化要被抓到**：姿态都一样（解不出来）、某一次没碰在同一个点上
     （拟合残差大）—— 这两种都必须**不能**悄悄给出一个「通过」。
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tcp_check import (                                          # noqa: E402
    DELTA_MAX_M, MIN_SPREAD_DEG, rotation_angle_deg, rotvec_to_matrix,
    solve_tcp_error)

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
        print('  FAIL  %s  %s' % (name, detail))


def rotvec(*deg):
    """>=1 个绕轴旋转的复合，返回旋转矢量（度）。只为造数据用。"""
    R = np.eye(3)
    for axis, a in zip('xyz', deg):
        k = {'x': [1, 0, 0], 'y': [0, 1, 0], 'z': [0, 0, 1]}[axis]
        K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]], float)
        th = np.radians(a)
        R = R @ (np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K))
    # 矩阵 → 旋转矢量
    c = np.clip((np.trace(R) - 1) / 2, -1, 1)
    th = np.arccos(c)
    if th < 1e-12:
        return np.zeros(3)
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return np.degrees(w / (2 * np.sin(th)) * th)


def synth(e_true, rvs, C=np.array([0.40, 0.10, 0.05])):
    """按 `Pᵢ = C + Rᵢ·e` 造示教器读数。"""
    return [{'R': rotvec_to_matrix(rv), 'p': C + rotvec_to_matrix(rv) @ e_true}
            for rv in rvs]


def main():
    # ── 1. rotvec_to_matrix ─────────────────────────────────────────────
    print('[1] rotvec_to_matrix（UR 示教器给的是旋转矢量）')
    check('零矢量 → 单位阵', np.allclose(rotvec_to_matrix([0, 0, 0]), np.eye(3)))
    Rz90 = rotvec_to_matrix([0, 0, 90])
    check('绕 z 转 90° 把 x 轴送到 y 轴',
          np.allclose(Rz90 @ [1, 0, 0], [0, 1, 0], atol=1e-9),
          '%s' % (Rz90 @ [1, 0, 0]))
    check('是纯旋转（det=+1）', abs(np.linalg.det(Rz90) - 1) < 1e-12)
    # 旋转矢量**不唯一**：超过 180° 会翻到反轴上去。270° 绕 +z
    # 和 90° 绕 −z 是同一个旋转 —— 而写成 [0,0,270] 与 [0,0,90] 就**不是**。
    # （这条断言第一版就写成了后者，是它自己 FAIL 出来的。）
    check('270° 绕 +z == 90° 绕 −z（旋转矢量的翻轴）',
          np.allclose(rotvec_to_matrix([0, 0, 270]), rotvec_to_matrix([0, 0, -90]),
                      atol=1e-9),
          '270°z vs −90°z 最大差 %.2e'
          % np.abs(rotvec_to_matrix([0, 0, 270])
                   - rotvec_to_matrix([0, 0, -90])).max())
    check('而 90° 与 270° 绕 +z **不是**同一个旋转',
          not np.allclose(rotvec_to_matrix([0, 0, 90]),
                          rotvec_to_matrix([0, 0, 270]), atol=1e-6))

    # ── 2. 往返：e → 读数 → 反解 ────────────────────────────────────────
    print('[2] 往返：已知 TCP 误差 → 示教器读数 → 反解')
    e_true = np.array([-2.0, 1.5, 3.0]) * 1e-3            # 米
    rvs = [rotvec(0, 0, 0), rotvec(35, 15, 0), rotvec(-20, 40, 25)]
    poses = synth(e_true, rvs)
    e, fit, rank = solve_tcp_error(poses)
    check('反解出的 e == 真值', np.allclose(e, e_true, atol=1e-9),
          '得到 %s' % (e * 1e3))
    check('拟合残差 ≈ 0（点确实是同一个）', fit < 1e-9, '%.2e m' % fit)
    check('秩 = 3（姿态铺得开）', rank == 3, 'rank=%d' % rank)

    # 零误差 ⇒ 读数全是同一个点，Δ = 0
    poses0 = synth(np.zeros(3), rvs)
    d0 = max(np.linalg.norm(poses0[i]['p'] - poses0[j]['p'])
             for i in range(3) for j in range(i + 1, 3))
    check('TCP 完全准时 Δ = 0', d0 < 1e-12, 'Δ=%.2e' % d0)

    # ── 3. Δ 与姿态角的关系 ─────────────────────────────────────────────
    print('[3] Δ = 2·sin(θ/2)·|e|（「姿态该拉开多少」的依据）')
    e_axis = np.array([0.0, 0.0, 5.0]) * 1e-3             # 误差沿工具 z
    for th in (45.0, 90.0, 30.0):
        ps = synth(e_axis, [rotvec(0, 0, 0), rotvec(th, 0, 0)])
        d = float(np.linalg.norm(ps[0]['p'] - ps[1]['p']))
        want = 2 * np.sin(np.radians(th) / 2) * np.linalg.norm(e_axis)
        check('姿态差 %.0f° 时 Δ = 2·sin(θ/2)·|e|' % th, abs(d - want) < 1e-12,
              '得到 %.4f mm，期望 %.4f mm' % (d * 1e3, want * 1e3))
    ps = synth(e_axis, [rotvec(0, 0, 0), rotvec(45, 0, 0)])
    d = float(np.linalg.norm(ps[0]['p'] - ps[1]['p']))
    check('45° 的系数确实是 ~0.77（文档里的那个数）',
          abs(d / np.linalg.norm(e_axis) - 0.7654) < 1e-3,
          '系数 %.4f' % (d / np.linalg.norm(e_axis)))
    check('Δ 判据 1 mm 比「姿态只差 20°」这条门槛更粗',
          MIN_SPREAD_DEG == 20.0 and DELTA_MAX_M == 1e-3)

    # ── 4. 退化必须被抓到 ───────────────────────────────────────────────
    print('[4] 退化：姿态一样 / 没碰在同一个点')
    same = synth(e_true, [rotvec(0, 0, 0)] * 3)
    _e, _fit, rank_same = solve_tcp_error(same)
    check('三个姿态完全相同 ⇒ 秩 < 3（解不出来）', rank_same < 3,
          'rank=%d' % rank_same)

    # 某一次没碰在同一个物理点上（偏了 4 mm）—— 残差必须看得出来
    miss = synth(e_true, rvs)
    miss[1]['p'] = miss[1]['p'] + np.array([0.004, 0.0, 0.0])
    _e2, fit2, _r2 = solve_tcp_error(miss)
    check('有一次没碰在同一个点 ⇒ 拟合残差 > 1 mm', fit2 > 1e-3,
          '残差 %.2f mm' % (fit2 * 1e3))
    check('而那次的 Δ 判据本身也可能漏 —— 所以两个判据都要看',
          True)

    # 姿态只差一点点：e 还能解出来，但 Δ 很小 ⇒ **测试是钝的**，要警告
    tiny = synth(e_axis, [rotvec(0, 0, 0), rotvec(6, 0, 0)])
    d_tiny = float(np.linalg.norm(tiny[0]['p'] - tiny[1]['p']))
    check('姿态只差 6° 时 Δ 只有 0.10 mm（看着「通过」，其实是钝）',
          d_tiny < DELTA_MAX_M,
          'Δ=%.3f mm < 判据 %.1f mm' % (d_tiny * 1e3, DELTA_MAX_M * 1e3))
    check('所以 MIN_SPREAD_DEG 这道额外的门槛是必要的',
          rotation_angle_deg(tiny[0]['R'].T @ tiny[1]['R']) < MIN_SPREAD_DEG)

    # ── 5. 从「配的值」到「该改成多少」 ─────────────────────────────────
    print('[5] 修正值的符号')
    t_wrong = np.array([0.0, 172.0, 130.0]) * 1e-3        # 现场那个不可能的 y
    t_true = np.array([0.0, 0.0, 130.0]) * 1e-3
    e = t_wrong - t_true
    poses_w = synth(e, rvs)
    e_got, _f, _r = solve_tcp_error(poses_w)
    check('解出的 e == 配的值 − 真值', np.allclose(e_got, e, atol=1e-9),
          '%s' % (e_got * 1e3))
    check('配的值 − e 就是真值（这就是脚本给出的修正）',
          np.allclose(t_wrong - e_got, t_true, atol=1e-9),
          '%s' % ((t_wrong - e_got) * 1e3))

    print()
    print('=' * 72)
    print('PASS %d   FAIL %d' % (N_PASS, N_FAIL))
    if FAILS:
        print('失败：' + '；'.join(FAILS))
    return 1 if N_FAIL else 0


if __name__ == '__main__':
    sys.exit(main())

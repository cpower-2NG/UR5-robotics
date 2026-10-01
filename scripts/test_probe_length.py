#!/usr/bin/env python3
"""`probe_length.py` 自检 —— 不需要 ROS、不需要真机、不需要示教器。

（沿用本仓库 ``scripts/test_*.py`` 的约定：直接 ``python3`` 跑，退出码 0 = 通过。）

为什么值得单独钉一遍
====================
这个方法要**取代**示教器的四点法，而四点法坏就坏在**它不报残差** ——
解歪了看不出来。所以新方法必须自己**证明两件事**：

  1. **能算对**：已知 `t` → 合成示教器读数 → 解，必须解回同一个 `t`；
  2. **算不对的时候不能装成对的**：姿态没铺开、探针端点其实在游走 ——
     这两种都必须**出警告或拒绝出数**，而不是悄悄给一个看着挺像的答案。

第 2 条是重点。现场那 ~12 mm 的不一致，正是因为旧方法两条都没有。

顺带钉三条**姿态判据**（用户问的就是这个）：
  * 竖直朝下 ⟺ 旋转矢量 `rz == 0` **且** `|rv| == 180`；
  * `rz=0` 时 `倾角 = 180 − |rv|`；
  * 单次竖直法的灵敏度 ≈ `离轴距离 × π/180` ≈ 0.8 mm/度。
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from probe_length import (                                        # noqa: E402
    RESID_MAX_M, VERTICAL_TOL_DEG, solve, solve_180, solve_vertical_once,
    tool_axis_tilt_deg, two_pose_180)
from tcp_check import rotvec_to_matrix                            # noqa: E402

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
        print('  FAIL  %s   %s' % (name, detail))


def mk(rows):
    """把 (rv, z) 列表变成 solve() 要的 rows。"""
    return [mk_R(rotvec_to_matrix(rv), z, rv) for rv, z in rows]


def mk_R(R, z, rv=(0.0, 0.0, 0.0)):
    return {'R': R, 'p': np.array([0.0, 0.0, z]), 'rv': np.array(rv, float),
            'tilt': tool_axis_tilt_deg(R)}


def synth(t_mm, rvs, table_z_mm=0.0):
    """已知 t（毫米）→ 合成「零 TCP 碰台面」时示教器报的 z（毫米）。"""
    t = np.asarray(t_mm, float) * 1e-3
    rows = []
    for rv in rvs:
        R = rotvec_to_matrix(rv)
        z = table_z_mm * 1e-3 - (R @ t)[2]
        rows.append((rv, z))
    return mk(rows)


def main():
    # ── 1. 姿态判据：竖直朝下 ⟺ rz == 0 且 |rv| == 180 ────────────────
    print('[1] 竖直判据（用户问的就是这条）')
    for rv in ([180, 0, 0], [0, 180, 0], [127.3, 127.3, 0],
               [127.3, -127.3, 0], [-180, 0, 0]):
        check('rv=%-18s 竖直（rz=0, |rv|=180）' % (rv,),
              abs(tool_axis_tilt_deg(rotvec_to_matrix(rv))) < 0.05
              and abs(rv[2]) < 1e-9 and abs(np.linalg.norm(rv) - 180) < 0.05)
    for rv, why in (([180, 0, 10], 'rz≠0'), ([170, 0, 0], '|rv|≠180'),
                    ([150, 50, 20], '两个都不满足')):
        tl = tool_axis_tilt_deg(rotvec_to_matrix(rv))
        check('rv=%-18s 判为**不**竖直（%s）' % (rv, why), tl > 1.0,
              '倾角只算到 %.3f°' % tl)
    # rz=0 时 倾角 = 180 − |rv|
    check('rz=0 时 倾角 == 180 − |rv|',
          abs(tool_axis_tilt_deg(rotvec_to_matrix([170, 0, 0])) - 10.0) < 1e-6)
    # 灵敏度 ≈ 离轴距离 × π/180
    t_mm = np.array([44.54, -11.36, 194.55])
    r_off = float(np.hypot(t_mm[0], t_mm[1]))
    check('单次竖直法的灵敏度 ≈ 离轴 × π/180',
          abs(r_off * np.pi / 180 - 0.802) < 0.01,
          '%.4f mm/度' % (r_off * np.pi / 180))

    # ── 2. 往返：≥3 个铺开的姿态 ───────────────────────────────────────
    print()
    print('[2] 三点最小二乘 —— 往返')
    rvs3 = [[180, 0, 0], [150, 30, 10], [160, -20, -15]]
    rows = synth(t_mm, rvs3)
    t, _tz, resid, rank, _sv = solve(rows, 0.0)
    check('解回同一个 t', np.allclose(t * 1e3, t_mm, atol=1e-6),
          '解出 %s' % (t * 1e3))
    check('残差 ≈ 0', resid < 1e-9, '%.3e' % resid)
    check('秩 = 3', rank == 3)
    check('|t| == 199.91（与文档里 乙 的 |t| 对上）',
          abs(np.linalg.norm(t) * 1e3 - 199.91) < 0.01,
          '%.4f' % (np.linalg.norm(t) * 1e3))

    # ── 3. 单次竖直 ───────────────────────────────────────────────────
    print()
    print('[3] 单次竖直 —— t_z = z − table_z')
    rows1 = synth(t_mm, [[180, 0, 0]], table_z_mm=0.0)
    check('读数 z 就是 t_z', abs(rows1[0]['p'][2] * 1e3 - 194.55) < 1e-6,
          '%.4f' % (rows1[0]['p'][2] * 1e3))
    # 台面不在 0 时，必须**减掉** table_z
    t_v = solve_vertical_once(rows1[0], 0.0)
    check('单次竖直那条路解出 t_z', abs(t_v[2] * 1e3 - 194.55) < 1e-6,
          '%.4f' % (t_v[2] * 1e3))
    check('t_x/t_y 归 0（竖直时它们不影响高度）',
          t_v[0] == 0.0 and t_v[1] == 0.0)
    rows1b = synth(t_mm, [[180, 0, 0]], table_z_mm=-7.5)
    t1b, _z, _r, _rk, _sv = solve(rows1b, -7.5e-3)
    check('台面不在 0 时，减掉 table_z 仍解对',
          abs(t1b[2] * 1e3 - 194.55) < 1e-6, '%.4f' % (t1b[2] * 1e3))
    t_vb = solve_vertical_once(rows1b[0], -7.5e-3)
    check('单次竖直那条路，减掉 table_z 后也对',
          abs(t_vb[2] * 1e3 - 194.55) < 1e-6, '%.4f' % (t_vb[2] * 1e3))
    check('读数本身是 187.05（= 194.55 − 7.5）⇒ 不减 table_z 就偏 7.5 mm',
          abs(rows1b[0]['p'][2] * 1e3 - 187.05) < 1e-6,
          '%.4f' % (rows1b[0]['p'][2] * 1e3))

    # ── 4. 两姿态绕工具轴 180° ─────────────────────────────────────────
    print()
    print('[4] 两姿态绕工具轴 180° —— 把倾角的一阶项消掉')
    # 同一个倾斜姿态，绕工具轴再转 180°
    Rz = rotvec_to_matrix([0, 0, 180])          # 绕**工具** z 转 180°
    rvA = [150, 30, 10]
    RA = rotvec_to_matrix(rvA)
    RB = RA @ Rz                                 # 同一个工具，换了个绕轴自转
    t = t_mm * 1e-3
    rowsAB = [mk_R(RA, -(RA @ t)[2]), mk_R(RB, -(RB @ t)[2])]
    check('两姿态被认成「绕工具轴 180°」', two_pose_180(rowsAB) is True)
    check('两次的倾角一样（绕的就是工具轴）',
          abs(rowsAB[0]['tilt'] - rowsAB[1]['tilt']) < 1e-9,
          '%.6f vs %.6f' % (rowsAB[0]['tilt'], rowsAB[1]['tilt']))
    t180 = solve_180(rowsAB, 0.0)
    check('平均后解出 t_z（有倾角也算对）', abs(t180[2] * 1e3 - 194.55) < 1e-6,
          '%.4f' % (t180[2] * 1e3))
    check('而**单**取一次会差多少（这就是 180° 平均的价值）',
          abs(rowsAB[0]['p'][2] * 1e3 - 194.55) > 5.0,
          '单次偏 %.2f mm' % (rowsAB[0]['p'][2] * 1e3 - 194.55))
    # 单次法只在**真竖直**时才对；同样是这个倾斜姿态，单次法会错
    _t_v = solve_vertical_once(rowsAB[0], 0.0)
    check('同一姿态若用「单次竖直」那条路 ⇒ 错',
          abs(_t_v[2] * 1e3 - 194.55) > 5.0,
          '单次竖直解出 %.2f' % (_t_v[2] * 1e3))
    # 不平行的两姿态不能被误认
    rows_bad = mk([([180, 0, 0], 0.0), ([150, 30, 10], 0.0)])
    check('随便两个姿态**不会**被当成 180° 对',
          two_pose_180(rows_bad) is None)

    # ── 5. table_z 连它一起解 ──────────────────────────────────────────
    print()
    print('[5] 不给 table_z：≥4 个姿态连它一起解')
    rvs4 = [[180, 0, 0], [150, 30, 10], [160, -20, -15], [140, -35, 25]]
    rows4 = synth(t_mm, rvs4, table_z_mm=-12.0)      # 台面故意不在 0
    t4, tz4, resid4, rank4, _sv4 = solve(rows4, None)
    check('t 解对', np.allclose(t4 * 1e3, t_mm, atol=1e-6), '%s' % (t4 * 1e3))
    check('table_z 也解对（−12 mm）', abs(tz4 * 1e3 + 12.0) < 1e-6,
          '%.4f' % (tz4 * 1e3))
    check('秩 = 4', rank4 == 4)
    check('残差 ≈ 0', resid4 < 1e-9)

    # ── 6. 坏数据必须被抓住（这一节是重点） ────────────────────────────
    print()
    print('[6] 坏数据不许装成对的')
    # 6a. 姿态几乎一样 ⇒ 铺不开
    rows_same = synth(t_mm, [[180, 0, 0], [180, 0, 1.0], [179.5, 0.5, 0]])
    tilts = [r['tilt'] for r in rows_same]
    check('三个几乎同向的姿态：最大倾角仍很小 ⇒ 单次法也救不了',
          max(tilts) < VERTICAL_TOL_DEG * 2,
          '倾角 %s' % ['%.2f' % t for t in tilts])
    # 6b. **3 次碰不出残差** —— 3 个未知数、3 个方程，方阵，多歪都能精确拟合。
    #     这条要钉死：它是「示教器向导为什么看不出解歪了」的同一个原因。
    rows_d3 = synth(t_mm, rvs3)
    rows_d3[1]['p'][2] += 5e-3
    _t3, _z3, resid_3, _rk3, _sv3 = solve(rows_d3, 0.0)
    check('3 次碰 ⇒ 残差恒为 0，**再歪也看不出来**', resid_3 < 1e-12,
          '残差 %.3e' % resid_3)
    # 实测：一次 5 mm 的接触误差能让**解出来的 t 偏 10~30 mm**，而残差是 0.000000。
    check('而这一次 5 mm 的扰动被整个吸收进解里（偏 10~30 mm）—— '
          '残差 0 **不等于**解对了',
          np.max(np.abs(_t3 * 1e3 - t_mm)) > 5.0,
          'Δt 最大 %.2f mm' % np.max(np.abs(_t3 * 1e3 - t_mm)))

    # 6c. 第 4 次一加进来，同一份坏数据立刻现形
    rows_d4 = synth(t_mm, rvs4)
    rows_d4[1]['p'][2] += 5e-3
    rows_d4[2]['p'][2] -= 4e-3
    _t4, _z4, resid_4, _rk4, _sv4 = solve(rows_d4, 0.0)
    check('端点游走 ⇒ 第 4 次一进来残差就超判据（会红、会被拒绝）',
          resid_4 > RESID_MAX_M,
          '残差 %.3f mm vs 判据 %.1f mm' % (resid_4 * 1e3, RESID_MAX_M * 1e3))
    # 同理：光加姿态、不动数据，残差仍然 ≈ 0
    _t5, _z5, resid_5, _rk5, _sv5 = solve(synth(t_mm, rvs4), 0.0)
    check('干净数据下，4 次碰的残差仍 ≈ 0（判据不是空转）',
          resid_5 < 1e-9, '%.3e' % resid_5)

    print()
    print('=' * 72)
    print('PASS %d   FAIL %d' % (N_PASS, N_FAIL))
    if FAILS:
        print('失败：' + '；'.join(FAILS))
    return 1 if N_FAIL else 0


if __name__ == '__main__':
    sys.exit(main())

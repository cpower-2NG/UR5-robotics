#!/usr/bin/env python3
"""用**台面**当基准，量探针（针尖 / 指尖）到法兰的距离 —— **不经过任何四点法解算**。

为什么要有这么一个东西
======================
现在要定的是 `tool0_to_fingertip`（规划器用它把法兰放到 `wz + tool0_to_fingertip`，
好让指尖落在 `wz`）。它被卡住的经过：

* 示教器四点法给指尖 **173.99**、给针尖 **194.55**；
* 两者反推针尖伸出 **20.56 mm**，而卷尺量出来只有 **7~8 mm** ⇒ **差 ~12 mm**；
* 至少有一个是歪的 —— 但**两个都是同一个向导解出来的**，它不报残差，
  所以从示教器上**看不出**是哪一次解歪了。

这里换一条**不经过四点法**的路：把 TCP 设成 **(0,0,0)**，示教器报的 Base 位姿
就**直接是法兰中心**；然后拿**台面**当基准去碰。台面是个大平面，每条读数都
只用到"碰上了"这一个事实，不需要在纸上找同一个笔点。

数学
====
探针端点在**法兰系**里的坐标记作 `t`（就是我们要的）。法兰→base 的旋转记作 `R`
（示教器上 `TCP=(0,0,0)` 时显示的那组 `rx/ry/rz` 就是它）。碰台面时：

    法兰中心的高度 + (R·t)_z = 台面高度
    zᵢ + (Rᵢ·t)_z = table_z

展开成三个未知数 `t_x, t_y, t_z` 的**线性**方程：

    Rᵢ[2,0]·t_x + Rᵢ[2,1]·t_y + Rᵢ[2,2]·t_z = table_z − zᵢ

* **碰 ≥3 个姿态**（朝向要铺开）⇒ 最小二乘解出 `t`，**残差**就是这个解的可信度
  —— 这是示教器向导**不给**的那一项。
* **碰 1 个姿态、且工具轴严格竖直** ⇒ `R[2,:] = [0, 0, −1]`，方程退化成
  `t_z = z − table_z`，一次就够。**这是最省事的用法，但姿态必须准。**
* **碰 2 个姿态、两个绕工具轴差 180°** ⇒ `R[2,0]/R[2,1]` 反号、`R[2,2]` 不变，
  两式相加正好把 `t_x/t_y` 消掉：`t_z = (z₁+z₂)/2 − table_z`（再除以 cos 倾角）。
  **任意倾角都成立** —— 姿态调不准时的兜底。

台面高度 `table_z`
==================
三种来源，优先级从高到低：

1. 记录里直接给 `table_z`（**推荐**）。怎么得到：把 TCP 切回 **TCP_1**，
   用它碰一下台面，读 Base 的 **z** —— 那时 TCP 就在台面上，所以那个读数
   **就是** `table_z`，而且**与姿态无关**。
   （臂如果是平放在工作台面上拧死的，这个读数通常就是 **0**。）
2. 不给 `table_z`，但**同一个探针碰了 ≥4 个朝向铺开的姿态** ⇒ 把 `table_z`
   当成第 4 个未知数一起解出来。**完全自足，不假设台面在哪。**
3. 都不满足 ⇒ 脚本拒绝出数（会说明缺什么），**不猜**。

用法
====
    python3 scripts/probe_length.py --data probe.json

`probe.json`（`docs/calib/probe_length.template.json` 是一份空模板）：

    {
      "unit": "mm",
      "table_z": 0.0,
      "probes": [
        {"name": "针尖", "touches": [
            {"note": "竖直",  "xyz": [x, y, z], "rpy": [rx, ry, rz]}
        ]},
        {"name": "指尖", "touches": [
            {"note": "A", "xyz": [...], "rpy": [...]},
            {"note": "B 绕工具轴转180°", "xyz": [...], "rpy": [...]}
        ]}
      ]
    }

`xyz` 单位随 `unit`（**必须是 TCP=(0,0,0) 下读的**，也就是法兰中心）；
`rpy` 一律是**度**，而且是 UR 示教器上的**旋转矢量（轴角）**，不是欧拉角。
"""

import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tcp_check import rotvec_to_matrix                        # noqa: E402

# 竖直判据：工具轴与"正下方"差多少度以内就算竖直（超过就别用单次法）
VERTICAL_TOL_DEG = 2.0
# 拟合残差判据（米）。超过它 = "探针端点是个固定点"这个前提没成立
RESID_MAX_M = 1e-3
# 残差小，但姿态铺得不够开时，解仍然可能是病态的 —— 看最小奇异值。
# 这是给"姿态没铺开"用的，单位是"每毫米误差"，越大越安全。
COND_WARN = 0.05


def tool_axis_tilt_deg(R):
    """工具轴（法兰 +z）偏离"正下方"多少度。0 = 严格竖直朝下。"""
    a = R[:, 2]
    return float(np.degrees(np.arccos(np.clip(-a[2], -1.0, 1.0))))


def parse(doc):
    unit = doc.get('unit', 'mm')
    if unit not in ('m', 'mm'):
        raise SystemExit('unit 只能是 m 或 mm，拿到 %r' % unit)
    scale = 1e-3 if unit == 'mm' else 1.0
    probes = doc.get('probes')
    if not isinstance(probes, list) or not probes:
        raise SystemExit('缺 probes')

    out = []
    for k, pr in enumerate(probes):
        if not isinstance(pr, dict):
            raise SystemExit('probes[%d] 不是一个对象' % k)
        touches = pr.get('touches')
        if not isinstance(touches, list) or not touches:
            raise SystemExit('probes[%d] 缺 touches' % k)
        rows = []
        for i, t in enumerate(touches):
            if not isinstance(t, dict):
                raise SystemExit('probes[%d].touches[%d] 不是一个对象' % (k, i))
            try:
                p = [float(v) for v in t['xyz']]
                rv = [float(v) for v in t['rpy']]
            except (KeyError, TypeError, ValueError):
                raise SystemExit('probes[%d].touches[%d] 的 xyz/rpy 要各 3 个数 —— '
                                 '**这一行没填**？' % (k, i))
            if len(p) != 3 or len(rv) != 3:
                raise SystemExit('probes[%d].touches[%d] 的 xyz/rpy 要各 3 个数'
                                 % (k, i))
            R = rotvec_to_matrix(rv)
            rows.append({'note': t.get('note', '碰%d' % (i + 1)),
                         'p': np.array(p, float) * scale,
                         'rv': np.array(rv, float),
                         'R': R,
                         'tilt': tool_axis_tilt_deg(R)})
        out.append({'name': pr.get('name', 'probe%d' % k), 'rows': rows})
    return unit, out


def spread_deg(rows):
    """这几次碰姿态之间最多差多少度。"""
    out = []
    for i in range(len(rows)):
        for j in range(i + 1, len(rows)):
            c = (np.trace(rows[i]['R'].T @ rows[j]['R']) - 1.0) / 2.0
            out.append(float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0)))))
    return max(out) if out else 0.0


def solve(rows, table_z):
    """最小二乘解 `t`（法兰系里探针端点到法兰的向量）。

    :param rows: `[{'R','p','tilt',...}]`
    :param table_z: 台面高度（米）；给 `None` 就连它一起解（需 ≥4 个姿态）
    :returns: `(t (3,), table_z, 残差, 秩, 最小奇异值)`
    """
    A = np.vstack([r['R'][2, :] for r in rows])
    if table_z is None:
        # 未知数变成 [tx, ty, tz, table_z]：`R·t − table_z = −p_z`
        A = np.hstack([A, -np.ones((len(rows), 1))])
        b = -np.array([r['p'][2] for r in rows])
    else:
        b = np.array([table_z - r['p'][2] for r in rows])
    sol, _res, rank, sv = np.linalg.lstsq(A, b, rcond=None)
    resid = float(np.linalg.norm(A @ sol - b))
    sv_min = float(sv[-1]) if len(sv) else 0.0
    t = sol[:3]
    tz = table_z if table_z is not None else float(sol[3])
    return t, tz, resid, int(rank), sv_min


def solve_vertical_once(row, table_z):
    """单次竖直：`R[2,:] = [0, 0, −1]` ⇒ `t_z = z − table_z`。`t_x/t_y` 定不了。"""
    return np.array([0.0, 0.0, row['p'][2] - table_z])


def solve_180(rows, table_z):
    """两姿态、绕工具轴差 180°：`t_x/t_y` 在平均里消掉，只剩 `t_z`。

    `R[2,0]/R[2,1]` 反号、`R[2,2]` 不变 ⇒ `z₁+z₂ = 2·table_z − 2·t_z·R[2,2]`。
    """
    R22 = float(rows[0]['R'][2, 2])
    if abs(R22) < 1e-6:
        return None
    z_mid = float(np.mean([r['p'][2] for r in rows]))
    return np.array([0.0, 0.0, (table_z - z_mid) / R22])


def two_pose_180(rows):
    """这两次碰是不是「绕工具轴差 ~180°」？（是才可以用 `solve_180`）

    条件不好判（180° 是个不动点），所以这里**只在正好 2 次**时用，
    并且要求 `R[2,2]` 两次一致（差 < 0.01）—— 那是"绕的是工具轴"的必要条件。
    """
    if len(rows) != 2:
        return None
    if abs(rows[0]['R'][2, 2] - rows[1]['R'][2, 2]) > 0.01:
        return None
    # R[2,0] / R[2,1] 应当反号 —— 这是"绕工具轴转了 180°"的判据
    if np.linalg.norm(rows[0]['R'][2, :2] + rows[1]['R'][2, :2]) > 0.02 * \
            max(1.0, float(np.linalg.norm(rows[0]['R'][2, :2]))):
        return None
    return True


def report_probe(pr, table_z, unit, table_z_from):
    rows = pr['rows']
    k = 1e3 if unit == 'mm' else 1.0
    print()
    print('━' * 72)
    print('探针 **%s** —— 碰了 %d 次' % (pr['name'], len(rows)))
    for r in rows:
        print('  %-22s xyz = [%9.3f %9.3f %9.3f]  rpy = [%8.3f %8.3f %8.3f]  '
              '倾角 %6.3f°' % (r['note'], *(r['p'] * k), *r['rv'], r['tilt']))

    sp = spread_deg(rows)
    vert = [r for r in rows if r['tilt'] <= VERTICAL_TOL_DEG]
    t = resid = None
    rank = 0
    sv_min = 0.0
    method = ''

    if len(rows) >= 3 and sp >= 20.0:
        method = '最小二乘（%d 次，姿态最大差 %.1f°）' % (len(rows), sp)
        t, tz_used, resid, rank, sv_min = solve(rows, table_z)
    elif two_pose_180(rows):
        method = '两姿态绕工具轴 180° 取平均（不需要 t_x/t_y）'
        if abs(float(rows[0]['R'][2, 2])) < 1e-6:
            print('  ⚠️ R[2,2] ≈ 0 —— 工具轴**横着**，这组数据用不了这个办法。')
            return None
        t = solve_180(rows, table_z)
        resid = 0.0
        print('  ⚠️ 这个办法**只解得 t_z**（t_x/t_y 被平均消掉了）—— '
              '对"探针伸出去多长"够用。')
    elif len(rows) == 1 and vert:
        method = '单次 + 竖直（倾角 %.2f° ≤ %.1f°）' % (vert[0]['tilt'],
                                                        VERTICAL_TOL_DEG)
        t = solve_vertical_once(vert[0], table_z)
        resid = 0.0
        print('  ⚠️ 单次竖直只解得 t_z，t_x/t_y 按 0 处理（竖直时它们本来就'
              '不影响高度）。')
    else:
        why = []
        if len(rows) < 3:
            why.append('只碰了 %d 次（要 ≥3，或 2 次绕工具轴差 180°）' % len(rows))
        if sp < 20.0:
            why.append('姿态最大只差 %.1f° < 20° （铺不开 ⇒ 解病态）' % sp)
        if not vert:
            why.append('没有一次是竖直的（最竖直的那次差 %.1f° > %.1f°）'
                       % (min(r['tilt'] for r in rows), VERTICAL_TOL_DEG))
        print('  ❌ 这组数据**解不出来**：%s' % '；'.join(why))
        return None

    if table_z is None and len(rows) < 4:
        print('  ❌ 记录里没给 `table_z`，而这组只有 %d 次 —— 不够连它一起解。'
              % len(rows))
        return None

    print()
    print('  解算办法：%s' % method)
    if table_z is None:
        _t, tz, _r, _rk, _sv = solve(rows, None)
        print('  台面高度 table_z = **%.3f mm**（连它一起解出来的，%s）'
              % (tz * k, table_z_from))
    print('  t = [%8.3f %8.3f %8.3f] mm   |t| = %.2f mm'
          % (*(t * k), np.linalg.norm(t) * k))
    if method.startswith('最小二乘'):
        print('  最小奇异值 = %.4f' % sv_min)
        if len(rows) <= 3:
            # 3 个未知数、3 个方程 ⇒ 方阵，**怎么歪都能精确拟合**，残差恒为 0。
            print('  ⚠️ **只碰了 %d 次 ⇒ 算不出残差**（3 个未知数、%d 个方程，'
                  '方阵，多歪都能精确拟合）。' % (len(rows), len(rows)))
            print('     残差正是示教器向导**不给**、而这次出事**就缺**的那一项 ——')
            print('     **再碰一次**（第 4 个姿态，换一个朝向）就有了。')
        else:
            print('  拟合残差 = %.3f mm' % (resid * k))
            if resid > RESID_MAX_M:
                print('  ❌ **残差超了 %.1f mm** ——「探针端点是个固定点」这个'
                      '前提没成立。' % (RESID_MAX_M * k))
                print('     探针是钝面（接触点在面上游走）？还是绑松了/被顶跑了？')
                print('     这个数**不能拿去用**。')
            elif sv_min < COND_WARN:
                print('  ⚠️ 残差小，但最小奇异值 %.4f 偏小 —— 姿态是铺开了，'
                      '朝向的**方向**可能仍偏单调。' % sv_min)
    return {'t': t, 'resid': resid, 'method': method, 'spread': sp,
            'n': len(rows)}


def main():
    ap = argparse.ArgumentParser(
        description='用台面当基准量探针到法兰的距离（不经过四点法）',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument('--data', required=True, help='见文件头的数据格式')
    args = ap.parse_args()

    with open(args.data) as fh:
        doc = json.load(fh)
    unit, probes = parse(doc)
    k = 1e3 if unit == 'mm' else 1.0
    table_z = doc.get('table_z')
    table_z = None if table_z is None else float(table_z) * (1e-3 if unit == 'mm'
                                                             else 1.0)
    tz_src = doc.get('table_z_from', '记录里给的')

    print('单位 %s，%d 个探针' % (unit, len(probes)))
    if table_z is not None:
        print('台面高度 table_z = %.3f mm（%s）' % (table_z * k, tz_src))
    else:
        print('记录里没给 table_z —— 会尝试连它一起解（每个探针要 ≥4 次）')

    got = {}
    for pr in probes:
        r = report_probe(pr, table_z, unit, tz_src)
        if r:
            got[pr['name']] = r

    # ── 汇总 ────────────────────────────────────────────────────────────
    print()
    print('=' * 72)
    if len(got) >= 2:
        names = list(got)
        print('两个探针之间（**这是最不依赖绝对基准的一个数**）：')
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                d = got[names[j]]['t'] - got[names[i]]['t']
                print('  %s − %s = [%8.3f %8.3f %8.3f] mm，沿工具轴 %+.3f mm'
                      % (names[j], names[i], *(d * k), d[2] * k))
        print('  ⚠️ 倾角各差 1° 时，t_x/t_y 那两项会串进 z（≈0.8 mm/度）——')
        print('     上面这两个探针的**姿态各自差多少度**，决定了这个差值可不可信。')

    ref = doc.get('reference')
    if isinstance(ref, dict):
        ref_t = np.array([float(v) for v in ref['xyz']]) * \
            (1e-3 if unit == 'mm' else 1.0)
        print()
        print('对照 `%s` = [%.3f %.3f %.3f] mm（%s）：'
              % (ref.get('name', 'reference'), *(ref_t * k),
                 ref.get('what', '示教器上配着的那个 TCP')))
        for nm, r in got.items():
            if nm != ref.get('name'):
                d = r['t'] - ref_t
                print('  %s − 对照 = [%8.3f %8.3f %8.3f] mm，|差| = %.3f mm'
                      % (nm, *(d * k), np.linalg.norm(d) * k))

    print()
    bad = 0
    for nm, r in got.items():
        if r['n'] <= 3:
            flag, tail = '⚠️', '**没有残差可看**（碰 %d 次）' % r['n']
        elif r['resid'] > RESID_MAX_M:
            flag, tail = '❌', '残差 %.3f mm **超判据**' % (r['resid'] * k)
            bad += 1
        else:
            flag, tail = '✅', '残差 %.3f mm' % (r['resid'] * k)
        print('%s %-8s  t_z = %8.3f mm   （%s；%s）'
              % (flag, nm, r['t'][2] * k, tail, r['method']))
    return 1 if (not got or bad) else 0


if __name__ == '__main__':
    sys.exit(main())

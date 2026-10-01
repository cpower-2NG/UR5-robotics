#!/usr/bin/env python3
"""TCP 标定**验收**：同一个物理点，换几个姿态各碰一次，看报出来的坐标散不散。

为什么要有这么一个东西
======================
TCP 是整条 B 路外参的**底座**。TCP 错了，碰出来的每个点都带同一个
（在**法兰系**里的）偏移；而那个偏移会随姿态**转**，于是：

* 若几个姿态**一样** —— 偏移是个常数，刚体配准**原样吸收进 t**，
  残差照样是 0.0 mm，**看不出来**；
* 若姿态**不同** —— 同一个物理点在示教器上会报出**不同的坐标**。

所以验收方法只有一条：**同一个笔点，换一个角度再碰一次，比两次报的坐标**。

    Δ = |P₁ − P₂|

Δ 差不多就是 TCP 误差本身（姿态差 45° 时系数约 0.77，见下）。

⚠️ 光把 TCP_1 的六个数抄给我**不算验收** ——
那只能查「这个值离不离谱」，查不出「对不对」。要判断对错，必须有第二个姿态。

用法
====
    python3 scripts/tcp_check.py --poses tcp_poses.json

`tcp_poses.json` 长这样（`docs/calib/tcp_poses.template.json` 是一份空模板）：

    {
      "tcp_name": "TCP_1",
      "unit": "mm",
      "tcp_xyz": [x, y, z],          // 示教器上 TCP_1 **现在配的**值；给了才能算修正值
      "tcp_rpy": [0, 0, 0],          // TCP 自身的姿态，4 点法标出来的一般是 0
      "poses": [
        {"note": "姿态①", "xyz": [...], "rpy": [...]},
        {"note": "姿态②（转了 ~45°）", "xyz": [...], "rpy": [...]}
      ]
    }

`xyz` 单位随 `unit`；`rpy` 一律是**度**，而且是 UR 示教器上的
**旋转矢量（轴角）**，不是欧拉角 —— UR 的 Rx/Ry/Rz 就是这个。
搞混了不会报错，只会让解出来的修正值歪掉，所以这里按旋转矢量处理。

除了 Δ，它还会**反解出 TCP 误差向量**并给出**该改成多少**
（两个以上姿态时 `(Rᵢ − Rⱼ)·e = Pᵢ − Pⱼ` 是可解的）——
于是这变成一个**收敛**的过程：改完再碰一次，Δ 应当塌到 1 mm 以内。
"""

import argparse
import json
import os
import sys

import numpy as np

# Δ 的判据（米）。超过它 = TCP 还没标好，别拿它去碰靶。
DELTA_MAX_M = 1e-3

# 两个姿态差多少度以下就算「没验」——角度越小这个测试越钝
# （|Δ| = 2·sin(θ/2)·|e|，θ=45° 时 0.765，θ=10° 时只剩 0.174）。
MIN_SPREAD_DEG = 20.0


def rotvec_to_matrix(rv_deg):
    """旋转矢量（轴角，度）→ 3×3。Rodrigues。"""
    rv = np.asarray(rv_deg, float)
    ang = np.linalg.norm(rv)
    if ang < 1e-12:
        return np.eye(3)
    k = rv / ang
    th = np.radians(ang)
    K = np.array([[0, -k[2], k[1]],
                  [k[2], 0, -k[0]],
                  [-k[1], k[0], 0]], float)
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)


def rotation_angle_deg(R):
    """两个旋转之间差多少度。"""
    c = (np.trace(R) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def parse_poses(doc):
    unit = doc.get('unit', 'mm')
    if unit not in ('m', 'mm'):
        raise SystemExit('unit 只能是 m 或 mm，拿到 %r' % unit)
    scale = 1e-3 if unit == 'mm' else 1.0
    poses = doc.get('poses')
    if not isinstance(poses, list) or not poses:
        raise SystemExit('缺 poses（要一组姿态，每个至少给 xyz 和 rpy）')

    def vec(row, key, i, n):
        if not isinstance(row, (list, tuple)) or len(row) != n:
            raise SystemExit('poses[%d] 的 %s 要 %d 个数，拿到 %r' % (i, key, n, row))
        try:
            v = [float(x) for x in row]
        except (TypeError, ValueError):
            raise SystemExit('poses[%d] 的 %s = %r 不是数字 —— **这一行没填**？'
                             % (i, key, row))
        if not all(np.isfinite(v)):
            raise SystemExit('poses[%d] 的 %s 里含 nan/inf' % (i, key))
        return np.array(v, float)

    out = []
    for i, p in enumerate(poses):
        if not isinstance(p, dict):
            raise SystemExit('poses[%d] 不是一个对象' % i)
        out.append({'note': p.get('note', '姿态%d' % (i + 1)),
                    'p': vec(p.get('xyz'), 'xyz', i, 3) * scale,
                    'rv': vec(p.get('rpy'), 'rpy', i, 3)})
    return unit, out


def solve_tcp_error(poses):
    """从一组「同一个物理点、不同姿态」的记录里反解 TCP 误差向量。

    设物理点是 `C`，示教器报的是 `Pᵢ = C + Rᵢ·e`（`e` = 配的值 − 真值，
    在**法兰系**里）。两两相减消掉 `C`：

        (Rᵢ − Rⱼ)·e = Pᵢ − Pⱼ

    全配对堆起来最小二乘。三个以上姿态时是过定的，**拟合残差**因此可以当质检：
    残差大 = 「碰的是同一个物理点」这个前提没成立。

    :param poses: `[{'R': (3,3) 法兰姿态, 'p': (3,) 示教器报的坐标}, ...]`，≥2 个
    :returns: `(e (3,), 拟合残差 (标量), 秩)`
    """
    A, b = [], []
    for i in range(len(poses)):
        for j in range(i + 1, len(poses)):
            A.append(poses[i]['R'] - poses[j]['R'])
            b.append(poses[i]['p'] - poses[j]['p'])
    A = np.vstack(A)
    b = np.concatenate(b)
    e, _res, rank, _sv = np.linalg.lstsq(A, b, rcond=None)
    return e, float(np.linalg.norm(A @ e - b)), int(rank)


def main():
    ap = argparse.ArgumentParser(
        description='TCP 标定验收：同一个点换姿态碰，比坐标散不散',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument('--poses', required=True, help='见文件头的数据格式')
    ap.add_argument('--max-mm', type=float, default=DELTA_MAX_M * 1e3,
                    help='Δ 的判据（mm）')
    args = ap.parse_args()

    with open(args.poses) as fh:
        doc = json.load(fh)
    unit, poses = parse_poses(doc)
    thr = args.max_mm * 1e-3

    name = doc.get('tcp_name', '(未写 tcp_name)')
    print('TCP = %s，%d 个姿态，记录单位 %s' % (name, len(poses), unit))
    for q in poses:
        print('  %-18s P = [%9.3f %9.3f %9.3f]  rpy = [%8.3f %8.3f %8.3f]'
              % (q['note'], *(q['p'] * (1e3 if unit == 'mm' else 1.0)),
                 *q['rv']))

    # TCP 自身的姿态：示教器给的是 **TCP** 的姿态，而误差向量在**法兰**系里。
    # 4 点法标出来的 TCP 一般不带旋转（rpy 全 0），那两者就是同一个东西。
    tcp_rpy = np.array(doc.get('tcp_rpy', [0.0, 0.0, 0.0]), float)
    if np.linalg.norm(tcp_rpy) > 1e-9:
        R_tcp = rotvec_to_matrix(tcp_rpy)
        for q in poses:
            q['R'] = rotvec_to_matrix(q['rv']) @ R_tcp.T
        print('（TCP 自身带旋转，已按 tcp_rpy 换算到法兰系）')
    else:
        for q in poses:
            q['R'] = rotvec_to_matrix(q['rv'])

    if len(poses) < 2:
        print()
        print('⚠️ 只有 1 个姿态 —— **这不算验收**，只能查「值离不离谱」。')
        print('   要判对错，必须换一个姿态再碰同一个点。')
        tcp_xyz = doc.get('tcp_xyz')
        if tcp_xyz:
            v = np.array(tcp_xyz, float)
            print('   配的 TCP 偏移 |t| = %.1f mm，方向 [%.2f %.2f %.2f]'
                  % (np.linalg.norm(v), *(v / max(np.linalg.norm(v), 1e-9))))
            print('   ⚠️ 判据：指尖类 TCP 应当**主要沿着工具 z**，'
                  '且 |t| 要落在夹爪的物理范围内。')
            print('      实测踩过：旧那个 TCP 的 y ≈ 172 mm —— 全开 95 mm 时'
                  '指尖角离轴最多 47.5 mm，三倍半还多，**物理上不可能**。')
        return 0

    # ── Δ 判据：两两比 ────────────────────────────────────────────────
    print()
    print('两两比（判据 Δ ≤ %.1f mm）：' % args.max_mm)
    worst = 0.0
    for i in range(len(poses)):
        for j in range(i + 1, len(poses)):
            d = float(np.linalg.norm(poses[i]['p'] - poses[j]['p']))
            ang = rotation_angle_deg(poses[i]['R'].T @ poses[j]['R'])
            worst = max(worst, d)
            print('  %s vs %s：Δ = %7.3f mm   姿态差 %6.1f°   ⇒ %s'
                  % (poses[i]['note'], poses[j]['note'], d * 1e3, ang,
                     'OK' if d <= thr else '**超了**'))

    # 姿态铺得开不开 —— 差得越小这个测试越钝
    spreads = [rotation_angle_deg(poses[i]['R'].T @ poses[j]['R'])
               for i in range(len(poses)) for j in range(i + 1, len(poses))]
    if max(spreads) < MIN_SPREAD_DEG:
        print('⚠️ 两个姿态只差 %.1f° < %.0f° —— 这个测试**太钝**：'
              '角度越小 Δ 越看不出误差（Δ ≈ 2·sin(θ/2)·|e|）。重新碰，'
              '把姿态拉开到 30° 以上。' % (max(spreads), MIN_SPREAD_DEG))

    # ── 反解 TCP 误差向量 ─────────────────────────────────────────────
    # 同一个物理点 C，示教器报的是 Pᵢ = C + Rᵢ·e， e = (配的TCP) − (真TCP)，
    # 在**法兰系**里。两两相减： (Rᵢ − Rⱼ)·e = Pᵢ − Pⱼ。
    # 全配对堆起来最小二乘 —— 三个以上姿态时还是过定的，残差能当质检。
    e, fit, rank = solve_tcp_error(poses)

    print()
    if rank < 3:
        print('⚠️ 解不出误差向量（秩 %d < 3）。' % rank)
        if len(poses) == 2:
            # 两个姿态**够判 Δ**（上面那行已经给了结论），只是**不够反解**：
            # `(R₁ − R₂)` 一定有个零特征方向（就是那次的转轴），秩最多 2。
            # 这一条容易误解成「刚才那次碰白碰了」，所以说清楚。
            print('   两个姿态**足够判 Δ**（上面的结论有效）。但不够反解误差向量 ——')
            print('   `(R₁ − R₂)` 必然沿转轴有个零特征方向，秩最多 2。'
                  '想要「该改成多少」，')
            print('   再换一个**绕另一个轴**转的姿态碰第三次。')
        else:
            print('   姿态还不够开。再换一个**绕不同轴**转的姿态碰一次。')
    else:
        print('反解出的 TCP 误差向量 e = [%7.2f %7.2f %7.2f] mm，|e| = %.2f mm'
              % (*(e * 1e3), np.linalg.norm(e) * 1e3))
        print('   （e = 现在配的值 − 真值，在**法兰系**里；'
              '拟合残差 %.2f mm）' % (fit * 1e3))
        if fit * 1e3 > max(1.0, args.max_mm):
            print('   ⚠️ 拟合残差偏大 —— 说明「碰的是同一个物理点」这个前提'
                  '没成立（尖端侧滑了，或者哪一次没顶住）。')
            print('      先重碰一遍，别信这个 e。')
        tcp_xyz = doc.get('tcp_xyz')
        if tcp_xyz:
            now = np.array(tcp_xyz, float)
            fixed = now - e * (1e3 if unit == 'mm' else 1.0)
            print()
            print('TCP_1 现在配的是 [%.2f %.2f %.2f]，**改成** [%.2f %.2f %.2f] mm'
                  '（保持 rpy 不变）' % (*now, *fixed))
            print('   改完**再碰一次同一个点**：Δ 应当塌到 %.0f mm 以内。'
                  '没塌就再来一轮 —— 这是个收敛过程。' % args.max_mm)
        else:
            print('   （记录里没给 `tcp_xyz`，所以算不出「该改成多少」；'
                  '把示教器上 TCP_1 现在的 x/y/z 填进去再跑一次就有了）')

    print()
    if worst <= thr:
        print('✅ Δ 最大 %.3f mm ≤ %.1f mm —— **TCP 验收通过**，可以拿去碰靶了。'
              % (worst * 1e3, args.max_mm))
    else:
        print('❌ Δ 最大 %.3f mm > %.1f mm —— **还没标好**。按顺序查：'
              % (worst * 1e3, args.max_mm))
        print('   ① 四次碰的是不是**同一个物理点**（纸上的笔点有没有被顶跑）')
        print('   ② 四个标定姿态是不是**绕不同轴**转的（都绕一个轴的话解不出来）')
        print('   ③ 参考点别用**纸角** —— 纸角有厚度、有毛边，顶上去没有明确的止感')
        print('   ④ 上面反解出来的修正值可以直接改进去，再碰一轮')
    return 0 if worst <= thr else 1


if __name__ == '__main__':
    sys.exit(main())

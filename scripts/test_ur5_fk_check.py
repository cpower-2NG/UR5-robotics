#!/usr/bin/env python3
"""`ur5_fk_check.py` 自检 —— 不需要 ROS、不需要真机、不需要示教器。

（沿用本仓库 ``scripts/test_*.py`` 的约定：直接 ``python3`` 跑，退出码 0 = 通过。）

为什么值得单独钉一遍
====================
这个脚本要判的是「示教器那个 `Base` 到底是不是机器人 base」，
而它给出的结论会**直接改写真机几何**（台面到底在 0 还是 −0.395）。
所以判决工具本身必须先被判过。这里钉四层：

  1. **FK 本身**：两条独立的链（URDF 关节原点 / 标准 DH）必须给出同一个法兰位置。
  2. **三个分支各自可达**：参考系一致 / 被偏移 / 判不了。
     —— 尤其要钉住「判不了就是判不了」，它**不许**硬给一个 A 或 B。
  3. **不变量**：`d` 与填的 TCP 无关（我一度推错、以为能顺手解出那个 4 mm，
     所以这条特别要钉 —— 否则脚本会被人当成「不用碰台面就能定 4 mm」）。
  4. **变异测试**：把 FK 或参考系**改坏**，自检/判决必须**变红**。
     绿得没有理由的护栏等于没有护栏。
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ur5_fk_check as U                                          # noqa: E402
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
        print('  FAIL  %s  %s' % (name, detail))


def mat_to_rotvec_deg(R):
    """3×3 → 旋转矢量（度）。示教器显示的就是这个。"""
    ang = np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    if ang < 1e-12:
        return np.zeros(3)
    k = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * np.sin(ang))
    return np.degrees(k * ang)


def synth_pose(q_deg, tcp_mm=(0.0, 0.0, 0.0), offset_m=(0.0, 0.0, 0.0)):
    """造一条「示教器会报出来的」位姿。

    示教器算的是 `法兰位姿 + R·TCP`，所以 `tcp_mm` 必须是**当时填的那个**，
    而且**必须拿同一个 TCP 去喂 `ur5_fk_check`** —— 两边不一致的话，
    `d` 里会多出一项 `R·(TCP填 − TCP填)`，测的就不是参考系了。
    （第一版自检就栽在这上面：合成用 `0,0,200.2`、回读用 `0,0,0`，
     于是「20 mm 偏移」那条其实是靠 200 mm 的 TCP 差**碰巧**通过的。）
    """
    q = np.radians(q_deg)
    M = U.T(U.rotz(U.PI), np.zeros(3)) @ U.urdf_fk(q)      # base 系（示教器同系）
    t = np.asarray(tcp_mm, float) / 1000.0
    p = (M[:3, 3] + M[:3, :3] @ t + np.asarray(offset_m, float)) * 1000.0
    rv = mat_to_rotvec_deg(M[:3, :3])
    return ('%s' % ','.join('%.6f' % v for v in p),
            '%s' % ','.join('%.6f' % v for v in rv))


def run(joints, pose, tcp='0,0,0', joints2=None, pose2=None, tcp2='0,0,0'):
    argv = ['--joints=%s' % joints, '--pose=%s' % pose, '--tcp=%s' % tcp]
    if joints2 is not None:
        argv += ['--joints2=%s' % joints2, '--pose2=%s' % pose2, '--tcp2=%s' % tcp2]
    import io
    import contextlib
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        code = U.main(argv)
    return code, buf.getvalue()


def main():
    Q = [10.0, -70.0, 80.0, -100.0, -90.0, 0.0]
    Q2 = [-30.0, -50.0, 60.0, -120.0, -80.0, 40.0]
    A = 200.2

    # ── 1. FK 本身 ────────────────────────────────────────────────────
    print('[1] FK：URDF 链 vs 标准 DH 链')
    check('两条独立链位置一致（自检通过）', U.self_test(verbose=False))

    # 与手算的 q=0 金标值对一遍（DH 链，标准 UR5 DH 表）
    p0 = U.dh_fk(np.zeros(6))[:3, 3] * 1000.0
    check('q=0 的 DH 法兰位置 = 手算金标 (−817.25, −191.45, −5.491) mm',
          np.allclose(p0, [-817.25, -191.45, -5.491], atol=0.01), '%s' % p0)

    JQ = '%s' % ','.join('%g' % v for v in Q)
    JQ2 = '%s' % ','.join('%g' % v for v in Q2)

    # ── 2. 三个分支（合成与回读**必须用同一个 TCP**）────────────────────
    print('[2] 判决分支')
    poseA, rvA = synth_pose(Q)
    code, out = run(JQ, poseA + ',' + rvA)
    check('参考系一致 ⇒ A 成立、退出码 0', code == 0 and 'A 成立' in out,
          'code=%d' % code)

    poseB, rvB = synth_pose(Q, offset_m=(0, 0, 0.395))
    code, out = run(JQ, poseB + ',' + rvB)
    check('参考系偏移 395 mm ⇒ B 成立、退出码 0', code == 0 and 'B 成立' in out,
          'code=%d' % code)

    # 含糊的偏移（20 mm）⇒ 必须拒绝判，不许挑一个方向猜
    poseX, rvX = synth_pose(Q, offset_m=(0.02, 0, 0))
    code, out = run(JQ, poseX + ',' + rvX)
    check('偏移 20 mm ⇒ **拒判**、退出码 3（不猜）',
          code == 3 and '我不猜' in out, 'code=%d' % code)

    # 换个姿态再抄一遍 ⇒ 能分辨纯平移 / 带旋转
    pose2, rv2 = synth_pose(Q2, offset_m=(0.02, 0, 0))
    code, out = run(JQ, poseX + ',' + rvX, joints2=JQ2, pose2=pose2 + ',' + rv2)
    check('两姿态下偏移一致 ⇒ 判「纯平移」', code == 0 and '纯平移' in out,
          'code=%d' % code)

    poseR, rvR = synth_pose(Q2, offset_m=(0.02, 0.03, 0))
    code, out = run(JQ, poseX + ',' + rvX, joints2=JQ2, pose2=poseR + ',' + rvR)
    check('两姿态下偏移不同 ⇒ 判「还带旋转」、退出码 3',
          code == 3 and '旋转' in out, 'code=%d' % code)

    # ── 3. 不变量：d 与填的 TCP 无关（**成对**地换才成立）─────────────
    print('[3] 不变量')
    for tcp in ('0,0,0', '0,0,200.2', '44.54,-11.36,194.55'):
        t = tuple(float(v) for v in tcp.split(','))
        p, rv = synth_pose(Q, tcp_mm=t)
        code, out = run(JQ, p + ',' + rv, tcp)
        check('填 TCP=(%s) 且用同一个 TCP 合成 ⇒ 参考系仍判一致' % tcp,
              code == 0 and 'A 成立' in out, 'code=%d' % code)

    # ── 4. 退化输入 ───────────────────────────────────────────────────
    print('[4] 退化输入')
    code, _ = run('10,-70,80', poseA + ',' + rvA)
    check('关节角只有 3 个 ⇒ 退出码 2', code == 2, 'code=%d' % code)
    code, _ = run('a,b,c,d,e,f', poseA + ',' + rvA)
    check('关节角不是数字 ⇒ 退出码 2', code == 2, 'code=%d' % code)

    code, out = run('%s' % ','.join('%g' % v for v in Q), poseA + ',' + rvA,
                    joints2='%s' % ','.join('%g' % v for v in Q), pose2=poseA + ',' + rvA)
    check('第二组抄了**同一个姿态** ⇒ 被拦下、退出码 3（差=0 不是证据）',
          code == 3 and '没有提供任何新信息' in out, 'code=%d' % code)

    # ── 5. 变异测试：把守卫改坏，必须变红 ─────────────────────────────
    print('[5] 变异测试（改坏 FK / 参考系，必须变红）')

    # 5a. a2 的符号翻掉（−0.425 → +0.425）⇒ 两条链必须不再一致
    orig_chain = list(U.URDF_CHAIN)
    try:
        U.URDF_CHAIN[3] = ((0.425, 0.0, 0.0), (0.0, 0.0, 0.0), True)
        check('变异：a2 符号翻掉 ⇒ FK 自检变红', not U.self_test(verbose=False))
    finally:
        U.URDF_CHAIN[:] = orig_chain
    check('还原后 FK 自检恢复绿色', U.self_test(verbose=False))

    # 5b. 漏掉 base_link→base 那个绕 z 的 π ⇒ d 从 ~0 变成 ~1271 mm
    orig_rotz = U.rotz
    U.rotz = lambda _t: np.eye(3)                      # 退化成「不转」
    try:
        code, out = run(JQ, poseA + ',' + rvA)
        check('变异：漏掉绕 z 的 π ⇒ **不再**报 A 成立（参考系错会被抓到）',
              'A 成立' not in out, 'code=%d' % code)
    finally:
        U.rotz = orig_rotz
    check('还原后判决恢复', 'A 成立' in run(JQ, poseA + ',' + rvA)[1])

    # 5c. `pendant_flange` 忘记扣 TCP ⇒ 只有**带 TCP** 的那条不变量会变红
    #     （这条专门验 [3] 那组不变量是有牙的，不是摆着好看）
    orig_pf = U.pendant_flange
    U.pendant_flange = lambda pose, rv, tcp: U.T(rotvec_to_matrix(rv),
                                                 np.asarray(pose, float) / 1000.0)
    try:
        p, rv = synth_pose(Q, tcp_mm=(0, 0, A))
        code, _ = run(JQ, p + ',' + rv, '0,0,200.2')
        check('变异：反推法兰时忘记扣 TCP ⇒ 带 TCP 的不变量变红',
              code != 0, 'code=%d' % code)
    finally:
        U.pendant_flange = orig_pf

    print()
    print('=' * 72)
    print('PASS %d   FAIL %d' % (N_PASS, N_FAIL))
    if FAILS:
        print('失败：' + '；'.join(FAILS))
    return 1 if N_FAIL else 0


if __name__ == '__main__':
    sys.exit(main())

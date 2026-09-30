#!/usr/bin/env python3
"""外参解算自检（B 路）—— 不需要 ROS、不需要真机、不需要机械臂。

（沿用本仓库 ``scripts/test_*.py`` 的约定：直接 ``python3`` 跑，退出码 0 = 通过。）

为什么值得单独钉一遍
====================
这条路上**每一步错了都不会报错**，只会得到一个看着挺像的外参：

  · 合成顺序写反（`R_bb·R_cb` 写成了 `R_cb·R_bb`）→ 还是正交矩阵，位姿照样合法；
  · 靶系原点取错（忘了「内角点比外角点各进去一整格」）→ 整体平移 20~30 mm，
    **残差依旧很小**（所有点一起平移，刚体配准吸收得掉）；
  · 光学系没换成 `camera_link` → 外参整体转 90°，**重投影误差一点不变**。

三样都只能靠断言发现，不能靠读代码或看结果「像不像」。所以这里的每一节
都对应上面一条。

七节：
  1. **Kabsch 精确复原**：已知 (R,t) 造点，解出来必须逐位对上。
  2. **有噪声时不跑飞**：加 1 mm 高斯噪声，误差应仍是毫米级。
  3. **退化输入仍返回真旋转**：点共线时朴素 SVD 会给出 det = −1 的**镜像**，
     那不是一个合法位姿。本仓库的写法必须把它掰回来（det = +1）。
  4. **四元数往返**：`matrix_to_quat` ∘ `quat_to_matrix` = 恒等（含 180° 那个
     朴素写法会崩的角落）。
  5. **合成的语义**：`compose` 出来的变换作用在点上，必须等于「先反解到靶系、
     再经 base←靶 搬过去」那两步的结果。**这一节专门钉住顺序**。
  6. **光学系 → camera_link**：det = +1（是旋转不是镜像），且
     向前(z_opt) → x_link、向右(x_opt) → −y_link。
  7. **靶点几何**：内角点比外角点**各进去一整格**，数量 = (cols−1)(rows−1)。
  8. **PnP 往返**（有 cv2 才跑）：合成一组投影 → solvePnP → 比 (R,t)。
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from board_extrinsics import (                                    # noqa: E402
    OPTICAL_TO_LINK, rigid_from_correspondences, quat_to_matrix, matrix_to_quat,
    board_inner_corners, board_pattern_corners, compose_base_from_camera,
    optical_to_link)

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


def rot(axis, deg):
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    if axis == 'x':
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], float)
    if axis == 'y':
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], float)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], float)


def main():
    rng = np.random.default_rng(57)

    # ── 1. Kabsch 精确复原 ────────────────────────────────────────────────
    print('[1] Kabsch 精确复原')
    R_true = rot('z', 23.0) @ rot('y', -11.0) @ rot('x', 7.0)
    t_true = np.array([0.31, -0.22, 0.64])
    P_src = rng.normal(size=(12, 3)) * 0.15
    P_dst = P_src @ R_true.T + t_true
    R, t, rms = rigid_from_correspondences(P_src, P_dst)
    check('R 逐位复原', np.allclose(R, R_true, atol=1e-9),
          '最大差 %.2e' % np.abs(R - R_true).max())
    check('t 逐位复原', np.allclose(t, t_true, atol=1e-9),
          '最大差 %.2e' % np.abs(t - t_true).max())
    check('残差 ~0', rms < 1e-12, 'rms=%.3e' % rms)

    # ── 2. 有噪声时不跑飞 ────────────────────────────────────────────────
    print('[2] 加 1 mm 噪声')
    P_noisy = P_dst + rng.normal(scale=1e-3, size=P_dst.shape)
    R2, t2, rms2 = rigid_from_correspondences(P_src, P_noisy)
    err_t = np.linalg.norm(t2 - t_true)
    ang = np.degrees(np.arccos(np.clip((np.trace(R2.T @ R_true) - 1) / 2, -1, 1)))
    check('平移误差 < 1 mm', err_t < 1e-3, '%.3f mm' % (err_t * 1e3))
    check('姿态误差 < 0.5°', ang < 0.5, '%.3f°' % ang)
    check('残差反映噪声量级', 0.2e-3 < rms2 < 3e-3, 'rms=%.3f mm' % (rms2 * 1e3))

    # ── 3. 退化输入仍返回真旋转 ──────────────────────────────────────────
    #
    # ⚠️ 这一节**差点写成假测试**。第一版只试了一个角度（z=40°），
    # 它「通过」了 —— 但去掉 det 修正再跑，**照样通过**。
    # 原因是：点共线时 H 秩 1，后两个奇异向量是**简并**的，LAPACK 给它们的
    # 符号是任意的，所以朴素的 `Vt.T @ U.T` 落在 +1 还是 −1 **看运气**。
    # 实测扫 300 个随机旋转，朴素版给出镜像 **132 次（44%）**；
    # z=40° 恰好是运气好的那一边（z=−40° 与 z=0.001° 则稳定翻成 −1）。
    # 所以断言不能只钉一个输入，必须**扫一批** —— 有修正时恒 +1，没有时约 44% 翻车。
    print('[3] 退化输入（共线点）仍返回真旋转')
    line = np.stack([np.linspace(0, 0.3, 6)] * 3, axis=1)   # 一条直线上的 6 个点
    n_neg = 0
    worst_det = +2.0
    for seed in range(200):
        r = np.random.default_rng(seed)
        A = r.normal(size=(3, 3))
        Q, _ = np.linalg.qr(A)
        if np.linalg.det(Q) < 0:
            Q[:, 0] *= -1
        R3, _, _ = rigid_from_correspondences(line, line @ Q.T + r.normal(size=3))
        d3 = float(np.linalg.det(R3))
        if d3 < 0:
            n_neg += 1
        worst_det = min(worst_det, d3)
    check('200 个随机姿态里一次镜像都没有（朴素版约 44% 会翻）', n_neg == 0,
          '翻了 %d 次' % n_neg)
    for tag, deg in (('z=+40°', 40.0), ('z=−40°', -40.0), ('z=90°', 90.0),
                     ('z=0.001°', 0.001)):
        R3, _, _ = rigid_from_correspondences(line, line @ rot('z', deg).T +
                                              np.array([0.1, 0.2, 0.3]))
        check('共线 %s → det = +1' % tag, abs(np.linalg.det(R3) - 1.0) < 1e-9,
              'det=%.6f' % np.linalg.det(R3))
    R3, _, _ = rigid_from_correspondences(line, line @ rot('z', 40.0).T +
                                          np.array([0.1, 0.2, 0.3]))
    check('R 仍是正交阵', np.allclose(R3 @ R3.T, np.eye(3), atol=1e-9))

    # 镜像输入：数据**要求**一个反射，而反射不是位姿。
    # 函数必须交回一个真旋转（det=+1）并把这件事如实报成**非零残差**，
    # 而不是悄悄返回 det=−1 的矩阵 —— 后者会让后面所有旋转合成全错，
    # 却依旧「看起来像一个矩阵」。
    print('[3b] 镜像输入（数据要求反射）')
    P_src_m = rng.normal(size=(10, 3)) * 0.2
    P_dst_m = P_src_m @ rot('z', 25.0).T
    P_dst_m[:, 2] *= -1.0                      # 过 xy 面翻个面 = 反射
    R_m, t_m, rms_m = rigid_from_correspondences(P_src_m, P_dst_m)
    check('反射输入下 det 仍是 +1（不给镜像）',
          abs(np.linalg.det(R_m) - 1.0) < 1e-9, 'det=%.6f' % np.linalg.det(R_m))
    check('反射的代价如实报成非零残差', rms_m > 1e-3, 'rms=%.3f mm' % (rms_m * 1e3))

    # ── 4. 四元数往返 ────────────────────────────────────────────────────
    print('[4] 四元数往返')
    for tag, Rq in (('一般', rot('z', 37) @ rot('x', 58)),
                    ('180°', rot('x', 180.0)),
                    ('近 180°', rot('y', 179.3)),
                    ('小角', rot('z', 0.4))):
        q = matrix_to_quat(Rq)
        check('往返 %s（|q|=1）' % tag, abs(np.linalg.norm(q) - 1) < 1e-12,
              '|q|=%.15f' % np.linalg.norm(q))
        check('往返 %s（矩阵复原）' % tag,
              np.allclose(quat_to_matrix(q), Rq, atol=1e-9),
              '最大差 %.2e' % np.abs(quat_to_matrix(q) - Rq).max())

    # ── 5. 合成的语义（**顺序**） ────────────────────────────────────────
    print('[5] compose 的顺序')
    R_bb, t_bb = rot('z', 15) @ rot('y', 5), np.array([0.5, 0.1, 0.75])
    R_cb, t_cb = rot('x', -70) @ rot('z', 33), np.array([0.02, -0.03, 0.61])
    R_bc, t_bc = compose_base_from_camera(R_bb, t_bb, R_cb, t_cb)
    bad = 0
    for _ in range(20):
        p_cam = rng.normal(size=3) * 0.2
        p_board = R_cb.T @ (p_cam - t_cb)          # 相机系 → 靶系
        two_step = R_bb @ p_board + t_bb           # 靶系 → base
        composed = R_bc @ p_cam + t_bc
        bad = max(bad, float(np.abs(two_step - composed).max()))
    check('合成 == 两步（钉住顺序）', bad < 1e-12, '最大差 %.3e m' % bad)

    # ── 6. 光学系 → camera_link ─────────────────────────────────────────
    print('[6] 光学系 → camera_link')
    check('det = +1（旋转不是镜像）',
          abs(np.linalg.det(OPTICAL_TO_LINK) - 1.0) < 1e-12,
          'det=%.6f' % np.linalg.det(OPTICAL_TO_LINK))
    check('向前 z_opt → x_link',
          np.allclose(OPTICAL_TO_LINK @ np.array([0, 0, 1.0]), [1, 0, 0]))
    check('向右 x_opt → −y_link（y 是左）',
          np.allclose(OPTICAL_TO_LINK @ np.array([1.0, 0, 0]), [0, -1, 0]))
    check('向下 y_opt → −z_link（z 是上）',
          np.allclose(OPTICAL_TO_LINK @ np.array([0, 1.0, 0]), [0, 0, -1]))
    R_l, t_l = optical_to_link(R_bc, t_bc)
    check('换帧不改平移', np.allclose(t_l, t_bc))
    check('换帧后仍是正交阵', np.allclose(R_l @ R_l.T, np.eye(3), atol=1e-12))

    # ── 7. 靶点几何 ─────────────────────────────────────────────────────
    print('[7] 靶点几何（外角点 / 内角点差一整格）')
    inner = board_inner_corners(20.0, 9, 12)
    outer = board_pattern_corners(20.0, 9, 12)
    check('内角点数 = (cols−1)(rows−1) = 88', len(inner) == 88, '得到 %d' % len(inner))
    check('第一个内角点 = (20,20,0)（各进去一整格）',
          np.allclose(inner[0], [20.0, 20.0, 0.0]), str(inner[0]))
    check('外角点 4 个', len(outer) == 4, '得到 %d' % len(outer))
    check('图案尺寸 = 180 × 240', np.allclose(outer[2], [180.0, 240.0, 0.0]),
          str(outer[2]))
    check('内角点全在外角点张成的框内',
          inner[:, 0].min() >= outer[:, 0].min() and
          inner[:, 0].max() <= outer[:, 0].max() and
          inner[:, 1].max() <= outer[:, 1].max())

    # ── 8. PnP 往返 ─────────────────────────────────────────────────────
    print('[8] solvePnP 往返（需要 cv2）')
    try:
        import cv2
    except ImportError:
        print('  SKIP  没有 cv2')
    else:
        K = np.array([[917.6, 0, 649.6], [0, 916.4, 344.9], [0, 0, 1.0]])
        R_true2 = rot('x', 143.5) @ rot('z', -18.0)   # 俯视相机：约 36.5° 俯角
        t_true2 = np.array([0.05, -0.02, 0.653])
        obj = board_inner_corners(20.0, 9, 12)
        proj, _ = cv2.projectPoints(obj, cv2.Rodrigues(R_true2)[0], t_true2, K,
                                    np.zeros(5))
        ok, rvec, tvec = cv2.solvePnP(obj, proj, K, np.zeros(5),
                                      flags=cv2.SOLVEPNP_ITERATIVE)
        R_pnp, _ = cv2.Rodrigues(rvec)
        check('PnP 解出 ok', bool(ok))
        check('PnP 姿态复原', np.allclose(R_pnp, R_true2, atol=1e-4),
              '最大差 %.2e' % np.abs(R_pnp - R_true2).max())
        check('PnP 平移复原', np.allclose(tvec.ravel(), t_true2, atol=1e-4),
              '最大差 %.2e' % np.abs(tvec.ravel() - t_true2).max())

    # ── 9. 命令行端到端 + 单位契约 ──────────────────────────────────────
    #
    # 这一节是**真的把 CLI 起来跑**，因为明天现场走的就是这条链。
    # 它钉住一个当场踩到的缺陷：`pnp` 写毫米、`solve-touch` 按米算，
    # 而 `compose` 把两者直接混合 —— 结果是「相机离基座 730 米」，
    # **全程没有任何一步报错**（1000 倍只是同一套数学上的一个尺度而已）。
    print('[9] CLI 端到端 + 单位契约')
    import json
    import subprocess
    import tempfile

    script = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          'board_extrinsics.py')

    def run(*argv):
        p = subprocess.run([sys.executable, script, *argv],
                           capture_output=True, text=True)
        return p.returncode, p.stdout + p.stderr

    S = 0.020                                   # 米
    R_bb_t = rot('z', 12.0)
    t_bb_t = np.array([0.42, -0.07, 0.02])
    R_cb_t = rot('x', 143.5) @ rot('z', -18.0)
    t_cb_t = np.array([0.05, -0.02, 0.653])
    corners = np.array([[0, 0, 0], [9 * S, 0, 0],
                        [9 * S, 12 * S, 0], [0, 12 * S, 0]], float)
    P_base = corners @ R_bb_t.T + t_bb_t

    with tempfile.TemporaryDirectory() as td:
        def w(name, obj):
            p = os.path.join(td, name)
            with open(p, 'w') as fh:
                json.dump(obj, fh)
            return p

        cam = w('cam_m.json', {'R': R_cb_t.tolist(), 't': t_cb_t.tolist(),
                               'unit': 'm'})
        touch_m = w('touch_m.json', {'R_board_pts': corners.tolist(),
                                     'P_base_pts': P_base.tolist(), 'unit': 'm'})
        out_m = os.path.join(td, 'ext_m.json')
        rc1, log1 = run('solve-touch', '--points', touch_m,
                        '--out', os.path.join(td, 'b_m.json'))
        rc2, log2 = run('compose', '--base-from-board', os.path.join(td, 'b_m.json'),
                        '--cam-from-board', cam, '--out', out_m)
        check('solve-touch 米制解码成功', rc1 == 0, log1.strip()[-200:])
        check('compose 米制合成成功', rc2 == 0, log2.strip()[-200:])
        if rc2 == 0:
            with open(out_m) as fh:
                got = json.load(fh)
            R_ok = R_bb_t @ R_cb_t.T @ OPTICAL_TO_LINK.T
            t_ok = t_bb_t - (R_bb_t @ R_cb_t.T) @ t_cb_t
            check('外参 == 真值（R）', np.allclose(got['R'], R_ok, atol=1e-9),
                  '最大差 %.2e' % np.abs(np.array(got['R']) - R_ok).max())
            check('外参 == 真值（t）', np.allclose(got['t'], t_ok, atol=1e-9),
                  '最大差 %.2e' % np.abs(np.array(got['t']) - t_ok).max())
            check('相机到基座距离合理（不是 730 米）',
                  np.linalg.norm(got['t'][:2]) < 5.0,
                  '%.3f m' % np.linalg.norm(got['t'][:2]))

        # 同一份记录改成毫米 —— 必须解出**同一个**外参
        touch_mm = w('touch_mm.json',
                     {'R_board_pts': (corners * 1000).tolist(),
                      'P_base_pts': (P_base * 1000).tolist(), 'unit': 'mm'})
        out_mm = os.path.join(td, 'ext_mm.json')
        run('solve-touch', '--points', touch_mm, '--out', os.path.join(td, 'b_mm.json'))
        rc3, log3 = run('compose', '--base-from-board', os.path.join(td, 'b_mm.json'),
                        '--cam-from-board', cam, '--out', out_mm)
        check('毫米记录也能正确解码', rc3 == 0, log3.strip()[-200:])
        if rc3 == 0 and rc2 == 0:
            with open(out_mm) as fh:
                got_mm = json.load(fh)
            check('毫米与米解出同一个外参',
                  np.allclose(got_mm['t'], got['t'], atol=1e-12),
                  't 差 %.3e m' % np.abs(np.array(got_mm['t'])
                                         - np.array(got['t'])).max())

        # 只有一个输入是毫米 —— 必须**停下来**，不能默默混算
        cam_bad = w('cam_bad.json', {'R': R_cb_t.tolist(),
                                     't': t_cb_t.tolist(), 'unit': 'mm'})
        rc4, log4 = run('compose', '--base-from-board', os.path.join(td, 'b_m.json'),
                        '--cam-from-board', cam_bad,
                        '--out', os.path.join(td, 'x.json'))
        check('单位不一致时停下来（退出码非 0）', rc4 != 0, 'rc=%d' % rc4)
        check('且说清了是谁的单位不对', 'mm' in log4, log4.strip()[-200:])

    print()
    print('=' * 72)
    print('PASS %d   FAIL %d' % (N_PASS, N_FAIL))
    if FAILS:
        print('失败项：')
        for f in FAILS:
            print('  - %s' % f)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())

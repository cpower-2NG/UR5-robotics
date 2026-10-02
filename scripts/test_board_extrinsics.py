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

十二节（编号即 `main()` 里的 `[n]`）：
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
  9. **CLI 端到端 + 单位契约**：`solve-touch` 的毫米/米、残差闸门、抄串行、
     缺行 —— 每一种都必须让退出码非 0。
 10. **`points.template.json`** 与 `board_pattern_corners` 逐点一致。
 11. **检测 ROI 逐级放宽**：紧框检不出时自动放宽、最后退到整幅（含变异检查）。
 12. **④ 标注器接口**：`靶←相机` 的求逆、与 ① 逐点一致、忘了求逆会错、
     台面方块该填的 z 符号，以及三道硬闸门。**这条路不动臂**，是采数据集用的。
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from board_extrinsics import (                                    # noqa: E402
    OPTICAL_TO_LINK, rigid_from_correspondences, quat_to_matrix, matrix_to_quat,
    board_inner_corners, board_pattern_corners, compose_base_from_camera,
    optical_to_link, board_from_camera_optical, cmd_labeler)

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

        # ── ② 的残差闸门：抄串行必须**停下来** ──────────────────────────
        # ② 的输入是**手抄的六个数**，抄错一位会解出一个「合法但错」的
        # base←靶。旧版只打一句 ⚠️、退出码仍是 0、文件照样写 —— 现场四条
        # 命令连着敲，那一行太容易滑过去，而合成出来的位姿看着完全正常。
        # 实测：把④抄成①的重复点，残差 94 mm。
        dup = w('touch_dup.json',
                {'R_board_pts': corners.tolist(),
                 'P_base_pts': [P_base[0].tolist(), P_base[1].tolist(),
                                P_base[2].tolist(), P_base[0].tolist()],
                 'unit': 'm'})
        rc5, log5 = run('solve-touch', '--points', dup,
                        '--out', os.path.join(td, 'b_dup.json'))
        check('抄串行（重复点）时 solve-touch 退出码非 0', rc5 != 0, 'rc=%d' % rc5)
        # 就算那份坏文件流到了 compose，也必须被**第二道**闸门拦住 ——
        # 闸门要放在**生成外参**的那一步才算真拦住。
        rc6, log6 = run('compose', '--base-from-board',
                        os.path.join(td, 'b_dup.json'),
                        '--cam-from-board', cam,
                        '--out', os.path.join(td, 'x2.json'))
        check('坏 base←靶 流到 compose 仍被拦住', rc6 != 0, 'rc=%d' % rc6)

        # 有一行没填 —— 报错要**指出是哪一行**。默认那两句
        # （`could not convert string to float` / `SVD did not converge`）
        # 没人能从中反推出「④那一行没填」。
        hole = w('touch_hole.json',
                 {'R_board_pts': corners.tolist(),
                  'P_base_pts': [P_base[0].tolist(), P_base[1].tolist(),
                                 P_base[2].tolist(), ['FILL', 'FILL', 'FILL']],
                  'unit': 'm'})
        rc7, log7 = run('solve-touch', '--points', hole,
                        '--out', os.path.join(td, 'b_hole.json'))
        check('有一行没填时退出码非 0', rc7 != 0, 'rc=%d' % rc7)
        check('且指出是第几行（P_base_pts[3]）', 'P_base_pts[3]' in log7,
              log7.strip()[-200:])

    # ── 10. 现场输入模板与靶系定义**不许漂** ─────────────────────────────
    # `docs/calib/points.template.json` 是给现场用的：靶系那四个角已经算好
    # 填在里面，人只填 Base 坐标。它一旦与 `board_pattern_corners` 不一致，
    # 后果正是这条路上**残差抓不到**的那种错 —— 四个点仍能配出一个刚体，
    # 只是配到别的靶系上去了。所以把「模板 == 函数输出」钉在这里。
    print('[10] points.template.json 与 board_pattern_corners 一致')
    tpl_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            'docs', 'calib', 'points.template.json')
    if not os.path.exists(tpl_path):
        # 这份自检**两边都跑**（本地 + 板子上 `/tmp/calib/`）。板子上那份是
        # 从 `scripts/` 单独拷过去的，没有 `docs/` —— 那不是失败，跳过。
        print('  SKIP  没有 %s（板子上只拷了 scripts/）' % tpl_path)
    else:
        with open(tpl_path) as fh:
            tpl = json.load(fh)
        want = board_pattern_corners(20.0, 9, 12)      # A4：9×12 格 / 20 mm
        got = np.array(tpl['R_board_pts'], float)
        check('R_board_pts == board_pattern_corners(20, 9, 12)',
              np.array_equal(got, want),
              '模板 %s' % got.tolist())
        check('R_board_pts 是四个点（①②③④）', got.shape == (4, 3),
              '形状 %s' % (got.shape,))
        # 外角必须**不共线** —— 共线的话 Kabsch 的 rotation 是不定的，
        # 而残差照样接近 0（这条路上残差抓不到的错又多一种）。
        # ⚠️ 看的是**第二大**奇异值，不是最小的那个：这四个点本来就**共面**
        # （靶系里 z 全是 0），所以最小奇异值**必然**是 0，拿它当判据是错的
        # —— 这条断言第一版就写错了，是它自己 FAIL 出来的。
        e = np.sort(np.linalg.eigvalsh(np.cov((want - want.mean(0)).T)))[::-1]
        check('四个外角不共线（第二奇异值 > 10 mm）', np.sqrt(max(e[1], 0)) > 0.010,
              '奇异值 %.1f / %.1f / %.1f mm'
              % tuple(np.sqrt(np.maximum(e, 0)) * 1e3))
        check('unit 是 mm（示教器读的就是 mm）', tpl.get('unit') == 'mm',
              'unit=%r' % tpl.get('unit'))
        # 占位符要**填不动**才安全：留着它必须让 ② 报错，而不是当 0 用
        ph = tpl['P_base_pts']
        check('P_base_pts 四行、每行三个（占位符形式）',
              len(ph) == 4 and all(len(r) == 3 for r in ph),
              '形状 %s' % ([len(r) for r in ph],))

    # ── 11. 检测的 ROI 必须**逐级放宽、最后退到整幅** ────────────────────
    # 2026-10-02 实测踩到的坑，且**不报错**：同一张靶、同一个位置（外接矩形
    # 逐像素相同），只因为画面里别的东西变了（机械臂吊进来、投在纸上的影子），
    # `_rough_roi` 的紧框（pad=0.35）下 2x/3x **双双检不出**，放宽到 0.90
    # 或整幅立刻检出。症状是「靶明明在画面中央，脚本却说找不到角点」。
    #
    # 这一节钉三件事：① 变体的顺序与覆盖（结构）；② 紧框全废时**整幅兜底
    # 仍能端到端检出**（功能）；③ 拿掉兜底那一步就会红（变异）——
    # 前两条只证明「现在能跑」，第三条才证明**兜底是承重的**。
    print('[11] 检测 ROI 逐级放宽 + 整幅兜底')
    try:
        import cv2
    except ImportError:
        print('  SKIP  没有 cv2')
    else:
        import board_extrinsics as _B

        # 造一张合成靶图：内角点 8×11 ⇒ 方格 9×12。
        # ⚠️ 每格 **12 px** 是**试出来的**，不是随手填的：方格大到 14 px 时
        #    1x 就能干净检出（ratio=1.00），整节就**一档都不会走到**，
        #    下面三条会「全绿地什么都没测」。12 px 时 1x 的 ratio = 0.00
        #    （重复行），与真机上那张 150×143 px 的靶同档。
        #    画布也要够大，否则两个 pad 裁出来的框会**退化成同一个**
        #    （实测：紧框与 0.90 都等于整幅，去重后就只剩一档）。
        sq, cols, rows, W, H, ox, oy = 12, 9, 12, 1280, 900, 600, 380
        img = np.full((H, W, 3), 235, np.uint8)
        for r in range(rows):
            for c in range(cols):
                if (r + c) % 2:
                    img[oy + r * sq:oy + (r + 1) * sq,
                        ox + c * sq:ox + (c + 1) * sq] = 25
        img = cv2.GaussianBlur(img, (3, 3), 0)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        PS = [(cols - 1, rows - 1)]

        # (前置) 这张合成图必须**在 1x 下就检出重复行** —— 否则整节什么
        #        都没走到，会全绿地骗人（上面那条注释说的就是这个坑）。
        ok1, c1 = cv2.findChessboardCorners(
            gray, PS[0],
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 60, 0.001)
        r1 = _B.adjacent_spacing_min_ratio(
            cv2.cornerSubPix(gray, c1, (11, 11), (-1, -1), crit)
            .reshape(PS[0][1], PS[0][0], 2)) if ok1 else 0.0
        check('(前置) 合成图 1x 判为退化（否则本节测了个空）', r1 < 0.45,
              'ratio=%.2f' % r1)

        # ① 结构：第一档是 1x 全幅；每个放大档**最后一档**是整幅
        vs = list(_B._roi_variants(gray, PS, (1, 2, 3), (0.35, 0.90)))
        scales_seen = sorted({v[0] for v in vs})
        check('_roi_variants 覆盖 1/2/3 三档', scales_seen == [1, 2, 3],
              '实际 %s' % scales_seen)
        check('第一档是 1x 全幅', vs[0][0] == 1 and vs[0][4] == '1x 全幅',
              vs[0][4])
        for sc in (2, 3):
            tags = [v[4] for v in vs if v[0] == sc]
            check('%dx 最后一档是整幅兜底' % sc, tags[-1] == '%dx 全幅' % sc,
                  '实际 %s' % tags)
            check('%dx 每个 pad 各一档 + 整幅' % sc, len(tags) == 3,
                  '实际 %s' % tags)

        # ② 功能：把 `_rough_roi` 打成**永远返回不含靶的小框**（模拟「紧框
        #    与宽框都不行」），端到端仍必须检出 —— 救它的是整幅兜底。
        ok_box = _B._rough_roi(gray, PS)
        bad_box = (0, 0, 40, 40)
        check('(前置) 造出来的坏框确实不含靶', bad_box != ok_box, str(ok_box))
        real_roi = _B._rough_roi
        tried = []
        real_variants = _B._roi_variants

        def spy(g, ps, scales, pads):
            for v in real_variants(g, ps, scales, pads):
                tried.append(v[4])
                yield v

        try:
            _B._rough_roi = lambda *a, **k: bad_box
            _B._roi_variants = spy
            got, found, sc = _B.detect_chessboard(gray, PS, verbose=False)
            n_want = (cols - 1) * (rows - 1)
            check('紧框全废时仍检出 %d 点' % n_want,
                  got is not None and len(got) == n_want,
                  'got=%s' % (None if got is None else len(got)))
            check('而且**确实走到了**整幅那一档',
                  any(t.endswith('全幅') for t in tried[1:]),
                  '试过 %s' % tried)

            # ③ 变异：把兜底那一档抽掉，同一个坏框下**必须检不出**。
            #    这一条才是「兜底是承重的」的证据 —— 少了它，上面那条
            #    也可能只是碰巧。
            def no_fallback(g, ps, scales, pads):
                for v in real_variants(g, ps, scales, pads):
                    if v[4].endswith('全幅') and v[0] != 1:
                        continue
                    yield v

            _B._roi_variants = no_fallback
            got2, _, _ = _B.detect_chessboard(gray, PS, verbose=False)
            check('变异：抽掉整幅兜底后**检不出**（证明兜底承重）',
                  got2 is None, '居然还是检出了 %d 点'
                  % (0 if got2 is None else len(got2)))
        finally:
            _B._rough_roi = real_roi
            _B._roi_variants = real_variants

    # ── 12. ④ 标注器接口：靶←相机（**不动臂**那条出口）────────────────────
    # ① 解的是 `相机←靶`，而 `auto_label_grid.py` 的 tf 后端要的是
    # `靶←相机`（tf2 lookup_transform(target, camera) 的语义）—— **恰好是逆**。
    # 喂错了**不报错**：逆矩阵照样是合法位姿，方块被标到画面里一个镜像位置，
    # 框还在、大小也对。这一节钉四件事：
    #   ① 求逆本身对（含「是 −Rᵀt 不是 −t」这一处）；
    #   ② 标注器的 `to_camera` 与 ① 的 `R·p+t` **逐点相同**（两份独立意见）；
    #   ③ 把 ① 直接喂进去（变异）**确实会错**，证明求逆承重；
    #   ④ 台面方块该填的 z 符号 —— 靶系 z 指向不定，填反号是**静默**错误。
    print('[12] labeler：靶←相机')
    import contextlib
    import io
    import json as _json
    import tempfile
    import types

    import auto_label_grid as L

    # 真实那组数的量级：相机在靶上方 0.88 m、斜着朝下看。
    # **用 look-at 造，不硬编码角度** —— 随手挑一组欧拉角很容易让靶点落到
    # 相机背后，而 `project_pinhole` 会把背后的点**丢掉**（返回值少几个），
    # 于是后面的比较就变成了拿长度不等的两个数组相减。实测踩过。
    #
    # 靶系 z 指**进台面**（findChessboardCorners 的右手定则：图像里 x 向右、
    # y 向下 ⇒ z 指进画面）；相机在上方，所以光心 z 是负的，世界「上」是 −z。
    t_bc_true = np.array([0.16, -0.58, -0.64])
    z_opt = -t_bc_true / np.linalg.norm(t_bc_true)         # 光轴：光心→靶原点
    x_opt = np.cross(np.array([0.0, 0.0, -1.0]), z_opt)     # 世界「上」× 光轴
    x_opt /= np.linalg.norm(x_opt)
    y_opt = np.cross(z_opt, x_opt)
    R_bc_true = np.column_stack([x_opt, y_opt, z_opt])     # 列 = 相机各轴在靶系
    R_cb_true = R_bc_true.T
    t_cb_true = -R_cb_true @ t_bc_true
    check('(前置) 造出来的位姿是真旋转、且原点在相机前方',
          np.allclose(R_bc_true @ R_bc_true.T, np.eye(3), atol=1e-12)
          and (R_cb_true @ np.zeros(3) + t_cb_true)[2] > 0,
          'det=%.6f' % np.linalg.det(R_bc_true))

    R_bc, t_bc = board_from_camera_optical(R_cb_true, t_cb_true)
    check('求逆：R 回到原样', np.allclose(R_bc, R_bc_true, atol=1e-12),
          '最大差 %.2e' % np.abs(R_bc - R_bc_true).max())
    check('求逆：t 回到相机光心在靶系的坐标',
          np.allclose(t_bc, t_bc_true, atol=1e-12),
          '最大差 %.2e' % np.abs(t_bc - t_bc_true).max())
    # 「−Rᵀt 写成 −t」这条变异的**唯一**症状就是光心跑到靶的另一侧，
    # 而 R 还是对的 —— 所以单独钉一条。
    check('求逆：**不是** −t（那会把光心放到靶另一侧）',
          not np.allclose(t_bc, -t_cb_true, atol=1e-6),
          '−t 恰好也对得上？那这条测试没测到东西')

    info = {'fx': 917.62, 'fy': 916.43, 'cx': 649.55, 'cy': 344.91,
            'width': 1280, 'height': 720}
    doc = {'target_frame': 'board', 'camera_frame': 'camera_color_optical_frame',
           'translation': t_bc.tolist(), 'rotation': matrix_to_quat(R_bc).tolist()}
    proj = L.TfProjector(info, doc)

    # 含离面高度的点：z≠0 那条路也要一起验（只验 z=0 的靶面点是测不到符号坑的）
    pts = np.array([[0.00, 0.00, 0.0], [0.09, 0.12, 0.0],
                    [0.18, 0.24, 0.0], [0.09, 0.12, -0.01425],
                    [0.03, 0.30, -0.0285]])
    via_labeler = np.array([proj.to_camera(p) for p in pts])
    via_pnp = (R_cb_true @ pts.T).T + t_cb_true
    check('标注器 to_camera ≡ ① 的 R·p+t（两份独立意见）',
          np.allclose(via_labeler, via_pnp, atol=1e-12),
          '最大差 %.2e m' % np.abs(via_labeler - via_pnp).max())

    def _px(cam_pts):
        uv, dropped = L.project_pinhole(list(cam_pts), info['fx'], info['fy'],
                                        info['cx'], info['cy'])
        return np.asarray(uv), dropped

    uv_ok, drop_ok = _px(via_labeler)
    check('正常路径全部点都在相机前方', drop_ok == 0, 'dropped=%d' % drop_ok)

    # 变异：**把 ① 的产物直接当 target←camera 喂进去**（忘了求逆）。
    wrong = {'target_frame': 'board', 'camera_frame': 'camera_color_optical_frame',
             'translation': t_cb_true.tolist(),
             'rotation': matrix_to_quat(R_cb_true).tolist()}
    wproj = L.TfProjector(info, wrong)
    cam_wrong = np.array([wproj.to_camera(p) for p in pts])
    # ⚠️ 判据取在**相机系（米）**，不取像素差。像素差依赖这组具体几何：
    #    这组位姿下喂错方向的点**全部落到相机背后**（project_pinhole 会把它们
    #    丢掉，返回空数组），换个角度就可能变成「镜面成像、框照样出得来」。
    #    米制差与几何无关，且同样证明这一步承重。
    d_m = float(np.abs(cam_wrong - via_pnp).max())
    check('变异：忘了求逆 → 相机系差 > 0.1 m（证明这一步承重）', d_m > 0.1,
          '只差 %.4f m —— 那这条测试没测到东西' % d_m)
    check('变异方向在这组几何下是**响的**（点全落到相机背后，会被丢掉）',
          _px(cam_wrong)[1] > 0 or np.abs(_px(cam_wrong)[0]).max() > 50.0,
          '居然投出了几乎一样的像素 —— 那它就是个静默错误')

    # z 符号：台面在靶系里是哪一侧。**填反号是静默的** —— 框照样出得来。
    cx, cy = info['cx'], info['cy']
    base = np.array([0.09, 0.12, 0.0])
    uv_base, _ = _px([proj.to_camera(base)])
    r_base = float(np.hypot(uv_base[0][0] - cx, uv_base[0][1] - cy))

    def _r_of(z):
        uv, _ = _px([proj.to_camera(np.array([0.09, 0.12, z]))])
        return float(np.hypot(uv[0][0] - cx, uv[0][1] - cy))

    h = 0.0285
    sign = 1.0 if t_bc[2] > 0 else -1.0        # ← 与 cmd_labeler 里那条规则逐字一致
    z_rule = sign * h / 2
    check('打印机给出的 z 符号规则 = 物理上正确的那一侧（抬高 → 离主点更远）',
          _r_of(z_rule) > r_base + 0.5,
          'z=%+.5f 时 r=%.2f，底面 r=%.2f' % (z_rule, _r_of(z_rule), r_base))
    check('变异：z 填反号 → 反而缩向主点（这就是「指进台面」那个坑）',
          _r_of(-z_rule) < r_base,
          'z=%+.5f 时 r=%.2f，底面 r=%.2f' % (-z_rule, _r_of(-z_rule), r_base))

    # 三道硬闸门。**每一道都要真的响** —— 否则文件写出去、数据集就歪了。
    def _run_labeler(pnp_doc, **kw):
        d = tempfile.mkdtemp()
        p = os.path.join(d, 'pnp.json')
        with open(p, 'w') as fh:
            _json.dump(pnp_doc, fh)
        a = types.SimpleNamespace(pnp=p, out=os.path.join(d, 'o.json'),
                                  target_frame='board', image=None, info=None,
                                  square_mm=None, cols=None, rows=None,
                                  block_mm=28.5)
        for k, v in kw.items():
            setattr(a, k, v)
        # 静音：`cmd_labeler` 正常时会把整段「下一步」打印出来，四条闸门
        # 各来一遍会把这一节的输出淹掉，看不出哪几条在测什么。
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                cmd_labeler(a)
            return None
        except SystemExit as e:
            return str(e)

    good = {'R': R_cb_true.tolist(), 't': t_cb_true.tolist(), 'unit': 'm'}
    check('闸门 0：合法输入放行', _run_labeler(good) is None,
          '被挡了：%s' % _run_labeler(good))
    msg = _run_labeler(dict(good, unit='mm'))
    check('闸门 1：单位不是 m → 停', msg is not None and '单位' in msg, str(msg))
    msg = _run_labeler(dict(good, R=(R_cb_true * 1.01).tolist()))
    check('闸门 2：R 不正交（手改/转存）→ 停',
          msg is not None and '不正交' in msg, str(msg))
    # 距离出界：t_cb 取成「相机离靶原点 5 cm」。这条拦住的是
    # --square-mm / --cols / --rows 填错 —— 那些错误解出来的位姿**照样合法**。
    msg = _run_labeler(dict(good, t=(R_cb_true @ np.array([0.0, 0.0, 0.05])).tolist()))
    check('闸门 3：相机离靶 0.05 m（尺度错）→ 停',
          msg is not None and '说得通' in msg, str(msg))

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

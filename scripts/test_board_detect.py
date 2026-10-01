#!/usr/bin/env python3
"""棋盘格检测自检 —— 不需要 ROS、不需要真机、不需要相机。

（沿用本仓库 ``scripts/test_*.py`` 的约定：直接 ``python3`` 跑，退出码 0 = 通过。）

为什么值得单独钉一遍
====================
`cv2.findChessboardCorners` 在**图案只占画面一小块**时会返回一个
**末行复制上一行**的网格：88 个点里 8 个是假的。它**返回 True**，
不抛异常、不打日志。而它的残差长相恰好骗过「看中位数」这种质检：

    俯视相机 2026-10-01 19:25 那一帧（同一个靶、同一张图）
      1x 检测：中位 0.271 px  95 分位 6.394  最大 6.981   ← 中位数漂亮
      2x 检测：中位 0.149 px  95 分位 0.311  最大 0.455   ← 同一个靶

差在哪：远端那 8 个假点把平面「拉」歪，近端 80 个点仍然亚像素。
**所以判据必须看最大值**，而且检测本身要先有一道闸。

四节：
  1. **比值判据本身**：正常网格 vs 重复行网格，必须分得开。
  2. **`detect_chessboard` 的契约**：它**永不**返回退化网格（跑真图或合成图）。
  3. **合成靶的往返**：已知位姿 → 投影 → 检测 → solvePnP，位姿要回来。
  4. **真图回归**（有 ``/tmp/topcam/frame.png`` 才跑）：最大值必须 < 1 px。
     这一节是**回归**不是前提 —— 真图不在就跳过，不算失败。
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from board_extrinsics import (                                    # noqa: E402
    adjacent_spacing_min_ratio, detect_chessboard, board_inner_corners)

try:
    import cv2
except ImportError:                                               # pragma: no cover
    cv2 = None

N_PASS = N_FAIL = N_SKIP = 0
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


def skip(name, why):
    global N_SKIP
    N_SKIP += 1
    print('  SKIP  %s  (%s)' % (name, why))


def clean_grid(rows=11, cols=8, step=20.0):
    g = np.zeros((rows, cols, 2), float)
    g[:, :, 0] = np.arange(cols) * step
    g[:, :, 1] = np.arange(rows)[:, None] * step
    return g


def duplicated_grid(rows=11, cols=8, step=20.0):
    """末行复制上一行 —— 就是实测踩到的那个退化。"""
    g = clean_grid(rows, cols, step)
    g[-1] = g[-2]
    return g


def synth_frame(quad, cols=9, rows=12, S=60, blur=1.2, noise=3.0,
                W=1280, H=720, seed=0):
    """按给定位姿把靶「贴」到画面上，返回灰度图。

    ``quad`` 是靶四个**外角**（① (0,0)、② (W_board,0)、③ (W,H)、④ (0,H)，
    顺序同 `board_pattern_corners`）在画面里的像素位置。
    """
    board = np.full((rows * S, cols * S), 235, np.uint8)
    for i in range(rows):
        for j in range(cols):
            if (i + j) % 2 == 0:
                board[i * S:(i + 1) * S, j * S:(j + 1) * S] = 25
    dst = np.float32([[0, 0], [cols * S, 0], [cols * S, rows * S], [0, rows * S]])
    Hm = cv2.getPerspectiveTransform(dst, np.float32(quad))
    mask = cv2.warpPerspective(np.full_like(board, 255), Hm, (W, H))
    warp = cv2.warpPerspective(board, Hm, (W, H))
    canvas = np.full((H, W), 110, np.uint8)
    canvas[mask > 0] = warp[mask > 0]
    canvas = cv2.GaussianBlur(canvas, (0, 0), blur)
    rng = np.random.default_rng(seed)
    return np.clip(canvas.astype(np.float32) + rng.normal(0, noise, canvas.shape),
                   0, 255).astype(np.uint8)


def main():
    if cv2 is None:
        print('没有 cv2，整份跳过（板子上要 pip install opencv-python 才有）')
        return 0

    # ── 1. 比值判据本身 ──────────────────────────────────────────────────
    print('[1] 相邻间距比值：正常网格 vs 重复行')
    r_ok = adjacent_spacing_min_ratio(clean_grid())
    r_dup = adjacent_spacing_min_ratio(duplicated_grid())
    check('正常网格比值 ≈ 1', r_ok > 0.99, '得到 %.4f' % r_ok)
    check('重复行比值 ≈ 0', r_dup < 0.01, '得到 %.4f' % r_dup)
    check('默认闸门 0.45 能把两者分开', r_dup < 0.45 <= r_ok,
          'ok=%.3f dup=%.3f' % (r_ok, r_dup))
    # 只重复**列**也要抓到（横放的靶踩的是这一支）
    g = clean_grid()
    g[:, -1] = g[:, -2]
    check('重复列同样抓到', adjacent_spacing_min_ratio(g) < 0.01,
          '得到 %.4f' % adjacent_spacing_min_ratio(g))

    # ── 2. detect_chessboard 的契约：永不返回退化网格 ────────────────────
    print('[2] detect_chessboard 的契约')
    quad = [(906.7, 258.6), (711.4, 248.2), (723.1, 83.7), (892.0, 89.6)]
    gray = synth_frame(quad)

    # 先确认**这份夹具真的会触发**那个退化 —— 不然下面几条就是在空转，
    # 哪天 OpenCV 改好了（或合成参数改了）也不会有人知道。
    _crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 60, 0.001)
    _ok, _c = cv2.findChessboardCorners(
        gray, (8, 11), cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    _r1 = float('nan')
    if _ok:
        _c = cv2.cornerSubPix(gray, _c, (11, 11), (-1, -1), _crit)
        _r1 = adjacent_spacing_min_ratio(_c.reshape(11, 8, 2))
    check('夹具确实触发退化（1x 比值 < 0.45）', _r1 < 0.45,
          '1x 比值 %.3f —— 夹具不再复现，这个测试要重做' % _r1)

    c, ps, scale = detect_chessboard(gray, ((8, 11), (11, 8)))
    check('小靶能检出', ps is not None and len(c) == 88,
          'ps=%s 点数=%s' % (ps, None if c is None else len(c)))
    if ps is not None:
        grid = c.reshape(ps[1], ps[0], 2)
        ratio = adjacent_spacing_min_ratio(grid)
        check('返回的网格非退化（比值 ≥ 0.45）', ratio >= 0.45, '得到 %.3f' % ratio)
        check('返回的坐标在原图范围内',
              c[:, 0, 0].min() >= 0 and c[:, 0, 0].max() < gray.shape[1] and
              c[:, 0, 1].min() >= 0 and c[:, 0, 1].max() < gray.shape[0],
              'u∈[%.0f,%.0f] v∈[%.0f,%.0f]' % (c[:, 0, 0].min(), c[:, 0, 0].max(),
                                                c[:, 0, 1].min(), c[:, 0, 1].max()))
        # 放大只在 ROI 上做，不该整幅放大 —— 整幅 3x 会慢一个量级
        check('未整幅放大（scale 受控）', scale in (1, 2, 3), 'scale=%s' % scale)

    # 这条是**变异检验**：把放大关掉（只剩 1x）应当拿不到东西 —— 宁可返回
    # None 让调用方报错，也**绝不**返回那份 88 点里有 8 个是假的网格。
    # 谁要是把闸门摘了，这条立刻红。
    _c1, _ps1, _s1 = detect_chessboard(gray, ((8, 11), (11, 8)), scales=(1,),
                                       verbose=False)
    check('只给 1x 时宁可返回 None，也不返回坏网格', _ps1 is None,
          '竟然返回了 ps=%s' % (_ps1,))

    # ── 3. 合成靶的位姿往返 ──────────────────────────────────────────────
    print('[3] 合成靶 → 检测 → solvePnP 往返')
    if ps is None:
        skip('位姿往返', '第 2 节没检出')
    else:
        K = np.array([[917.62, 0, 649.55], [0, 916.43, 344.91], [0, 0, 1]])
        c_eff, r_eff = ps[0] + 1, ps[1] + 1
        objp = board_inner_corners(0.020, c_eff, r_eff)
        ok, rvec, tvec = cv2.solvePnP(objp, c.reshape(-1, 2).astype(float), K,
                                      np.zeros(5), flags=cv2.SOLVEPNP_ITERATIVE)
        pp, _ = cv2.projectPoints(objp, rvec, tvec, K, np.zeros(5))
        err = np.linalg.norm(pp.reshape(-1, 2) - c.reshape(-1, 2), axis=1)
        check('solvePnP 收敛', bool(ok))
        # 判据与真图那条一致（1 px）。合成图的模糊/噪声是我自己造的，
        # 比真相机的略糙（实测 0.70 px vs 真图 0.46 px），不必更严。
        check('重投影最大 < 1 px', err.max() < 1.0, '最大 %.3f px' % err.max())
        # 靶面法向必须朝着相机（合成时就是这么摆的）
        R, _ = cv2.Rodrigues(rvec)
        check('相机在靶面 +z 侧', float(tvec.ravel() @ R[:, 2]) > 0,
              't·n = %.4f' % float(tvec.ravel() @ R[:, 2]))

    # ── 4. 真图回归（真图不在就跳过）────────────────────────────────────
    print('[4] 真图回归（可选）')
    real = '/tmp/topcam/frame.png'
    real_info = '/tmp/topcam/info.json'
    if not (os.path.exists(real) and os.path.exists(real_info)):
        skip('真图回归', '没有 %s' % real)
    else:
        import json
        with open(real_info) as fh:
            info = json.load(fh)
        K = np.array(info['K'], float).reshape(3, 3)
        D = np.array(info['D'], float).reshape(-1)
        g = cv2.cvtColor(cv2.imread(real), cv2.COLOR_BGR2GRAY)
        c, ps, scale = detect_chessboard(g, ((8, 11), (11, 8)))
        if ps is None:
            check('真图能检出', False, '没检出')
        else:
            c_eff, r_eff = ps[0] + 1, ps[1] + 1
            objp = board_inner_corners(0.020, c_eff, r_eff)
            ok, rvec, tvec = cv2.solvePnP(objp, c.reshape(-1, 2).astype(float), K,
                                          D, flags=cv2.SOLVEPNP_ITERATIVE)
            pp, _ = cv2.projectPoints(objp, rvec, tvec, K, D)
            err = np.linalg.norm(pp.reshape(-1, 2) - c.reshape(-1, 2), axis=1)
            print('        放大 %dx，中位 %.3f px，最大 %.3f px' % (scale, np.median(err), err.max()))
            check('真图重投影最大 < 1 px（判据看最大值）', err.max() < 1.0,
                  '最大 %.3f px' % err.max())

    print()
    print('=' * 72)
    print('PASS %d   FAIL %d   SKIP %d' % (N_PASS, N_FAIL, N_SKIP))
    if FAILS:
        print('失败：' + '；'.join(FAILS))
    return 1 if N_FAIL else 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""方块位置提案器自检 —— 不需要 ROS、不需要真机、不需要机械臂、不需要数据集。

（沿用本仓库 ``scripts/test_*.py`` 的约定：直接 ``python3`` 跑，退出码 0 = 通过。）

为什么值得单独钉一遍
====================
这个脚本的产物是**训练标签的输入**。它错了不会报错，只会把一批框标偏，
而偏掉的标签训出来的模型「看着还行、就是抓不准」—— 事后极难归因。
实测已经抓到过两个这种错：

  · **`_rough_seed` 用了 `R.T` 而不是 `R`。** 正向是 `p_target = R·p_cam + t`
    （见 `TfProjector.__init__`），所以「相机系方向 → 靶系方向」要乘 `R`。
    写成 `R.T` 不报错、不崩溃，仍然给出一根像样的射线，只是**每次固定偏 ~80 mm**。
    旧版被后面的最小二乘顺手救回来了，于是这个错一直没露头；改成倒角精修后
    窗口只有 ±12 mm，立刻全体崩掉 —— 是它把问题顶出来的。

  · **提案与标注必须是同一套投影。** `project_cubes` 是向量化重写的
    `to_camera` + `project_pinhole`，写错一个转置就会让提案和标注差一个镜像，
    而两边单看都「像那么回事」。

五节（编号即 `main()` 里的 `[n]`）：
  1. **`_rough_seed` 往返**：真值 → 投影 → 框中心 → 反投，必须回到毫米级。
     **这一节就是抓 `R.T`/`R` 之别的。**
  2. **变异检查**：把 `R` 换成 `R.T`，往返必须坏掉 —— 证明第 1 节真的在承重，
     而不是碰巧通过。
  3. **向量化投影 == labeler 的元素级投影**：逐点一致，钉住「提案和标注同一套数学」。
  4. **倒角场**：合成图里画一个立方体轮廓，真位置分数≈0，挪开 30 mm 明显变大。
  5. **尺寸体检**：画 28.5 mm 的立方体应体检出 28.5 mm；画 20 mm 的应体检出 20 mm
     （即「真方块」与「形状不对的东西」在这个量上确实分得开）。
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from board_extrinsics import matrix_to_quat                            # noqa: E402
import auto_label_grid as L                                            # noqa: E402
import propose_block_positions as P                                    # noqa: E402

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
        print('  FAIL  %s%s' % (name, ('   ← %s' % detail) if detail else ''))


# ------------------------------------------------------------------ 合成一个相机

W, H = 1280, 720
FX, FY, CX, CY = 917.62, 916.43, 649.55, 344.91
# 与现场实测同一量级的位姿：相机在靶系 (0.16, -0.58, -0.64)，俯视靶原点。
T_CAM = np.array([0.1591, -0.5776, -0.6442])


def make_projector():
    """造一个 `TfProjector`：正向约定 p_target = R·p_cam + t。"""
    z_opt = -T_CAM / np.linalg.norm(T_CAM)
    x_opt = np.cross(np.array([0.0, 0.0, -1.0]), z_opt)
    x_opt /= np.linalg.norm(x_opt)
    y_opt = np.cross(z_opt, x_opt)
    R = np.column_stack([x_opt, y_opt, z_opt])       # 相机 → 靶
    extr = {'target_frame': 'board', 'camera_frame': 'camera_color_optical_frame',
            'rotation': matrix_to_quat(R), 'translation': T_CAM.tolist()}
    info = {'fx': FX, 'fy': FY, 'cx': CX, 'cy': CY, 'width': W, 'height': H}
    return L.TfProjector(info, extr)


def main():
    proj = make_projector()
    size = 0.0285
    zsign = 1.0 if proj.t[2] > 0 else -1.0
    zc = zsign * size / 2.0

    # ------------------------------------------------------------ [1] 往返
    print('\n[1] _rough_seed 往返（真值 → 投影 → 框中心 → 反投）')
    pts = [(0.276, -0.100), (0.206, -0.038), (-0.208, 0.032),
           (0.158, 0.306), (0.398, 0.261)]
    errs = []
    for (x, y) in pts:
        bb = P.pixel_bbox(proj, (x, y, zc), size)
        if bb is None:
            errs.append(1e9)
            continue
        s = P._rough_seed(proj, np.array(bb), zc)
        errs.append(float(np.linalg.norm(np.array([x, y]) - s)))
    e1 = max(errs)
    # 框中心不是立方体中心的投影，几毫米的差是几何本身带来的，不是 bug。
    check('五个位置往返误差 < 5 mm', e1 < 0.005, '最大 %.1f mm' % (e1 * 1000))

    # ------------------------------------------------------------ [2] 变异
    print('\n[2] 变异：把 R 换成 R.T，往返必须坏掉（证明 [1] 在承重）')

    def rough_seed_mutated(proj_, obs, zc_):
        u = (obs[0] + obs[2]) / 2.0
        v = (obs[1] + obs[3]) / 2.0
        d_cam = np.array([(u - proj_.cx) / proj_.fx, (v - proj_.cy) / proj_.fy, 1.0])
        d = proj_.R.T @ d_cam                       # ← 故意写错的那一行
        lam = (zc_ - proj_.t[2]) / d[2]
        return (proj_.t + lam * d)[:2]

    x, y = pts[0]
    bb = P.pixel_bbox(proj, (x, y, zc), size)
    s_bad = rough_seed_mutated(proj, np.array(bb), zc)
    e2 = float(np.linalg.norm(np.array([x, y]) - s_bad))
    check('变异后误差 > 20 mm（错得足够明显）', e2 > 0.020,
          '实际 %.1f mm —— 若这条不过，说明 [1] 是碰巧过的' % (e2 * 1000))

    # ------------------------------------------------ [3] 向量化 == labeler
    print('\n[3] project_cubes（向量化）与 labeler 的元素级投影逐点一致')
    uv, ok = P.project_cubes(proj, np.array([[x, y]]), zc, P._corner_offsets(size))
    ref = []
    for c in L.box_corners(x, y, zc, size):
        px, dropped = L.project_pinhole([proj.to_camera(c)], FX, FY, CX, CY)
        ref.append(px[0])
    d3 = float(np.abs(uv[0] - np.asarray(ref, float)).max())
    check('bool 与 8 个角点都一致（<1e-9 px）', bool(ok[0]) and d3 < 1e-9,
          '最大差 %.3g px' % d3)

    # ------------------------------------------------------------ [4] 倒角场
    print('\n[4] 倒角场：合成图里画出立方体轮廓，真位置≈0、挪开明显变大')
    import cv2

    img = np.full((H, W, 3), 255, np.uint8)
    uv_c, ok_c = P.project_cubes(proj, np.array([[x, y]]), zc, P._corner_offsets(size))
    check('合成的立方体整个落在画面内', bool(ok_c[0]))
    hull = cv2.convexHull(np.ascontiguousarray(uv_c[0], dtype=np.float32))
    cv2.fillPoly(img, [hull.astype(np.int32)], (0, 0, 0))
    dt = P.chamfer_field(img)
    s_true, *_ = (P.chamfer_score(dt, uv_c[0], W, H),)
    s_far = P.score_at(proj, dt, x + 0.030, y, zc, size, W, H)
    check('真位置倒角 < 1.5 px', s_true is not None and s_true < 1.5,
          '实际 %s' % s_true)
    check('挪开 30 mm 后倒角 > 5 px', s_far is not None and s_far > 5.0,
          '实际 %s' % s_far)

    # ---------------------------------------------------------- [5] 尺寸体检
    print('\n[5] 尺寸体检：画出多大的方块，就该体检出多大')
    got = []
    for true_mm in (28.5, 20.0):
        s = true_mm / 1000.0
        uv_s, ok_s = P.project_cubes(proj, np.array([[x, y]]), zsign * s / 2.0,
                                     P._corner_offsets(s))
        if not ok_s[0]:
            got.append(None)
            continue
        im2 = np.full((H, W, 3), 255, np.uint8)
        h2 = cv2.convexHull(np.ascontiguousarray(uv_s[0], dtype=np.float32))
        cv2.fillPoly(im2, [h2.astype(np.int32)], (0, 0, 0))
        dt2 = P.chamfer_field(im2)
        res = P.refine_chamfer(proj, dt2, W, H, x, y, size, zsign)
        got.append(None if res is None else res[2] * 1000.0)
    check('28.5 mm 的轮廓 → 体检 %.1f mm（±1.5）' % (got[0] or -1),
          got[0] is not None and abs(got[0] - 28.5) < 1.5)
    check('20.0 mm 的轮廓 → 体检 %.1f mm（±1.5）' % (got[1] or -1),
          got[1] is not None and abs(got[1] - 20.0) < 1.5)

    print('\n%d PASS  %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('未通过：' + '; '.join(FAILS))
    return 1 if N_FAIL else 0


if __name__ == '__main__':
    sys.exit(main())

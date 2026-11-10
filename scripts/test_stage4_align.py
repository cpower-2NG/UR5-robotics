#!/usr/bin/env python3
"""E4b 对中工具（``stage4_align.py``）自检：纯数学 + 合成场景，不需要真机/相机。

钉住五件事：

1. **纯几何自洽**：``project`` / ``axis_shift_uv`` / ``uv_to_xy_base`` 三者互逆，
   已知 base 位移经像素再解回来 ≤1e-6 mm；轴向微分的符号（抬高 → v 变小）；
2. **合成场景端到端**：按真实俯视外参（B1 实测）渲染台面 + 方块 + 两根深色指，
   块的 base 位移已知 ⇒ 工具必须把它解出来（悬停 ≤0.5 mm）；正负号两向都试；
3. **抓后（held）走颜色掩码**：指与块物理贴合，几何分割会把它们连成一簇 ——
   自检必须证明「指是黑的 ⇒ 不进掩码」，块类错/类别没给都要按预期处理；
4. **共识与拒帧**：中位 + 3 mm 剔除、90° 周期圆均值（±45° 边界）、帧数不足 ⇒
   ``ok=false``，暖机坏帧被 ``drop_first`` 丢掉；
5. **失败注入必须变红**：burst 不存在 / 帧不够 / 块换成灰色（没有可抓的彩色块）
   —— 三种都要 ``ok=false``，不许悄悄出个 0。

跑法：``python3 scripts/test_stage4_align.py``；退出码 0 = 全通过，2 = 有失败。
"""

import json
import math
import os
import shutil
import sys
import tempfile

import numpy as np
from PIL import Image

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import graspnet_candidates as gc                                 # noqa: E402
import precision_plan as pp                                       # noqa: E402
import stage4_align as A                                          # noqa: E402
import urkin                                                      # noqa: E402

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
        print('  FAIL  %s%s' % (name, ('\n        ' + detail) if detail else ''))


def approx(a, b, tol):
    return abs(float(a) - float(b)) <= tol


def find_urdf():
    env = os.environ.get('PRECISION_URDF')
    if env and os.path.exists(env):
        return env
    import glob
    hits = sorted(glob.glob(os.path.expanduser(
        '~/robotics/notes/ur5-vision-grasp/runs/*/robot.urdf')),
        key=os.path.getmtime)
    return hits[-1] if hits else None


# =====================================================================
# 俯视相机的外参：B1 实测（2026-11-08，与 docs/当前状态.md 同一份）
KCAM = np.array([[1376.43, 0.0, 974.33],
                 [0.0, 1374.64, 517.36],
                 [0.0, 0.0, 1.0]])
RCAM = np.array([[-0.9961, 0.0306, -0.0830],
                 [0.0824, 0.6637, -0.7434],
                 [0.0324, -0.7474, -0.6636]])
TCAM = np.array([404.1, 1075.5, 801.2]) / 1000.0
SHAPE = (1036, 1948)                       # (H, W)，B1 真实画幅

TABLE = np.array([205, 205, 200], np.uint8)
YELLOW = np.array([255, 200, 60], np.uint8)      # H≈43°，落在 yellow
BLACK = np.array([24, 24, 26], np.uint8)


def _ray_box(o, D, lo, hi, eps=1e-12):
    """向量化 slab 法：返回 (hit, t_near)。``o`` (3,)，``D`` (N,3)。"""
    tmin = np.full(D.shape[:-1], -np.inf)
    tmax = np.full(D.shape[:-1], np.inf)
    for k in range(3):
        a = D[..., k]
        par = np.abs(a) < eps
        with np.errstate(divide='ignore', invalid='ignore'):
            t1 = (lo[k] - o[k]) / a
            t2 = (hi[k] - o[k]) / a
        near = np.minimum(t1, t2)
        far = np.maximum(t1, t2)
        inside = (o[k] > lo[k]) & (o[k] < hi[k])
        near = np.where(par, np.where(inside, -np.inf, np.inf), near)
        far = np.where(par, np.where(inside, np.inf, -np.inf), far)
        tmin = np.maximum(tmin, near)
        tmax = np.minimum(tmax, far)
    hit = tmax >= np.maximum(tmin, 0.0)
    return hit, tmin


def render_scene(tool0_T, block_xy, block_top=0.0295, block_side=0.050,
                 opening=0.055, finger_thick=0.006, finger_wide=0.012,
                 body_half=0.0, block_color=YELLOW, cam=(KCAM, RCAM, TCAM),
                 shape=SHAPE, joints=None, fingers=True):
    """按针孔模型渲染台面 + 方块 + 两根深色指 → 一帧（``load_frame`` 同构）。

    ``depth`` 是**相机 z**（米），与 ``gc.backproject`` 的口径一致；没打中的像素
    填 0。块的横向位置由 ``block_xy`` 给定，与 ``tool0`` 的 xy 之差就是工具该
    解出来的 ``(dx, dy)``。
    """
    K, R, t = cam
    H, W = shape
    ys, xs = np.mgrid[0:H, 0:W]
    d = np.stack([(xs - K[0, 2]) / K[0, 0], (ys - K[1, 2]) / K[1, 1],
                  np.ones_like(xs, dtype=float)], -1).reshape(-1, 3)
    D = d @ R.T                                  # base 系方向（z_cam 分量为 1）
    o = t
    N = D.shape[0]
    depth = np.zeros(N)
    color = np.full((N, 3), 255, np.uint8)
    zc_of = lambda tt: tt                              # noqa: E731  相机 z = tt

    # 台面 z = 0
    with np.errstate(divide='ignore', invalid='ignore'):
        tt = (0.0 - o[2]) / D[:, 2]
    ok = np.isfinite(tt) & (tt > 0)
    depth[ok] = tt[ok]
    color[ok] = TABLE

    # 方块
    bx, by = float(block_xy[0]), float(block_xy[1])
    hit, tt = _ray_box(o, D, np.array([bx - block_side / 2, by - block_side / 2,
                                       0.0]),
                       np.array([bx + block_side / 2, by + block_side / 2,
                                 block_top]))
    m = hit & (tt > 0) & (tt < depth)
    depth[m] = tt[m]
    color[m] = block_color

    # 两根指：沿 tool0 的 x（开口轴），从 tool0 一路垂到指尖
    R0, p0 = tool0_T[:3, :3], tool0_T[:3, 3]
    if not fingers:
        depth[~np.isfinite(depth)] = 0.0
        return {'dir': '<synthetic>', 'depth': depth.reshape(H, W),
                'color': color.reshape(H, W, 3), 'K': K.copy(),
                'R_bc': R.copy(), 't_bc': t.copy(), 'joints': joints}
    axis_x = R0[:, 0]
    half = opening / 2.0
    z_lo = p0[2] - A.TOOL0_TO_BARE_FINGERTIP
    for sgn in (+1.0, -1.0):
        c = p0 + sgn * (half + finger_thick / 2.0) * axis_x
        ex = finger_thick / 2.0
        hit, tt = _ray_box(o, D,
                           np.array([c[0] - ex, c[1] - finger_wide / 2, z_lo]),
                           np.array([c[0] + ex, c[1] + finger_wide / 2, p0[2]]))
        m = hit & (tt > 0) & (tt < depth)
        depth[m] = tt[m]
        color[m] = BLACK

    # 两指之间的**深色本体**（s10 实测：指间还有一条 10 px 的黑色竖条，
    # 只配对「相邻串」会先碰上它 —— 见 detect_fingers 的配对说明）
    if body_half > 0.0:
        hit, tt = _ray_box(o, D,
                           np.array([p0[0] - body_half, p0[1] - finger_wide / 2,
                                     z_lo]),
                           np.array([p0[0] + body_half, p0[1] + finger_wide / 2,
                                     p0[2]]))
        m = hit & (tt > 0) & (tt < depth)
        depth[m] = tt[m]
        color[m] = BLACK

    depth[~np.isfinite(depth)] = 0.0
    return {'dir': '<synthetic>',
            'depth': depth.reshape(H, W),
            'color': color.reshape(H, W, 3),
            'K': K.copy(), 'R_bc': R.copy(), 't_bc': t.copy(),
            'joints': joints}


def hover_pose(th, uv=(0.435, 0.315), z=0.360, yaw=0.0):
    """IK 出一个**真的** tool0 位姿（悬停高度 z），返回 (T, q)。IK 不收敛返回 (None, None)。"""
    T = pp.target_matrix([uv[0], uv[1], z], yaw)
    q, err_mm, err_deg, _it = th.ik(T, [0.0] * 6)
    if q is None or err_mm > 2.0 or err_deg > 1.0:
        return None, None
    return th.fk(q), q


def as_joints(q, th):
    return {'name': list(th.move_joints),
            'position': [float(v) for v in q]}


def write_frame(d, idx, frame):
    """把内存帧落成 ``load_frame`` 认的目录结构。"""
    Image.fromarray((frame['depth'] * 1000.0).astype(np.uint16)).save(
        os.path.join(d, 'depth_%03d.png' % idx))
    Image.fromarray(frame['color']).save(
        os.path.join(d, 'color_%03d.png' % idx))
    from scipy.spatial.transform import Rotation
    q = Rotation.from_matrix(frame['R_bc']).as_quat()      # x,y,z,w
    entry = {'rotation': {k: float(v) for k, v in zip('xyzw', q)},
             'translation': {k: float(v) for k, v in
                             zip('xyz', frame['t_bc'])}}
    json.dump({'ok': {'base_link:%s' % gc.TOPCAM_TIP: entry}},
              open(os.path.join(d, 'tf_%03d.json' % idx), 'w'))
    json.dump({'K': frame['K'].reshape(-1).tolist()},
              open(os.path.join(d, 'camera_info_%03d.json' % idx), 'w'))
    if frame.get('joints'):
        json.dump(frame['joints'],
                  open(os.path.join(d, 'joint_states_%03d.json' % idx), 'w'))


# =====================================================================
def _res(converged=True, dx=0.8, dy=-0.4, e=None, ok=True):
    cons = {'n_used': 8, 'dx_mm': dx, 'dy_mm': dy, 'spread_mm': 0.9,
            'converged': converged, 'e_mm': e}
    return {'ok': ok, 'held': e is not None, 'consensus': cons,
            'frames': [], 'reason': None if ok else '量不出来'}


def _summary_section():
    td = tempfile.mkdtemp(prefix='e4b-sum-')

    def put(name, obj):
        p = os.path.join(td, name)
        with open(p, 'w', encoding='utf-8') as f:
            json.dump(obj, f)
        return p

    # 三场全收敛，e 都在判据内
    spec = []
    for i, e in enumerate((1.2, 1.8, 2.6)):
        spec.append({'scene': 's%d' % i,
                     'pre': [put('p%d.json' % i, _res())],
                     'held': put('h%d.json' % i, _res(e=e))})
    spec_p = put('spec.json', spec)
    r = A.summarise(spec)
    check('三场全收敛 ⇒ 收敛率 100%%、E4b 通过',
          r['ok'] is True and r['aggregate']['conv_rate'] == 1.0
          and r['reasons'] == [], str(r['reasons']))
    check('e 中位 1.80 mm / p90 2.44 mm',
          approx(r['aggregate']['e_median_mm'], 1.8, 0.01)
          and approx(r['aggregate']['e_p90_mm'], 2.44, 0.05),
          str(r['aggregate']))

    # 一场改了 3 轮 ⇒ 收敛率 2/3 < 90%，且点名「没收敛」
    spec2 = [dict(x) for x in spec]
    spec2[0] = dict(spec2[0], pre=[put('a1.json', _res()),
                                   put('a2.json', _res()),
                                   put('a3.json', _res())])
    r = A.summarise(spec2)
    check('有一场修正 3 轮 ⇒ E4b 不通过、报「没收敛」',
          r['ok'] is False and any('没收敛' in x for x in r['scene_problems']),
          str(r['scene_problems']))

    # 一场残差没进容差
    spec3 = [dict(x) for x in spec]
    spec3[1] = dict(spec3[1], pre=[put('b1.json', _res(converged=False))])
    r = A.summarise(spec3)
    check('某场残差超容差 ⇒ 该场不收敛、整体不通过',
          r['ok'] is False and r['scenes'][1]['converged'] is False,
          str(r['scenes'][1]))

    # e 超判据
    spec4 = []
    for i, e in enumerate((1.5, 2.5, 3.0)):
        spec4.append({'scene': 'q%d' % i, 'pre': [put('q%d.json' % i, _res())],
                      'held': put('qh%d.json' % i, _res(e=e))})
    r = A.summarise(spec4)
    check('e 中位 2.50 mm > 2 mm ⇒ 不通过（哪怕收敛率 100%）',
          r['ok'] is False and any('中位' in x for x in r['reasons']),
          str(r['reasons']))
    check('e 大但仍在 p90 内 ⇒ 只报中位那条',
          len([x for x in r['reasons'] if 'p90' in x]) == 0, str(r['reasons']))

    # 缺 held 不算通过
    spec5 = [{'scene': 'x0', 'pre': [put('x0.json', _res())]}]
    r = A.summarise(spec5)
    check('没有抓后偏心 ⇒ 不算 E4b 通过、点名缺 e',
          r['ok'] is False and any('偏心' in x for x in r['scene_problems']),
          str(r['scene_problems']))

    # 量不出来的 pre
    spec6 = [{'scene': 'y0', 'pre': [put('y0.json', _res(ok=False))],
              'held': put('y0h.json', _res(e=1.0))}]
    r = A.summarise(spec6)
    check('pre 本身 ok=false ⇒ 该场不收敛',
          r['scenes'][0]['converged'] is False, str(r['scenes'][0]))

    # CLI
    out = os.path.join(td, 'sum.json')
    rc = A.main(['--summary', spec_p, '--out-summary', out])
    check('CLI --summary 通过 ⇒ rc=0 且写盘',
          rc == 0 and os.path.exists(out), str(rc))
    rc = A.main(['--summary', put('spec_bad.json', spec4)])
    check('CLI --summary 不通过 ⇒ rc=2', rc == 2, str(rc))
    try:
        A.main(['--summary', spec_p, '--burst', td])
        check('--summary 与 --burst 同给 ⇒ 拒绝', False)
    except SystemExit:
        check('--summary 与 --burst 同给 ⇒ 拒绝', True)
    try:
        A.main([])
        check('既不给 --summary 也不给 --burst ⇒ 拒绝', False)
    except SystemExit:
        check('既不给 --summary 也不给 --burst ⇒ 拒绝', True)


def main():
    global N_PASS, N_FAIL
    print('--- A. 纯几何 ---')
    p_base = np.array([0.43, 0.29, 0.0295])
    uv = A.project_pt(p_base, KCAM, RCAM, TCAM)
    check('投影落在画幅内', uv is not None and 0 < uv[0] < SHAPE[1]
          and 0 < uv[1] < SHAPE[0], repr(uv))

    # 抬高 100 mm → v 变小（相机在上方），u 变化小（B1 实测 dv/dz≈1.1 px/mm）
    d_uv = A.axis_shift_uv((p_base[0], p_base[1]), p_base[2], p_base[2] + 0.100,
                           KCAM, RCAM, TCAM)
    check('轴向微分：抬高 → v 变小', d_uv is not None and d_uv[1] < -50,
          'duv=%s' % (None if d_uv is None else np.round(d_uv, 2)))
    check('轴向微分：每 100 mm 落在 60–160 px（倾角决定，量级对就行）',
          d_uv is not None and -160.0 < d_uv[1] < -60.0,
          'dv=%s' % (None if d_uv is None else round(d_uv[1], 2)))

    # uv → base 的往返：同高两点，已知位移应被原样解回
    for (dx0, dy0) in ((0.0030, -0.0020), (-0.0045, 0.0012), (0.0, 0.0)):
        pa = A.project_pt([0.43, 0.29, 0.0295], KCAM, RCAM, TCAM)
        pb = A.project_pt([0.43 + dx0, 0.29 + dy0, 0.0295], KCAM, RCAM, TCAM)
        conv = A.uv_to_xy_base(pb[0] - pa[0], pb[1] - pa[1], pa[2], KCAM, RCAM)
        # 线性化（雅可比只含 R）在 3 mm 量级有 ~5% 的二阶残差；闭环会把它吃掉，
        # 但自检必须把这条误差**钉住**：超过 8% 说明雅可比写错了，不是近似问题。
        err = None if conv is None else max(abs(conv[0] - dx0), abs(conv[1] - dy0))
        mag = max(abs(dx0), abs(dy0), 1e-4)
        check('往返 (%+.1f, %+.1f) mm（线性化误差 ≤8%%）' % (dx0 * 1e3, dy0 * 1e3),
              err is not None and err <= 0.08 * mag,
              '→ %s  误差 %.3f mm' % (None if conv is None else
                                      (round(conv[0] * 1e3, 4),
                                       round(conv[1] * 1e3, 4)),
                                      -1.0 if err is None else err * 1e3))

    check('fold_deg: 89 ≡ -1', approx(A.fold_deg(89.0), -1.0, 1e-9))
    check('fold_deg: -46 ≡ 44', approx(A.fold_deg(-46.0), 44.0, 1e-9))
    cm = A.circular_mean_deg([0.0, 88.0])
    check('周期均值跨 ±45 边界', cm is not None and approx(cm, -1.0, 0.5),
          'mean=%s' % (None if cm is None else round(cm, 3)))
    check('全零输入不瞎猜', A.circular_mean_deg([]) is None)

    # ---- 外参闸门：垫面重投影残差 ----
    # 造 12 个「垫面点」：把三维点按**同一组** R,t 投到像素 ⇒ 残差必须为 0；
    # 再把 t 挪 5 mm，残差必须显著起来（否则闸门是个摆设）。
    pad_xy = [(0.35 + 0.02 * i, -0.20 + 0.03 * j, 0.0004)
              for i in range(4) for j in range(3)]
    imgs = [A.project_pt(p, KCAM, RCAM, TCAM) for p in pad_xy]
    rms0, du0, dv0, n0 = A.pad_reprojection_rms(pad_xy, imgs, KCAM, RCAM, TCAM)
    check('垫面残差：同参数自投 ⇒ 0 px', rms0 < 1e-6 and n0 == 12,
          'rms=%.4f n=%d' % (rms0, n0))
    # 沿**相机系 y**（像里的竖直方向）挪 5 mm —— 沿 base y 挪主要落在相机 z 上，
    # 那只是尺度变化，残差小得多，测不出闸门该有的灵敏度。
    rms1, _du1, _dv1, _n1 = A.pad_reprojection_rms(
        pad_xy, imgs, KCAM, RCAM, TCAM + RCAM @ np.array([0.0, 0.005, 0.0]))
    check('垫面残差：平移 5 mm（相机系 y）⇒ 闸门必红',
          rms1 > A.EXTR_MAX_RMS_PX,
          'rms=%.2f px（阈值 %.1f）' % (rms1, A.EXTR_MAX_RMS_PX))

    # ---- 视差闸门：逐帧标记 + 共识拦截 ----
    check('视差闸门常量与判据同量级', 5.0 <= A.PARALLAX_MAX_PX <= 30.0,
          'PARALLAX_MAX_PX=%.1f（2 mm ≈ 1.5 px ÷ 桥误差 10%%）'
          % A.PARALLAX_MAX_PX)
    r_ok = [{'dx_mm': 0.5, 'dy_mm': 0.5, 'u_mid_px': 800.0,
             'parallax_px': 10.0, 'parallax_mm': 12.0, 'dy_trusted': True}
            for _ in range(5)]
    c_ok = A.consensus(r_ok)
    check('视差小 ⇒ dy_trusted 通过', c_ok['checks'].get('dy_trusted') is True,
          repr(c_ok['checks']))
    r_bad = [dict(r) for r in r_ok]
    r_bad[2]['parallax_px'] = 120.0
    r_bad[2]['dy_trusted'] = False
    c_bad = A.consensus(r_bad)
    check('5 帧里 1 帧视差大 ⇒ 不许报收敛（中位数会抹平）',
          c_bad['checks'].get('dy_trusted') is False
          and c_bad['converged'] is False, repr(c_bad['checks']))

    # ---- 视线遮挡：两指分离轴（tool y）vs 相机水平视线 ----
    # 存的外参给出的水平视线 ≈ (-0.111, -0.994) ⇒ 相机朝 −y 看。
    # s10 的姿态：tool y ≈ base x ⇒ 与水平视线近垂直 = 侧向（实测 86.9°）。
    R_lat = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
    v = A.view_clearance_deg(R_lat, RCAM)
    check('俯仰角报得对（仰角 ~41.6°）', v['elevation_deg'] is not None
          and approx(v['elevation_deg'], 41.6, 1.0), repr(v))
    check('tool y = base x ⇒ 近垂直 = 侧向（不挡视线）',
          v['angle_deg'] is not None and v['angle_deg'] >= 70.0
          and v['lateral'] is True, repr(v))
    # 反过来：分离轴顺着相机水平视线（一前一后）⇒ 近侧手指的影子盖住块顶。
    look = RCAM[:, 2]
    h = np.array([look[0], look[1], 0.0])
    h = h / np.linalg.norm(h)
    xb = np.cross(h, np.array([0.0, 0.0, 1.0]))
    xb = xb / np.linalg.norm(xb)
    zb = np.cross(xb, h)
    R_bad = np.column_stack([xb, h, zb])       # column 1 = tool y = 视线方向
    v2 = A.view_clearance_deg(R_bad, RCAM)
    check('分离轴顺着视线 ⇒ 挡视线（闸门必须红）',
          v2['angle_deg'] is not None and v2['angle_deg'] < 15.0
          and v2['lateral'] is False, repr(v2))
    check('近垂直俯视 ⇒ 没有「水平视线」可言，返回 None 不瞎判',
          A.view_clearance_deg(np.eye(3), np.diag([1.0, -1.0, -1.0]))
          ['angle_deg'] is None,
          repr(A.view_clearance_deg(np.eye(3), np.diag([1.0, -1.0, -1.0]))))
    # 共识：逐帧都侧向才算（同视差的道理）
    for r in r_ok:
        r['view_clearance_deg'] = 84.0
        r['view_lateral'] = True
    c_v = A.consensus(r_ok)
    check('视线角全部侧向 ⇒ 通过', c_v['checks'].get('view_lateral') is True,
          repr(c_v['checks']))
    r_vbad = [dict(r) for r in r_ok]
    r_vbad[2]['view_clearance_deg'] = 8.0
    r_vbad[2]['view_lateral'] = False
    c_vbad = A.consensus(r_vbad)
    check('5 帧里 1 帧挡视线 ⇒ 不许报收敛',
          c_vbad['checks'].get('view_lateral') is False
          and c_vbad['converged'] is False, repr(c_vbad['checks']))

    # =====================================================================
    print('\n--- B. 合成场景：悬停 ---')
    URDF = find_urdf()
    if not URDF:
        print('  (跳过：没有 URDF，PRECISION_URDF 或 ~/robotics/notes/.../robot.urdf)')
        th = None
    else:
        th = urkin.Chain(URDF)
        T_hover, q_hover = hover_pose(th)
        check('悬停位 IK 收敛', T_hover is not None)
        if T_hover is None:
            th = None

    if th is not None:
        tool0 = T_hover[:3, 3]
        block_top = 0.0295

        def hover_rec(dx0, dy0, cls='yellow'):
            xy = (tool0[0] + dx0, tool0[1] + dy0)
            fr = render_scene(T_hover, xy, block_top=block_top,
                              joints=as_joints(q_hover, th))
            return A.measure_frame(None, 0, th, block_top * 1000.0,
                                   grasp_yaw_deg=0.0, expect_class=cls,
                                   prior_xy_mm=(xy[0] * 1e3, xy[1] * 1e3),
                                   frame=fr)

        # 绝对读数有 ~2 mm 的固定偏置（顶面点云混进近侧竖壁，见 detect_block
        # 的说明；规划器走同一个 uv，所以是共模的）。判的是**差分**：基准位与
        # 偏移位之间的差必须等于真实位移，任何定值偏置在这条判据里被约掉。
        try:
            base = hover_rec(0.0, 0.0)
            check('悬停基准位可测', True, '')
            check('基准位偏置 ≤3 mm（竖壁共模，不去它）',
                  abs(base['dx_mm']) <= 3.0 and abs(base['dy_mm']) <= 3.0,
                  '基准 = (%+.2f, %+.2f) mm' % (base['dx_mm'], base['dy_mm']))
        except Exception as e:                                   # noqa: BLE001
            base = None
            check('悬停基准位可测', False, '抛异常：%s' % e)

        if base is not None:
            for (dx0, dy0) in ((0.0030, -0.0020), (-0.0030, 0.0020),
                               (0.0060, 0.0040)):
                try:
                    rec = hover_rec(dx0, dy0)
                    gx = rec['dx_mm'] - base['dx_mm']
                    gy = rec['dy_mm'] - base['dy_mm']
                    ok = approx(gx, dx0 * 1e3, 0.5) and approx(gy, dy0 * 1e3, 0.5)
                    check('悬停差分 (%+.1f, %+.1f) mm' % (dx0 * 1e3, dy0 * 1e3), ok,
                          '→ (%+.2f, %+.2f)，期望 ±0.5 mm' % (gx, gy))
                except Exception as e:                               # noqa: BLE001
                    check('悬停差分 (%+.1f, %+.1f) mm' % (dx0 * 1e3, dy0 * 1e3),
                          False, '抛异常：%s' % e)

        # 类别门：期望的类不存在 ⇒ 必须报错，不许拿别的块顶数
        try:
            hover_rec(0.003, 0.0, cls='cyan')
            check('类错 ⇒ 拒（不许将就）', False, '居然出了数')
        except RuntimeError as e:
            check('类错 ⇒ 拒（不许将就）', True, str(e))

        # 判类确实在起作用：同一个块，类名对就对得上
        try:
            r = hover_rec(0.0, 0.0)
            check('判类正确（yellow）', r['block_class'] == 'yellow',
                  str(r['block_class']))
        except Exception as e:                                   # noqa: BLE001
            check('判类正确（yellow）', False, '抛异常：%s' % e)

        # 灰色块（没有可抓的彩色） ⇒ 拒
        xy = (tool0[0] + 0.003, tool0[1])
        fr = render_scene(T_hover, xy, block_top=block_top,
                          block_color=np.array([190, 190, 190], np.uint8),
                          joints=as_joints(q_hover, th))
        try:
            A.measure_frame(None, 0, th, block_top * 1000.0,
                            expect_class='yellow',
                            prior_xy_mm=(xy[0] * 1e3, xy[1] * 1e3), frame=fr)
            check('灰块 ⇒ 拒（给了类名就不许拿它顶）', False, '居然出了数')
        except RuntimeError as e:
            check('灰块 ⇒ 拒（给了类名就不许拿它顶）', True, str(e))

    # =====================================================================
    print('\n--- C. 合成场景：抓后（held） ---')
    if th is not None:
        T_g, q_g = hover_pose(th, z=0.384, yaw=0.0)
        if T_g is None:
            print('  (跳过：held 位 IK 不收敛)')
        else:
            tool0g = T_g[:3, 3]
            top_h = 0.180

            def held_rec(dx0, dy0, cls=None):
                xy = (tool0g[0] + dx0, tool0g[1] + dy0)
                fr = render_scene(T_g, xy, block_top=top_h, block_side=0.026,
                                  opening=0.0262, joints=as_joints(q_g, th))
                return A.measure_frame(None, 0, th, top_h * 1000.0, held=True,
                                       expect_class=cls, frame=fr)

            try:
                baseg = held_rec(0.0, 0.0)
                check('held 基准位可测', True, '')
                check('held 自动判类（不给 --expect-class）',
                      baseg['block_class'] == 'yellow', baseg['block_class'])
            except Exception as e:                               # noqa: BLE001
                baseg = None
                check('held 基准位可测', False, '抛异常：%s' % e)

            if baseg is not None:
                for (dx0, dy0) in ((0.0020, -0.0015), (-0.0018, 0.0022)):
                    try:
                        rec = held_rec(dx0, dy0)
                        gx = rec['dx_mm'] - baseg['dx_mm']
                        gy = rec['dy_mm'] - baseg['dy_mm']
                        ok = approx(gx, dx0 * 1e3, 0.8) and approx(gy, dy0 * 1e3,
                                                                  0.8)
                        check('held 差分 (%+.1f, %+.1f) mm' % (dx0 * 1e3, dy0 * 1e3),
                              ok, '→ (%+.2f, %+.2f)，期望 ±0.8 mm' % (gx, gy))
                    except Exception as e:                           # noqa: BLE001
                        check('held 差分 (%+.1f, %+.1f) mm'
                              % (dx0 * 1e3, dy0 * 1e3), False, '抛异常：%s' % e)

            # 抓后指与块贴合：几何分割会并簇 —— 证明颜色掩码没把指算进去
            xy = (tool0g[0] + 0.002, tool0g[1])
            fr = render_scene(T_g, xy, block_top=top_h, block_side=0.026,
                              opening=0.0262, joints=as_joints(q_g, th))
            info = A.held_block_uv(fr, 0.0, expect_class='yellow')
            check('held 顶面高度 ≈ 真值（±10 mm）',
                  approx(info['top_depth_mm'], top_h * 1e3, 10.0),
                  'top=%s' % info['top_depth_mm'])
            check('held 顶面点云横向极差 ≤ 30 mm（指没混进来）',
                  info['spread_mm'] <= 30.0, 'spread=%s' % info['spread_mm'])
            # 指是黑的：把块的颜色换成黑的，掩码就该空 ⇒ 证明隔离靠的是颜色
            fr2 = render_scene(T_g, xy, block_top=top_h, block_side=0.026,
                               opening=0.0262, block_color=BLACK,
                               joints=as_joints(q_g, th))
            try:
                A.held_block_uv(fr2, 0.0)
                check('块与指同色 ⇒ 拒（颜色掩码失效就停）', False, '居然出了数')
            except RuntimeError as e:
                check('块与指同色 ⇒ 拒（颜色掩码失效就停）', True, str(e))

    # =====================================================================
    print('\n--- D. 共识与拒帧 ---')


    def rec(dx, dy, dyaw=None, u=0.0):
        return {'dx_mm': dx, 'dy_mm': dy, 'dyaw_deg': dyaw, 'u_mid_px': u}


    good = [rec(1.2 + 0.05 * i, -0.8 + 0.03 * i, 1.0) for i in range(8)]
    good.append(rec(40.0, 40.0, 1.0))          # 一帧野值
    c = A.consensus(good)
    check('中位抗野值', approx(c['dx_mm'], 1.4, 0.2) and c['n_used'] >= 6,
          'dx=%.2f n=%d' % (c['dx_mm'], c['n_used']))
    check('共识收敛标记', c['converged'] is True, str(c['checks']))

    c2 = A.consensus([rec(0.0, 0.0, 0.0), rec(1.0, 0.0, 0.0)])
    check('帧数 <3 ⇒ n_used < 3（调用方不许当收敛）', c2['n_used'] < 3, str(c2))

    yaw_wrap = [rec(0.0, 0.0, 44.0), rec(0.0, 0.0, -44.0), rec(0.0, 0.0, 44.5),
                rec(0.0, 0.0, -45.5)]
    c3 = A.consensus(yaw_wrap)
    check('dyaw 在 ±45 边界不出 ±90 的假值',
          c3.get('dyaw_deg') is None or abs(c3['dyaw_deg']) <= 46,
          str(c3.get('dyaw_deg')))
    c4 = A.consensus([rec(5.0, 0.0, 0.0) for _ in range(5)])
    check('|e| > 2 mm ⇒ within_2mm 假', c4['checks']['within_2mm'] is False,
          str(c4['checks']))
    c5 = A.consensus([rec(5.0 + 4.0 * i, 0.0, 0.0) for i in range(5)])
    check('逐帧发散 ⇒ stable 假', c5['checks']['stable'] is False, str(c5['checks']))

    # =====================================================================
    print('\n--- E. 失败注入（必须变红） ---')
    with tempfile.TemporaryDirectory() as td:
        r = A.run_burst(os.path.join(td, 'nope'), URDF or '/dev/null', 29.5)
        check('burst 不存在 ⇒ ok=false', r['ok'] is False, r.get('reason'))

        empty = os.path.join(td, 'empty')
        os.makedirs(empty)
        r = A.run_burst(empty, URDF or '/dev/null', 29.5)
        check('空目录 ⇒ ok=false', r['ok'] is False, r.get('reason'))

    if th is not None and URDF:
        with tempfile.TemporaryDirectory() as td:
            # 好 burst：5 帧（2 暖机坏帧 + 3 好帧）应被 drop_first 丢掉坏帧
            burst = os.path.join(td, 'hover')
            os.makedirs(burst)
            xy = (tool0[0] + 0.003, tool0[1] - 0.002)
            for i in range(5):
                if i < 2:
                    # 暖机坏帧：两指还没进画面（曝光/机械延迟期那种帧）
                    fr = render_scene(T_hover, xy, fingers=False,
                                      joints=as_joints(q_hover, th))
                else:
                    fr = render_scene(T_hover, xy, joints=as_joints(q_hover, th))
                write_frame(burst, i, fr)
            r = A.run_burst(burst, URDF, 29.5, expect_class='yellow',
                            prior_xy_mm=(xy[0] * 1e3, xy[1] * 1e3),
                            drop_first=2, max_frames=5)
            check('目录端到端 ok', r['ok'] is True, r.get('reason'))
            if r['ok']:
                # 绝对读数带着顶面近壁那 ~2 mm 的共模偏置，扣掉基准再比位移
                fr0 = render_scene(T_hover, (tool0[0], tool0[1]),
                                   joints=as_joints(q_hover, th))
                b0 = A.measure_frame(None, 0, th, 29.5, expect_class='yellow',
                                     frame=fr0)
                gx = r['consensus']['dx_mm'] - b0['dx_mm']
                gy = r['consensus']['dy_mm'] - b0['dy_mm']
                check('目录端到端解出 (+3.0, -2.0) mm',
                      approx(gx, 3.0, 0.6) and approx(gy, -2.0, 0.6),
                      '差分 = (%+.2f, %+.2f)' % (gx, gy))
                check('坏帧没进共识', r['consensus']['n_used'] >= 3
                      and not r.get('errors'),
                      'n_used=%d err=%d' % (r['consensus']['n_used'],
                                            len(r.get('errors') or [])))

            # 注入：同样 5 帧但 drop_first=0 ⇒ 坏帧被计入，必须留下痕迹
            r2 = A.run_burst(burst, URDF, 29.5, expect_class='yellow',
                             prior_xy_mm=(xy[0] * 1e3, xy[1] * 1e3),
                             drop_first=0, max_frames=5)
            bad_frames = len(r2.get('errors') or [])
            spread = r2.get('consensus', {}).get('spread_mm')
            check('不丢暖机帧 ⇒ 要么报错、要么极差炸开',
                  (not r2['ok']) or bad_frames > 0 or (spread or 0) > 2.0,
                  'ok=%s err=%d spread=%s' % (r2['ok'], bad_frames, spread))

            # 注入：只剩 2 帧 ⇒ 不够下限
            short = os.path.join(td, 'short')
            os.makedirs(short)
            for i in range(2):
                write_frame(short, i, render_scene(T_hover, xy,
                                                   joints=as_joints(q_hover, th)))
            r3 = A.run_burst(short, URDF, 29.5, drop_first=0, max_frames=4)
            check('帧数 <3 ⇒ ok=false', r3['ok'] is False, r3.get('reason'))

    # =====================================================================
    print('\n--- F. 真帧回归（B1，缺则跳过） ---')
    B1 = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      'runs/2026-11-08-stage4-entry/e2/set4/b1/topcam')
    if th is not None and os.path.isdir(B1):
        fr = gc.load_frame(B1, tip=gc.TOPCAM_TIP, chain=gc.TOPCAM_CHAIN, idx=10)
        tz = A._table_z_m(fr['depth'], fr['K'], fr['R_bc'], fr['t_bc'])
        js = fr['joints']
        q = [float(js['position'][js['name'].index(n)]) for n in th.move_joints]
        T = th.fk(q)
        tip = T[:3, 3] + T[:3, :3] @ np.array([0, 0, A.TOOL0_TO_BARE_FINGERTIP])
        p_tip = A.project_pt(tip, fr['K'], fr['R_bc'], fr['t_bc'])
        f = A.detect_fingers(fr['color'], fr['depth'], tz or 0.0, fr['K'],
                             fr['R_bc'], fr['t_bc'], p_tip[0], p_tip[1])
        check('B1 两指可分辨', f is not None, repr(f))
        if f:
            check('B1 指中点 ≈785 px（±3）', approx(f['u_mid'], 785.0, 3.0),
                  'u_mid=%.2f' % f['u_mid'])
            check('B1 两指间距 ≈55 px（±6）', approx(f['sep_px'], 55.0, 6.0),
                  'sep=%.2f' % f['sep_px'])
    else:
        print('  (跳过：没有 B1 burst 或 URDF —— 真机帧不随仓库分发)')

    # =====================================================================
    print('\n--- H. 2026-11-10 s10 真帧暴露的两个坑 ---')
    if th is not None:
        xy0 = (tool0[0], tool0[1])
        fr_plain = render_scene(T_hover, xy0, block_top=block_top,
                                joints=as_joints(q_hover, th))
        # 指间再画一条 4 mm 半宽的黑色本体（s10 实测：三串深色，中间那串
        # 才是本体，两指在外面）
        fr_body = render_scene(T_hover, xy0, block_top=block_top,
                               body_half=0.004, joints=as_joints(q_hover, th))

        def _fingers(fr):
            js = fr['joints']
            q = [float(js['position'][js['name'].index(n)])
                 for n in th.move_joints]
            T = th.fk(q)
            tip = (T[:3, 3] + T[:3, :3]
                   @ np.array([0, 0, A.TOOL0_TO_BARE_FINGERTIP]))
            p = A.project_pt(tip, fr['K'], fr['R_bc'], fr['t_bc'])
            tz = A._table_z_m(fr['depth'], fr['K'], fr['R_bc'], fr['t_bc'])
            return A.detect_fingers(fr['color'], fr['depth'], tz or 0.0,
                                    fr['K'], fr['R_bc'], fr['t_bc'], p[0],
                                    p[1])

        f_plain = _fingers(fr_plain)
        f_body = _fingers(fr_body)
        check('H1 两指（指间无本体）可分辨', f_plain is not None, repr(f_plain))
        check('H1 指间多一条黑本体仍可分辨', f_body is not None, repr(f_body))
        if f_plain and f_body:
            check('H1 取的是最外一对（span），不是相邻串',
                  f_body.get('pair_mode') == 'span',
                  'pair_mode=%s n_pairs=%s' % (f_body.get('pair_mode'),
                                               f_body.get('n_pairs')))
            check('H1 至少有 2 个候选对（最宽优先真的在做选择）',
                  int(f_body.get('n_pairs') or 0) >= 2,
                  'n_pairs=%s' % f_body.get('n_pairs'))
            check('H1 本体不改变指中点 / 间距（±4 px）',
                  approx(f_body['u_mid'], f_plain['u_mid'], 4.0)
                  and approx(f_body['sep_px'], f_plain['sep_px'], 4.0),
                  'Δmid=%+.1f px  Δsep=%+.1f px' % (
                      f_body['u_mid'] - f_plain['u_mid'],
                      f_body['sep_px'] - f_plain['sep_px']))

        # 抓前小窗：v 下界必须压在指尖下方
        K0, R0, t0 = fr_plain['K'], fr_plain['R_bc'], fr_plain['t_bc']
        js = fr_plain['joints']
        qq = [float(js['position'][js['name'].index(n)])
              for n in th.move_joints]
        Tt = th.fk(qq)
        tip3 = (Tt[:3, 3] + Tt[:3, :3]
                @ np.array([0, 0, A.TOOL0_TO_BARE_FINGERTIP]))
        p_tip = A.project_pt(tip3, K0, R0, t0)
        prior = A.project_pt([xy0[0], xy0[1], block_top], K0, R0, t0)
        tz0 = A._table_z_m(fr_plain['depth'], K0, R0, t0) or 0.0
        u_rng, v_rng = A.grasp_roi(prior, p_tip[1], fr_plain['depth'].shape[0])
        check('H2 小窗上界压在指尖下方', v_rng is not None
              and v_rng[0] > p_tip[1] + 1.0,
              'v_rng=%s v_tip=%.1f' % (v_rng, p_tip[1]))
        check('H2 小窗含块顶投影', v_rng is not None
              and u_rng[0] <= prior[0] <= u_rng[1]
              and v_rng[0] <= prior[1] <= v_rng[1],
              'u_rng=%s v_rng=%s prior=(%.1f, %.1f)'
              % (u_rng, v_rng, prior[0], prior[1]))
        kw = dict(expect_class='yellow', min_h_mm=10.0, max_h_mm=120.0,
                  z_expect_mm=block_top * 1e3, z_tol_mm=60.0)
        blk_ok = A.detect_block(fr_plain, tz0, prior[0], prior[1],
                                u_range=u_rng, v_range=v_rng, **kw)
        check('H2 小窗内检出块', blk_ok is not None, repr(blk_ok))
        # 变异：裁切抬回指尖（= s10 的裁切）⇒ 窗里开始有手指像素。
        # s10 真帧里这些像素把指、块、台面桥接成一个 145×209 mm 的大簇，
        # max_h 挡不住、类别投票被非黄的像素冲掉 ⇒ 抓前 20 帧有 13 帧不出块。
        # 合成场景没有那层低处桥接（真帧才复现），所以这里守卫的是**可判定的
        # 那一半**：裁切窗必须一点手指像素都不含。
        def _black_count(vr):
            img = fr_plain['color']
            v0, v1 = max(0, int(vr[0])), min(img.shape[0] - 1, int(vr[1]))
            u0, u1 = max(0, int(u_rng[0])), min(img.shape[1] - 1, int(u_rng[1]))
            sub = img[v0:v1 + 1, u0:u1 + 1].astype(int)
            return int((np.abs(sub - BLACK.astype(int)) <= 6).all(-1).sum())

        n_ok = _black_count(v_rng)
        n_bad = _black_count((p_tip[1] - 30.0, v_rng[1]))
        check('H2 抓前小窗里一颗手指像素都没有（抬回指尖就有一大片）',
              n_ok == 0 and n_bad >= 40,
              '正常 %d px / 变异 %d px' % (n_ok, n_bad))

        # 抓后：块被举高过曝，色相漂到 yellowgreen ⇒ 类别只作核对
        T_g2, q_g2 = hover_pose(th, z=0.384, yaw=0.0)
        if T_g2 is None:
            print('  (跳过 held 过曝检查：位姿 IK 不收敛)')
        else:
            BLOWN = np.array([255, 229, 30], np.uint8)   # 色相 ≈53° yellowgreen
            xyb = (T_g2[0, 3] + 0.002, T_g2[1, 3])
            frb = render_scene(T_g2, xyb, block_top=0.180, block_side=0.026,
                               opening=0.0262, block_color=BLOWN,
                               joints=as_joints(q_g2, th))
            try:
                info = A.held_block_uv(frb, 0.0)
                check('H3 过曝块的读数是非 yellow（前提成立）',
                      info.get('class') != 'yellow', 'class=%s' % info.get('class'))
            except RuntimeError as e:                            # noqa: BLE001
                check('H3 过曝块的读数是非 yellow（前提成立）', False, str(e))
            try:
                recb = A.measure_frame(None, 0, th, 180.0, held=True,
                                       expect_class='yellow', frame=frb)
                check('H3 类别对不上仍量得出（类名降级为核对）', True,
                      'class=%s class_src=%s' % (recb.get('block_class'),
                                                 recb.get('class_src')))
                check('H3 走的是窄 ROI 分支（class_src=roi-any）',
                      recb.get('class_src') == 'roi-any',
                      str(recb.get('class_src')))
                check('H3 class_match 记 False（不假装对上了）',
                      recb.get('class_match') is False,
                      str(recb.get('class_match')))
            except RuntimeError as e:                            # noqa: BLE001
                check('H3 类别对不上仍量得出（类名降级为核对）', False, str(e))
            try:
                A.held_block_uv(frb, 0.0, expect_class='yellow')
                check('H3 变异：整幅 + 严格类过滤 ⇒ 拒（旧行为）', False,
                      '居然出了数')
            except RuntimeError as e:                            # noqa: BLE001
                check('H3 变异：整幅 + 严格类过滤 ⇒ 拒（旧行为）', True, str(e))

    # =====================================================================
    print('\n--- G. E4b 多场汇总 ---')
    _summary_section()

    # =====================================================================
    print('\n%s' % ('=' * 60))
    print('PASS %d   FAIL %d' % (N_PASS, N_FAIL))
    if FAILS:
        for f in FAILS:
            print('  ✗ %s' % f)

    sys.exit(0 if N_FAIL == 0 else 2)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""阶段 4 入口（E2/E3b 侧）的纯函数自检：**不需要 ROS、相机、图片**。

钉住三件事：

1. `topcam_marker_calib` 的帧名白名单与前缀 —— 帧名认错/前缀打错都**不报错**，
   只会让静态 TF 永远不生效，最后表现成「机械臂抓偏」；
2. `stage4_joint_calib` 的代数：两路相除、发布 TF 复核、覆盖投影；
3. 端到端合成：第 1 轮（derived）必须 ok，第 2 轮（published-tf）一致必须 ok，
   发布值偏 3 mm 必须**判红**（证明 two_way 不是恒等式）。

关键的一条：第 1 轮的 two_way 由定义恒等于 0，**不能**当证据。这里显式断言
``extrinsic.source == 'derived-this-round'``，第 2 轮才断言 `published-tf`。

跑法：``python3 scripts/test_stage4_topcam.py``；退出码 0 = 全通过。
"""

import argparse
import json
import os
import sys
import tempfile

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np                                                # noqa: E402

import stage4_joint_calib as jc                                   # noqa: E402
import topcam_marker_calib as tmc                                 # noqa: E402
import topcam_extrinsic_gate as gate                              # noqa: E402
import graspnet_candidates as gc                                  # noqa: E402

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


def safe(fn, *a, **kw):
    try:
        return fn(*a, **kw)
    except SystemExit as e:
        return f'!! 拒绝：{e}'
    except Exception as e:                                        # noqa: BLE001
        return f'!! 异常：{type(e).__name__}: {e}'


X = np.array([1.0, 0.0, 0.0])
Y = np.array([0.0, 1.0, 0.0])
Z = np.array([0.0, 0.0, 1.0])


def rot(axis, deg):
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    if axis == 'x':
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == 'y':
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def T(R, t):
    M = np.eye(4)
    M[:3, :3] = np.asarray(R, dtype=float)
    M[:3, 3] = np.asarray(t, dtype=float)
    return M


# =====================================================================
print('--- A. topcam_marker_calib：帧名白名单与前缀 ---')

R_ID = np.eye(3)
DOC_LINK = {'frame_child': 'topcam_link', 'R': R_ID.tolist(),
            't': [0.42, 0.0, 1.3]}
out = safe(tmc.base_from_camera, DOC_LINK)
check('topcam_link 被认（不再「拒绝猜」）', not isinstance(out, str), str(out))
if not isinstance(out, str):
    check('  topcam_link ⇒ 左乘 link→optical',
          np.allclose(out[0], tmc.R_LINK_TO_OPTICAL), str(out[0]))
    check('  平移不参与旋转（光学系原点=link 原点）',
          np.allclose(out[1], [0.42, 0.0, 1.3]))

out = safe(tmc.base_from_camera, {'frame_child': 'topcam_color_optical_frame',
                                  'R': R_ID.tolist(), 't': [0, 0, 0]})
check('topcam_color_optical_frame 被认且原样返回',
      not isinstance(out, str) and np.allclose(out[0], R_ID), str(out))

out = safe(tmc.base_from_camera, {'frame_child': 'topcam_optical_frame',
                                  'R': R_ID.tolist(), 't': [0, 0, 0]})
check('topcam_optical_frame 别名也认', not isinstance(out, str), str(out))

# 实测链 vs 理想旋转：必须真的不一样（否则「必须给 --tf-json」就是空话）
R_MEAS, t_MEAS = tmc.R_LINK_TO_OPTICAL @ rot('y', 0.6), np.array([0.012, -0.005, 0.009])
a_ideal = tmc.base_from_camera(DOC_LINK)
a_meas = tmc.base_from_camera(DOC_LINK, (R_MEAS, t_MEAS))
check('带实测 link→optical 时结果与理想值不同（差 >10 mm）',
      np.linalg.norm(np.asarray(a_meas[1]) - np.asarray(a_ideal[1])) > 0.010,
      'ideal=%s meas=%s' % (np.round(a_ideal[1], 4), np.round(a_meas[1], 4)))

q = tmc.matrix_to_quat(np.eye(3))
s = tmc.fmt_launch(np.eye(3), np.array([0.4, -0.1, 1.3]), prefix='topcam')
check('fmt_launch(topcam) 打的是 topcam_publish_tf', 'publish_topcam_tf:=true' in s)
check('  且参数名是 topcam_x..topcam_qw',
      all(('%s_%s:=' % ('topcam', k)) in s
          for k in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')), s)
check('  且**不含** camera_x（前缀没混）', 'camera_x:=' not in s)
s_old = tmc.fmt_launch(np.eye(3), np.array([0.4, -0.1, 1.3]))
check('旧调用（无 prefix）仍是 camera_*（回归）',
      'publish_camera_tf:=true' in s_old and 'camera_x:=0.400000' in s_old)

# tf.json 链
tf_doc = {'ok': {
    'topcam_link:topcam_color_frame': {
        'translation': {'x': 0.0, 'y': 0.0, 'z': 0.0},
        'rotation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}},
    'topcam_color_frame:topcam_color_optical_frame': {
        'translation': {'x': 0.012, 'y': -0.005, 'z': 0.009},
        'rotation': {'x': -0.5, 'y': 0.5, 'z': -0.5, 'w': 0.5}},
}}
Rc, tc = tmc.chain_from_tf(tf_doc, tmc.TOPCAM_LINK_TO_OPTICAL_KEYS)
check('chain_from_tf 取到驱动实测的 link→optical',
      np.allclose(tc, [0.012, -0.005, 0.009]), str(tc))
check('  旋转与 REP-103 理想值一致（这份 tf 就是理想链）',
      np.allclose(Rc, tmc.R_LINK_TO_OPTICAL))
bad = safe(tmc.chain_from_tf, {'ok': {}}, tmc.TOPCAM_LINK_TO_OPTICAL_KEYS)
check('缺一段 ⇒ 拒绝（不拿理想值顶替）', isinstance(bad, str), str(bad))

# =====================================================================
print('\n--- B. stage4_joint_calib：差值代数 ---')

d = jc._rel_delta(np.eye(4), np.eye(4))
check('同一变换 ⇒ 0 mm / 0°',
      d['translation_mm'] < 1e-9 and d['angle_deg'] < 1e-6, str(d))

Tb = T(rot('z', 0.5), [0.003, 0.0, 0.0])
d = jc._rel_delta(np.eye(4), Tb)
check('偏 3 mm ⇒ translation_mm≈3', abs(d['translation_mm'] - 3.0) < 1e-6, str(d))
check('偏 0.5° ⇒ angle_deg≈0.5', abs(d['angle_deg'] - 0.5) < 1e-6, str(d))
check('  且 yaw_deg 也≈0.5（口径与 stage2 一致）',
      abs(d['yaw_deg'] - 0.5) < 1e-6, str(d))

Tb2 = T(rot('x', 0.5), [0.0, 0.0, 0.0])
d = jc._rel_delta(np.eye(4), Tb2)
check('纯 tilt 0.5°：angle≈0.5 但 yaw≈0（两口径不是一回事）',
      abs(d['angle_deg'] - 0.5) < 1e-6 and abs(d['yaw_deg']) < 1e-6, str(d))

# =====================================================================
print('\n--- C. 从文件 / tf.json 取已发布外参 ---')

T_LINK = T(rot('x', 170.0), [0.42, 0.0, 1.3])
got = jc._extrinsic_link_from_doc({'R_link': T_LINK[:3, :3].tolist(),
                                   't_link': T_LINK[:3, 3].tolist()})
check('R_link/t_link 写法被认', np.allclose(got, T_LINK))
got = jc._extrinsic_link_from_doc({'frame_child': 'topcam_link',
                                   'R': T_LINK[:3, :3].tolist(),
                                   't': T_LINK[:3, 3].tolist()})
check('frame_child=topcam_link 的 R/t 写法被认', np.allclose(got, T_LINK))
bad = safe(jc._extrinsic_link_from_doc,
           {'frame_child': 'camera_link', 'R': np.eye(3).tolist(), 't': [0, 0, 0]})
check('别的 frame_child ⇒ 拒绝（不猜）', isinstance(bad, str), str(bad))

doc = {'ok': {'base_link:topcam_link': {
    'translation': {'x': 0.42, 'y': 0.0, 'z': 1.3},
    'rotation': {k: float(v) for k, v in zip(
        'xyzw', tmc.matrix_to_quat(rot('x', 170.0)))}}}}
got = jc._published_link(doc)
check('tf.json 里的 base_link:topcam_link 被取到',
      got is not None and np.allclose(got, T_LINK), str(got))
check('没有那一段 ⇒ None（第 1 轮就该走 derived）',
      jc._published_link({'ok': {}}) is None)

# =====================================================================
print('\n--- D. 覆盖投影 ---')

K = np.array([[600.0, 0.0, 640.0], [0.0, 600.0, 360.0], [0.0, 0.0, 1.0]])
DIST = np.zeros((5, 1))
META = {'pitch_mm': 30.0,
        'grid': {'cells': [{'center_pad_mm': [dx, dy]}
                           for dx in (-30.0, 0.0, 30.0)
                           for dy in (-30.0, 0.0, 30.0)]}}
T_BASE_PAD = T(rot('z', 12.0), [0.422, -0.115, 0.0039])
R_CAM = rot('x', 180.0)                     # 光轴朝 −z（俯视）
T_BASE_CAM = T(R_CAM, [0.42, 0.0, 1.30])
T_CAM_BASE = np.linalg.inv(T_BASE_CAM)

cov = jc.coverage(META, T_BASE_PAD, T_CAM_BASE, K, DIST, 1280, 720)
check('合成俯视：拾取区在画幅内', cov['pick_zone']['in_frame'],
      json.dumps(cov['pick_zone']['uv_px']))
check('  宫格四角在画幅内', cov['grid_corners']['in_frame'],
      json.dumps(cov['grid_corners']['uv_px']))
check('  悬停点在画幅内', cov['hover_points']['in_frame'],
      json.dumps(cov['hover_points']['uv_px']))
check('  无标记信息时不做标记判据', cov['ok'] and 'markers' not in cov)

cov_small = jc.coverage(META, T_BASE_PAD, T_CAM_BASE, K, DIST, 200, 200)
check('画幅缩到 200×200 ⇒ 宫格跑出画面，判红',
      not cov_small['grid_corners']['in_frame'] and not cov_small['ok'])

cov_miss = jc.coverage(META, T_BASE_PAD, T_CAM_BASE, K, DIST, 1280, 720,
                       detected={'a': [0, 1, 2, 3], 'b': [0, 1, 2, 3],
                                 'a_missing': [4], 'b_missing': []})
check('页 A 缺一个标记 ⇒ 判红（两页必须全检出）',
      not cov_miss['markers']['all_detected'] and not cov_miss['ok'])

# =====================================================================
print('\n--- E. 端到端合成：第 1 轮 derived / 第 2 轮 published ---')

T_CAM_PAD_TRUE = np.linalg.inv(T_BASE_CAM) @ T_BASE_PAD
R_LO, t_LO = tmc.R_LINK_TO_OPTICAL, np.array([0.012, -0.005, 0.009])
T_LINK_OPT = T(R_LO, t_LO)
T_BASE_LINK_TRUE = T_BASE_CAM @ np.linalg.inv(T_LINK_OPT)

TF_CHAIN = {
    'topcam_link:topcam_color_frame': {
        'translation': {'x': 0.0, 'y': 0.0, 'z': 0.0},
        'rotation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}},
    'topcam_color_frame:topcam_color_optical_frame': {
        'translation': {k: float(v) for k, v in zip('xyz', t_LO)},
        'rotation': {k: float(v) for k, v in zip('xyzw', tmc.matrix_to_quat(R_LO))}},
}


def _tf_with_published(T_bl):
    d = {'ok': dict(TF_CHAIN)}
    if T_bl is not None:
        d['ok']['base_link:topcam_link'] = {
            'translation': {k: float(v) for k, v in zip('xyz', T_bl[:3, 3])},
            'rotation': {k: float(v) for k, v in zip(
                'xyzw', tmc.matrix_to_quat(T_bl[:3, :3]))}}
    return d


_MAT = tempfile.mktemp(suffix='.json')
with open(_MAT, 'w') as fh:
    json.dump(META, fh)


def make_args(**kw):
    a = argparse.Namespace(
        wrist_frame='/tmp/wrist', topcam_frame='/tmp/topcam', mat=_MAT,
        base_topcam_link=None, max_trans_mm=2.0, max_rot_deg=0.5,
        max_rms_px=1.5, min_markers=4,
        margin_px=20.0, pick_zone=list(jc.PICK_ZONE),
        table_z_mm=jc.TABLE_Z_MM, block_mm=29.5, hover_mm=80.0)
    for k, v in kw.items():
        setattr(a, k, v)
    return a


def run_case(tf_doc, **kw):
    """打桩掉帧 I/O 与 PnP，只让 joint_calib 的代数跑。"""
    orig = (jc.gc.load_frame, jc.load_camera_frame, jc.load_tf,
            jc.sp.solve_pad)
    calls = {'n': 0}

    def stub_solve(color, K_, R_bc, t_bc, meta, **kws):
        calls['n'] += 1
        is_top = np.allclose(np.asarray(R_bc), np.eye(3)) and \
            np.allclose(np.asarray(t_bc), 0.0)
        Tw = T_BASE_PAD if calls['n'] == 1 else T_CAM_PAD_TRUE
        if is_top != (calls['n'] == 2):
            raise AssertionError('solve_pad 调用顺序/身份不对：%d %s'
                                 % (calls['n'], is_top))
        return {'ok': True, 'adopted': 'joint', 'T_A_B': None,
                'T_base_pad': Tw,
                'checks': {'rms_adopted_px': 0.5, 'adopted': 'joint',
                           'page_a': {'ids': list(range(10)), 'missing_ids': []},
                           'page_b': {'ids': list(range(10)), 'missing_ids': []}}}

    jc.gc.load_frame = lambda *a, **k: {
        'color': np.zeros((720, 1280, 3), dtype=np.uint8), 'K': K,
        'R_bc': T_BASE_PAD[:3, :3], 't_bc': T_BASE_PAD[:3, 3]}
    jc.load_camera_frame = lambda *a, **k: {
        'color': np.zeros((720, 1280, 3), dtype=np.uint8), 'K': K,
        'dist': DIST, 'width': 1280, 'height': 720, 'meta': META}
    jc.load_tf = lambda *a, **k: tf_doc
    jc.sp.solve_pad = stub_solve
    try:
        return jc.run(make_args(**kw))
    finally:
        (jc.gc.load_frame, jc.load_camera_frame, jc.load_tf,
         jc.sp.solve_pad) = orig


r1 = run_case(_tf_with_published(None))
check('第 1 轮：extrinsic.source == derived-this-round',
      r1['extrinsic']['source'] == 'derived-this-round',
      r1['extrinsic']['source'])
check('  且 two_way 数值≈0（代数恒等式，**不是**证据）',
      r1['two_way']['translation_mm'] < 1e-9)
check('  记录里明确写了「不是独立证据」', '代数恒等式' in r1.get('note', ''))
check('  link→optical 来源是 tf-json（不是理想值）',
      r1['link_to_optical']['source'] == 'tf-json')
check('  整体 ok', r1['ok'], str(r1['reasons']))

r2 = run_case(_tf_with_published(T_BASE_LINK_TRUE))
check('第 2 轮：extrinsic.source == published-tf',
      r2['extrinsic']['source'] == 'published-tf', r2['extrinsic']['source'])
check('  一致时 two_way≈0 且 ok', r2['two_way']['translation_mm'] < 1e-6
      and r2['ok'], '%.3g mm' % r2['two_way']['translation_mm'])
check('  写出的 link 外参与真值一致',
      np.allclose(np.array(r2['extrinsic_link']['R_link']),
                  T_BASE_LINK_TRUE[:3, :3], atol=1e-9)
      and np.allclose(r2['extrinsic_link']['t_link'],
                      T_BASE_LINK_TRUE[:3, 3], atol=1e-9))

T_BAD = T_BASE_LINK_TRUE.copy()
T_BAD[0, 3] += 0.003
r_bad = run_case(_tf_with_published(T_BAD))
check('第 2 轮发布值偏 3 mm ⇒ 判红', not r_bad['ok'], str(r_bad['reasons']))
check('  差值约 3 mm', abs(r_bad['two_way']['translation_mm'] - 3.0) < 0.05,
      '%.3f mm' % r_bad['two_way']['translation_mm'])
check('  判红原因里点了「两路」',
      any('两路' in x for x in r_bad['reasons']), str(r_bad['reasons']))

# 旋到 5° 也应判红（防止只查平移）
T_ROT = T_BASE_LINK_TRUE.copy()
T_ROT[:3, :3] = rot('z', 5.0) @ T_ROT[:3, :3]
r_rot = run_case(_tf_with_published(T_ROT))
check('第 2 轮发布值转 5° ⇒ 判红', not r_rot['ok'])
check('  转角 ≈5°', abs(r_rot['two_way']['angle_deg'] - 5.0) < 0.05,
      '%.3f°' % r_rot['two_way']['angle_deg'])

# 帧的 tf.json 不完整 ⇒ 致命，不拿理想值顶
r_nochain = run_case({'ok': {}})
check('tf.json 缺 link→optical ⇒ fatal（不拿理想值顶替）',
      r_nochain.get('fatal') is True, str(r_nochain.get('reasons')))

os.unlink(_MAT)

# =====================================================================
print('\n--- F. E3b 外参门：增量门与 fail-closed 次序 ---')


def _T2(R, t):
    return T(R, t)


d0 = gate._diff(_T2(np.eye(3), [0, 0, 0]), _T2(np.eye(3), [0, 0, 0]))
check('同一外参 ⇒ 0 mm / 0°', d0[0] < 1e-9 and d0[1] < 1e-9, str(d0))
d1 = gate._diff(T_BASE_CAM, _T2(T_BASE_CAM[:3, :3], T_BASE_CAM[:3, 3] + [0.001, 0, 0]))
check('偏 1 mm ⇒ 1.0 mm', abs(d1[0] - 1.0) < 1e-6, str(d1))
d2 = gate._diff(T_BASE_CAM, _T2(rot('z', 0.2) @ T_BASE_CAM[:3, :3],
                                T_BASE_CAM[:3, 3]))
check('偏 0.2° ⇒ 0.2°', abs(d2[1] - 0.2) < 1e-6, str(d2))

T_OPT_TRUE = T_BASE_CAM
doc_opt = {'frame_child': 'topcam_color_optical_frame', 'R': T_OPT_TRUE[:3, :3].tolist(),
           't': T_OPT_TRUE[:3, 3].tolist()}
check('optical_from_doc：光学系 R/t 原样用',
      np.allclose(gate.optical_from_doc(doc_opt), T_OPT_TRUE))
doc_link2 = {'R_link': T_BASE_LINK_TRUE[:3, :3].tolist(),
             't_link': T_BASE_LINK_TRUE[:3, 3].tolist()}
check('optical_from_doc：R_link 左乘 link→optical',
      np.allclose(gate.optical_from_doc(doc_link2, (R_LO, t_LO)),
                  T_BASE_CAM, atol=1e-9))
check('optical_from_doc：别的帧名 ⇒ GateData（退出 2，不猜）',
      isinstance(safe(gate.optical_from_doc, {'frame_child': 'foo',
                                              'R': np.eye(3).tolist(),
                                              't': [0, 0, 0]}), str))

# -- solve_extrinsic：用打桩的板检测，检查 base←board ∘ cam←board 的代数
_STORE = tempfile.mktemp(suffix='.json')
_BOARD = tempfile.mktemp(suffix='.json')
_INFO = tempfile.mktemp(suffix='.json')
T_BB = T_BASE_PAD                      # 就拿垫位姿当「板在 base 系」
# cam←板 = (base←cam)^-1 ∘ (base←板)
R_cb, t_cb = tmc.compose(tmc.invert((T_BASE_CAM[:3, :3], T_BASE_CAM[:3, 3])),
                         (T_BB[:3, :3], T_BB[:3, 3]))
with open(_STORE, 'w') as fh:
    json.dump({'base_from_board_R': T_BB[:3, :3].tolist(),
               'base_from_board_t': T_BB[:3, 3].tolist()}, fh)
with open(_BOARD, 'w') as fh:
    json.dump({'square_mm': 30, 'marker_mm': 22.5, 'squares_x': 5,
               'squares_y': 5, 'dictionary': 'DICT_4X4_50'}, fh)
with open(_INFO, 'w') as fh:
    json.dump({'fx': 600, 'fy': 600, 'cx': 640, 'cy': 360,
               'width': 1280, 'height': 720, 'D': [0, 0, 0, 0, 0]}, fh)


def _with_detect(fn):
    orig = (gate.tmc.load_camera, gate.tmc.detect_board)
    gate.tmc.load_camera = lambda p: (K, np.zeros((5, 1)), (1280, 720))
    gate.tmc.detect_board = fn
    try:
        return gate.solve_extrinsic('x.png', _INFO, _BOARD, _STORE, None)
    finally:
        gate.tmc.load_camera, gate.tmc.detect_board = orig


sol = _with_detect(lambda *a, **k: ((R_cb, t_cb), 16, 0.4))
check('solve_extrinsic：base←cam = (base←板)∘(cam←板)^-1',
      np.allclose(sol['T_optical'], T_BASE_CAM, atol=1e-9),
      str(np.round(sol['T_optical'], 4)))
check('  RMS 超限 ⇒ GateData（退出 2）',
      isinstance(safe(_with_detect, lambda *a, **k: ((R_cb, t_cb), 16, 3.0)),
                 str))
check('  板没检出 ⇒ GateData（退出 2）',
      isinstance(safe(_with_detect, lambda *a, **k: None), str))
check('  link→optical 来源标成 ideal（没给 tf.json）',
      sol['link_to_optical_source'] == 'ideal')

# -- run() 编排：用打桩的 solve / joint / drift
_ARGS_TMP = tempfile.mkdtemp(prefix='gate-test-')


def gate_args(**kw):
    a = argparse.Namespace(
        round=1, image='/tmp/new.png', info=_INFO,
        prev_image=None, new_image=None, board=_BOARD, store=_STORE,
        tf_json=None, prev_extrinsics=None, reimage=None, reinfo=None,
        wrist_frame=None, topcam_frame=None, mat=_MAT,
        max_rms_px=1.0, out_dir=_ARGS_TMP,
        max_drift_mm=1.0, max_drift_deg=0.2,
        max_trans_mm=2.0, max_rot_deg=0.5)
    for k, v in kw.items():
        setattr(a, k, v)
    return a


def gate_run(drift_rc=None, sol_shift=None, sol_shifts=None, joint_ok=True,
             **kw):
    orig = (gate.run_drift, gate.solve_extrinsic, gate.joint_check)
    gate.run_drift = lambda *a, **k: {
        'ran': True, 'rc': drift_rc, 'verdict': '打桩', 'tail': []}
    seq = list(sol_shifts) if sol_shifts is not None else [sol_shift]
    idx = {'n': 0}

    def stub_solve(*a, **k):
        sh = seq[min(idx['n'], len(seq) - 1)]
        idx['n'] += 1
        Tm = (T_OPT_TRUE if sh is None else
              _T2(T_OPT_TRUE[:3, :3], T_OPT_TRUE[:3, 3] + np.asarray(sh)))
        return {'T_optical': Tm, 'R': Tm[:3, :3], 't': Tm[:3, 3],
                'R_link': T_BASE_LINK_TRUE[:3, :3],
                't_link': T_BASE_LINK_TRUE[:3, 3], 'rms_px': 0.5,
                'n_corners': 16, 'link_to_optical_source': 'tf-json',
                'image': 'x'}

    gate.solve_extrinsic = stub_solve
    gate.joint_check = lambda *a, **k: {
        'ok': joint_ok, 'two_way': {'translation_mm': 0.2, 'angle_deg': 0.01},
        'extrinsic': {'source': 'published-tf'},
        'reasons': [] if joint_ok else ['打桩不过']}
    try:
        return gate.run(gate_args(**kw))
    finally:
        gate.run_drift, gate.solve_extrinsic, gate.joint_check = orig


rep, code = gate_run(drift_rc=2, prev_image='/tmp/old.png',
                     new_image='/tmp/new.png',
                     wrist_frame='/tmp/w', topcam_frame='/tmp/t')
check('drift 返回 2（证据不足）⇒ **继续**进 PnP 并最终通过（不放行旧值）',
      rep['drift']['rc'] == 2 and rep['pnp']['ok'] and code == 0,
      'code=%s reasons=%s' % (code, rep['reasons']))

rep, code = gate_run(drift_rc=0, prev_image='/tmp/old.png',
                     new_image='/tmp/new.png')
check('drift 返回 0（不该出现）⇒ 停车', code == 1 and any(
    '只允许 1/2' in x for x in rep['reasons']), str(rep['reasons']))

rep, code = gate_run(drift_rc=1, prev_image='/tmp/old.png',
                     new_image='/tmp/new.png')
check('给了 --prev-image/--new-image ⇒ drift 真的跑了',
      rep['drift']['ran'] and rep['drift'].get('policy'))

rep, code = gate_run(drift_rc=None)
check('第 1 轮无上次图 ⇒ drift 明确记 skipped（不假装做过）',
      rep['drift']['ran'] is False and '第 1 轮' in rep['drift']['reason'])

PREV = os.path.join(_ARGS_TMP, 'prev.json')
with open(PREV, 'w') as fh:
    json.dump({'frame_child': 'topcam_color_optical_frame',
               'R': T_OPT_TRUE[:3, :3].tolist(),
               't': T_OPT_TRUE[:3, 3].tolist()}, fh)

rep, code = gate_run(drift_rc=None, prev_extrinsics=PREV,
                     sol_shift=[0.005, 0.0, 0.0],
                     wrist_frame='/tmp/w', topcam_frame='/tmp/t')
check('增量超 5 mm 且没给 --reimage ⇒ 判红（停车）',
      code == 1 and any('没给 --reimage' in x for x in rep['reasons']),
      str(rep['reasons']))
check('  compare 里差 ≈5 mm', abs(rep['compare']['t_mm'] - 5.0) < 0.05,
      str(rep['compare']))

rep, code = gate_run(drift_rc=None, prev_extrinsics=PREV, reimage='/tmp/re.png',
                     sol_shifts=[[0.005, 0.0, 0.0], [0.0, 0.0, 0.0]],
                     wrist_frame='/tmp/w', topcam_frame='/tmp/t')
check('给了 --reimage 且重解后与存盘一致 ⇒ resolved 且整体通过',
      code == 0 and rep['compare'].get('resolved') is True, str(rep['reasons']))

rep, code = gate_run(drift_rc=None, prev_extrinsics=None,
                     wrist_frame='/tmp/w', topcam_frame='/tmp/t')
check('第 1 轮（无存盘外参）⇒ baseline，不是「差 0 mm」',
      rep['compare'].get('baseline') is True and '基准' in rep['compare']['reason'])

rep, code = gate_run(drift_rc=None, prev_extrinsics=None)
check('缺端到端两路复核 ⇒ 不算通过（fail-closed）',
      code == 1 and any('端到端' in x for x in rep['reasons']),
      str(rep['reasons']))

rep, code = gate_run(drift_rc=None, prev_extrinsics=None, joint_ok=False,
                     wrist_frame='/tmp/w', topcam_frame='/tmp/t')
check('端到端两路复核不过 ⇒ 判红', code == 1)

os.unlink(_STORE)
os.unlink(_BOARD)
os.unlink(_INFO)

print('\n通过 %d，失败 %d' % (N_PASS, N_FAIL))
sys.exit(0 if N_FAIL == 0 else 1)

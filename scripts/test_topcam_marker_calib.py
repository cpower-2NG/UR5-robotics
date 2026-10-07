#!/usr/bin/env python3
"""`topcam_marker_calib.py` 的纯函数自检（**不需要 ROS、相机、图片、标定板**）。

要钉住的是**这一条链的数学** —— 它整条建立在一个复合上：

    base←cam = (base←板) ∘ (cam←板)^-1

复合的顺序、求逆的写法、以及「板位姿怎么由一次已知可信的外参推出来」，
只要错一处，结果会是一个**看着完全合理**的相机位姿（数值都是三位数），
而外参错了的表现是「机械臂抓偏」—— 与坐标系错的表现一模一样，很难倒推。
所以这里逐条校验代数性质。

跑法：
    python3 scripts/test_topcam_marker_calib.py
退出码 0 = 全通过。
"""

import importlib.util
import math
import os
import shutil
import sys
import tempfile

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np                                                # noqa: E402
from topcam_marker_calib import (                                 # noqa: E402
    R_LINK_TO_OPTICAL, base_from_camera, baseline_fingerprint, baseline_state,
    compose, invert, matrix_to_quat, optical_to_link, quat_to_matrix)

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
    except SystemExit as e:                   # 本文件里「拒绝」是用 SystemExit 表达的
        return f'!! 拒绝：{e}'
    except Exception as e:                    # noqa: BLE001
        return f'!! 崩了：{type(e).__name__}: {e}'


# 一个任意的刚体变换，当测试素材
R_TEST = quat_to_matrix([0.3, -0.2, 0.5, 0.78])
T_TEST = np.array([0.35, 0.87, 0.65])
A = (R_TEST, T_TEST)

print('--- A. 四元数 ↔ 矩阵 ---')
check('往返一致（矩阵 → 四元数 → 矩阵）',
      np.allclose(quat_to_matrix(matrix_to_quat(R_TEST)), R_TEST, atol=1e-12))
check('单位化：任意缩放的四元数给出同一个矩阵',
      np.allclose(quat_to_matrix([0.6, -0.4, 1.0, 1.56]),
                  quat_to_matrix([0.3, -0.2, 0.5, 0.78]), atol=1e-12))
check('det = +1（是旋转不是镜像）',
      abs(float(np.linalg.det(quat_to_matrix(matrix_to_quat(R_TEST)))) - 1.0) < 1e-12)
check('零四元数 ⇒ 抛（不返回垃圾矩阵）',
      isinstance(safe(quat_to_matrix, [0, 0, 0, 0]), str))
# 覆盖 matrix_to_quat 的四个分支：绕 x / y / z 各 180°，以及小角度
for axis, name in ((np.array([1.0, 0, 0]), 'x'), (np.array([0, 1.0, 0]), 'y'),
                   (np.array([0, 0, 1.0]), 'z')):
    for deg in (180.0, 179.0, 1.0):
        th = math.radians(deg)
        K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]],
                      [-axis[1], axis[0], 0]])
        R = np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * (K @ K)
        check(f'绕 {name} 轴 {deg:g}° 的往返（trace 分支的另一条路）',
              np.allclose(quat_to_matrix(matrix_to_quat(R)), R, atol=1e-9))

print('\n--- B. 复合与求逆 ---')
check('逆：A ∘ A⁻¹ = 单位变换',
      all(np.allclose(x, y, atol=1e-12)
          for x, y in zip(compose(A, invert(A)), (np.eye(3), np.zeros(3)))))
check('逆：A⁻¹ ∘ A = 单位变换',
      all(np.allclose(x, y, atol=1e-12)
          for x, y in zip(compose(invert(A), A), (np.eye(3), np.zeros(3)))))
check('★ 求逆用的是 −Rᵀt，不是 −t（把两者写反在数值上也是「像样的三位数」）',
      np.allclose(invert(A)[1], -R_TEST.T @ T_TEST)
      and not np.allclose(invert(A)[1], -T_TEST))
B = (quat_to_matrix([-0.1, 0.4, 0.2, 0.88]), np.array([-0.2, 0.1, 0.05]))
check('复合不交换：A∘B ≠ B∘A（顺序有意义）',
      not np.allclose(compose(A, B)[1], compose(B, A)[1]))

print('\n--- C. 帧名判定（拒绝猜） ---')
ARCH = {'R': R_TEST.tolist(), 't': T_TEST.tolist(),
        'frame_parent': 'base_link', 'frame_child': 'camera_link'}
Rb, tb = safe(base_from_camera, ARCH)
check('camera_link ⇒ 左乘固定旋转',
      np.allclose(Rb, R_TEST @ R_LINK_TO_OPTICAL) and np.allclose(tb, T_TEST))
check('  t 不参与旋转（光心在 parent 系就是 t）', np.allclose(tb, T_TEST))
Rb2, tb2 = safe(base_from_camera, {**ARCH, 'frame_child': 'camera_color_optical_frame'})
check('★ 已是光学系 ⇒ 原样返回，不重复左乘', np.allclose(Rb2, R_TEST))
check('  且与 camera_link 那支不相等（说明这一步真做了事）',
      not np.allclose(Rb, Rb2))
check('`camera_frame` 这个键名也认',
      np.allclose(safe(base_from_camera,
                       {k: v for k, v in ARCH.items() if k != 'frame_child'}
                       | {'camera_frame': 'camera_color_optical_frame'})[0], R_TEST))
check('★ 不认识的帧名 ⇒ 拒绝（不猜）',
      isinstance(safe(base_from_camera, {**ARCH, 'frame_child': 'some_frame'}), str))
check('  缺帧名也拒绝', isinstance(safe(base_from_camera, {'R': R_TEST.tolist(),
                                                          't': T_TEST.tolist()}), str))
check('四元数形式（q）也认，且与 R 一致',
      np.allclose(safe(base_from_camera, {'q': matrix_to_quat(R_TEST).tolist(),
                                          't': T_TEST.tolist(),
                                          'frame_child': 'camera_link'})[0],
                  R_TEST @ R_LINK_TO_OPTICAL, atol=1e-9))

print('\n--- D. ★ 整条链的闭环：init 之后再 solve，应当拿回原外参 ---')
# 设相机真值 base←cam；由它和某次观测 cam←板 推出 base←板（--init）；
# 再用**另一次**观测（板没动、相机漂了）反推 base←cam（--solve）。
base_from_cam_true = (quat_to_matrix([0.02, 0.31, 0.64, 0.70]),
                      np.array([0.35, 0.87, 0.65]))
cam_from_board_init = (quat_to_matrix([0.1, -0.5, 0.2, 0.83]),
                       np.array([0.05, -0.02, 0.62]))
base_from_board = compose(base_from_cam_true, cam_from_board_init)
# 相机漂了一个小旋转：新的 cam←板 = 漂移 ∘ 旧的
R_drift = quat_to_matrix([0.0, 0.0, math.sin(math.radians(0.5) / 2),
                          math.cos(math.radians(0.5) / 2)])
cam_from_board_new = compose((R_drift, np.zeros(3)), cam_from_board_init)
base_from_cam_solved = compose(base_from_board, invert(cam_from_board_new))
ang = math.degrees(math.acos(min(1.0, max(-1.0,
    (np.trace(base_from_cam_solved[0].T @ base_from_cam_true[0]) - 1.0) / 2.0))))
check('★ 无漂移时 solve 精确还原 init 的外参',
      all(np.allclose(x, y, atol=1e-12) for x, y in zip(
          compose(base_from_board, invert(cam_from_board_init)),
          base_from_cam_true)))
check(f'★ 漂 0.5° 时 solve 出来的位姿也正好偏 0.5°（差 {ang:.3f}°）',
      abs(ang - 0.5) < 1e-6, f'got {ang}')
check('  位移不受纯转动漂移影响（相机光心没动）',
      np.allclose(base_from_cam_solved[1], base_from_cam_true[1], atol=1e-12))
check('★ 反过来：板若被写错（把 cam←板 当成 base←板），闭环就崩 —— 值得显式记',
      not np.allclose(compose(A, invert(B))[0], compose(A, invert(A))[0]))


def angle_deg(R1, R2):
    """两个旋转之间的夹角（度）。"""
    return math.degrees(math.acos(min(1.0, max(-1.0,
        (np.trace(np.asarray(R1, float).T @ np.asarray(R2, float)) - 1.0) / 2.0))))


print('\n--- E. ★ 约定闭环：--solve 印的必须与 --init 喂的同一个约定 ---')
# `real.launch.py` 把相机 TF 发在 base_link → camera_link 上，站点配置的
# camera_q* 也是这个约定；而工具内部（以及 OpenCV 的 cam←板）跑的是光学系。
# 两者差一个**固定 120°**的轴约定旋转。漏掉换算，TF 就整体转错 120°，
# 而**不会报任何错** —— 只让 /targets_3d 全错，表现与「标定不准」一模一样。
#
# ⚠️ D 段抓不到这个：D 用的是合成位姿，从没经过 base_from_camera，
# 所以「把换算去掉」这种改动在 D 段**改前改后都绿**。
ANG_LO = angle_deg(R_LINK_TO_OPTICAL, np.eye(3))
check(f'前提：link↔optical 这个固定旋转就是 120°（实测 {ANG_LO:.3f}°）',
      abs(ANG_LO - 120.0) < 1e-9)

OPT = base_from_camera(ARCH)[0]          # camera_link 文件 → 工具内部的光学系
check('★ optical_to_link(base_from_camera(文件)) 拿回文件里的 R（不差 120°）',
      np.allclose(optical_to_link(OPT), R_TEST))
check('  未换算的那个确实不等于文件里的 R（说明这一步真做了事）',
      not np.allclose(OPT, R_TEST))
check(f'  两者的夹角正是那个固定旋转（{ANG_LO:.3f}°）',
      abs(angle_deg(OPT, R_TEST) - ANG_LO) < 1e-9)
check('  且与源文件里明写的 q 也对得上（不只是与 R 自洽）',
      np.allclose(matrix_to_quat(optical_to_link(OPT)),
                  matrix_to_quat(R_TEST), atol=1e-9))

# 端到端：照 --init / --solve 的实际算法走一遍（同一观测 ⇒ 应当精确还原）
OBS = (quat_to_matrix([0.1, -0.5, 0.2, 0.83]), np.array([0.05, -0.02, 0.62]))
STORE = compose(base_from_camera(ARCH), OBS)     # --init 存进 ~/topcam_marker.json 的
SOLVED = compose(STORE, invert(OBS))             # --solve（板没动、同一次观测）
check('★ 端到端：同一观测下 --solve 印回来的 == --init 喂进去的 R',
      np.allclose(optical_to_link(SOLVED[0]), R_TEST, atol=1e-12))
check('  不换算则差 120°（这就是「印错系」那个缺陷的签名）',
      abs(angle_deg(SOLVED[0], R_TEST) - ANG_LO) < 1e-9)
check('  t 不受约定影响（光心在 parent 系就是 t，与子帧朝向无关）',
      np.allclose(SOLVED[1], T_TEST, atol=1e-12))

print('\n--- F. ★ 基准闸门：`--solve` 是**相对的**，基准换了必须拒绝算 ---')
# 这一节防的是恒久层记过的那条「**基准要一起重写**」。它原来的形状是文档里的一句
# 前提 —— 而前提没人执行。现在做成闸门，这里就钉住「闸门真的拦得住」。

ARCH_DOC = {'frame_child': 'camera_link',
            'R': R_TEST.tolist(), 't': T_TEST.tolist()}
OTHER_DOC = {'frame_child': 'camera_link',
             'R': quat_to_matrix([0.2, 0.4, -0.1, 0.89]).tolist(),
             't': [0.36, 0.87, 0.65]}


def _store_of(doc, with_fp=True):
    """照 `--init` 实际写进去的形状造一份 store。"""
    d = {'from_extrinsics': '/tmp/某一份外参.json'}
    if with_fp:
        fp = baseline_fingerprint(doc)
        d['from_extrinsics_R'], d['from_extrinsics_t'] = fp['R'], fp['t']
    return d


ST_OK = _store_of(ARCH_DOC)
check('基准那份外参照旧 ⇒ ok（放行）',
      baseline_state(ST_OK, ARCH_DOC)[0] == 'ok')
st_changed, why = baseline_state(ST_OK, OTHER_DOC)
check('★ 基准那份外参**被换过了** ⇒ changed，且**拒绝**（不是警告）',
      st_changed == 'changed', '%s / %s' % (st_changed, why[:60]))
check('  文案说清「这**不是**相机漂移，是旧基准 vs 新真值的差」'
      '（不然那个数会被读成「漂了多少」）',
      '不是**相机漂移' in why, why[:200])
check('  并且给出**下一步**（用新外参重跑 --init），不是只喊一句错',
      '重跑一次 `--init`' in why, why[-140:])
st, why = baseline_state(ST_OK, None, load_error='FileNotFoundError: 没有那个文件')
check('基准文件读不到了 ⇒ missing、拒绝（唯一的自检正是拿它比的）',
      st == 'missing' and '拒绝算' in why, '%s / %s' % (st, why[:80]))
st, why = baseline_state(_store_of(ARCH_DOC, with_fp=False), ARCH_DOC)
check('旧格式 store（没记指纹）⇒ no_fingerprint、拒绝（**不假装还对**）',
      st == 'no_fingerprint' and '重跑' in why, '%s / %s' % (st, why[:80]))
check('  fail-closed：这一档**不是** ok（宁可当作不知道，不可当作还对）',
      baseline_state(_store_of(ARCH_DOC, with_fp=False), ARCH_DOC)[0] != 'ok')
check('路径不变、内容变了 ⇒ 也抓得住（这正是「重标一次写回同一个文件」的形状）',
      _store_of(ARCH_DOC)['from_extrinsics']
      == _store_of(OTHER_DOC)['from_extrinsics']
      and baseline_state(ST_OK, OTHER_DOC)[0] == 'changed')

# 指纹本身：约定要跟着 `base_from_camera` 走（内部光学系），否则同一份外参
# 换个写法就会被判成「变了」—— 那会让闸门变成噪声源，人就开始绕过它。
check('指纹记的是**内部光学系**的值（与 base_from_camera 同源）',
      np.allclose(np.array(baseline_fingerprint(ARCH_DOC)['t']), T_TEST))
_opt_doc = {'frame_child': 'camera_color_optical_frame',
            'R': (R_TEST @ R_LINK_TO_OPTICAL).tolist(), 't': T_TEST.tolist()}
check('同一份外参写成光学系 ⇒ 指纹**相同**（两种写法是同一件事）',
      baseline_fingerprint(ARCH_DOC) == baseline_fingerprint(_opt_doc))

# ---- 变异：把闸门拆掉（当它永远 ok）⇒ 上面那几条必须翻面 ----
# 临时文件写到**仓库外**的临时目录：写进仓库根会污染 `git status`，
# 而崩在中途时 `finally` 也保不住它（本仓是公开副本，红线 3）。
_SRC = open(os.path.join(HERE, 'topcam_marker_calib.py'), encoding='utf-8').read()
_ANCHOR = "    if (cur['R'] != fp[0]) or (cur['t'] != fp[1]):"
if _ANCHOR not in _SRC:
    check('变异锚点还在（baseline_state 里那次比较）', False, _ANCHOR)
else:
    _tmp = tempfile.mkdtemp(prefix='mut_topcam_marker_calib_')
    _d = os.path.join(_tmp, 'mut_topcam_marker_calib.py')
    with open(_d, 'w', encoding='utf-8') as _fh:
        _fh.write(_SRC.replace(_ANCHOR, "    if False:", 1))
    try:
        _spec = importlib.util.spec_from_file_location('mut_topcam_marker_calib', _d)
        _m = importlib.util.module_from_spec(_spec)
        _spec.loader.exec_module(_m)
        _mst, _mwhy = _m.baseline_state(ST_OK, OTHER_DOC)
        check('变异：拆掉「基准被换过」那一档 ⇒ 它照样 ok，于是上面那个差会被印成'
              '「漂移了几个毫米」（★ 断言吃劲：改前 ≠ 改后）',
              _mst == 'ok' and st_changed != 'ok',
              '变异后 %s（%r）；改前 %s' % (_mst, _mwhy, st_changed))
    finally:
        shutil.rmtree(_tmp, ignore_errors=True)

print('\n通过 %d，失败 %d' % (N_PASS, N_FAIL))
if FAILS:
    print('失败项：')
    for f in FAILS:
        print('  - %s' % f)
sys.exit(1 if N_FAIL else 0)

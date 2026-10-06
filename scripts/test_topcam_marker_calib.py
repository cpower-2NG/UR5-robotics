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

import math
import os
import re
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np                                                # noqa: E402
import draw_zone_on_frame as dzf                                  # noqa: E402
from topcam_marker_calib import (                                 # noqa: E402
    STORE_VERSION, T_LINK_TO_OPTICAL, R_LINK_TO_OPTICAL, base_from_camera,
    compose, fmt_launch, invert, matrix_to_quat, quat_to_matrix,
    store_version_error)

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

print('\n--- C. 帧名判定（拒绝猜） + ★ 那 15 mm 必须跟着转 ---')
ARCH = {'R': R_TEST.tolist(), 't': T_TEST.tolist(),
        'frame_parent': 'base_link', 'frame_child': 'camera_link'}
Rb, tb = safe(base_from_camera, ARCH)
check('camera_link ⇒ 左乘固定旋转',
      np.allclose(Rb, R_TEST @ R_LINK_TO_OPTICAL))
# ★ 2026-10-06 改：原来这里断言「t 不参与旋转」，那是在**钉住缺陷**。
#   `t` 是 camera_link 原点在 base 里的位置，而我们要的是**光学系**原点 ——
#   两者差 R·T_LINK_TO_OPTICAL（15 mm，正好在光心上）。旋转转了、平移不转，
#   得到的是一个**哪个帧都不是**的混合体。
check('★ 平移也必须跟着转：光学系原点 = R·T_LINK_TO_OPTICAL + t',
      np.allclose(tb, R_TEST @ T_LINK_TO_OPTICAL + T_TEST))
check('  而它与原来那个（错的）t 差了 15 mm 量级 —— 不是四舍五入级别的差别',
      abs(float(np.linalg.norm(tb - T_TEST)) * 1000 - 15.0) < 0.5,
      '%.2f mm' % (float(np.linalg.norm(tb - T_TEST)) * 1000))
check('★ 且与 draw_zone_on_frame.compose_base_optical 逐位一致'
      '（两个工具共用一份定义，不再各写各的）',
      np.allclose(Rb, dzf.compose_base_optical(ARCH)[0])
      and np.allclose(tb, dzf.compose_base_optical(ARCH)[1]))
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

print('\n--- C2. ★ fmt_launch：打印的必须是 launch 要的帧（base←camera_link）---')
# `real.launch.py` 那个 static_transform_publisher 是硬编码的
# `--frame-id base_link --child-frame-id camera_link`，它自己的注释也写着
# 「手眼标定的结果必须表达在 camera_link 上」。而这条链内部算出来的是
# **base←光学系** —— 直接打印出去就错一个 90° 级旋转，而且**不报错**。
R_opt, t_opt = safe(base_from_camera, ARCH)
line = fmt_launch(R_opt, t_opt)


def _arg(name):
    """从 launch 那几行里取 camera_<name>:= 的数值。"""
    m = re.search(r'camera_%s:=(-?[\d.]+)' % name, line)
    return float(m.group(1))

t_launch = np.array([_arg('x'), _arg('y'), _arg('z')])
R_launch = quat_to_matrix([_arg('qx'), _arg('qy'), _arg('qz'), _arg('qw')])
# ⚠️ 容差按**打印精度**定：launch 那几行是 `%.6f`，所以四元数只剩 1e-6 量级，
#    还原回来的 R 与 t 也就差 ~1e-6（不是 1e-9）。钉 1e-9 会假红。
ang_launch = math.degrees(math.acos(min(1, max(-1,
    (np.trace(R_launch.T @ R_TEST) - 1) / 2))))
d_launch = float(np.linalg.norm(t_launch - T_TEST)) * 1000
check('★ fmt_launch 还原出文件里的 camera_link 位姿（这里是构造它的那个 R,t）'
      '（残差只来自 %.6f 的打印精度）',
      ang_launch < 1e-3 and d_launch < 1e-2,
      'R 差 %.6f°，t 差 %.6f mm' % (ang_launch, d_launch))
check('  且它**不是**原样打印光学系那组（否则这一步等于没做）',
      not np.allclose(R_launch, R_opt, atol=1e-6))

print('\n--- C3. ★ 存盘版本闸门（旧语义不能悄悄沿用）---')
check('★ 没有 store_version 字段（= v1 的旧存盘）⇒ 拒绝，并说明差 15 mm',
      isinstance(store_version_error({'base_from_board_R': [[1, 0, 0], [0, 1, 0],
                                                             [0, 0, 1]]}), str)
      and '15 mm' in store_version_error({}))
check('  当前版本 ⇒ 放行（None）', store_version_error(
    {'store_version': STORE_VERSION}) is None)
check('★ 比当前**新**的存盘也拒绝（本脚本读不懂它的语义，不猜）',
      isinstance(store_version_error({'store_version': STORE_VERSION + 1}), str))

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

print('\n--- E. 变异：改坏一处，对应断言必须失效 ---')


def _load(src_text, tag):
    """把一份源码文本当模块导入（临时文件，避免与真模块的 .pyc 混）。"""
    import importlib.util
    import tempfile
    d = tempfile.mkdtemp(prefix='mkcalib_')
    p = os.path.join(d, 'topcam_marker_calib_%s.py' % tag)
    with open(p, 'w') as f:
        f.write(src_text)
    spec = importlib.util.spec_from_file_location('topcam_marker_calib_%s' % tag, p)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _mutate(old, new):
    """对真源码做一次文本替换；没命中就抛（防「变异其实没生效」）。"""
    s = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          'topcam_marker_calib.py')).read()
    assert old in s, '变异锚点没找到：%r' % old
    return s.replace(old, new)


def _mutate_dzf(old, new):
    s = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          'draw_zone_on_frame.py')).read()
    assert old in s, 'dzf 变异锚点没找到：%r' % old
    return s.replace(old, new)


# M1：那 15 mm 又被丢掉（换算只转旋转、不转平移）——这正是 2026-10-06 修掉的
#     那个缺陷。注意换算现在住在 dzf 里（两个工具共用一份），所以变异要打在
#     dzf 上、再把变异版塞给 marker_calib —— 这本身也证明了「共用一份」生效了。
m1_dzf = _load(_mutate_dzf("        return (R @ R_LINK_TO_OPTICAL,\n"
                           "                R @ T_LINK_TO_OPTICAL + t)",
                           "        return (R @ R_LINK_TO_OPTICAL, t)"), 'dzf_m1')
m1 = _load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             'topcam_marker_calib.py')).read(), 'm1')
m1.dzf = m1_dzf
Rb_m1, tb_m1 = safe(m1.base_from_camera, ARCH)
check('M1 平移不跟着转 → 光学系原点退回 t，15 mm 那三条断言全失效',
      np.allclose(tb_m1, T_TEST) and not np.allclose(tb_m1, tb),
      '变异后 t = %s' % np.round(tb_m1, 4).tolist())

# M2：fmt_launch 不做帧转换（把 base←光学系 当 launch 值打印）——
#     这正是改之前的行为：填进 launch 就错一个 90° 级旋转，而且不报错。
m2 = _load(_mutate("    R_link = np.asarray(R_opt, dtype=float) @ R_LINK_TO_OPTICAL.T\n"
                   "    t_link = np.asarray(t_opt, dtype=float) - R_link @ T_LINK_TO_OPTICAL",
                   "    R_link = np.asarray(R_opt, dtype=float)\n"
                   "    t_link = np.asarray(t_opt, dtype=float)"), 'm2')
line_m2 = m2.fmt_launch(R_opt, t_opt)


def _arg_of(line, name):
    return float(re.search(r'camera_%s:=(-?[\d.]+)' % name, line).group(1))


R_m2 = quat_to_matrix([_arg_of(line_m2, 'qx'), _arg_of(line_m2, 'qy'),
                       _arg_of(line_m2, 'qz'), _arg_of(line_m2, 'qw')])
ang_m2 = math.degrees(math.acos(min(1, max(-1,
    (np.trace(R_m2.T @ R_TEST) - 1) / 2))))
check('M2 fmt_launch 不转换 → 打印的与 launch 要的差一个 90° 级旋转'
      '（所以 C2 那两条不是空转）',
      ang_m2 > 80.0, '变异后与正确值差 %.1f°' % ang_m2)

# M3：版本闸门写成「非零就放行」——未来版本的存盘会被读进来（本脚本读不懂
#     它的语义）。这正是「不猜」原则要挡住的。
m3 = _load(_mutate("    if have == STORE_VERSION:\n        return None",
                   "    if have:\n        return None"), 'm3')
check('M3 闸门写成「非零即放行」→ 比当前新的存盘也被放行（所以 C3 那条不是空转）',
      m3.store_version_error({'store_version': STORE_VERSION + 1}) is None
      and store_version_error({'store_version': STORE_VERSION + 1}) is not None)

print('\n通过 %d，失败 %d' % (N_PASS, N_FAIL))
if FAILS:
    print('失败项：')
    for f in FAILS:
        print('  - %s' % f)
sys.exit(1 if N_FAIL else 0)

#!/usr/bin/env python3
"""`draw_zone_on_frame.py` 的纯函数自检（**不碰 ROS、不碰相机、不读图片**）。

要钉住的是**方向**这一件事 —— 这个工具唯一会错得离谱的地方：

1. `base_link ← camera_link` 与 `base_link ← 光学系` 差一个**固定旋转**。
   漏了它，针孔公式照样吐出一个「看着像样」的框，只是整个歪掉。
2. 光轴与台面法线的夹角是**可检验的不变式**（文档实测 35.9°）。
3. 点在相机背后必须返回 `None`，不能裁一个数出来。

最有力的一条是 **E 段的「光轴上的点必落在主点」**：它把
「旋转用对了没有」变成一个**解析上必然成立**的等式，而不是「看着差不多」。

跑法：
    python3 scripts/test_draw_zone_on_frame.py
退出码 0 = 全通过。
"""

import json
import math
import os
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np                                                   # noqa: E402
from draw_zone_on_frame import (                                     # noqa: E402
    RC_PARENT, RQ_TOL_DEG, R_LINK_TO_OPTICAL, T_LINK_TO_OPTICAL, axis_angles,
    compose_base_optical, parse_zone, project, quat_to_matrix,
    validate_extrinsics)

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


# 留档那份（docs/calib/extrinsics-2026-10-02.json）的逐字数值。
# ⚠️ 那份文件的 `frame_parent` 是 **`base`**（UR 控制器帧，2026-10-06 才改对）；
#    它自称 `base_link` 的那段时间整条链是镜像的，臂跑到反侧（台账 §10.2）。
#    这里**故意不写 frame_parent** —— 这组数当「一组真实量级的矩阵」用就够了，
#    把那个谎字段再抄进测试，等于让它换个地方继续活着。
ARCHIVE = {
    "R": [[-0.034660042571149956, -0.999396046293701, 0.0024948149197288206],
          [0.5828767786583874, -0.0181868375995216, 0.8123570027019832],
          [-0.8118210038855782, 0.02961049798038296, 0.5831551046330354]],
    "t": [-0.34694477757210807, -0.8693043936314018, 0.6454397556505314],
    "frame_child": "camera_link",
}
K = (917.6205444335938, 916.4271240234375, 649.554931640625, 344.9062805175781)


def safe(fn, *a, **kw):
    try:
        return fn(*a, **kw)
    except Exception as e:                       # noqa: BLE001
        return f"!! 崩了：{type(e).__name__}: {e}"


# ----------------------------------------------------------------------
# A. 那个固定旋转本身
# ----------------------------------------------------------------------
print("\n--- A. camera_link → 光学系 的固定旋转 ---")
M = R_LINK_TO_OPTICAL
check("正交：|RᵀR − I| < 1e-12",
      float(np.abs(M.T @ M - np.eye(3)).max()) < 1e-12)
check("★ 是**纯旋转**（det = +1，不是镜像）", abs(float(np.linalg.det(M)) - 1.0) < 1e-12,
      f"det={np.linalg.det(M)}")
# ⚠️ 这里**不能**再断言「严格等于理想旋转」：常数已换成**实机实测值**，
#    它与理想值差 0.2° 量级（D435i 的彩色模块相对 camera_link 的真实安装）。
#    断言要钉的是「接近理想（在 1° 内）」＋「正交」。
IDEAL = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
_ang = math.degrees(math.acos(min(1.0, max(-1.0, (np.trace(M.T @ IDEAL) - 1) / 2))))
check(f"★ 与理想旋转相差 {_ang:.3f}°（应在 1° 内 —— 实机不是理想安装）",
      _ang < 1.0, f"got {_ang}")
check("★ opt_z（朝前）≈ link_x", np.allclose(M @ np.array([0., 0., 1.]), [1., 0., 0.], atol=0.02))
check("★ opt_x（朝右）≈ −link_y", np.allclose(M @ np.array([1., 0., 0.]), [0., -1., 0.], atol=0.02))
check("★ opt_y（朝下）≈ −link_z", np.allclose(M @ np.array([0., 1., 0.]), [0., 0., -1.], atol=0.02))
check("★★ **平移非零且约 15 mm** —— 这一条钉的正是「只转不平移」那个 bug；"
      "理想旋转没有平移，漏掉它会让离线投影偏约 16 mm",
      abs(float(np.linalg.norm(T_LINK_TO_OPTICAL)) - 0.01496) < 0.001,
      f"|T|={np.linalg.norm(T_LINK_TO_OPTICAL)*1000:.2f} mm")
check("★ 平移主要在 link 的 +y 方向（实测 14.96 mm）",
      abs(T_LINK_TO_OPTICAL[1] - 0.014957) < 1e-5)


# ----------------------------------------------------------------------
# B. compose：帧名决定要不要左乘
# ----------------------------------------------------------------------
print("\n--- B. compose_base_optical ---")
R_a, t_a = safe(compose_base_optical, ARCHIVE)
check("camera_link ⇒ 左乘那个固定旋转",
      np.allclose(R_a, np.array(ARCHIVE["R"]) @ M))
# ⚠️ 这条**曾经断言错了方向**：原文是「camera_link ⇒ t 不变」，而那恰恰把
#    「漏掉 link→光学系的 15 mm 平移」锁死成了「正确行为」—— 一条**把 bug
#    写成契约**的断言。正确语义：光学系的光心在 parent 系里是 `t + R·T_lo`。
check("★ camera_link ⇒ 光心 = t + R·T_lo（不是 t 本身）",
      np.allclose(t_a, np.array(ARCHIVE["t"])
                  + np.array(ARCHIVE["R"]) @ T_LINK_TO_OPTICAL))
check("  且确实与 t 相差 ≈15 mm（钉住那个漏平移的 bug）",
      not np.allclose(t_a, ARCHIVE["t"], atol=1e-4)
      and abs(float(np.linalg.norm(np.array(t_a) - np.array(ARCHIVE["t"]))) - 0.01496) < 0.002)
R_o, t_o = safe(compose_base_optical, {**ARCHIVE, "frame_child":
                                      "camera_color_optical_frame"})
check("★ 已经是光学系 ⇒ **原样返回，不重复左乘**",
      np.allclose(R_o, np.array(ARCHIVE["R"])))
check("  且与 camera_link 那支**不相等**（说明这一步真的做了事）",
      not np.allclose(R_a, R_o))
# ⚠️ 必须**去掉** frame_child：留着它就走旧支，这条就成了恒真式（第一次就是
#    这么写的 —— 在一个专测「方向」的文件里把方向搞错）。
_NO_CHILD = {k: v for k, v in ARCHIVE.items() if k != "frame_child"}
check("`camera_frame` 这个键名也认（标注那份用的是它，且没 frame_child）",
      np.allclose(safe(compose_base_optical, {**_NO_CHILD, "camera_frame":
                                              "camera_color_optical_frame"})[0], R_o))
check("四元数形式（`q`）也认，且与 R 一致",
      np.allclose(safe(compose_base_optical, {
          "q": [-0.316374655430605, 0.32913450910630504,
                0.6395314661538762, 0.6185281368827056],
          "t": ARCHIVE["t"], "frame_child": "camera_link"})[0], R_a, atol=1e-9))
r = safe(compose_base_optical, {**ARCHIVE, "frame_child": "some_unknown_frame"})
check("★ 不认识的帧名 ⇒ **拒绝**（不猜帧关系）",
      isinstance(r, str) and "崩了" in r and "ValueError" in r, f"got {r!r}")
check("  缺 frame_child 也拒绝（同上）",
      isinstance(safe(compose_base_optical, {"R": ARCHIVE["R"], "t": ARCHIVE["t"]}), str))


# ----------------------------------------------------------------------
# C. 不变式：光轴 vs 台面
# ----------------------------------------------------------------------
print("\n--- C. 光轴夹角（可检验的不变式）---")
to_n, to_p = axis_angles(R_a)
check("★ 与台面法线 ≈ 35.9°（文档实测）", abs(to_n - 35.9) < 0.5, f"得到 {to_n:.2f}°")
check("★ 与台面 ≈ 54.1°", abs(to_p - 54.1) < 0.5, f"得到 {to_p:.2f}°")
check("  两者互补（和 = 90）", abs(to_n + to_p - 90.0) < 1e-9)
check("★ 相机是**朝下**看的（光轴 z 分量为负）",
      float(np.asarray(R_a)[2, 2]) < 0, f"{np.asarray(R_a)[2,2]:.4f}")
check("用 camera_link 版的 R 去算会**得到不同的角**（所以那一步不能省）",
      abs(axis_angles(np.array(ARCHIVE["R"]))[0] - to_n) > 1.0,
      f"不左乘时 {axis_angles(np.array(ARCHIVE['R']))[0]:.2f}°")


# ----------------------------------------------------------------------
# D. project：背后必须 None
# ----------------------------------------------------------------------
print("\n--- D. project ---")
# ⚠️ 方向要算，不能猜：光轴朝下，(0,0,-5) 反而在**前方** 5.08 m（第一次写错了）。
#    真正的背后 = 从光心沿光轴**反方向**退出去。
_behind = np.asarray(t_a) - 5.0 * np.asarray(R_a)[:, 2]
check("★ 相机背后的点 ⇒ None（不是裁一个数出来）",
      safe(project, R_a, t_a, tuple(_behind), K) is None)
check("  那条不是空转：同一个点若按「前方」算是有投影的",
      safe(project, R_a, t_a, tuple(np.asarray(t_a) + 5.0 * np.asarray(R_a)[:, 2]), K)
      is not None)
check("  正好在光心平面上（z_cam≈0）⇒ None",
      safe(project, R_a, t_a, tuple(t_a), K) is None)


# ----------------------------------------------------------------------
# E. ★ 最有力的一条：光轴上的点必落在主点
# ----------------------------------------------------------------------
print("\n--- E. 光轴上的点 → 主点（解析上必然成立）---")
axis = np.asarray(R_a)[:, 2]                      # 光轴方向（base 系）
for d in (0.5, 0.9, 1.5, 3.0):
    p = np.asarray(t_a) + d * axis
    u, v = safe(project, R_a, t_a, tuple(p), K)
    check(f"沿光轴 {d:.1f} m 的点 ⇒ 像素 = 主点 ({K[2]:.0f},{K[3]:.0f})",
          abs(u - K[2]) < 1e-6 and abs(v - K[3]) < 1e-6,
          f"得到 ({u:.3f}, {v:.3f})")
# ⚠️ 这一条我写错过**两次**，值得留在这儿：
#   ① 拿「错 R 自己的第三列」当方向 ⇒ 必然映射回 e_z ⇒ 恒落主点，测了个恒真式；
#   ② 改用真光轴后，那个点被错 R 映射到相机系的 **z=0 平面**上（算出 −0.0）
#      ⇒ 按设计返回 None —— 不是「打偏」，是**整个光轴射线的像塌进了像平面**。
#   真正能读数、也最好懂的是拿一个**画面里的实点**比：base 原点。
_right = safe(project, R_a, t_a, (0.0, 0.0, 0.0), K)
_wrong = safe(project, np.array(ARCHIVE["R"]), t_a, (0.0, 0.0, 0.0), K)
check("★ base 原点：左乘对了 ⇒ 落在一个像样的像素上",
      _right is not None and 0 <= _right[0] < 1280 and 0 <= _right[1] < 720,
      f"got {_right}")
check("★ 同一个点、**漏掉那一步** ⇒ 差出几百像素（不是「差不多」）",
      _wrong is not None and abs(_wrong[0] - _right[0]) > 100,
      f"对 {_right} / 错 {_wrong}")
check("  而且错的那一版把该点投到了画面外或极边缘",
      _wrong is not None and (_wrong[1] < 0 or _wrong[1] > 720
                              or _wrong[0] < 0 or _wrong[0] > 1280),
      f"got {_wrong}")


# ----------------------------------------------------------------------
# F. parse_zone
# ----------------------------------------------------------------------
print("\n--- F. parse_zone ---")
name, corners, z = safe(parse_zone, "pick:0.40,0.75,-0.45,0.45")
check("名字与 z 默认 0", name == "pick" and z == 0.0)
check("四角顺序是逆时针（左下→右下→右上→左上）",
      corners == [(0.40, -0.45, 0.0), (0.75, -0.45, 0.0),
                  (0.75, 0.45, 0.0), (0.40, 0.45, 0.0)])
check("★ 负数也认（**工作区就在 −x 侧**，这条是给那个用的）",
      safe(parse_zone, "pick:-0.75,-0.40,-0.45,0.45")[1][0] == (-0.75, -0.45, 0.0))
check("第五个数是 z", safe(parse_zone, "z:-0.75,-0.40,-0.45,0.45,0.03")[2] == 0.03)
check("上下界反了 ⇒ 拒绝", isinstance(safe(parse_zone, "p:0.75,0.40,-0.45,0.45"), str))
check("少给数 ⇒ 拒绝", isinstance(safe(parse_zone, "p:0.40,0.75,-0.45"), str))
check("没有冒号 ⇒ 拒绝", isinstance(safe(parse_zone, "0.40,0.75,-0.45,0.45"), str))

def _q_from_R(R):
    """R → 四元数（只在本测试里用，所以不引 topcam_marker_calib —— 它要 cv2/aruco）。"""
    R = np.array(R, dtype=float)
    tr = float(np.trace(R))
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = np.zeros(4)
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return np.array([q[0], q[1], q[2], q[3]])


def _rot_nudged(R, deg):
    """把 R 绕某个斜轴转 deg 度，返回它的四元数（用来试容差的边界）。"""
    th = math.radians(deg)
    ax = np.array([0.3, -0.5, 0.81])
    ax = ax / np.linalg.norm(ax)
    K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    Rn = (np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * (K @ K)) @ np.array(R, float)
    return [float(v) for v in _q_from_R(Rn)]


# ----------------------------------------------------------------------
# G. validate_extrinsics —— 一份看着正常的 JSON 给出一个静默错的相机位姿
# ----------------------------------------------------------------------
# 这一层要挡的三件事，在**正常文件上都不会出现**，所以只有喂合成字典才验得动。
# 它们共同的特征：数值都是三位数、量纲也对，症状却是「臂抓偏」—— 与坐标系错
# 的表现一模一样，倒推不出来。
print("\n--- G. validate_extrinsics（外参文件能不能用）---")
_real = lambda n: json.load(open(os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'docs', 'calib', n),
    encoding='utf-8'))
_10_03 = _real('extrinsics-2026-10-03-base_link.json')
_10_02 = _real('extrinsics-2026-10-02.json')

check('干净的一份（只有 R/t、没写 frame_parent）⇒ 放行',
      validate_extrinsics(ARCHIVE) is None,
      str(validate_extrinsics(ARCHIVE)))
check('父帧如实写 base_link ⇒ 放行',
      validate_extrinsics({**ARCHIVE, 'frame_parent': RC_PARENT}) is None)
check('★ 父帧是 `base` ⇒ **拒绝**，且点出「绕 z 的 π / 镜像 / 臂跑到反侧」',
      isinstance(validate_extrinsics({**ARCHIVE, 'frame_parent': 'base'}), str)
      and 'π' in validate_extrinsics({**ARCHIVE, 'frame_parent': 'base'})
      and '镜像' in validate_extrinsics({**ARCHIVE, 'frame_parent': 'base'}),
      str(validate_extrinsics({**ARCHIVE, 'frame_parent': 'base'}))[:200])
check('   `base_link_inertia`（同一个帧的 URDF 名字）也拒',
      isinstance(validate_extrinsics({**ARCHIVE, 'frame_parent': 'base_link_inertia'}), str))
check('★ 认不出的父帧名一律拒（不猜帧关系）',
      isinstance(validate_extrinsics({**ARCHIVE, 'frame_parent': 'world'}), str))
check('★ 真实文件回归：10-03 那份现在**过**（它的 R 与 q 已改自洽）',
      validate_extrinsics(_10_03) is None, str(validate_extrinsics(_10_03)))
check('★ 真实文件回归：10-02 留档那份现在**拒**，原因是父帧（它其实在 base 帧）',
      isinstance(validate_extrinsics(_10_02), str)
      and 'base' in validate_extrinsics(_10_02),
      str(validate_extrinsics(_10_02))[:200])
check('   而这条拒绝不是空转：把它的父帧改成 base_link 就放行（R/t 本身没问题）',
      validate_extrinsics({**_10_02, 'frame_parent': 'base_link'}) is None)

# ★ R 与 q 对不上：2026-10-06 实际发生的形态 —— 整体绕 z 转 180° 时改了 R/t，
#   q 是上一份抄过来的。这类错**不会**被父帧检查、也不会被正交性检查拦住。
_q_wrong = {**_10_03, 'q': _10_02['q']}                       # 抄错的那版
_m = validate_extrinsics(_q_wrong)
check('★ R 与 q 差 180°（q 是从上一份抄来的）⇒ 拒绝，且说明自相矛盾',
      isinstance(_m, str) and '180' in _m and '自相矛盾' in _m, str(_m)[:240])
check('   对照：把 q 换成由自己的 R 反解的 ⇒ 放行（说明上面拒的是 q，不是别的）',
      validate_extrinsics({**_10_03, 'q': (-np.array(_10_03['q'])).tolist()}) is None
      and validate_extrinsics(
          {**_10_03, 'q': [float(v) for v in
                           np.round(_q_from_R(_10_03['R']), 9)]}) is None)
check('   容差确实是 %g°：差 0.4° 放行、差 0.6° 拒绝' % RQ_TOL_DEG,
      validate_extrinsics({**_10_03, 'q': _rot_nudged(_10_03['R'], 0.4)}) is None
      and isinstance(validate_extrinsics({**_10_03, 'q': _rot_nudged(_10_03['R'], 0.6)}),
                     str),
      '容差 %g' % RQ_TOL_DEG)

check('★ det = −1（镜像）⇒ 拒绝（「父帧标反」的典型形态就是整体镜像）',
      isinstance(validate_extrinsics({**ARCHIVE, 'R': (-np.array(ARCHIVE['R'])).tolist()}), str))
check('R 不正交 ⇒ 拒绝',
      isinstance(validate_extrinsics(
          {**ARCHIVE, 'R': (np.array(ARCHIVE['R']) * np.array([1.0, 1.0, 1.5])).tolist()}), str))
check('R 不是 3×3 ⇒ 拒绝',
      isinstance(validate_extrinsics({**ARCHIVE, 'R': [[1, 0], [0, 1]]}), str))
check('只有 q、没有 R ⇒ 放行（不强行要 R，两份来源本来就只写一种）',
      validate_extrinsics({'q': _10_03['q'], 't': _10_03['t'],
                           'frame_child': 'camera_link'}) is None)
check('既没 R 也没 q ⇒ 拒绝', isinstance(validate_extrinsics({'t': [0, 0, 1]}), str))
check('没有 t ⇒ 拒绝', isinstance(validate_extrinsics({'R': ARCHIVE['R']}), str))
check('根本不是字典 ⇒ 拒绝', isinstance(validate_extrinsics(['R']), str))
check('★ 而且它真的挂在路上：compose_base_optical 会抛，不是写了个没人调的函数',
      isinstance(safe(compose_base_optical, {**ARCHIVE, 'frame_parent': 'base'}), str)
      and isinstance(safe(compose_base_optical, _10_02), str))

# ---- 变异：每一条闸门拿掉之后，上面哪条断言会红 ----
print("\n--- G2. 变异（闸门拿掉之后必须变红）---")


def _load(src_text, tag):
    """把一份源码文本当模块导入（临时文件，避免与真模块的 .pyc 混）。"""
    import importlib.util
    import tempfile
    d = tempfile.mkdtemp(prefix='dzf_')
    p = os.path.join(d, 'draw_zone_on_frame_%s.py' % tag)
    with open(p, 'w') as f:
        f.write(src_text)
    spec = importlib.util.spec_from_file_location('draw_zone_on_frame_%s' % tag, p)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _mutate(old, new):
    s = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          'draw_zone_on_frame.py')).read()
    assert old in s, '变异锚点没找到：%r' % old
    return s.replace(old, new)


_g1 = _load(_mutate('    if parent and parent != RC_PARENT:',
                    '    if False and parent and parent != RC_PARENT:'), 'g1')
check('G1 去掉父帧闸门 → `base` 也被放行（所以「父帧」那三条不是空转）',
      _g1.validate_extrinsics({**ARCHIVE, 'frame_parent': 'base'}) is None
      and validate_extrinsics({**ARCHIVE, 'frame_parent': 'base'}) is not None)

_g2 = _load(_mutate('RQ_TOL_DEG = 0.5', 'RQ_TOL_DEG = 200.0'), 'g2')
check('G2 容差放到 200° → R/q 差 180° 也被放行（所以「抄错 q」那条不是空转）',
      _g2.validate_extrinsics(_q_wrong) is None
      and validate_extrinsics(_q_wrong) is not None)

_g3 = _load(_mutate('        if abs(float(np.linalg.det(R)) - 1.0) > 1e-6:',
                    '        if False:'), 'g3')
check('G3 去掉 det 检查 → 镜像被放行（所以 det 那条不是空转）',
      _g3.validate_extrinsics(
          {**ARCHIVE, 'R': (-np.array(ARCHIVE['R'])).tolist()}) is None)

print("\n通过 %d，失败 %d" % (N_PASS, N_FAIL))
if FAILS:
    print("失败项：")
    for f in FAILS:
        print("  - %s" % f)
sys.exit(1 if N_FAIL else 0)

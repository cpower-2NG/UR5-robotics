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

import math
import os
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np                                                   # noqa: E402
from draw_zone_on_frame import (                                     # noqa: E402
    R_LINK_TO_OPTICAL, axis_angles, compose_base_optical, parse_zone, project)

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


# 留档那份（docs/calib/extrinsics-2026-10-02.json）的逐字数值
ARCHIVE = {
    "R": [[-0.034660042571149956, -0.999396046293701, 0.0024948149197288206],
          [0.5828767786583874, -0.0181868375995216, 0.8123570027019832],
          [-0.8118210038855782, 0.02961049798038296, 0.5831551046330354]],
    "t": [-0.34694477757210807, -0.8693043936314018, 0.6454397556505314],
    "frame_parent": "base_link", "frame_child": "camera_link",
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
check("★ opt_z（朝前）= link_x：R_lo @ (0,0,1) = (1,0,0)",
      np.allclose(M @ np.array([0., 0., 1.]), [1., 0., 0.]))
check("★ opt_x（朝右）= −link_y：R_lo @ (1,0,0) = (0,−1,0)",
      np.allclose(M @ np.array([1., 0., 0.]), [0., -1., 0.]))
check("★ opt_y（朝下）= −link_z：R_lo @ (0,1,0) = (0,0,−1)",
      np.allclose(M @ np.array([0., 1., 0.]), [0., 0., -1.]))


# ----------------------------------------------------------------------
# B. compose：帧名决定要不要左乘
# ----------------------------------------------------------------------
print("\n--- B. compose_base_optical ---")
R_a, t_a = safe(compose_base_optical, ARCHIVE)
check("camera_link ⇒ 左乘那个固定旋转",
      np.allclose(R_a, np.array(ARCHIVE["R"]) @ M))
check("t 不参与旋转（相机光心在 parent 系就是 t）",
      np.allclose(t_a, ARCHIVE["t"]))
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

print("\n通过 %d，失败 %d" % (N_PASS, N_FAIL))
if FAILS:
    print("失败项：")
    for f in FAILS:
        print("  - %s" % f)
sys.exit(1 if N_FAIL else 0)

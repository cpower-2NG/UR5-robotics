#!/usr/bin/env python3
"""
标注投影自检 —— 不需要仿真器、不需要 colcon build、不需要真机（W4）
===================================================================
本脚本把「训练标签的几何 == 在线推理的几何」这条**契约**钉死成可执行断言。

为什么非要这么较真：`auto_label_grid.py` 与 `perception_3d/projector_node.py`
必须互为逆运算。两边一旦分家，没有任何一步会报错 —— 症状只是「检测框看着
挺好、3D 坐标系统性偏一点」，是全项目最难查的一类偏差。而且**历史上真的分过
家**：`world_to_pixel()` 里没有任何旋转，在仿真的光学约定下它其实是个**转置**
（第 2 节把它证成可执行断言）。这种错误靠肉眼看代码看不出来，只能靠断言。

五节：
  1. **往返恒等**：随机相机位姿/内参/点，投影下去再反解回来必须回到原点。
     对应 `projector_node.py` 的 Step 3（正向）与 `auto_label_grid.py` 的
     `project_pinhole`（反向）。
  2. **旧数学的错法被钉住**：证明「光心 x+ → world −y、y+ → world −x」这个
     已实测的仿真约定下，旧 `world_to_pixel` 给出的是
     `u_true = cx+cy−v_old`、`v_true = cx+cy−u_old` —— 转置 + 关于 (cx+cy)/2
     的点反演。谁要是把旧写法搬回来，这一节会红。
  3. **退化情形**：相机背后的点必须被丢弃；完全出画的方块必须返回 None。
  4. **两条后端互为第二意见**：单应后端与 tf 后端对台面上同一点必须给出同一个
     像素。这是计划里「同一帧跑两条路、对得上才放心」那一步的自动化版本
     （标定板单应不含轴向假设，所以是真正独立的对照）。并验证「底面 ∪ 顶面」
     的两张单应能把立方体的包围盒算成**精确等号**，而单张 H 的零厚度近似
     确实在边缘偏十几像素 —— 两种档位各有断言，免得「精确」是白捡的。
  5. **仿真 meta 端到端**：真实外参 + 真实 meta 走一遍，并钉住「bbox 尺寸随
     离轴距离**有 13~19% 的自然变化**」这个事实 —— 它同时是一条**反过度断言**
     的护栏：曾经有人（我）据此误以为「正确数学应当给出完全相同的尺寸」，
     从而得出「现存数据集不是这份 meta 生成的」这个**理由错误的结论**
     （结论碰巧对：那批图来自 `generate_synthetic_data.py` 用 40 个随机 seed
     跑出来的场景，不是这份固定 meta）。

用法:
    python3 scripts/test_label_projection.py
退出码 0 = 全通过。只依赖 numpy（第 5 节需要 meta 文件存在）。
"""

import importlib.util
import math
import random
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent
LABELER_PY = PROJECT_ROOT / "scripts" / "auto_label_grid.py"
SIM_META = (PROJECT_ROOT / "ros2_ws" / "src" / "yolo_bringup"
            / "worlds" / "block_grid_world_meta.json")

# 仿真相机的光学外参（与 sim.launch.py:48 一致）
SIM_Q = [0.7071067811865476, -0.7071067811865476, 0.0, 0.0]

_failed = []


def check(name, cond, detail=''):
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}" + (f"  {detail}" if detail else ''))
    if not cond:
        _failed.append(name)


def load_labeler():
    """按文件路径加载 auto_label_grid.py（scripts/ 不是包，只能这么导）。"""
    spec = importlib.util.spec_from_file_location("auto_label_grid", LABELER_PY)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def quat_mul(a, b):
    """Hamilton 积 (x,y,z,w)，与 tf2 的四元数语义一致。"""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def angle_axis(theta, axis):
    """绕单位轴的转角 → 四元数 (x,y,z,w)。"""
    n = math.sqrt(sum(c * c for c in axis))
    s = math.sin(theta / 2.0) / n
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(theta / 2.0))


def px_of(L, proj, point):
    """把单个 3D 点投到像素 —— 走 projector 自己的链，**精确、不经包围盒**。

    ⚠️ 别拿 `project_box` 的中心当「投影中心」：包围盒中心是极值的**中点**，
    离轴时与真正的投影中心差零点几像素（第 5 节把那个差值量化了）。
    做「往返恒等」这种 1e-9 量级的断言必须用单点投影。
    """
    cam = proj.to_camera(point)
    out, dropped = L.project_pinhole([cam], proj.fx, proj.fy, proj.cx, proj.cy)
    return (out[0] if out else None), dropped


def box_center_px(proj, center, size):
    """投影一个立方体，返回**归一化包围盒的中心**（像素）与包围盒。"""
    bbox, dropped = proj.project_box(center, size)
    if bbox is None:
        return None, None, dropped
    return (bbox[0] * proj.width, bbox[1] * proj.height), bbox, dropped


# ── 1. 往返恒等 ────────────────────────────────────────────────────────
def test_round_trip(L):
    print("\n--- 1. 往返恒等（推理链的正向 ↔ 标注链的反向）---")

    def forward_project(u, v, depth, fx, fy, cx, cy):
        """projector_node.py Step 3 的原文，一字不改。"""
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
        return x, y, z

    rng = random.Random(20260927)
    worst = 0.0
    for _ in range(2000):
        fx = rng.uniform(300.0, 1200.0)
        fy = rng.uniform(300.0, 1200.0)
        cx = rng.uniform(200.0, 500.0)
        cy = rng.uniform(150.0, 400.0)
        u = rng.uniform(0.0, 640.0)
        v = rng.uniform(0.0, 480.0)
        depth = rng.uniform(0.1, 3.0)

        x, y, z = forward_project(u, v, depth, fx, fy, cx, cy)
        out, dropped = L.project_pinhole([(x, y, z)], fx, fy, cx, cy)
        assert dropped == 0, "深度为正的点不该被丢弃"
        worst = max(worst, abs(out[0][0] - u), abs(out[0][1] - v))

    check("2000 组随机 (内参, 像素, 深度) 往返误差 < 1e-9 px",
          worst < 1e-9, f"最大 {worst:.3e} px")

    out, _ = L.project_pinhole([(0.1, -0.05, 0.5)], 600.0, 600.0, 320.0, 240.0)
    check("project_pinhole 返回的是**像素**而不是归一化（D7）",
          abs(out[0][0] - 440.0) < 1e-9 and abs(out[0][1] - 180.0) < 1e-9,
          f"{tuple(round(v, 3) for v in out[0])}")

    # fx != fy（真机 D435i 会有微小差异）时必须各自用自己的焦距
    out, _ = L.project_pinhole([(0.1, 0.1, 0.5)], 600.0, 300.0, 320.0, 240.0)
    check("fx ≠ fy 时两条轴各用自己的焦距",
          abs(out[0][0] - 440.0) < 1e-9 and abs(out[0][1] - 300.0) < 1e-9,
          f"{tuple(round(v, 3) for v in out[0])}")


# ── 2. 旧数学的错法 ────────────────────────────────────────────────────
def test_old_math_is_transposed(L):
    print("\n--- 2. 旧 world_to_pixel 在仿真约定下是个转置（钉住这条）---")

    def world_to_pixel_old(wx, wy, wz, cam_x, cam_y, cam_z, fx, fy, cx, cy):
        """旧写法原文（scripts/generate_synthetic_data.py:61-69 与
        git 历史里的 auto_label_grid.py 都是这一份）。"""
        Xc = wx - cam_x
        Yc = wy - cam_y
        Zc = cam_z - wz
        if Zc <= 0:
            return None
        return (fx * (Xc / Zc) + cx, fy * (Yc / Zc) + cy)

    info = {"fx": 554.256, "fy": 554.256, "cx": 320.0, "cy": 240.0,
            "width": 640, "height": 480}
    CAM = (0.5, 0.0, 1.145)
    extr = {"target_frame": "world",
            "camera_frame": "overhead_camera/camera_link/color_camera",
            "translation": list(CAM), "rotation": SIM_Q}
    proj = L.TfProjector(info, extr)

    # 先锚定「我说的约定」= 「sim.launch.py:42 注释里写的约定」
    ax = proj.R @ np.array([1.0, 0.0, 0.0])   # 光心 x+ 在世界系里
    ay = proj.R @ np.array([0.0, 1.0, 0.0])
    az = proj.R @ np.array([0.0, 0.0, 1.0])
    check("光心 x+ → world −y", np.allclose(ax, [0, -1, 0], atol=1e-9), f"{np.round(ax,6).tolist()}")
    check("光心 y+ → world −x", np.allclose(ay, [-1, 0, 0], atol=1e-9), f"{np.round(ay,6).tolist()}")
    check("光心 z+ → world −z", np.allclose(az, [0, 0, -1], atol=1e-9), f"{np.round(az,6).tolist()}")
    check("该旋转是合法旋转 det(R) = +1（参考项目称其「无法用纯旋转表示」是错的）",
          abs(np.linalg.det(proj.R) - 1.0) < 1e-9, f"det = {np.linalg.det(proj.R):.9f}")

    rng = random.Random(7)
    worst, max_shift, shift_at = 0.0, 0.0, None
    for _ in range(500):
        w = (rng.uniform(0.15, 0.85), rng.uniform(-0.25, 0.25), rng.uniform(0.70, 0.80))
        old = world_to_pixel_old(w[0], w[1], w[2], *CAM,
                                 info["fx"], info["fy"], info["cx"], info["cy"])
        if old is None:
            continue
        true, _ = px_of(L, proj, w)
        worst = max(worst, abs(true[0] - (info["cx"] + info["cy"] - old[1])),
                    abs(true[1] - (info["cx"] + info["cy"] - old[0])))
        d = math.hypot(true[0] - old[0], true[1] - old[1])
        if d > max_shift:
            max_shift, shift_at = d, (old, true)

    check("u_true = cx+cy−v_old 且 v_true = cx+cy−u_old（500 个台面点，< 1e-9 px）",
          worst < 1e-9, f"最大偏差 {worst:.3e} px")

    # 量级：这个错法不是几像素的偏移。⚠️ 注意**别用单个手挑的点**去证 ——
    # 这个映射是「关于直线 u+v=cx+cy 的镜像」，任何落在该直线上的点都是它的
    # **不动点**，会给出 0 偏差（我第一版就挑了 (0.30,0.20,0.765)，
    # 恰好 u+v=560=cx+cy，于是断言自己红了）。取整个样本上的最大位移才稳。
    check("旧写法的位置偏差是**几十~几百像素**量级（不是几像素）",
          max_shift > 50.0,
          f"样本内最大 {max_shift:.1f} px"
          + (f"（旧 {shift_at[0][0]:.0f},{shift_at[0][1]:.0f} → "
             f"真 {shift_at[1][0]:.0f},{shift_at[1][1]:.0f}）" if shift_at else ""))

    # 尺寸上却几乎看不出来 —— 这正是它能潜伏这么久的原因，值得写明。
    w = (0.30, 0.05, 0.765)
    half = 0.02
    span_old = abs(world_to_pixel_old(w[0] + half, w[1], w[2], *CAM, info["fx"],
                                      info["fy"], info["cx"], info["cy"])[0]
                   - world_to_pixel_old(w[0] - half, w[1], w[2], *CAM, info["fx"],
                                        info["fy"], info["cx"], info["cy"])[0])
    a, _ = px_of(L, proj, (w[0] - half, w[1], w[2]))
    b, _ = px_of(L, proj, (w[0] + half, w[1], w[2]))
    span_true = math.hypot(a[0] - b[0], a[1] - b[1])
    check("但**尺寸**两者几乎相同（所以「框看着挺对」掩盖了位置全错）",
          abs(span_old - span_true) < 3.0,
          f"旧 {span_old:.1f} px vs 真 {span_true:.1f} px")

    # ── 场景级后果（用 **grid world** 那台相机 z=1.5、桌面 z=0.765）─────
    # 旧渲染把 0.8 m 那条边映到图像**宽**、0.6 m 映到图像**高**；真投影正好
    # 对调，于是 0.8 m 落到只有 480 行的**高**上、直接上下出画。
    # 这就是 block_dataset 里那批合成图与仿真/真相机视角的可见差异。
    gp = L.TfProjector(info, {"target_frame": "world",
                              "camera_frame": "overhead_camera/camera_link/color_camera",
                              "translation": [0.5, 0.0, 1.5], "rotation": SIM_Q})
    fx, Zc = info["fx"], 1.5 - 0.765
    true_v = [px_of(L, gp, (x, y, 0.765))[0][1]
              for x, y in [(0.1, -0.3), (0.9, -0.3), (0.9, 0.3), (0.1, 0.3)]]
    old_v_lo, old_v_hi = 240 - fx * 0.3 / Zc, 240 + fx * 0.3 / Zc
    check("真投影下 0.8×0.6 桌面上下出画，而旧渲染「刚好」装得下",
          (min(true_v) < 0 or max(true_v) > 479)
          and 0 <= old_v_lo and old_v_hi <= 479,
          f"真 v ∈ [{min(true_v):.0f}, {max(true_v):.0f}]（480 行，出画）"
          f" vs 旧 v ∈ [{old_v_lo:.0f}, {old_v_hi:.0f}]（装得下）")


# ── 3. 退化情形 ────────────────────────────────────────────────────────
def test_degenerate(L):
    print("\n--- 3. 退化情形（宁可不标，也不要标错）---")

    out, dropped = L.project_pinhole([(0.0, 0.0, -1.0), (0.0, 0.0, 0.0),
                                      (0.0, 0.0, 0.5)], 500.0, 500.0, 320.0, 240.0)
    check("相机背后的点被丢弃（Z_c ≤ 0），只有 1 个活下来",
          len(out) == 1 and dropped == 2, f"留下 {len(out)}，丢弃 {dropped}")

    info = {"fx": 500.0, "fy": 500.0, "cx": 320.0, "cy": 240.0,
            "width": 640, "height": 480}
    extr = {"target_frame": "base_link", "camera_frame": "cam_optical",
            "translation": [0.0, 0.0, 1.0], "rotation": SIM_Q}
    proj = L.TfProjector(info, extr)

    bbox, drop = proj.project_box((0.0, 0.0, 1.5), 0.04)
    check("方块在相机**背后**时返回 None（不是一条横跨画面的巨框）",
          bbox is None and drop > 0, f"bbox={bbox} dropped={drop}")

    check("方块整个在画面外时返回 None",
          proj.project_box((50.0, 50.0, 0.0), 0.04)[0] is None)

    b = L.bbox_from_pixels([(-500.0, -500.0), (10.0, 10.0)], 640, 480)
    check("越界框被裁剪到图像范围内，且仍归一化",
          b is not None and all(0.0 <= v <= 1.0 for v in b),
          f"{tuple(round(v, 4) for v in b)}")

    check("空点集 → None（不是除零）", L.bbox_from_pixels([], 640, 480) is None)
    check("裁剪后退化（宽=0）→ None",
          L.bbox_from_pixels([(-5.0, -5.0), (0.0, 100.0)], 640, 480) is None)

    check("四元数未归一化时仍给出合法旋转（外参文件常常不是单位四元数）",
          abs(np.linalg.det(L.quat_to_matrix([0.3, -0.5, 0.2, 0.788])) - 1.0) < 1e-9)


# ── 4. 两条后端互为第二意见 ────────────────────────────────────────────
def test_homography_agrees_with_tf(L):
    print("\n--- 4. 单应后端 vs tf 后端：台面上同一点必须同一个像素 ---")

    info = {"fx": 554.256, "fy": 554.256, "cx": 320.0, "cy": 240.0,
            "width": 640, "height": 480}
    # 故意用一个**带俯仰和滚转**的位姿，免得又靠仿真那个对称姿态碰巧过关。
    # 构造方式：仿真朝向 ⊗ 俯仰 15° ⊗ 滚转 20°。必须在仿真朝向上叠，
    # 否则相机可能压根没看向台面，Z_c ≤ 0 的点会被丢弃、单应解不出来
    # （第一版随手写了个 [0.3,-0.5,0.2,0.788] 就栽在这里）。
    q = quat_mul(quat_mul(SIM_Q, angle_axis(0.2618, (1, 0, 0))),   # 俯仰 15°
                 angle_axis(0.3491, (0, 0, 1)))                     # 滚转 20°
    extr = {"target_frame": "base_link", "camera_frame": "cam_optical",
            "translation": [0.05, -0.02, 1.10], "rotation": q}
    proj = L.TfProjector(info, extr)
    check("该测试位姿确实看向台面（光轴与 world −z 夹角 < 30°）",
          math.degrees(math.acos(min(1.0, max(-1.0,
              float((proj.R @ np.array([0.0, 0.0, 1.0])) @ np.array([0.0, 0.0, -1.0]))))))
          < 30.0)

    # 由 tf 链**自己**解出台面 (z=0) 的单应矩阵：取 4 个平面点 → 相机系 → 像素。
    # 然后要求两后端对**其它**随机点一致 —— 这才是独立对照，不是自证。
    def solve_h(z):
        """用 tf 链解出「平面 z=const」的单应（等价于现场把标定板垫高 z 重标一次）。"""
        src = [(0.2, -0.2), (0.8, -0.2), (0.8, 0.2), (0.2, 0.2)]
        A = []
        for (x, y) in src:
            p, _ = px_of(L, proj, (x, y, z))
            if p is None:
                return None
            u, v = p
            A.append([-x, -y, -1, 0, 0, 0, u * x, u * y, u])
            A.append([0, 0, 0, -x, -y, -1, v * x, v * y, v])
        H = np.linalg.svd(np.array(A))[2][-1].reshape(3, 3)
        return (H / H[2, 2]).tolist()

    H0, H1 = solve_h(0.0), solve_h(0.04)
    check("单应的 4 个源点全部可投（相机确实看得见台面）", H0 is not None and H1 is not None)
    if H0 is None or H1 is None:
        return
    hp = L.HomographyProjector({"H": H0, "width": 640, "height": 480})

    rng = random.Random(99)
    worst = 0.0
    for _ in range(500):
        pt = (rng.uniform(0.15, 0.85), rng.uniform(-0.25, 0.25), 0.0)
        ut, vt = px_of(L, proj, pt)[0]
        uh, vh = hp.project_point(pt[0], pt[1])
        worst = max(worst, abs(ut - uh), abs(vt - vh))
    check("500 个台面点上两条后端一致（含俯仰+滚转的位姿，< 1e-6 px）",
          worst < 1e-6, f"最大偏差 {worst:.3e} px")

    # ── 关键：给了 H_top 之后，单应后端对**整个立方体**必须精确 ──────────
    # 凸体的轮廓 = 其顶点投影的凸包，而方块的顶点只落在底面和顶面两个平面上，
    # 所以「底面 ∪ 顶面」的包围盒就是真轮廓 —— 这一条把它验成等号。
    hp2 = L.HomographyProjector({"H": H0, "H_top": H1,
                                 "width": 640, "height": 480})
    worst2 = 0.0
    for _ in range(200):
        c = (rng.uniform(0.2, 0.8), rng.uniform(-0.2, 0.2), 0.02)
        bt = proj.project_box(c, 0.04)[0]
        bh = hp2.project_box(c, 0.04)[0]
        worst2 = max(worst2, max(abs(p - q) for p, q in zip(bt, bh)) * 640)
    check("给了 H_top 后，单应后端与 tf 后端对**整个立方体**完全一致（< 1e-6 px）",
          worst2 < 1e-6, f"最大偏差 {worst2:.3e} px")

    # 反面：只有 H（零厚度近似）时，边缘处的偏差必须**真的存在** ——
    # 否则说明上面那条「精确」是白捡的，两种档位没有区别。
    worst3, at = 0.0, None
    for _ in range(200):
        c = (rng.uniform(0.2, 0.8), rng.uniform(-0.2, 0.2), 0.02)
        bt = proj.project_box(c, 0.04)[0]
        bh = hp.project_box(c, 0.04)[0]
        d = max(abs(p - q) for p, q in zip(bt, bh)) * 640
        if d > worst3:
            worst3, at = d, c
    check("只有 H 的零厚度档位确实偏（量级与 docstring 声明的边缘误差相符）",
          worst3 > 2.0,
          f"最大 {worst3:.1f} px（@ x={at[0]:.2f}, y={at[1]:.2f}）")

    # 反面：**离面**的点两后端必须不一致 —— 否则说明这一节根本没测到东西。
    ut, vt = px_of(L, proj, (0.5, 0.0, 0.04))[0]
    uh, vh = hp.project_point(0.5, 0.0)
    off = math.hypot(ut - uh, vt - vh)
    check("离面 40mm 时两后端**有意**不同（单应的零厚度近似，量化已知误差）",
          off > 0.0, f"该点差 {off:.2f} px（计划已接受该量级）")

    check("H[2][2] ≈ 0 时明确报错，而不是静默给出垃圾",
          _raises(lambda: L.HomographyProjector({"H": [[1, 0, 0], [0, 1, 0], [0, 0, 0]],
                                                 "width": 640, "height": 480})))


def _raises(fn, exc=Exception):
    try:
        fn()
    except exc:
        return True
    except Exception:
        return False
    return False


# ── 5. 仿真 meta 端到端 ────────────────────────────────────────────────
def test_sim_meta(L):
    print("\n--- 5. 仿真 meta 端到端（W4 的仿真回归）---")
    if not SIM_META.exists():
        check(f"仿真 meta 存在（{SIM_META}）", False)
        return
    info, extr, blocks = L.sim_inputs(SIM_META)
    proj = L.TfProjector(info, extr)

    check("9 个方块全部投进画面内",
          len(blocks) == 9 and
          all(proj.project_box((b["x"], b["y"], b["z"]), b["size"])[0] is not None
              for b in blocks))

    ws, hs, offs = [], [], []
    for b in blocks:
        c, bbox, _ = box_center_px(proj, (b["x"], b["y"], b["z"]), b["size"])
        p, _ = px_of(L, proj, (b["x"], b["y"], b["z"]))
        ws.append(bbox[2] * proj.width)
        hs.append(bbox[3] * proj.height)
        offs.append(math.hypot(c[0] - p[0], c[1] - p[1]))
    check("方块全部落在画面内、9 个框都在",
          all(0 < w < proj.width and 0 < h < proj.height for w, h in zip(ws, hs)))

    # 正中的那一块：相机在它正上方，轮廓由**顶面**决定，因此
    # 尺寸 = 0.04*fx/(1.5-0.805)。这条是纯解析值，不含任何拟合。
    on_axis = [w for w, b in zip(ws, blocks) if abs(b["x"] - 0.5) < 1e-9
               and abs(b["y"]) < 1e-9]
    expect = 0.04 * proj.fx / (1.5 - 0.805)
    check("正中方块的包围盒宽 = 0.04·fx/顶面距离（解析值）",
          on_axis and abs(on_axis[0] - expect) < 0.05,
          f"实测 {on_axis[0]:.2f} px，解析 {expect:.2f} px")

    # ⚠️ 反过度断言：尺寸**不是**常数。相机虽垂直向下（Z_c 恒为 0.715），
    #    但离轴的方块会露出侧面，轮廓因此随离轴距离涨到 ~38 px。
    #    曾经（我）据此误称「正确数学应给出完全相同的尺寸」—— 那是错的，
    #    这条断言就是为了防止那个错误结论被再写一遍。
    spread = 100 * (max(ws) - min(ws)) / min(ws)
    check("bbox 尺寸随离轴距离有 5~25% 的自然变化（**不是**常数）",
          5.0 < spread < 25.0,
          f"宽 {min(ws):.2f}~{max(ws):.2f} px（{spread:.1f}%），"
          f"高 {min(hs):.2f}~{max(hs):.2f} px")

    check("包围盒中心与真实投影中心差不到 1 px（所以标签可用，但不能拿它当精确投影）",
          max(offs) < 1.0, f"最大 {max(offs):.3f} px")

    # 最终验收：拿**真实外参**跑正向链，必须精确回到方块本身。
    # ⚠️ 反变换是 p_target = R·p_cam + t（`to_camera` 的逆），不是 R·(p_cam − t)。
    # 第一版写成后者，误差恰为 |t + R·t| = 0.5 m —— 一个看起来很「物理」、
    # 其实纯属自己写错的数字。
    cam_z = extr["translation"][2]
    worst = 0.0
    for b in blocks:
        p, _ = px_of(L, proj, (b["x"], b["y"], b["z"]))
        depth = cam_z - b["z"]                 # 相机垂直向下 ⇒ Z_c 就是高差
        cam = np.array([(p[0] - info["cx"]) * depth / info["fx"],
                        (p[1] - info["cy"]) * depth / info["fy"], depth])
        back = proj.R @ cam + proj.t
        worst = max(worst, *(abs(back - np.array([b["x"], b["y"], b["z"]]))))
    check("框中心经**正向链**解回 base_link 精确回到方块（< 1e-9 m）",
          worst < 1e-9, f"最大偏差 {worst:.3e} m")


# ── 6. 多类别的**写标签**那一半 ────────────────────────────────────────
def test_multiclass_labels(L):
    """端到端跑一遍 auto_label_grid，只问一件事：**写出来的类别号对不对**。

    为什么单开一节：前面五节全在验几何，而多类别的失效**不在几何上** ——
    投影可以精确到 1e-16，类别号却全部写成 0，而且没有任何一步会报错。
    所以这一节不看坐标，只数类别号。
    """
    import json
    import subprocess
    import tempfile

    print("\n--- 6. 多类别：类别号必须按方块逐个写进标签 ---")
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        img_dir, out_dir = td / "img", td / "lbl"
        img_dir.mkdir()
        # 用真实 meta 的相机与几何，只把类别改成 4 类、方块缩到 6 个，
        # 好让每类的框数可以心算（6 个方块轮发 4 类 → 2/2/1/1）。
        meta = json.load(open(SIM_META))
        meta["classes"] = ["red", "yellow", "cyan", "purple"]
        names = ["red", "yellow", "cyan", "purple"]
        meta["blocks"] = meta["blocks"][:6]
        for i, b in enumerate(meta["blocks"]):
            b["class_name"] = names[i % 4]
            b["class_id"] = i % 4
        meta_path = td / "meta.json"
        meta_path.write_text(json.dumps(meta))

        # 一张黑图就够：投影几何与图像内容无关，这里只关心写出的标签行。
        import cv2
        cv2.imwrite(str(img_dir / "f0.jpg"), np.zeros((480, 640, 3), np.uint8))

        r = subprocess.run(
            [sys.executable, str(LABELER_PY), "--sim-meta", str(meta_path),
             "--images", str(img_dir), "--out", str(out_dir)],
            capture_output=True, text=True)
        check("auto_label_grid 退出码 0", r.returncode == 0,
              (r.stdout + r.stderr)[-500:])
        txt = out_dir / "f0.txt"
        check("写出了标签文件", txt.exists())
        if not txt.exists():
            return
        cls = [int(l.split()[0]) for l in txt.read_text().splitlines() if l.strip()]
        check("每个方块一行，共 6 行", len(cls) == 6, str(cls))
        # ⬇⬇ 本节的核心：类别号**逐块**来自 meta，不是统一写 0
        check("**类别号逐块来自 meta（不是统一写 0）**",
              cls == [0, 1, 2, 3, 0, 1], str(cls))
        check("  四类都出现过（不是只有第 0 类）", sorted(set(cls)) == [0, 1, 2, 3],
              str(sorted(set(cls))))
        check("  按类别报框数", "0=2" in r.stdout and "3=1" in r.stdout,
              r.stdout[-400:])

    print("  -- 类别的兜底与拒绝 --")
    # 没有 class_id 又没有 --class-id：必须报错退出，**不能默认 0**
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        img_dir, out_dir = td / "img", td / "lbl"
        img_dir.mkdir()
        import cv2
        cv2.imwrite(str(img_dir / "f0.jpg"), np.zeros((480, 640, 3), np.uint8))
        meta = json.load(open(SIM_META))
        for b in meta["blocks"]:
            b.pop("class_id", None)
            b.pop("class_name", None)
        meta_path = td / "meta.json"
        meta_path.write_text(json.dumps(meta))
        r = subprocess.run(
            [sys.executable, str(LABELER_PY), "--sim-meta", str(meta_path),
             "--images", str(img_dir), "--out", str(out_dir)],
            capture_output=True, text=True)
        check("meta 没有 class_id → 非 0 退出 + 说清原因",
              r.returncode != 0 and "class_id" in (r.stdout + r.stderr),
              (r.stdout + r.stderr)[-300:])
        check("  没有 Traceback（是拒绝，不是崩）",
              "Traceback" not in r.stderr, r.stderr[-300:])


def main():
    print("=" * 62)
    print("标注投影自检（W4）—— 训练几何必须 == 推理几何")
    print("=" * 62)
    L = load_labeler()
    test_round_trip(L)
    test_old_math_is_transposed(L)
    test_degenerate(L)
    test_homography_agrees_with_tf(L)
    test_sim_meta(L)
    test_multiclass_labels(L)

    print("\n" + "=" * 62)
    if _failed:
        print(f"FAILED: {len(_failed)} 项 -> {_failed}")
        return 1
    print("全部通过。")
    return 0


if __name__ == "__main__":
    sys.exit(main())

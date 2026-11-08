#!/usr/bin/env python3
"""阶段 4 的独立碰撞核验器（E8 的 M1/M2 用）。

**为什么要另写一份**：MoveIt 自己会算碰撞，但 M2 说的是「20/20 无碰撞」。
如果拿 MoveIt 的结论去证明 MoveIt 规划得对，那和 E5 那轮的「评估器自证」是同一个
坑（台账第二百七十七轮）。所以规划交给 MoveIt，**核验交给这里**：位姿由 FK 服务给，
几何求交在本文件里自己算。

口径
----
- 物体一律是 OBB（有朝向的长方体）：桌面、方块、夹爪本体、两根手指。
- **间隙判据**：``gap(a, b) <= margin`` 即判「碰撞/过近」。
  ``gap`` 是 15 条 SAT 轴上最大间隙 —— 它是**真实距离的下界**，所以
  「gap > margin」是真结论，反过来的「gap ≤ margin」可能偏保守。
  保守方向是对的：宁可多报一次碰撞，不可漏报。
- 方块 OBB 的 z 由「台面 + 边长/2」给；真值里的 ``top`` 是块顶（米）。
- 夹爪几何**逐字取自** ``arm_grasp_sim/models/ag95.urdf.xacro``：
  本体 100×75×60、单指 22×30×144.2、行程 ±47.5、tool0→指尖外沿 204.2。
  URDF 改了就同步 ``GRIPPER`` 里的数（自检里有一条核对这几个数）。

这里**不做 FK**：位姿由调用方给（ROS 侧走 ``/compute_fk``）。这样几何可以脱离
ROS 单测，也避免「同一张图里两套运动学」。
"""
import math

import numpy as np

# AG-95 仿真代理的几何（arm_grasp_sim/models/ag95.urdf.xacro）
GRIPPER = {
    "body": (0.100, 0.075, 0.060),      # 本体，中心在 base_link 上方 body_z/2
    "body_z": 0.060,
    "finger": (0.022, 0.030, 0.1442),   # 单指碰撞盒 = tool0→指尖外沿 − 本体高
    "stroke_half": 0.0475,              # 95 mm 全开 / 2
    "tip_z": 0.2042,                    # tool0 → 指尖外沿（闭合时）
}

# 抓取深度：指尖外沿越过被接近面 18.5 mm（A3 定案，与块高无关 —— 见
# docs/阶段/阶段1-点对点抓放/A3精密抓放-实验记录.md §3）
GRASP_DEPTH = 0.0185


def rpy_to_quat(rpy):
    """XYZ 欧拉角（弧度）→ 四元数 (x, y, z, w)。

    与 ``grasp_planner/planner_node.py:rpy_to_quat`` **同一算式** —— 探针用它造
    抓取位姿，必须和规划节点给出的姿态口径一致（rpy=(π,0,ψ) = 竖直朝下）。
    """
    cr, sr = math.cos(rpy[0] / 2), math.sin(rpy[0] / 2)
    cp, sp = math.cos(rpy[1] / 2), math.sin(rpy[1] / 2)
    cy, sy = math.cos(rpy[2] / 2), math.sin(rpy[2] / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def down_quat(yaw_rad=0.0):
    """夹爪竖直朝下、绕竖直轴转 yaw：rpy=(π, 0, ψ)。"""
    return rpy_to_quat((math.pi, 0.0, yaw_rad))


def interp_positions(points, step=0.05):
    """关节空间等步长插值，返回 N×M 数组列表（含两端点）。

    为什么要补点：轨迹服务只给航点，**航点之间可能撞**。判据是「整条路径无碰撞」，
    所以必须按关节位移细分到 ``step`` 弧度以内再逐点核验。只有 1 个航点时原样返回。
    """
    pts = [np.asarray(p, dtype=float) for p in points]
    if len(pts) <= 1:
        return pts
    out = []
    for a, b in zip(pts[:-1], pts[1:]):
        n = max(1, int(np.ceil(np.max(np.abs(b - a)) / step)))
        for i in range(n):
            out.append(a + (b - a) * i / n)
    out.append(pts[-1])
    return out


def quat_to_mat(q):
    """(x, y, z, w) → 3×3 旋转矩阵。"""
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Obb:
    """有朝向的长方体：中心 + 三个半边长 + 旋转矩阵。"""

    def __init__(self, center, half, rot=None, name=""):
        self.center = np.asarray(center, dtype=float).reshape(3)
        self.half = np.asarray(half, dtype=float).reshape(3)
        self.rot = np.eye(3) if rot is None else np.asarray(rot, dtype=float)
        self.name = name

    @classmethod
    def from_pose(cls, center, size, quat=None, name=""):
        rot = quat_to_mat(quat) if quat is not None else None
        return cls(center, np.asarray(size, dtype=float) / 2.0, rot, name)

    @property
    def axes(self):
        return [self.rot[:, i] for i in range(3)]

    def corner(self, sx, sy, sz):
        """s* 取 ±1，返回该角的坐标。"""
        local = np.array([sx * self.half[0], sy * self.half[1], sz * self.half[2]])
        return self.center + self.rot.dot(local)

    def corners(self):
        return [self.corner(sx, sy, sz)
                for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]

    def expand(self, margin):
        """沿自身三轴各胀 margin/2 —— 用来把「间隙门」折进重叠判定。"""
        return Obb(self.center, self.half + margin / 2.0, self.rot, self.name)

    def __repr__(self):
        return "Obb(%s c=%s h=%s)" % (
            self.name or "?", np.round(self.center, 4).tolist(),
            np.round(self.half, 4).tolist())


def _cross_axes(a, b):
    out = []
    for u in a.axes:
        for v in b.axes:
            c = np.cross(u, v)
            n = np.linalg.norm(c)
            if n > 1e-9:
                out.append(c / n)
    return out


def gap(a, b):
    """两 OBB 的间隙下界（米）。重叠时为 0，不会给负数。

    对 15 条候选分离轴逐条算「投影间隙」，取最大 —— 凸体分离时真实距离 =
    max 方向上的间隙，而我们只查这 15 条，所以这是**下界**。
    """
    d = b.center - a.center
    best = -float("inf")
    for axis in a.axes + b.axes + _cross_axes(a, b):
        ra = sum(a.half[i] * abs(axis.dot(a.axes[i])) for i in range(3))
        rb = sum(b.half[i] * abs(axis.dot(b.axes[i])) for i in range(3))
        best = max(best, abs(axis.dot(d)) - (ra + rb))
    return max(0.0, best)


def collides(a, b, margin=0.0):
    """间隙 ≤ margin 就算碰撞。margin=0 即纯重叠判定。"""
    return gap(a, b) <= margin


def first_hit(link_obbs, scene_obbs, margin=0.0):
    """返回第一对过近的 (link, scene)；全干净返回 None。"""
    for lk in link_obbs:
        for sc in scene_obbs:
            if collides(lk, sc, margin):
                return lk, sc
    return None


# ---------------------------------------------------------------- 场景构造
def table_obb(table_top_z, size_xy=(2.0, 2.0), thickness=0.05, name="table"):
    """桌面：顶面在 table_top_z，往下长 thickness。"""
    return Obb([0.0, 0.0, table_top_z - thickness / 2.0],
               [size_xy[0] / 2.0, size_xy[1] / 2.0, thickness / 2.0], name=name)


def block_obb(block, table_top_z, name=None):
    """真值块 → OBB。``block`` **就是** ``scene_truth.json`` 里的一条。

    三个容易踩的单位/口径：
    - ``x/y/z`` 是**块心**的 base 系坐标（米），``z`` 已经含层高，不要再加台面；
    - ``yaw`` 是**度**（gen_scene 里只在写 SDF 时才转弧度）；
    - ``size`` 是立方边长（米）；只有 ``top`` 时用 ``top − table_top_z`` 反推。
    """
    size = block.get("size")
    if size is None:
        top = block.get("top")
        if top is None:
            raise ValueError("块既没有 size 也没有 top：%r" % (block,))
        size = top - table_top_z
    z = block.get("z")
    if z is None:
        z = table_top_z + size / 2.0
    yaw_rad = math.radians(block.get("yaw", 0.0))
    return Obb.from_pose([block["x"], block["y"], z], [size, size, size],
                         (0.0, 0.0, math.sin(yaw_rad / 2.0),
                          math.cos(yaw_rad / 2.0)),
                         name=name or "block-%s" % block.get("id"))


def gripper_obbs(base_pose, finger_q, extra=None):
    """夹爪 → 三个 OBB（本体 + 左右指），位姿来自 FK。

    ``base_pose`` = (center(xyz), quat(xyzw)) of ``ag95_base_link``；
    手指盒的中心在 base 系 x=±(finger_x/2 + q)、z=body_z+finger_z/2。

    ``extra`` 可给 ``{"left": pose, "right": pose}`` 直接用 FK 的两指位姿
    （推荐：少一层手算）。给了就忽略 ``finger_q`` 的平移推导。
    """
    center, quat = base_pose
    rot = quat_to_mat(quat)
    center = np.asarray(center, dtype=float)
    bx, by, bz = GRIPPER["body"]
    fx, fy, fz = GRIPPER["finger"]
    out = [Obb(center + rot.dot(np.array([0.0, 0.0, GRIPPER["body_z"] / 2.0])),
               [bx / 2.0, by / 2.0, bz / 2.0], rot, name="ag95_base")]

    if extra and "left" in extra and "right" in extra:
        for side in ("left", "right"):
            c, q = extra[side]
            out.append(Obb(np.asarray(c, dtype=float), [fx / 2.0, fy / 2.0, fz / 2.0],
                           quat_to_mat(q), name="ag95_%s_finger" % side))
        return out

    q = float(finger_q)
    finger_z = fz / 2.0 + GRIPPER["body_z"]
    for side, sign in (("left", 1.0), ("right", -1.0)):
        c = center + rot.dot(np.array([sign * (fx / 2.0 + q), 0.0, finger_z]))
        out.append(Obb(c, [fx / 2.0, fy / 2.0, fz / 2.0], rot,
                       name="ag95_%s_finger" % side))
    return out


def mat_to_quat(rot):
    """3×3 旋转矩阵 → 四元数 (x, y, z, w)。

    用 Shepperd 的分支写法（选最大对角元开方），避免迹接近 −1 时的数值放大；
    常规迹 > 0 分支就是最直白的那个公式。
    """
    m = np.asarray(rot, dtype=float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
        w = 0.25 * s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
        w = (m[2, 1] - m[1, 2]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
        w = (m[0, 2] - m[2, 0]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
        w = (m[1, 0] - m[0, 1]) / s
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / n, y / n, z / n, w / n)


def block_axes(block):
    """块的三条世界系单位轴（yaw 绕 base z）。

    顺序与 ``block_obb`` 的局部 x/y/z 一致；第三轴恒为 (0,0,1)（gen_scene 只给
    绕竖直轴的 yaw，块不会翻面）。
    """
    yaw = math.radians(block.get("yaw", 0.0))
    c, s = math.cos(yaw), math.sin(yaw)
    return [np.array([c, s, 0.0]),
            np.array([-s, c, 0.0]),
            np.array([0.0, 0.0, 1.0])]


def grasp_candidates(block, table_top_z=0.0, grasp_depth=GRASP_DEPTH):
    """E7：一个块的全部平行指抓取候选（12 个**摆位** × 2 个腕部朝向）。

    约定（这是判据口径，别改）：
    - ``a`` = tool0 的 **z 轴**，指向块心 = **接近方向**；夹爪在块的 −a 一侧；
    - ``c`` = tool0 的 **x 轴** = **合拢轴**，两指沿 ±c 开合；
    - y = a × c，保证右手系；
    - 指尖外沿越过被接近的那个面 ``grasp_depth``（A3 的 18.5 mm，与块高无关）；
      于是 tool0 在 ``face + a·grasp_depth − a·tip_z``（tip_z=204.2，闭指尖外沿）。

    摆位 = 3 轴 × 2 方向（a 的正负）× 2 个合拢轴（垂直于 a 的另两轴）= 12。
    ``variant`` 0/1 是绕 a 转 180° 的两个腕部朝向：**碰撞几何完全相同**
    （本体与两指都是中心在 tool 轴上的长方体，180° 自映射），只有 IK 分支不同。

    ``vertical`` = True 即 B 档（只允许竖直向下接近，a=(0,0,−1)）：每块 2 个摆位。
    """
    size = block.get("size")
    if size is None:
        size = block["top"] - table_top_z
    size = float(size)
    z = block.get("z")
    if z is None:
        z = table_top_z + size / 2.0
    center = np.array([float(block["x"]), float(block["y"]), float(z)])
    axes = block_axes(block)
    tip_z = GRIPPER["tip_z"]

    out = []
    for i in range(3):
        for sign in (1.0, -1.0):
            a = axes[i] * sign
            face = center - a * (size / 2.0)          # 被接近的面（块心 − a·半边）
            pos = face - a * (tip_z - grasp_depth)    # 指尖恰越过该面 grasp_depth
            for j in range(3):
                if j == i:
                    continue
                c = axes[j]
                yv = np.cross(a, c)
                for variant, (cc, yy) in enumerate(((c, yv), (-c, -yv))):
                    rot = np.column_stack([cc, yy, a])
                    out.append({
                        "block": block.get("id"),
                        "axis": i, "sign": int(sign), "closing": j,
                        "variant": variant,
                        "vertical": bool(np.allclose(a, (0.0, 0.0, -1.0),
                                                     atol=1e-9)),
                        "approach": a.tolist(),
                        "pos": pos.tolist(),
                        "quat": mat_to_quat(rot),
                        "pose": (pos.tolist(), mat_to_quat(rot)),
                    })
    return out


def fingertip(base_pose, q=0.0):
    """两指尖外沿的连线中点（tool0 系 z=tip_z），闭合时为一点。

    ``base_pose`` 是 ag95_base_link 的位姿；tool0 在 base 系 z=-? 不假定 ——
    夹爪本体下沿即 tool0 安装面，所以 base 系 z=0 就是 tool0 所在平面。
    """
    center, quat = base_pose
    rot = quat_to_mat(quat)
    return np.asarray(center, dtype=float) + rot.dot(
        np.array([0.0, 0.0, GRIPPER["tip_z"]]))

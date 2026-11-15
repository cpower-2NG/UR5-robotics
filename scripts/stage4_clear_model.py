#!/usr/bin/env python3
"""E9 自主移遮挡的**纯几何**部分（不 import rclpy，秒级可单测）。

这个模块只回答四个问题，全部是「规划后的块位」上的几何判断：

1. **遮挡是谁**：从 E7 口径的候选位姿逐条结果里，统计哪些块在夹爪占据的空间
   里（``blocker_scores``）。
2. **自由区落点在哪**：``free_destinations`` 给出桌上可放的候选点 —— 在拾取区
   与放置垫之外，避免「挪开的块又挡住别人」或压到宫格。
3. **指定格在哪**：垫面上的 3×3 宫格（``cell_center`` / ``cell_contains``）。
   几何**不在这里发明**：垫心 / 垫偏航 / 宫格间距 / 厚度直接取
   ``arm_grasp_sim/scripts/gen_scene.py`` 的常量（仿真世界与真机垫是同一份
   实测值），自检会反查那份源码。
4. **终态是否合法**：``terminal_report`` —— 契约 §4.3 的「所有块在合法格 /
   自由区，不重叠、不落地、不越界」。⚠️ S3/S4 允许块间**接触**，所以判「重叠」
   必须用带符号的 ``sm.separation`` 加容差，**不能**用 ``sm.gap``（接触与穿插
   都返回 0，见 ``test_stage4_scene_model`` 里那条对照）。

不在这里的东西：IK / 碰撞服务、场景增删、递归清障的搜索 —— 那些要 ROS，
在 ``stage4_clear_probe.py``。
"""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import stage4_scene_model as sm                                  # noqa: E402


# ── 场景几何的**唯一家**：仿真场景生成器 ──────────────────────────────
def _load_gen_scene():
    """按源码路径 import ``arm_grasp_sim/scripts/gen_scene.py``。

    不走 ``ros2_ws/install``：改了源码不用 colcon build（与 sim.launch.py 的
    取向一致）。找不到就直接报错 —— 静默用一份抄来的常量会让「宫格在哪」
    悄悄和仿真世界分家。
    """
    root = os.path.dirname(HERE)          # 仓根（HERE = <仓根>/scripts）
    path = os.path.join(root, "ros2_ws", "src", "arm_grasp_sim", "scripts")
    if path not in sys.path:
        sys.path.insert(0, path)
    import gen_scene                                                # noqa: E402
    return gen_scene


_GS = _load_gen_scene()

BLOCK_M = _GS.BLOCK_M                               # 29.5 mm 方块
PICK_ZONE = tuple(_GS.ZONE)                         # (xmin,xmax,ymin,ymax)
PAD_CENTER = (float(_GS.PAD_CENTER_MM[0]) / 1000.0,
              float(_GS.PAD_CENTER_MM[1]) / 1000.0)
PAD_YAW_RAD = math.radians(_GS.PAD_YAW_DEG)
PAD_PITCH = float(_GS.PAD_PITCH_MM) / 1000.0        # 宫格 60 mm
PAD_THICK = float(_GS.PAD_THICK_M)                  # 垫厚（真机实测值，见 gen_scene）
PAD_FOOTPRINT = tuple(float(v) for v in _GS.PAD_FOOTPRINT_M)   # 垫外框 (x, y) 米
PAD_HALF = 0.15                                     # 兼容旧调用：正方形半边长

# ── 垫位姿：真机每场开跑实测，不许沿用常量 ──────────────────────────
# 上面 ``PAD_CENTER`` / ``PAD_YAW_RAD`` 是**模型默认值**（= gen_scene 的仿真
# 世界），只在没有实测 ``grid.json`` 时成立。真机每场开跑都要用
# ``stage2_pad.py --camera topcam`` 从俯视帧重解一次垫位姿（见
# ``docs/待执行命令.md``「每轮开跑前重解垫位姿」），再由 ``load_grid()`` 覆盖进来。
#
# 为什么不能沿用常量：垫是会被挪动的物体，它同时决定**往哪放**（计划落点）
# 和**判对不对**（落格）—— 一旦错了，计划与判分同源一起错，还互相「自洽」，
# 谁都发现不了。2026-11-09 实测模型常量与印刷垫差 **87.5 mm / 88.7°**
# （台账第二百九十五轮）：计划把块放到 88 mm 外，判分再拿同一个错格心去量。
PAD_POSE_SOURCE = 'gen_scene 模型常量（未实测）'

# ── 垫参数的「未实测」哨兵（C1，2026-11-13）──────────────────────────
# 历史事故（台账第 295 轮）：模型常量与印刷垫实测差 87.5 mm / 88.7°，且同时喂
# 「放哪」和「判对没」—— 放错判错同源、互相自洽。原先的防法是「每个入口记得
# configure_pad」，漏一个入口就静默用错（score 单步重跑就真漏过一次）。
# 现在的防法是**毒值**：垫参数未实测时是下面这个哨兵，任何数值使用（算格心、
# 判垫面、写 manifest、进 f-string）都在**使用处**当场抛 PadNotMeasured ——
# 36 个读取点一个都不用改，静默通道被结构性堵死。

_PAD_MODEL_CENTER = PAD_CENTER            # gen_scene 模型常量，仅仿真复现用
_PAD_MODEL_YAW_RAD = PAD_YAW_RAD
_PAD_MODEL_THICK = PAD_THICK
_PAD_MODEL_FOOTPRINT = PAD_FOOTPRINT
_MODEL_PAD_SOURCE = PAD_POSE_SOURCE


class PadNotMeasured(RuntimeError):
    """垫参数没有实测值就被读了。消息里带该做什么。"""


class _UnmeasuredPad(float):
    """未实测哨兵：``isinstance(float)``（保住类型分派），但一切数值使用都抛。

    变异友好：谁把「未实测」当数字用，谁就在自己那一行红，报错带指引。
    """

    def __new__(cls, label):
        self = super().__new__(cls, 'nan')
        self._label = label
        return self

    def _fail(self, *a, **kw):
        raise PadNotMeasured(
            "垫参数 %s 没有实测值 —— 真机流程先 load_grid()/configure_pad()；"
            "仿真复现显式调 install_model_pad_defaults()（%s）"
            % (self._label, PAD_POSE_SOURCE))

    __add__ = __radd__ = _fail
    __sub__ = __rsub__ = _fail
    __mul__ = __rmul__ = _fail
    __truediv__ = __rtruediv__ = _fail
    __lt__ = __le__ = __gt__ = __ge__ = _fail
    __neg__ = _fail
    __float__ = _fail
    __round__ = _fail
    __format__ = _fail

    def __repr__(self):
        return 'PAD_UNMEASURED(%s)' % self._label

    __str__ = __repr__


def _unmeasured(label):
    return _UnmeasuredPad(label)


def _install_pad(center, yaw_rad, thick, footprint, source):
    global PAD_CENTER, PAD_YAW_RAD, PAD_POSE_SOURCE, PAD_THICK, PAD_FOOTPRINT
    PAD_CENTER = (float(center[0]), float(center[1]))
    PAD_YAW_RAD = float(yaw_rad)
    PAD_THICK = float(thick)
    PAD_FOOTPRINT = (float(footprint[0]), float(footprint[1]))
    PAD_POSE_SOURCE = str(source)


def configure_pad(center_mm=None, yaw_deg=None, source=None,
                  thickness_m=None, footprint_m=None):
    """把宫格参考换成实测垫位姿 / 垫厚 / 外框。

    ``center_mm`` 是 base 系**毫米**；``thickness_m``、``footprint_m``（``(x, y)``
    米）来自真机实测（v0.6 §20.5：仿真跟真机，不反过来）。给 None 的字段**保持
    原值** —— 未实测的就保持未实测（哨兵继续毒着），**不回退模型常量**（C1：
    隐式回退正是要拆掉的东西）。
    """
    global PAD_CENTER, PAD_YAW_RAD, PAD_POSE_SOURCE, PAD_THICK, PAD_FOOTPRINT
    if center_mm is not None:
        PAD_CENTER = (float(center_mm[0]) / 1000.0,
                      float(center_mm[1]) / 1000.0)
    if yaw_deg is not None:
        PAD_YAW_RAD = math.radians(float(yaw_deg))
    if thickness_m is not None:
        PAD_THICK = float(thickness_m)
    if footprint_m is not None:
        PAD_FOOTPRINT = (float(footprint_m[0]), float(footprint_m[1]))
    if source is not None:
        PAD_POSE_SOURCE = str(source)


def install_model_pad_defaults():
    """显式安装 gen_scene 模型常量 —— **只许仿真复现/自检调用**。

    真机流程没有这一步的容身之处：模型常量与印刷垫实测差 87.5 mm / 88.7°，
    装上它 = 放错判错同源。
    """
    _install_pad(_PAD_MODEL_CENTER, _PAD_MODEL_YAW_RAD, _PAD_MODEL_THICK,
                 _PAD_MODEL_FOOTPRINT, 'gen_scene 模型常量（显式安装）')


def reset_pad():
    """回到「未实测」哨兵（自检的干净起点；真机运行不该调它装模型常量）。"""
    global PAD_CENTER, PAD_YAW_RAD, PAD_POSE_SOURCE, PAD_THICK, PAD_FOOTPRINT
    PAD_CENTER = (_unmeasured('PAD_CENTER.x'), _unmeasured('PAD_CENTER.y'))
    PAD_YAW_RAD = _unmeasured('PAD_YAW_RAD')
    PAD_THICK = _unmeasured('PAD_THICK')
    PAD_FOOTPRINT = (_unmeasured('PAD_FOOTPRINT.x'),
                     _unmeasured('PAD_FOOTPRINT.y'))
    PAD_POSE_SOURCE = '未实测（须先解垫：stage4_pad_from_burst.py）'


# 缺省状态 = **未实测**（C1 的核心：没有隐式模型回退）。
reset_pad()


def load_grid(path):
    """读 grid 文档，按**实测垫参数**配置宫格（C1 后：位姿 + 厚度 + 足迹）。

    新文档（stage4_pad_from_burst --pad-spec 起）自带 ``pad_thickness_m`` /
    ``pad_footprint_m`` ⇒ 一份文件就是全部实测垫真值，探针/判分的**新进程**
    不再有「厚度足迹从哪来」的问题。旧文档缺厚度时回退 ``T_base_pad[2,3]``
    （解垫垫面 z，同是实测）；缺足迹时不回退（哨兵毒着，下游用到即抛——
    足迹只有一个家：pad-spec）。
    """
    with open(path, encoding='utf-8') as f:
        doc = json.load(f)
    if doc.get('kind') != 'stage2-grid':
        raise ValueError('%s 不是 stage2-grid（kind=%r）'
                         % (path, doc.get('kind')))
    try:
        center = doc['grid_center_base_mm']
        yaw = doc['grid_yaw_base_deg']
    except KeyError as exc:
        raise ValueError('%s 缺字段 %s' % (path, exc))
    configure_pad(center, yaw, source=os.path.abspath(path))
    thick = doc.get('pad_thickness_m')
    if thick is None and doc.get('T_base_pad'):
        try:
            thick = float(doc['T_base_pad'][2][3])
        except (TypeError, IndexError, ValueError):
            thick = None
    if thick is not None:
        configure_pad(thickness_m=float(thick))
    foot = doc.get('pad_footprint_m')
    if foot:
        configure_pad(footprint_m=[float(foot[0]), float(foot[1])])
    return doc


def find_grid(hint_dir):
    """在一堆可能的目录里找本场的实测垫位姿。返回路径或 ``None``。

    约定文件名（按优先级）：``grid-pre.json``（每场跑前那一次）、``grid.json``。
    """
    if not hint_dir:
        return None
    for name in ('grid-pre.json', 'grid.json'):
        p = os.path.join(hint_dir, name)
        if os.path.exists(p):
            return p
    return None

# 仿真桌面（1.15×1.10×0.04，位姿 (0.225, 0.10, −0.02)）。这份字面量在
# gen_scene.build_world 的 SDF 字符串里，没有名字；这里自检会反查源码，
# 改了桌子这个文件会红。
TABLE_CENTER = (0.225, 0.10)
TABLE_SIZE = (1.15, 1.10)
TABLE_XMIN = TABLE_CENTER[0] - TABLE_SIZE[0] / 2.0
TABLE_XMAX = TABLE_CENTER[0] + TABLE_SIZE[0] / 2.0
TABLE_YMIN = TABLE_CENTER[1] - TABLE_SIZE[1] / 2.0
TABLE_YMAX = TABLE_CENTER[1] + TABLE_SIZE[1] / 2.0

# ── 判据参数（E9 口径，改这里 = 改判据，须记录）──────────────────────
PEN_TOL = 0.0002      # 终态判「穿插」的容差 0.2 mm。S4 的接触对是精确贴面
                      # （数值上贴 0），任何真实叠放至少差几 mm —— 0.2 mm 把
                      # 浮点噪声和真穿插分得很开。
#: 声明贴合对（中心距 ∈ 贴合带，见 ``_pair_ids``）的终态穿插容差 2 mm。
#: 背景（台账第三百三十九轮）：S1:1 红/橙贴合对的真值质心各被拆分边界内偏
#: ~1.6 mm ⇒ 探针终态这对 OBB 测得 3.84 mm「穿插」，把 4 个无关目标的终态
#: 全判非法。物理上贴合对本来就贴着，终态「保持接触」合法。质心钳制（方案②）
#: 后真机回放的残余还有两类：① 位置残差 s1-1 −0.56 mm（钳制在深度系做、
#: 共识按 x/y 各自取中位）；② 拆分片形状被扭曲 ⇒ 彩色掩码 yaw 噪声
#: （s1-4 cyan 共识 yaw 0.1° vs 场上其余块 49–88°，±3° yaw 对贴面对就是
#: ~1.5 mm 表观穿插）。容差按这两条实测残余取 2 mm；超出仍是真穿插。
#: 只在显式传 ``pair_pen_tol`` 的调用方生效（探针），打分/报告的旧口径不动。
PAIR_PEN_TOL = 0.002
FREE_CLEAR = 0.005    # 挪到自由区的块，离别的块至少 5 mm（不贴面，免得又被算遮挡）
# 自由区：拾取区北侧的空台面（拾取区 y ≤ 0.42，垫在 y ≤ 0.038，都不碰）
FREE_X = (0.28, 0.75)
FREE_Y = (0.46, 0.62)
FREE_STEP = 0.04
FREE_ANCHOR = (0.515, 0.54)   # 落点排序锚点（带中央）


# ---------------------------------------------------------------- 垫宫格
#: 正式轮默认用哪几个格（块 id 升序 → 依次取）。四个角 + 垫心：奇偶性一致
#: （同一组斜线，互不挤格），两两格心 ≥ 84.85 mm（= 60·√2）。S0 的 5 块按
#: id 升序落 8、6、4、2、0 —— 远格先填，后来的块进垫通道不被挡。
#: 这是**轮级决定**，不由探针或布局各自现编（同一布局两次重复要能逐格对照）。
DEFAULT_CELLS = (8, 6, 4, 2, 0)

#: 按批缺省格（2026-11-15 S1/S2 接入编排器）：S0 五格不变；S1/S2 六块六格 ——
#: 第 6 格取边中格 **1**（与 0/2 相邻各 60 mm，29.5 mm 块物理不挤；执行次序
#: 仍是远格先填 8,6,4,2 → 1 → 0）。S3（7 块）/S4（9 块）进阶批等接入时再定。
DEFAULT_CELLS_BY_BATCH = {"S0": DEFAULT_CELLS,
                          "S1": (8, 6, 4, 2, 1, 0),
                          "S2": (8, 6, 4, 2, 1, 0)}


def parse_cells(spec, n=None, batch=None):
    """``"8,6,4,2,0"``（允许中文逗号）→ ``[8, 6, 4, 2, 0]``。

    ``spec`` 为空 ⇒ 取按批缺省（``DEFAULT_CELLS_BY_BATCH[batch]``，未知批次
    回退 ``DEFAULT_CELLS``）；``n`` 给了就要求个数正好等于 ``n``（少一个格＝
    有一个目标无处可放，宁可现在停）。格号重复 / 越界一律报错。显式 ``spec``
    优先于按批缺省（行为不变）。
    """
    if spec is None or (isinstance(spec, str) and not spec.strip()):
        base = DEFAULT_CELLS_BY_BATCH.get(batch, DEFAULT_CELLS)
        cells = list(base[:n] if n is not None else base)
    else:
        text = str(spec).strip().replace("，", ",")
        try:
            cells = [int(p.strip()) for p in text.split(",")]
        except ValueError:
            raise ValueError("--cells 只收整数格号，实得 %r" % (spec,))
    if n is not None and len(cells) != int(n):
        raise ValueError("--cells 给了 %d 个格（%s），但有 %d 个目标 —— "
                         "个数必须一致" % (len(cells), cells, int(n)))
    for c in cells:
        if not 0 <= c < 9:
            raise ValueError("宫格下标 0–8，收到 %r" % (c,))
    if len(set(cells)) != len(cells):
        raise ValueError("--cells 有重复格号：%s" % (cells,))
    return cells


def assign_cells(ids, cells=None):
    """块 id 列表 → ``{str(id): 格号}``：**id 升序**依次配 ``cells``。

    ``cells`` 缺省 = ``DEFAULT_CELLS``（8,6,4,2,0）。这是「哪个块进哪个格」的
    唯一出处 —— 探针、计划、判分三处都读它，免得各自排序排得不一样。
    """
    cells = list(cells) if cells is not None else list(DEFAULT_CELLS)
    if len(cells) < len(ids):
        raise ValueError("%d 个目标只有 %d 个格（%s）"
                         % (len(ids), len(cells), cells))
    for c in cells[:len(ids)]:
        if not 0 <= int(c) < 9:
            raise ValueError("宫格下标 0–8，收到 %r" % (c,))

    def _key(v):
        s = str(v)
        return (0, int(s), s) if s.isdigit() else (1, 0, s)

    return {str(bid): int(cells[i]) for i, bid in enumerate(sorted(ids, key=_key))}


def cell_center(index):
    """宫格 ``index``（0–8，行主序）的世界坐标 (x, y)。与 gen_scene 的
    ``_pad_grid_sdf`` 同一个式子：局部 ((c−1)·pitch, (1−r)·pitch) 再按垫偏航旋转。"""
    if not 0 <= index < 9:
        raise ValueError("宫格下标 0–8，收到 %r" % (index,))
    r, c = divmod(index, 3)
    lx = (c - 1) * PAD_PITCH
    ly = (1 - r) * PAD_PITCH
    ca, sa = math.cos(PAD_YAW_RAD), math.sin(PAD_YAW_RAD)
    return (PAD_CENTER[0] + ca * lx - sa * ly,
            PAD_CENTER[1] + sa * lx + ca * ly)


def to_pad_frame(x, y):
    """世界 (x, y) → 垫系局部 (u, v)。"""
    dx, dy = x - PAD_CENTER[0], y - PAD_CENTER[1]
    ca, sa = math.cos(PAD_YAW_RAD), math.sin(PAD_YAW_RAD)
    return (ca * dx + sa * dy, -sa * dx + ca * dy)


def cell_index_of(x, y):
    """(x, y) 落在哪个宫格里（按中心最近、且不超过半格距）；不在垫上给 None。"""
    u, v = to_pad_frame(x, y)
    c = round(u / PAD_PITCH) + 1
    r = 1 - round(v / PAD_PITCH)       # 局部 ly = (1−r)·pitch（行 0 在 +v 侧）
    if not (0 <= c <= 2 and 0 <= r <= 2):
        return None
    uu, vv = (c - 1) * PAD_PITCH, (1 - r) * PAD_PITCH
    if abs(u - uu) > PAD_PITCH / 2.0 or abs(v - vv) > PAD_PITCH / 2.0:
        return None
    return r * 3 + c


def cell_contains(x, y, yaw_deg, size):
    """块（中心 x,y、yaw、边长 size）的四角是否都在**同一个** 60 mm 宫格里。

    这是「不压格」的判据。29.5 mm 方块任意 yaw 的半对角 20.9 mm < 30 mm，
    所以在格心处永远成立；用它是为了拦住「中心偏了 / 块太大」的将来。
    """
    idx = cell_index_of(x, y)
    if idx is None:
        return False, None
    ca, sa = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    ca_p, sa_p = math.cos(PAD_YAW_RAD), math.sin(PAD_YAW_RAD)
    half = size / 2.0
    for sx in (-1, 1):
        for sy in (-1, 1):
            lx = sx * half * ca - sy * half * sa
            ly = sx * half * sa + sy * half * ca
            wx, wy = x + lx, y + ly
            dx, dy = wx - PAD_CENTER[0], wy - PAD_CENTER[1]
            u = ca_p * dx + sa_p * dy
            v = -sa_p * dx + ca_p * dy
            r, c = divmod(idx, 3)
            uu, vv = (c - 1) * PAD_PITCH, (1 - r) * PAD_PITCH
            if abs(u - uu) > PAD_PITCH / 2.0 or abs(v - vv) > PAD_PITCH / 2.0:
                return False, idx
    return True, idx


# ---------------------------------------------------------------- 自由区
def free_destinations(step=FREE_STEP):
    """自由区里的候选落点（按离锚点的距离排序，近的先试）。"""
    xs = np.arange(FREE_X[0], FREE_X[1] + 1e-9, step)
    ys = np.arange(FREE_Y[0], FREE_Y[1] + 1e-9, step)
    pts = [(float(x), float(y)) for x in xs for y in ys]
    pts.sort(key=lambda p: math.hypot(p[0] - FREE_ANCHOR[0],
                                      p[1] - FREE_ANCHOR[1]))
    return pts


def in_free_band(x, y):
    return FREE_X[0] <= x <= FREE_X[1] and FREE_Y[0] <= y <= FREE_Y[1]


# ---------------------------------------------------------------- 块位
def ghost(block, x, y, z_bottom=None):
    """真值块 → 搬到 (x, y) 的副本（保持 id / size / yaw）。

    ``z_bottom`` 默认落台面（0）；放垫上就传 ``PAD_THICK``。
    """
    size = float(block["size"])
    z = (0.0 if z_bottom is None else float(z_bottom)) + size / 2.0
    out = dict(block)
    out.update({"x": float(x), "y": float(y), "z": z})
    return out


def place_in_cell(block, index):
    """把块放到宫格 ``index`` 的格心、垫面上。"""
    cx, cy = cell_center(index)
    return ghost(block, cx, cy, z_bottom=PAD_THICK)


# ---------------------------------------------------------------- 终态
def _pair_ids(blocks, lo=0.72, hi=1.02):
    """块表里**贴合对**的下标对集合（探针终态的贴合对豁免用）。

    口径与 ``stage4_layouts.contact_pairs`` 同一贴合带（中心距 ∈ lo–hi×块边、
    同层同高）—— 那边是「摆块提示/布局校验」的家；这里不直接 import 它，
    因为 layouts 反向 import 本模块，会环。
    """
    out = set()
    for i in range(len(blocks)):
        for j in range(i + 1, len(blocks)):
            a, c = blocks[i], blocks[j]
            size = float(a["size"])
            if abs(a.get("layer", 0) - c.get("layer", 0)) >= 1:
                continue
            if abs((float(a["z"]) - float(a["size"]) / 2.0)
                   - (float(c["z"]) - float(c["size"]) / 2.0)) > 0.001:
                continue                       # 不同高：不是并排贴合对
            d = math.hypot(a["x"] - c["x"], a["y"] - c["y"])
            if lo * size <= d <= hi * size:
                out.add((i, j))
    return out


def terminal_report(blocks, pen_tol=PEN_TOL, pair_pen_tol=None):
    """全桌终态合法性（契约 §4.3 的那条）。

    每块：``zone`` ∈ {slot, free}、``in_bounds``、``legal``；整桌：
    ``overlaps``（穿插对数）、``off_table``（不在台面/垫上或掉下桌）、
    ``out_of_bounds``（四角出桌）。

    「有支撑」三种都算：落台面、落垫面、**压在另一块上**（S4 的塔本来就
    叠着 —— 只看台面会把合法的塔判成悬空）。

    ``pair_pen_tol`` 给了（探针传 ``PAIR_PEN_TOL``）⇒ 终态里中心距落在贴合带
    的对（``_pair_ids``，物理上本就贴着的手摆贴合对）用这个放宽容差判穿插，
    其余对仍用 ``pen_tol``；用了宽容差的对记在返回值 ``pair_tol_pairs``。
    缺省 ``None`` = 行为与旧版完全一致（打分/报告口径不动）。
    """
    obbs = [sm.block_obb(b, 0.0, name="block-%s" % b.get("id")) for b in blocks]
    tops = [float(b["z"]) + float(b["size"]) / 2.0 for b in blocks]

    rows = []
    for k, b in enumerate(blocks):
        size = float(b["size"])
        z_bottom = float(b["z"]) - size / 2.0
        half = size / 2.0
        ca = math.cos(math.radians(b.get("yaw", 0.0)))
        sa = math.sin(math.radians(b.get("yaw", 0.0)))
        corners = []
        for sx in (-1, 1):
            for sy in (-1, 1):
                corners.append((b["x"] + sx * half * ca - sy * half * sa,
                                b["y"] + sx * half * sa + sy * half * ca))
        in_bounds = all(TABLE_XMIN <= x <= TABLE_XMAX and
                        TABLE_YMIN <= y <= TABLE_YMAX for x, y in corners)

        on_table = abs(z_bottom) <= 0.001
        # 垫面判据用**实测**厚度与外框（§20.5），不是模型常量；坐标转垫系判矩形。
        # 高度容差**必须小于垫厚的一半**：垫厚 0.17 mm 时，±1 mm 会把「块就在
        # 台面上、只是落在垫的外框内」也算成「在垫上」（两者高度只差 0.17 mm），
        # 于是拿台面上的位置去判宫格 → 假「压格」。实测垫厚下容差 0.085 mm，
        # 只有真的垫在垫面上（仿真真值精确到 0.1 µm）才成立。
        pad_tol = min(0.001, PAD_THICK / 2.0)
        pu, pv = to_pad_frame(b["x"], b["y"])
        on_pad = (abs(z_bottom - PAD_THICK) <= pad_tol
                  and abs(pu) <= PAD_FOOTPRINT[0] / 2.0
                  and abs(pv) <= PAD_FOOTPRINT[1] / 2.0)
        on_block = False
        if not (on_table or on_pad):
            for j, other in enumerate(blocks):
                if j == k:
                    continue
                if abs(z_bottom - tops[j]) > 0.001:
                    continue
                # 高度对上还不够：**必须是同一块真的在它下面**。两个 OBB 在
                # 3D 里贴着（separation ≈ 0）才说明有接触；只在高度上对齐、XY
                # 离得很远的块不能当支撑 —— 上一版只查「不穿插」，于是塔顶块
                # 在塔底被挪走后仍被判「有支撑」，这是假绿灯。
                if sm.separation(obbs[k], obbs[j]) <= 0.001:
                    on_block = True
                    break
        supported = on_table or on_pad or on_block
        off_table = (not supported) and z_bottom < 0.0
        # 悬空（没落地也没落垫/落块）既不是「掉下桌」也不是合法终态，单列
        floating = (not supported) and z_bottom > 0.0

        zone = "slot" if on_pad else "free"
        reasons = []
        if not in_bounds:
            reasons.append("出桌")
        if floating:
            reasons.append("悬空")
        if off_table:
            reasons.append("不在台面")
        if on_pad:
            inside, idx = cell_contains(b["x"], b["y"], b.get("yaw", 0.0), size)
            if not inside:
                reasons.append("压格")
            else:
                zone = "slot"
        rows.append({"id": b.get("id"), "zone": zone, "in_bounds": in_bounds,
                     "legal": not reasons, "reasons": reasons,
                     "on_block": on_block,
                     "z_bottom_mm": round(z_bottom * 1000.0, 2),
                     "cell": cell_index_of(b["x"], b["y"])})

    overlaps = 0
    pairs = []
    tol_pairs = []
    declared = (_pair_ids(blocks) if pair_pen_tol is not None else set())
    for i in range(len(obbs)):
        for j in range(i + 1, len(obbs)):
            is_pair = (i, j) in declared
            tol = pair_pen_tol if is_pair else pen_tol
            sep = sm.separation(obbs[i], obbs[j])
            if sep < -tol:
                overlaps += 1
                pairs.append([blocks[i].get("id"), blocks[j].get("id"),
                              round(sep * 1000.0, 2)])
            elif is_pair:
                # 豁免且通过的对（sep ≥ −pair_pen_tol）：记下来供人复核——
                # 若某天这里出现大负值，说明贴合对的真值质量在恶化。
                tol_pairs.append([blocks[i].get("id"), blocks[j].get("id"),
                                  round(sep * 1000.0, 2)])

    n_bad = len([r for r in rows if not r["legal"]])
    n_off = len([r for r in rows if "不在台面" in r["reasons"]])
    n_out = len([r for r in rows if "出桌" in r["reasons"]])
    ok = (overlaps == 0 and n_bad == 0)
    return {"ok": ok, "overlaps": overlaps, "overlap_pairs": pairs,
            "pair_tol_pairs": tol_pairs,
            "off_table": n_off, "out_of_bounds": n_out,
            "illegal": n_bad, "blocks": rows}


def dependents(blocks, bid, tol=0.001):
    """压在块 ``bid`` 上的块 id 列表（必须**先**挪走，才能动它）。

    这是第二种「遮挡」：不是夹爪碰不到，而是**一动它上面那块就会掉**。
    判据与 ``terminal_report`` 的支撑判定同一口径：z_bottom 贴着 bid 的
    顶面（±1 mm），且两块 OBB 真的接触（separation ≤ 1 mm）。
    """
    idx = None
    tops = [float(b["z"]) + float(b["size"]) / 2.0 for b in blocks]
    # 按字符串比：``blocker_scores`` 的键是 ``str(id)``（探针递归时 bid 是 str），
    # 直接用 == 会让「被挡住的块自己上面还压着块」这种情况静默漏掉依赖。
    for k, b in enumerate(blocks):
        if str(b.get("id")) == str(bid):
            idx = k
            break
    if idx is None:
        return []
    obbs = {k: sm.block_obb(b, 0.0, name="block-%s" % b.get("id"))
            for k, b in enumerate(blocks)}
    out = []
    for j, b in enumerate(blocks):
        if j == idx:
            continue
        z_bottom = float(b["z"]) - float(b["size"]) / 2.0
        if abs(z_bottom - tops[idx]) > tol:
            continue
        if sm.separation(obbs[j], obbs[idx]) <= tol:
            out.append(b.get("id"))
    return out


# ---------------------------------------------------------------- 遮挡归属
def blocker_scores(rows):
    """E7 候选结果 → 每块「挡了多少个候选位姿」。

    ``rows`` 是 ``stage4_clear_probe`` 每块评测的逐位姿记录，带
    ``per_other``（块 id → 该位姿下夹爪与它的最小间隙，米）与 ``verdict``。
    只统计**夹爪真的碰到它**（间隙 ≤ 0）的位姿：``ok`` 的位姿不算遮挡，但一个
    位姿若同时挨着两块，两块都记 —— 挪走一块可能还不够，靠重测决定。

    返回 ``{id: {"n": 命中的位姿数, "worst_mm": 最负的间隙}}``；按 ``n`` 降序。
    """
    out = {}
    for r in rows:
        per = r.get("per_other") or {}
        hit = [(k, g) for k, g in per.items() if g is not None and g <= 0.0]
        for k, g in hit:
            e = out.setdefault(k, {"n": 0, "worst_mm": 0.0})
            e["n"] += 1
            e["worst_mm"] = min(e["worst_mm"], round(g * 1000.0, 1))
    return dict(sorted(out.items(), key=lambda kv: (-kv[1]["n"],
                                                    kv[1]["worst_mm"])))


def pick_blocker(scores, exclude=()):
    """按「挡得最多」选一个遮挡块；``exclude`` 里的（正在清/已挪走）跳过。"""
    for bid, _ in scores.items():
        if bid not in exclude:
            return bid
    return None

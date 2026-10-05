#!/usr/bin/env python3
"""
方块仿真世界生成器
==================
生成 block_grid_world.sdf：随机位置方块 + 桌面 + 俯瞰相机。

默认行为：方块在桌面上随机散布（位置随机、颜色随机、1~3个圆柱），
同时输出 block_grid_world_meta.json（记录各方块实际坐标）供自动标注使用。

用法:
    python3 scripts/generate_grid_world.py                       # 默认随机
    python3 scripts/generate_grid_world.py --seed 42             # 固定种子可复现
    python3 scripts/generate_grid_world.py --num_blocks 15       # 15 个方块
    python3 scripts/generate_grid_world.py --pos_jitter 0.10     # 更大扰动
    python3 scripts/generate_grid_world.py --fixed               # 固定 3×3 网格（调试用）
    python3 scripts/generate_grid_world.py --classes red,yellow,cyan   # 只摆这 3 类

输出:
    ros2_ws/src/yolo_bringup/worlds/block_grid_world.sdf
    ros2_ws/src/yolo_bringup/worlds/block_grid_world_meta.json

类别（多类别分拣）
------------------
「类别」在仿真里就是**颜色**，定义在 `scripts/block_classes.py`（唯一来源）。
本脚本负责把它写进 meta 的 `classes` 与每个方块的 `class_name`/`class_id`，
`auto_label_grid.py` 再照着它写标签 —— 所以**改类别表只改那一个文件**。

⚠️ `class_id` = 名字在 `--classes` 列表里的**下标**，传参顺序即类别号顺序，
必须与 `build_dataset.py --class-names` 完全一致。顺序错了不会报任何错
（每个类别号都还在 [0,nc) 里），只是语义全错 —— 详见 block_classes.py。
"""

import os
import sys
import json
import random
import argparse
import math

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from block_classes import (                                        # noqa: E402
    DEFAULT_CLASSES, color_of, class_index, parse_classes, describe,
)


# ============================================================
# 默认参数（可由命令行覆盖）
# ============================================================
BLOCK_SIZE = 0.04               # 方块边长 4cm
TABLE_X = 0.5                   # 桌面中心 X
TABLE_Y = 0.0                   # 桌面中心 Y
TABLE_W = 0.8                   # 桌面宽
TABLE_H = 0.6                   # 桌面深
TABLE_SURFACE_Z = 0.765         # 桌面表面高度
BLOCK_Z = TABLE_SURFACE_Z + BLOCK_SIZE / 2.0  # 方块中心 Z

CAMERA_X, CAMERA_Y, CAMERA_Z = 0.5, 0.0, 1.5
CAMERA_WIDTH, CAMERA_HEIGHT = 640, 480
CAMERA_FOV = 1.047              # 弧度 ≈ 60°

NUM_BLOCKS = 9                  # 默认方块数
POS_JITTER = 0.06               # 默认位置扰动幅度（米）
MIN_GAP = 0.06                  # 方块间最小间距（避免重叠）

# 颜色**不再在这里定义** —— 移到 `scripts/block_classes.py` 的 `PALETTE`。
#
# 原先这里是一个 12 色的 `COLOR_POOL`，颜色与类别**毫无关系**（随机抽取、
# 一个类别号都没记下来），所以那个数据集只能训单类。现在颜色就是类别，
# 而类别表被四个脚本共用，必须只有一份定义。
#
# 新表与旧池子的实质区别：旧池子里有成对的**同色相、只差明度**的颜色
# （蓝/钢蓝、绿/深绿、橙/深橙）。这类「颜色」在真机光照下根本分不开 ——
# 实测同一张照片内部，青色展开只有 2.0°，而最弱的紫色已经到 13.4°，
# 明度差异还会被高光和阴影整个吃掉。所以默认表只取**色相彼此拉开**的 5 个。

# ---- 输出路径 ----
# 默认输出位置：yolo_bringup 包内的 worlds/。
# （原 yolo_vision/worlds/ 已随教学包 yolo_vision/ 一并移除，见 docs/RESTRUCTURE_PLAN.md P1）
#
# ⚠️ **这个目录里的 `block_grid_world.sdf` 与 `block_grid_world_meta.json` 都被 git 跟踪**，
# 所以「默认输出」= 「改仓库」。只想生成草稿的调用方请用 `--out-dir`（或传 `out_dir=`），
# 别让一次「顺手生成」变成两个被跟踪文件的 diff。踩过的实例见 RESTRUCTURE_PLAN.md 第九节。
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUTPUT_DIR = os.path.join(PROJECT_ROOT, "ros2_ws", "src", "yolo_bringup", "worlds")


def clamp(v, lo=0.0, hi=1.0):
    return max(lo, min(hi, v))


# ============================================================
# 随机位置生成（默认模式）
# ============================================================
def generate_random_blocks(num_blocks, jitter, seed):
    """在桌面范围内随机生成方块位置，保证不重叠。"""
    rng = random.Random(seed)
    blocks = []

    # 桌面可用区域（留边距）
    margin = BLOCK_SIZE + 0.02
    x_min = TABLE_X - TABLE_W / 2 + margin
    x_max = TABLE_X + TABLE_W / 2 - margin
    y_min = TABLE_Y - TABLE_H / 2 + margin
    y_max = TABLE_Y + TABLE_H / 2 - margin

    max_attempts = 500
    for i in range(num_blocks):
        placed = False
        for _ in range(max_attempts):
            wx = rng.uniform(x_min, x_max)
            wy = rng.uniform(y_min, y_max)
            ok = True
            for b in blocks:
                if abs(wx - b["x"]) < MIN_GAP and abs(wy - b["y"]) < MIN_GAP:
                    ok = False
                    break
            if ok:
                blocks.append({"x": wx, "y": wy, "z": BLOCK_Z})
                placed = True
                break

        if not placed:
            for _ in range(max_attempts):
                wx = rng.uniform(x_min, x_max)
                wy = rng.uniform(y_min, y_max)
                ok = True
                for b in blocks:
                    dist = math.sqrt((wx - b["x"])**2 + (wy - b["y"])**2)
                    if dist < BLOCK_SIZE * 2:
                        ok = False
                        break
                if ok:
                    blocks.append({"x": wx, "y": wy, "z": BLOCK_Z})
                    placed = True
                    break

        if not placed:
            print(f"[WARN] 方块 {i+1}/{num_blocks} 无法放置（空间不足），跳过。")

    return blocks


def assign_colors_and_shapes(blocks, seed, class_names=None):
    """为方块分配**类别（=颜色）**与形状（1~3 个圆柱体）。

    与旧版的区别：旧版从 12 色池里随机抽颜色，**不记类别号**，所以只能训单类。
    现在每个方块都带上 `class_name` / `class_id`，颜色由类别查 `PALETTE` 得到。

    类别用 round-robin 轮发，保证**类别均衡**（各类方块数最多差 1）；
    但轮发顺序要先**打乱**：

    ⚠️ 方块是按「拒绝采样」逐个找位置生成的，靠前的方块位置自由度更大，
    于是「第 i 个方块」与「它在桌面上的位置」存在弱相关。若按顺序轮发类别，
    类别就会与位置相关 —— 模型完全可能学会「左下角那一类」而不是「青色那一类」，
    而仿真集上的指标照样好看，直到真机换个摆法才暴露。打乱赋值把这捷径掐掉。
    """
    rng = random.Random(seed + 1)
    class_names = list(class_names or DEFAULT_CLASSES)

    # 轮发 + 打乱：先造出均衡的类别序列，再洗掉它与生成顺序的关联
    seq = [class_names[i % len(class_names)] for i in range(len(blocks))]
    rng.shuffle(seq)

    n_cylinders = rng.randint(1, min(3, len(blocks) - 1))
    cylinder_indices = set(rng.sample(range(len(blocks)), n_cylinders))

    for i, b in enumerate(blocks):
        _apply_class(b, seq[i], class_names, rng)
        b["shape"] = "cylinder" if i in cylinder_indices else "box"

    return blocks


# 同一类别内部的**明度抖动**：同一种积木在不同光照、不同朝向下亮度会变，
# 模型必须学会「按色相认，不按明度认」，这里主动把这种差异造出来。
# ⚠️ 幅度不能大：亮度乘子会把色相也拽偏一点（黄这种 R≈G 的颜色最敏感，
#    0.8 倍就把黄压成橄榄色了），而现场紫块的明度实测已经只有 0.31。
CLASS_VALUE_JITTER = (0.85, 1.15)


def _apply_class(b, name, class_names, rng):
    """把类别写进方块：名字、号、颜色。颜色 = 类别基色 × 明度抖动。

    ⚠️ `class_index()` 在名字不在类别表里时**抛异常**（不返回 -1）。
    理由见 block_classes.py：-1 之后最可能的写法是 max(0,idx)，
    于是认不出的颜色悄悄塌成第 0 类，混淆矩阵上完全看不出来。

    ⚠️ 类别号必须按**本次的类别表** `class_names` 算，不能按默认表 ——
    否则 `--classes cyan,red` 会把 cyan 记成 3（默认表里的位置）而不是 0，
    标签里的类别号与 data.yaml 的 names 当场错位。
    """
    base_r, base_g, base_b = color_of(name)
    f = rng.uniform(*CLASS_VALUE_JITTER)
    b["class_name"] = name
    b["class_id"] = class_index(name, class_names)
    b["color"] = (clamp(base_r * f), clamp(base_g * f), clamp(base_b * f))
    return b


# ============================================================
# 固定网格模式（--fixed）
# ============================================================
def generate_fixed_grid(class_names=None):
    """生成固定 3×3 网格方块（调试用）。

    **一行 = 一个类别**。这个模式本来就是给「肉眼可验证的定排布」用的，
    行列结构让「哪一类摆在哪」一眼看得出来，比随机散布更适合查类别问题。

    ⚠️ 类别行数会**按类别表循环**：类别表有 N 个、网格只有 3 行时，
    行 r 用 `class_names[r % N]`。所以 N<3 时有的类别占两行（这是有意的：
    同一类摆两行，正好能看出模型是不是把同一类的两行判成了两类）；
    N>3 时多出来的类别这个模式**摆不下** —— 要覆盖全部类别请用随机模式。

    列方向的 `COL_VARIATION` 是**类内明度抖动**（同一类、不同亮度），
    不是不同类别。它教模型「按色相认，别按明度认」。
    """
    class_names = list(class_names or DEFAULT_CLASSES)
    GRID_ROWS, GRID_COLS = 3, 3
    X_SPACING = 0.16
    Y_SPACING = 0.12
    X_START = TABLE_X - (GRID_COLS - 1) * X_SPACING / 2.0
    Y_START = -(GRID_ROWS - 1) * Y_SPACING / 2.0

    COL_VARIATION = [0.8, 1.0, 1.2]
    CYLINDER_POSITIONS = {(1, 1)}

    blocks = []
    for row in range(GRID_ROWS):
        name = class_names[row % len(class_names)]
        base_r, base_g, base_b = color_of(name)
        for col in range(GRID_COLS):
            factor = COL_VARIATION[col]
            blocks.append({
                "x": X_START + col * X_SPACING,
                "y": Y_START + row * Y_SPACING,
                "z": BLOCK_Z,
                "class_name": name,
                "class_id": class_index(name, class_names),
                "color": (clamp(base_r * factor), clamp(base_g * factor), clamp(base_b * factor)),
                "shape": "cylinder" if (row, col) in CYLINDER_POSITIONS else "box",
            })
    return blocks


# ============================================================
# SDF 生成
# ============================================================
def generate_sdf(blocks):
    """生成 SDF 世界文件内容。"""
    lines = []
    lines.append('<?xml version="1.0" ?>')
    lines.append('<!--')
    lines.append('方块仿真环境')
    lines.append('用于 YOLO 视觉模型训练数据采集')
    lines.append(f'方块数: {len(blocks)}')
    lines.append('-->')
    lines.append('<sdf version="1.11">')
    lines.append('  <world name="block_grid_world">')
    lines.append('')
    lines.append('    <!-- 物理 -->')
    lines.append('    <physics name="1ms" type="bullet">')
    lines.append('      <max_step_size>0.001</max_step_size>')
    lines.append('      <real_time_factor>1.0</real_time_factor>')
    lines.append('      <bullet>')
    lines.append('        <solver><type>sequential_impulse</type></solver>')
    lines.append('      </bullet>')
    lines.append('    </physics>')
    lines.append('')
    lines.append('    <!-- 传感器系统 -->')
    lines.append('    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">')
    lines.append('    </plugin>')
    lines.append('')
    lines.append('    <!-- 用户交互 -->')
    lines.append('    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands">')
    lines.append('    </plugin>')
    lines.append('')
    lines.append('    <!-- 场景信息广播 -->')
    lines.append('    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster">')
    lines.append('    </plugin>')
    lines.append('')
    lines.append('    <!-- 光源 -->')
    lines.append('    <light type="directional" name="sun">')
    lines.append('      <cast_shadows>true</cast_shadows>')
    lines.append('      <pose>0 0 10 0 0 0</pose>')
    lines.append('      <diffuse>1.0 1.0 1.0 1.0</diffuse>')
    lines.append('      <specular>0.5 0.5 0.5 1.0</specular>')
    lines.append('      <attenuation>')
    lines.append('        <range>1000</range>')
    lines.append('        <constant>0.9</constant>')
    lines.append('        <linear>0.01</linear>')
    lines.append('        <quadratic>0.001</quadratic>')
    lines.append('      </attenuation>')
    lines.append('      <direction>-0.5 0.1 -0.9</direction>')
    lines.append('    </light>')
    lines.append('')
    lines.append('    <!-- 地面 -->')
    lines.append('    <model name="ground_plane">')
    lines.append('      <static>true</static>')
    lines.append('      <link name="link">')
    lines.append('        <collision name="collision">')
    lines.append('          <geometry><plane><normal>0 0 1</normal></plane></geometry>')
    lines.append('        </collision>')
    lines.append('        <visual name="visual">')
    lines.append('          <geometry>')
    lines.append('            <plane><normal>0 0 1</normal><size>10 10</size></plane>')
    lines.append('          </geometry>')
    lines.append('          <material>')
    lines.append('            <ambient>0.3 0.3 0.3</ambient>')
    lines.append('            <diffuse>0.5 0.5 0.5</diffuse>')
    lines.append('            <specular>0.1 0.1 0.1</specular>')
    lines.append('          </material>')
    lines.append('        </visual>')
    lines.append('      </link>')
    lines.append('    </model>')
    lines.append('')
    lines.append('    <!-- 桌子 -->')
    lines.append('    <model name="table">')
    lines.append('      <static>true</static>')
    lines.append('      <pose>{:.1f} {:.1f} 0.0 0 0 0</pose>'.format(TABLE_X, TABLE_Y))
    lines.append('')
    lines.append('      <link name="table_top">')
    lines.append('        <pose>0 0 0.75 0 0 0</pose>')
    lines.append('        <collision name="collision">')
    lines.append('          <geometry><box><size>{:.1f} {:.1f} 0.03</size></box></geometry>'.format(TABLE_W, TABLE_H))
    lines.append('        </collision>')
    lines.append('        <visual name="visual">')
    lines.append('          <geometry><box><size>{:.1f} {:.1f} 0.03</size></box></geometry>'.format(TABLE_W, TABLE_H))
    lines.append('          <material>')
    lines.append('            <ambient>0.6 0.4 0.2 1.0</ambient>')
    lines.append('            <diffuse>0.6 0.4 0.2 1.0</diffuse>')
    lines.append('          </material>')
    lines.append('        </visual>')
    lines.append('      </link>')
    lines.append('')
    for name, lx, ly in [("leg_fl", 0.35, 0.25), ("leg_fr", 0.35, -0.25),
                          ("leg_bl", -0.35, 0.25), ("leg_br", -0.35, -0.25)]:
        lines.append(f'      <link name="{name}">')
        lines.append(f'        <pose>{lx} {ly} 0.375 0 0 0</pose>')
        lines.append('        <collision name="collision">')
        lines.append('          <geometry><cylinder><radius>0.025</radius><length>0.75</length></cylinder></geometry>')
        lines.append('        </collision>')
        lines.append('        <visual name="visual">')
        lines.append('          <geometry><cylinder><radius>0.025</radius><length>0.75</length></cylinder></geometry>')
        lines.append('          <material>')
        lines.append('            <ambient>0.5 0.3 0.15 1.0</ambient>')
        lines.append('            <diffuse>0.5 0.3 0.15 1.0</diffuse>')
        lines.append('          </material>')
        lines.append('        </visual>')
        lines.append('      </link>')
    lines.append('    </model>')
    lines.append('')
    lines.append('    <!-- RGBD 相机 -->')
    lines.append('    <model name="camera_model">')
    lines.append('      <static>true</static>')
    lines.append('      <pose>{:.1f} {:.1f} {:.1f} 0.0 1.5708 0.0</pose>'.format(CAMERA_X, CAMERA_Y, CAMERA_Z))
    lines.append('')
    lines.append('      <link name="camera_link">')
    lines.append('        <visual name="visual">')
    lines.append('          <geometry><box><size>0.05 0.05 0.03</size></box></geometry>')
    lines.append('          <material>')
    lines.append('            <ambient>0.2 0.2 0.2 1.0</ambient>')
    lines.append('            <diffuse>0.2 0.2 0.2 1.0</diffuse>')
    lines.append('          </material>')
    lines.append('        </visual>')
    lines.append('')
    lines.append('        <sensor name="rgb_camera" type="camera">')
    lines.append('          <topic>/camera/color/image_raw</topic>')
    lines.append('          <update_rate>15</update_rate>')
    lines.append('          <camera>')
    lines.append('            <horizontal_fov>{:.4f}</horizontal_fov>'.format(CAMERA_FOV))
    lines.append('            <image>')
    lines.append('              <width>{}</width>'.format(CAMERA_WIDTH))
    lines.append('              <height>{}</height>'.format(CAMERA_HEIGHT))
    lines.append('              <format>R8G8B8</format>')
    lines.append('            </image>')
    lines.append('            <clip>')
    lines.append('              <near>0.05</near>')
    lines.append('              <far>10.0</far>')
    lines.append('            </clip>')
    lines.append('          </camera>')
    lines.append('        </sensor>')
    lines.append('')
    lines.append('        <sensor name="depth_camera" type="depth_camera">')
    lines.append('          <topic>/camera/depth/image_raw</topic>')
    lines.append('          <update_rate>15</update_rate>')
    lines.append('          <camera>')
    lines.append('            <horizontal_fov>{:.4f}</horizontal_fov>'.format(CAMERA_FOV))
    lines.append('            <image>')
    lines.append('              <width>{}</width>'.format(CAMERA_WIDTH))
    lines.append('              <height>{}</height>'.format(CAMERA_HEIGHT))
    lines.append('              <format>R_FLOAT32</format>')
    lines.append('            </image>')
    lines.append('            <clip>')
    lines.append('              <near>0.05</near>')
    lines.append('              <far>10.0</far>')
    lines.append('            </clip>')
    lines.append('          </camera>')
    lines.append('        </sensor>')
    lines.append('      </link>')
    lines.append('    </model>')
    lines.append('')

    # 方块
    n_box = sum(1 for b in blocks if b["shape"] == "box")
    n_cyl = sum(1 for b in blocks if b["shape"] == "cylinder")
    lines.append(f'    <!-- 方块：{len(blocks)} 个（{n_box} 正方体 + {n_cyl} 圆柱）-->')
    lines.append('')

    for i, b in enumerate(blocks):
        name = f"block_{i}"
        x, y, z = b["x"], b["y"], b["z"]
        r, g, bl = b["color"]

        lines.append(f'    <model name="{name}">')
        lines.append(f'      <static>false</static>')
        lines.append(f'      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>')
        lines.append(f'      <link name="link">')
        lines.append(f'        <gravity>false</gravity>')
        lines.append(f'        <inertial>')
        lines.append(f'          <mass>0.05</mass>')
        lines.append(f'          <inertia>')
        lines.append(f'            <ixx>0.00001</ixx><ixy>0</ixy><ixz>0</ixz>')
        lines.append(f'            <iyy>0.00001</iyy><iyz>0</iyz>')
        lines.append(f'            <izz>0.00001</izz>')
        lines.append(f'          </inertia>')
        lines.append(f'        </inertial>')

        if b["shape"] == "cylinder":
            radius = BLOCK_SIZE / 2.0
            length = BLOCK_SIZE
            lines.append(f'        <collision name="collision">')
            lines.append(f'          <geometry><cylinder><radius>{radius:.4f}</radius><length>{length:.4f}</length></cylinder></geometry>')
            lines.append(f'        </collision>')
            lines.append(f'        <visual name="visual">')
            lines.append(f'          <geometry><cylinder><radius>{radius:.4f}</radius><length>{length:.4f}</length></cylinder></geometry>')
            lines.append(f'          <material>')
            lines.append(f'            <ambient>{r:.3f} {g:.3f} {bl:.3f} 1.0</ambient>')
            lines.append(f'            <diffuse>{r:.3f} {g:.3f} {bl:.3f} 1.0</diffuse>')
            lines.append(f'          </material>')
            lines.append(f'        </visual>')
        else:
            s = BLOCK_SIZE
            lines.append(f'        <collision name="collision">')
            lines.append(f'          <geometry><box><size>{s:.4f} {s:.4f} {s:.4f}</size></box></geometry>')
            lines.append(f'        </collision>')
            lines.append(f'        <visual name="visual">')
            lines.append(f'          <geometry><box><size>{s:.4f} {s:.4f} {s:.4f}</size></box></geometry>')
            lines.append(f'          <material>')
            lines.append(f'            <ambient>{r:.3f} {g:.3f} {bl:.3f} 1.0</ambient>')
            lines.append(f'            <diffuse>{r:.3f} {g:.3f} {bl:.3f} 1.0</diffuse>')
            lines.append(f'          </material>')
            lines.append(f'        </visual>')

        lines.append(f'      </link>')
        lines.append(f'    </model>')
        lines.append('')

    lines.append('  </world>')
    lines.append('</sdf>')
    return '\n'.join(lines)


# ============================================================
# 元数据输出（JSON，供自动标注读取）
# ============================================================
def save_metadata(blocks, seed, is_fixed_mode, class_names, out_dir=None):
    """输出方块位姿元数据。

    `out_dir` 缺省仍是 `OUTPUT_DIR`（`yolo_bringup/worlds/`）—— 但那个目录
    **被 git 跟踪**，往里写等于改仓库。只想生成一份草稿、不想碰仓库的调用方
    **必须显式传 `out_dir` 或 `--out-dir`**；`generate_synthetic_data.py`
    就是把 40 次生成全部指到临时目录的（它原先没这么做，于是每跑一次
    都静默覆写 `block_grid_world.{sdf,meta.json}`）。

    顶层多出的 `classes` 是**类别号的定义**：`classes[i]` 就是类别号 i 的名字。
    `auto_label_grid.py` 靠它把方块的名字翻成写进标签的号 —— 没有这个字段，
    标注脚本只能自己猜一套类别表，而两边一旦不一致，标签里的号与
    `data.yaml` 的 names 就整体错位（且不会报错）。
    """
    meta = {
        "seed": seed,
        "fixed_mode": is_fixed_mode,
        "num_blocks": len(blocks),
        "classes": list(class_names),
        "camera": {
            "position": [CAMERA_X, CAMERA_Y, CAMERA_Z],
            "width": CAMERA_WIDTH,
            "height": CAMERA_HEIGHT,
            "horizontal_fov": CAMERA_FOV,
        },
        "blocks": [
            {
                "id": i,
                "x": b["x"],
                "y": b["y"],
                "z": b["z"],
                "shape": b["shape"],
                "color": list(b["color"]),
                "class_name": b["class_name"],
                "class_id": b["class_id"],
            }
            for i, b in enumerate(blocks)
        ],
    }

    out_dir = out_dir or OUTPUT_DIR
    os.makedirs(out_dir, exist_ok=True)
    meta_path = os.path.join(out_dir, "block_grid_world_meta.json")
    with open(meta_path, 'w') as f:
        json.dump(meta, f, indent=2)
    print(f"[OK] 元数据: {meta_path}")


# ============================================================
# 主入口
# ============================================================
def main():
    parser = argparse.ArgumentParser(
        description="生成方块仿真世界（默认随机位置）"
    )
    parser.add_argument("--fixed", action="store_true",
                        help="固定 3×3 网格模式（调试用，默认关闭）")
    parser.add_argument("--seed", type=int, default=None,
                        help="随机种子（固定可复现；不指定则每次不同）")
    parser.add_argument("--num_blocks", type=int, default=NUM_BLOCKS,
                        help="方块数量（默认 %(default)s）")
    parser.add_argument("--pos_jitter", type=float, default=POS_JITTER,
                        help="位置扰动幅度/米（默认 %(default)s）")
    parser.add_argument("--out-dir", default=None,
                        help=f"输出目录（默认 {OUTPUT_DIR}，**该目录被 git 跟踪**）。"
                             f"只想生成草稿就指到别处，别默认写进仓库。")
    parser.add_argument("--classes", default=",".join(DEFAULT_CLASSES),
                        help="类别表，逗号分隔，**顺序即类别号**"
                             "（默认 %(default)s）。\n"
                             "⚠️ 必须与 build_dataset.py --class-names 传同一个列表；"
                             "⚠️ 改顺序等于重编所有已标注数据。")
    args = parser.parse_args()

    try:
        class_names = parse_classes(args.classes)
    except ValueError as e:
        # 类别表非法是**配置错误**，当场退出比带着半套类别往下跑强：
        # 往下跑的话，class_index 会在写进 meta 之前才炸，而那时 SDF 可能
        # 已经落盘，留下一份「有方块但没类别」的世界。
        parser.error("--classes %s" % e)

    seed = args.seed if args.seed is not None else random.randint(0, 10**9)
    print(f"[INFO] 类别表（共 {len(class_names)} 类）：{describe(class_names)}")

    if args.fixed:
        print(f"[INFO] 固定网格模式")
        blocks = generate_fixed_grid(class_names)
        is_fixed = True
    else:
        print(f"[INFO] 随机位置模式 (seed={seed}, blocks={args.num_blocks}, jitter={args.pos_jitter}m)")
        blocks = generate_random_blocks(args.num_blocks, args.pos_jitter, seed)
        blocks = assign_colors_and_shapes(blocks, seed, class_names)
        is_fixed = False

    n_box = sum(1 for b in blocks if b["shape"] == "box")
    n_cyl = sum(1 for b in blocks if b["shape"] == "cylinder")
    print(f"[INFO] 生成 {len(blocks)} 个方块 ({n_box} 正方体 + {n_cyl} 圆柱)")

    # 每类的方块数：类别太不均衡时，混淆矩阵里样本少的那几类会被噪声主导，
    # 所以这里主动报出来（round-robin 正常情况下各类最多差 1）。
    per_class = {}
    for b in blocks:
        per_class[b["class_name"]] = per_class.get(b["class_name"], 0) + 1
    print("[INFO] 各类方块数：" + "  ".join(
        f"{n}={per_class.get(n, 0)}" for n in class_names))

    # 生成 SDF
    out_dir = args.out_dir or OUTPUT_DIR
    sdf_content = generate_sdf(blocks)
    os.makedirs(out_dir, exist_ok=True)
    sdf_path = os.path.join(out_dir, "block_grid_world.sdf")
    with open(sdf_path, 'w') as f:
        f.write(sdf_content)
    print(f"[OK] SDF: {sdf_path}")

    # 输出元数据
    save_metadata(blocks, seed, is_fixed, class_names, out_dir)

    # ⚠️ 原打印的是 simulation.launch.py——该文件第九轮已删除（第十轮由
    #    grid_world.launch.py 接管训练世界职责），照抄会报 file not found。
    print(f"\n[完成] 启动: ros2 launch yolo_bringup grid_world.launch.py")
    print(f"       或: gz sim -r {sdf_path}")


if __name__ == "__main__":
    main()

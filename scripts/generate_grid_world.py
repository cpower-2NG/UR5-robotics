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

输出:
    yolo_vision/worlds/block_grid_world.sdf
    yolo_vision/worlds/block_grid_world_meta.json
"""

import os
import json
import random
import argparse
import math


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

# 颜色池：多种可区分的颜色，随机抽取
COLOR_POOL = [
    (0.85, 0.15, 0.15),   # 红
    (0.15, 0.15, 0.85),   # 蓝
    (0.15, 0.80, 0.15),   # 绿
    (1.00, 0.55, 0.00),   # 橙
    (0.60, 0.20, 0.80),   # 紫
    (0.90, 0.80, 0.10),   # 黄
    (0.10, 0.70, 0.70),   # 青
    (0.80, 0.30, 0.50),   # 粉
    (0.40, 0.50, 0.90),   # 钢蓝
    (0.70, 0.55, 0.35),   # 棕
    (0.30, 0.70, 0.30),   # 深绿
    (0.90, 0.45, 0.00),   # 深橙
]

# ---- 输出路径 ----
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUTPUT_DIR = os.path.join(PROJECT_ROOT, "yolo_vision", "worlds")
OUTPUT_DIR_ALT = os.path.join(PROJECT_ROOT, "ros2_ws", "src", "yolo_bringup", "worlds")


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


def assign_colors_and_shapes(blocks, seed):
    """为方块随机分配颜色和形状（2~4个圆柱体）。"""
    rng = random.Random(seed + 1)

    colors = list(COLOR_POOL)
    rng.shuffle(colors)
    assigned_colors = []
    for i in range(len(blocks)):
        assigned_colors.append(colors[i % len(colors)])

    n_cylinders = rng.randint(1, min(3, len(blocks) - 1))
    cylinder_indices = set(rng.sample(range(len(blocks)), n_cylinders))

    for i, b in enumerate(blocks):
        b["color"] = assigned_colors[i]
        b["shape"] = "cylinder" if i in cylinder_indices else "box"

    return blocks


# ============================================================
# 固定网格模式（--fixed）
# ============================================================
def generate_fixed_grid():
    """生成固定 3×3 网格方块（兼容旧版）。"""
    GRID_ROWS, GRID_COLS = 3, 3
    X_SPACING = 0.16
    Y_SPACING = 0.12
    X_START = TABLE_X - (GRID_COLS - 1) * X_SPACING / 2.0
    Y_START = -(GRID_ROWS - 1) * Y_SPACING / 2.0

    ROW_COLORS = [
        (0.85, 0.15, 0.15),
        (0.15, 0.15, 0.85),
        (0.15, 0.80, 0.15),
    ]
    COL_VARIATION = [0.8, 1.0, 1.2]
    CYLINDER_POSITIONS = {(1, 1)}

    blocks = []
    for row in range(GRID_ROWS):
        base_r, base_g, base_b = ROW_COLORS[row]
        for col in range(GRID_COLS):
            factor = COL_VARIATION[col]
            blocks.append({
                "x": X_START + col * X_SPACING,
                "y": Y_START + row * Y_SPACING,
                "z": BLOCK_Z,
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
def save_metadata(blocks, seed, is_fixed_mode):
    """输出方块位姿元数据。"""
    meta = {
        "seed": seed,
        "fixed_mode": is_fixed_mode,
        "num_blocks": len(blocks),
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
            }
            for i, b in enumerate(blocks)
        ],
    }

    for out_dir in [OUTPUT_DIR, OUTPUT_DIR_ALT]:
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
    args = parser.parse_args()

    seed = args.seed if args.seed is not None else random.randint(0, 10**9)

    if args.fixed:
        print(f"[INFO] 固定网格模式")
        blocks = generate_fixed_grid()
        is_fixed = True
    else:
        print(f"[INFO] 随机位置模式 (seed={seed}, blocks={args.num_blocks}, jitter={args.pos_jitter}m)")
        blocks = generate_random_blocks(args.num_blocks, args.pos_jitter, seed)
        blocks = assign_colors_and_shapes(blocks, seed)
        is_fixed = False

    n_box = sum(1 for b in blocks if b["shape"] == "box")
    n_cyl = sum(1 for b in blocks if b["shape"] == "cylinder")
    print(f"[INFO] 生成 {len(blocks)} 个方块 ({n_box} 正方体 + {n_cyl} 圆柱)")

    # 生成 SDF
    sdf_content = generate_sdf(blocks)
    for out_dir in [OUTPUT_DIR, OUTPUT_DIR_ALT]:
        os.makedirs(out_dir, exist_ok=True)
        sdf_path = os.path.join(out_dir, "block_grid_world.sdf")
        with open(sdf_path, 'w') as f:
            f.write(sdf_content)
        print(f"[OK] SDF: {sdf_path}")

    # 输出元数据
    save_metadata(blocks, seed, is_fixed)

    print(f"\n[完成] 启动: ros2 launch yolo_vision sim_detect.launch.py")
    print(f"       或: gz sim -r yolo_vision/worlds/block_grid_world.sdf")


if __name__ == "__main__":
    main()

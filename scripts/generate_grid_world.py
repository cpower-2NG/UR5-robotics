#!/usr/bin/env python3
"""
方阵世界文件生成器
==================
生成 block_grid_world.sdf：5×5 方块方阵 + 桌面 + 俯瞰相机。

用法:
    python3 scripts/generate_grid_world.py

输出:
    ros2_ws/src/yolo_bringup/worlds/block_grid_world.sdf
"""

import os

# ---- 配置 ----
GRID_ROWS = 3
GRID_COLS = 3
BLOCK_SIZE = 0.04          # 方块边长 4cm（稍大方便看清）
TABLE_X = 0.5              # 桌面中心 X
TABLE_Y = 0.0              # 桌面中心 Y
TABLE_SURFACE_Z = 0.765    # 桌面表面高度
BLOCK_Z = TABLE_SURFACE_Z + BLOCK_SIZE / 2.0  # 方块中心 Z
X_SPACING = 0.16           # 列间距（3列跨度0.32m，居中桌面）
Y_SPACING = 0.12           # 行间距（3行跨度0.24m，居中桌面）
X_START = 0.50 - (GRID_COLS-1) * X_SPACING / 2.0  # 居中起始
Y_START = -(GRID_ROWS-1) * Y_SPACING / 2.0        # 居中起始

# 颜色方案：按行区分色系，按列微调深浅
# 每行一个主色调：(R, G, B) 基准
ROW_COLORS = [
    (0.85, 0.15, 0.15),   # 红色系
    (0.15, 0.15, 0.85),   # 蓝色系
    (0.15, 0.80, 0.15),   # 绿色系
]

# 列颜色微调因子（让每列略有深浅变化）
COL_VARIATION = [0.8, 1.0, 1.2]

# 哪些位置放圆柱（用 (row, col) 指定，0-indexed）
CYLINDER_POSITIONS = {(1, 1)}  # 中心圆柱增加形状多样性

# ---- 输出路径 ----
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUTPUT_PATH = os.path.join(
    PROJECT_ROOT, "yolo_vision", "worlds", "block_grid_world.sdf"
)
OUTPUT_PATH2 = os.path.join(
    PROJECT_ROOT, "ros2_ws", "src", "yolo_bringup", "worlds", "block_grid_world.sdf"
)


def clamp(v, lo=0.0, hi=1.0):
    return max(lo, min(hi, v))


def generate_sdf():
    """生成 SDF 世界文件内容。"""
    lines = []
    lines.append('<?xml version="1.0" ?>')
    lines.append('<!--')
    lines.append('方阵仿真环境：5×5 方块网格')
    lines.append('用于 YOLO 视觉模型训练数据采集')
    lines.append('')
    lines.append('用法:')
    lines.append('  gz sim -r block_grid_world.sdf')
    lines.append('  或:')
    lines.append('  ros2 launch yolo_bringup simulation.launch.py world:=block_grid_world.sdf')
    lines.append('-->')
    lines.append('<sdf version="1.11">')
    lines.append('  <world name="block_grid_world">')
    lines.append('')
    lines.append('    <!-- 物理 (bullet: 兼容ros2_control+关节驱动) -->')
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
    lines.append('    <!-- 用户交互（鼠标/键盘控制视角） -->')
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
    lines.append('    <!-- ============================================ -->')
    lines.append('    <!-- 桌子：80×60cm 桌面，高 75cm                  -->')
    lines.append('    <!-- ============================================ -->')
    lines.append('    <model name="table">')
    lines.append('      <static>true</static>')
    lines.append('      <pose>0.5 0.0 0.0 0 0 0</pose>')
    lines.append('')
    lines.append('      <!-- 桌面 -->')
    lines.append('      <link name="table_top">')
    lines.append('        <pose>0 0 0.75 0 0 0</pose>')
    lines.append('        <collision name="collision">')
    lines.append('          <geometry><box><size>0.8 0.6 0.03</size></box></geometry>')
    lines.append('        </collision>')
    lines.append('        <visual name="visual">')
    lines.append('          <geometry><box><size>0.8 0.6 0.03</size></box></geometry>')
    lines.append('          <material>')
    lines.append('            <ambient>0.6 0.4 0.2 1.0</ambient>')
    lines.append('            <diffuse>0.6 0.4 0.2 1.0</diffuse>')
    lines.append('          </material>')
    lines.append('        </visual>')
    lines.append('      </link>')
    lines.append('')
    lines.append('      <!-- 桌腿 -->')
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
    lines.append('    <!-- ============================================ -->')
    lines.append('    <!-- RGBD 相机：固定俯瞰桌面                      -->')
    lines.append('    <!-- ============================================ -->')
    lines.append('    <model name="camera_model">')
    lines.append('      <static>true</static>')
    lines.append('      <pose>0.5 0.0 1.5 0.0 1.5708 0.0</pose>')
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
    lines.append('        <!-- RGB 相机 -->')
    lines.append('        <sensor name="rgb_camera" type="camera">')
    lines.append('          <topic>/camera/color/image_raw</topic>')
    lines.append('          <update_rate>15</update_rate>')
    lines.append('          <camera>')
    lines.append('            <horizontal_fov>1.047</horizontal_fov>')
    lines.append('            <image>')
    lines.append('              <width>640</width>')
    lines.append('              <height>480</height>')
    lines.append('              <format>R8G8B8</format>')
    lines.append('            </image>')
    lines.append('            <clip>')
    lines.append('              <near>0.05</near>')
    lines.append('              <far>10.0</far>')
    lines.append('            </clip>')
    lines.append('          </camera>')
    lines.append('        </sensor>')
    lines.append('')
    lines.append('        <!-- 深度相机 -->')
    lines.append('        <sensor name="depth_camera" type="depth_camera">')
    lines.append('          <topic>/camera/depth/image_raw</topic>')
    lines.append('          <update_rate>15</update_rate>')
    lines.append('          <camera>')
    lines.append('            <horizontal_fov>1.047</horizontal_fov>')
    lines.append('            <image>')
    lines.append('              <width>640</width>')
    lines.append('              <height>480</height>')
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
    lines.append('    <!-- ============================================ -->')
    lines.append(f'    <!-- 方阵方块：{GRID_ROWS}×{GRID_COLS} 网格         -->')
    lines.append('    <!-- 桌面表面 Z={:.3f}，方块中心 Z={:.3f}          -->'.format(TABLE_SURFACE_Z, BLOCK_Z))
    lines.append('    <!-- ============================================ -->')
    lines.append('')

    for row in range(GRID_ROWS):
        base_r, base_g, base_b = ROW_COLORS[row]
        for col in range(GRID_COLS):
            x = X_START + col * X_SPACING
            y = Y_START + row * Y_SPACING
            z = BLOCK_Z

            # 列颜色微调
            factor = COL_VARIATION[col]
            r = clamp(base_r * factor)
            g = clamp(base_g * factor)
            b = clamp(base_b * factor)

            # 命名规则：block_r{row}_c{col}
            name = f"block_r{row}_c{col}"
            is_cylinder = (row, col) in CYLINDER_POSITIONS

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

            if is_cylinder:
                # 圆柱
                radius = BLOCK_SIZE / 2.0
                length = BLOCK_SIZE
                lines.append(f'        <collision name="collision">')
                lines.append(f'          <geometry><cylinder><radius>{radius:.4f}</radius><length>{length:.4f}</length></cylinder></geometry>')
                lines.append(f'        </collision>')
                lines.append(f'        <visual name="visual">')
                lines.append(f'          <geometry><cylinder><radius>{radius:.4f}</radius><length>{length:.4f}</length></cylinder></geometry>')
                lines.append(f'          <material>')
                lines.append(f'            <ambient>{r:.3f} {g:.3f} {b:.3f} 1.0</ambient>')
                lines.append(f'            <diffuse>{r:.3f} {g:.3f} {b:.3f} 1.0</diffuse>')
                lines.append(f'          </material>')
                lines.append(f'        </visual>')
            else:
                # 正方体
                s = BLOCK_SIZE
                lines.append(f'        <collision name="collision">')
                lines.append(f'          <geometry><box><size>{s:.4f} {s:.4f} {s:.4f}</size></box></geometry>')
                lines.append(f'        </collision>')
                lines.append(f'        <visual name="visual">')
                lines.append(f'          <geometry><box><size>{s:.4f} {s:.4f} {s:.4f}</size></box></geometry>')
                lines.append(f'          <material>')
                lines.append(f'            <ambient>{r:.3f} {g:.3f} {b:.3f} 1.0</ambient>')
                lines.append(f'            <diffuse>{r:.3f} {g:.3f} {b:.3f} 1.0</diffuse>')
                lines.append(f'          </material>')
                lines.append(f'        </visual>')

            lines.append(f'      </link>')
            lines.append(f'    </model>')
            lines.append('')

    lines.append('  </world>')
    lines.append('</sdf>')
    return '\n'.join(lines)


def main():
    content = generate_sdf()
    for out in [OUTPUT_PATH, OUTPUT_PATH2]:
        os.makedirs(os.path.dirname(out), exist_ok=True)
        with open(out, 'w') as f:
            f.write(content)
    print(f"[OK] {OUTPUT_PATH}")
    print(f"[OK] {OUTPUT_PATH2}")
    print(f"     方块: {GRID_ROWS}×{GRID_COLS}={GRID_ROWS*GRID_COLS}")
    print(f"     正方体: {GRID_ROWS*GRID_COLS-len(CYLINDER_POSITIONS)}, 圆柱: {len(CYLINDER_POSITIONS)}")
    print(f"     ros2 launch yolo_vision sim_detect.launch.py")


if __name__ == "__main__":
    main()

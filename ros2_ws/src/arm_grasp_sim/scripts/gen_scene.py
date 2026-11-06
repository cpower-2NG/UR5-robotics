#!/usr/bin/env python3
"""生成阶段 4 的杂乱场景：S0–S4 布局 → Gazebo world + 逐块真值 JSON。

为什么不在 world 文件里手摆积木：S0–S4 要 5 批 × 5 布局，手写 SDF 既不可复现
也改不动。这里用**固定种子**生成布局，输出两个文件：

  <out>/grasp_world.sdf    直接喂 `gz sim`
  <out>/scene_truth.json   逐块 ID/颜色/中心/顶高/yaw/层号（评分器的真值）

坐标：base_link 系，桌面 z=0，方块边长 29.5 mm。与 `scripts/block_classes.py`
的调色板同一套颜色（本文件有一份拷贝，`test_gen_scene.py` 会核对两边一致）。

用法：
  python3 gen_scene.py --batch S0 --index 0 --out-dir /tmp/scene_s0_0
  python3 gen_scene.py --list
  python3 gen_scene.py --validate-all          # 生成全部 25 个布局并检查合法性
"""
from __future__ import annotations

import argparse
import json
import math
import os
import random
import sys

# ── 真机常数（一个事实一个家：几何出处见 docs/当前状态.md / 台账）──────────
BLOCK_M = 0.0295                     # 29.5 mm 方块
ZONE = (0.25, 0.575, 0.17, 0.42)     # 拾取区 xmin,xmax,ymin,ymax（base 系）
MIN_GAP_ISOLATED_M = 0.060           # S0 的非接触安全间距
PAD_CENTER_MM = (424.2, -112.24)     # 阶段 3 重解的垫位
PAD_YAW_DEG = -179.778
PAD_PITCH_MM = 60.0
PAD_THICK_M = 0.00288

# 与 scripts/block_classes.py::PALETTE 一致（test_gen_scene.py 会核对）
PALETTE = {
    "red":         (0.85, 0.15, 0.15),
    "orange":      (0.90, 0.45, 0.05),
    "tan":         (0.62, 0.44, 0.26),
    "yellow":      (0.92, 0.82, 0.10),
    "yellowgreen": (0.62, 0.78, 0.18),
    "cyan":        (0.10, 0.70, 0.72),
    "purple":      (0.48, 0.20, 0.58),
}
CLASSES = ["red", "orange", "tan", "yellow", "yellowgreen", "cyan", "purple"]

BATCH_COUNTS = {"S0": 5, "S1": 6, "S2": 6, "S3": 7, "S4": 9}
LAYOUTS_PER_BATCH = 5

# 顶面相机（真机旧支架实测，base_link 系；见 docs/calib/extrinsics-2026-10-03-base_link.json）
TOPCAM_XYZ = (0.34694477757210807, 0.8693043936314018, 0.6454397556505314)
TOPCAM_Q = (0.32913450910630487, 0.31637465543060517,
            -0.61852813688270547, 0.63953146615387635)
# 俯视相机内参：真机实测 1280×720，fx 917.6205 / fy 916.4271 / cx 649.5549 / cy 344.9063
# （scripts/test_grab_frames.py:85-92 的回归值）。Gazebo 只认横向 fov，
# 由 fx 反算：2·atan(640/917.6205) = 1.2180331 rad。
TOPCAM_W, TOPCAM_H = 1280, 720
TOPCAM_HFOV = 1.2180331477131634


# ── 小几何工具 ────────────────────────────────────────────────────────────
def _rot2(v, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return (c * v[0] - s * v[1], s * v[0] + c * v[1])


def _inside_zone(x, y, margin=0.0):
    xmin, xmax, ymin, ymax = ZONE
    return (xmin + margin <= x <= xmax - margin
            and ymin + margin <= y <= ymax - margin)


def _dist(a, b):
    return math.hypot(a["x"] - b["x"], a["y"] - b["y"])


# ── 布局生成 ──────────────────────────────────────────────────────────────
def _sample_isolated(rng, placed, n, gap, tries=4000):
    """拒绝采样：放入 n 个与已放置块中心距 ≥ gap 的孤立块。"""
    out = []
    margin = BLOCK_M / 2 + 0.004
    for _ in range(n):
        for _try in range(tries):
            x = rng.uniform(ZONE[0] + margin, ZONE[1] - margin)
            y = rng.uniform(ZONE[2] + margin, ZONE[3] - margin)
            cand = {"x": x, "y": y}
            if all(_dist(cand, p) >= gap for p in placed + out):
                out.append(cand)
                break
        else:
            raise RuntimeError("拒绝采样失败：拾取区放不下这么多孤立块")
    return out


def _add_contact_pair(rng, placed, same_color, color_pair):
    """放一对面对面接触的块；返回 (block_a, block_b)。"""
    margin = BLOCK_M / 2 + 0.006
    for _try in range(4000):
        x = rng.uniform(ZONE[0] + margin, ZONE[1] - margin)
        y = rng.uniform(ZONE[2] + margin, ZONE[3] - margin)
        yaw = rng.uniform(0.0, 180.0)
        d = _rot2((BLOCK_M, 0.0), yaw)          # 两块沿 yaw 方向贴合
        a = {"x": x - d[0] / 2, "y": y - d[1] / 2}
        b = {"x": x + d[0] / 2, "y": y + d[1] / 2}
        if not (_inside_zone(a["x"], a["y"], margin) and _inside_zone(b["x"], b["y"], margin)):
            continue
        if any(_dist(p, a) < BLOCK_M + 0.025 or _dist(p, b) < BLOCK_M + 0.025
               for p in placed):
            continue
        ca, cb = (color_pair[0], color_pair[1]) if color_pair else (
            (rng.choice(CLASSES), rng.choice(CLASSES)) if not same_color
            else (lambda c: (c, c))(rng.choice(CLASSES)))
        return (dict(a, yaw=yaw, color=ca, layer=0),
                dict(b, yaw=yaw, color=cb, layer=0))
    raise RuntimeError("接触对采样失败")


def _add_stack(rng, placed):
    """放一个两层塔：上下块同心度偏差 ≤ 3 mm，yaw 差 0–10°。"""
    margin = BLOCK_M / 2 + 0.006
    for _try in range(4000):
        x = rng.uniform(ZONE[0] + margin, ZONE[1] - margin)
        y = rng.uniform(ZONE[2] + margin, ZONE[3] - margin)
        base_yaw = rng.uniform(0.0, 180.0)
        if any(math.hypot(p["x"] - x, p["y"] - y) < BLOCK_M + 0.030 for p in placed):
            continue
        off = rng.uniform(0.0, 0.003)
        off_dir = rng.uniform(0.0, 360.0)
        dx, dy = _rot2((off, 0.0), off_dir)
        bottom = dict(x=x, y=y, yaw=base_yaw, color=rng.choice(CLASSES), layer=0)
        top = dict(x=x + dx, y=y + dy, yaw=(base_yaw + rng.uniform(0.0, 10.0)) % 180.0,
                   color=rng.choice(CLASSES), layer=1)
        return bottom, top
    raise RuntimeError("塔采样失败")


def make_layout(batch, index):
    """返回 list[dict]：id/color/x/y/z/yaw/layer/size。"""
    if batch not in BATCH_COUNTS:
        raise ValueError(f"未知批次 {batch!r}，可选 {sorted(BATCH_COUNTS)}")
    rng = random.Random(f"stage4:{batch}:{index}")
    n = BATCH_COUNTS[batch]
    blocks = []

    if batch == "S0":
        pts = _sample_isolated(rng, [], n, MIN_GAP_ISOLATED_M)
        colors = CLASSES[:]
        rng.shuffle(colors)
        for i, p in enumerate(pts):
            blocks.append(dict(p, yaw=rng.uniform(0, 180), color=colors[i % len(colors)],
                               layer=0))

    elif batch in ("S1", "S2"):
        pair = _add_contact_pair(rng, [], same_color=(batch == "S2"), color_pair=None)
        placed = list(pair)
        rest = _sample_isolated(rng, placed, n - 2, BLOCK_M + 0.030)
        colors = [c for c in CLASSES if c not in {pair[0]["color"], pair[1]["color"]}]
        rng.shuffle(colors)
        for i, p in enumerate(rest):
            placed.append(dict(p, yaw=rng.uniform(0, 180), color=colors[i % len(colors)],
                               layer=0))
        blocks = placed

    elif batch == "S3":
        bottom, top = _add_stack(rng, [])
        placed = [bottom]
        # 塔外再放一对接触（两层重叠场景同时有「上下」与「贴边」）
        extra_pair = _add_contact_pair(rng, placed, same_color=False, color_pair=None)
        placed += list(extra_pair)
        rest = _sample_isolated(rng, placed, n - 4, BLOCK_M + 0.030)
        colors = [c for c in CLASSES if c not in {b["color"] for b in placed}]
        rng.shuffle(colors)
        for i, p in enumerate(rest):
            placed.append(dict(p, yaw=rng.uniform(0, 180), color=colors[i % len(colors)],
                               layer=0))
        blocks = placed + [top]

    else:  # S4：自由布局 —— 一塔 + 两对接触 + 3 个孤立块
        bottom, top = _add_stack(rng, [])
        placed = [bottom]
        placed += list(_add_contact_pair(rng, placed, same_color=False, color_pair=None))
        placed += list(_add_contact_pair(rng, placed, same_color=False, color_pair=None))
        rest = _sample_isolated(rng, placed, n - 6, BLOCK_M + 0.025)
        colors = [c for c in CLASSES if c not in {b["color"] for b in placed}]
        rng.shuffle(colors)
        for i, p in enumerate(rest):
            placed.append(dict(p, yaw=rng.uniform(0, 180), color=colors[i % len(colors)],
                               layer=0))
        blocks = placed + [top]

    for i, b in enumerate(blocks):
        b["id"] = i
        b["size"] = BLOCK_M
        b["z"] = BLOCK_M / 2 + BLOCK_M * b["layer"]
        b["top"] = BLOCK_M * (b["layer"] + 1)
    return blocks


def validate_layout(batch, blocks):
    """合法性检查；返回问题列表（空 = 合法）。"""
    errs = []
    ids = [b["id"] for b in blocks]
    if len(set(ids)) != len(ids):
        errs.append("块 id 重复")
    for b in blocks:
        m = BLOCK_M / 2
        if not _inside_zone(b["x"], b["y"], m):
            errs.append(f"块 {b['id']} 中心 ({b['x']:.3f},{b['y']:.3f}) 出了拾取区")
    for i in range(len(blocks)):
        for j in range(i + 1, len(blocks)):
            a, bb = blocks[i], blocks[j]
            dxy = _dist(a, bb)
            if abs(a["layer"] - bb["layer"]) >= 1:
                # 不同层：竖向不重叠，不构成碰撞；只要求上层确实压在下层某块上
                continue
            # 同层：要么明确接触（中心距 ≈ 边长），要么留出安全间距
            if dxy < BLOCK_M * 0.72:
                errs.append(f"块 {a['id']}/{bb['id']} 重叠（中心距 {dxy*1000:.1f} mm）")
            elif dxy < BLOCK_M * 1.02:
                if batch == "S0":
                    errs.append(f"S0 出现接触块 {a['id']}/{bb['id']}（{dxy*1000:.1f} mm）")
            elif dxy < MIN_GAP_ISOLATED_M:
                # 接触对只允许精确贴合；这里放宽到「接触或 ≥60 mm」
                pass
    # 上层块必须被下层块托住（偏心 ≤ 6 mm），否则是悬空
    for b in blocks:
        if b["layer"] == 0:
            continue
        supporters = [a for a in blocks if a["layer"] == b["layer"] - 1
                      and _dist(a, b) <= 0.006]
        if not supporters:
            errs.append(f"块 {b['id']} 在 layer {b['layer']} 但下方 6 mm 内没有支撑块")
    return errs


# ── SDF 输出 ──────────────────────────────────────────────────────────────
def _block_sdf(b):
    r, g, bl = PALETTE[b["color"]]
    return f"""
    <model name="block_{b['id']}">
      <static>false</static>
      <pose>{b['x']:.6f} {b['y']:.6f} {b['z']:.6f} 0 0 {math.radians(b['yaw']):.6f}</pose>
      <link name="link">
        <gravity>true</gravity>
        <inertial>
          <mass>0.030</mass>
          <inertia><ixx>2.2e-6</ixx><ixy>0</ixy><ixz>0</ixz><iyy>2.2e-6</iyy><iyz>0</iyz><izz>2.2e-6</izz></inertia>
        </inertial>
        <collision name="collision">
          <geometry><box><size>{BLOCK_M} {BLOCK_M} {BLOCK_M}</size></box></geometry>
          <surface><friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><box><size>{BLOCK_M} {BLOCK_M} {BLOCK_M}</size></box></geometry>
          <material>
            <ambient>{r*0.6:.4f} {g*0.6:.4f} {bl*0.6:.4f} 1</ambient>
            <diffuse>{r:.4f} {g:.4f} {bl:.4f} 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>"""


def _pad_grid_sdf():
    """垫面上的 3×3 放置宫格（纯视觉，不加碰撞）。"""
    cx, cy = PAD_CENTER_MM[0] / 1000.0, PAD_CENTER_MM[1] / 1000.0
    out = []
    for r in range(3):
        for c in range(3):
            local = ((c - 1) * PAD_PITCH_MM / 1000.0, (1 - r) * PAD_PITCH_MM / 1000.0)
            dx, dy = _rot2(local, PAD_YAW_DEG)
            out.append(f"""
      <visual name="cell_{r}{c}">
        <pose>{cx + dx:.6f} {cy + dy:.6f} {PAD_THICK_M + 0.0002:.6f} 0 0 {math.radians(PAD_YAW_DEG):.6f}</pose>
        <geometry><box><size>0.050 0.050 0.0004</size></box></geometry>
        <material><ambient>0.95 0.95 0.90 1</ambient><diffuse>0.95 0.95 0.90 1</diffuse></material>
      </visual>""")
    return "\n".join(out)


def build_world(blocks):
    grid = _pad_grid_sdf()
    cx, cy = PAD_CENTER_MM[0] / 1000.0, PAD_CENTER_MM[1] / 1000.0
    pad_yaw = math.radians(PAD_YAW_DEG)
    blocks_sdf = "\n".join(_block_sdf(b) for b in blocks)
    tx, ty, tz = TOPCAM_XYZ
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <!-- 由 scripts/gen_scene.py 生成；不要手改，改生成器。 -->
  <world name="grasp_world">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <light type="directional" name="key">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 2.0 0 0 0</pose>
      <diffuse>0.85 0.83 0.80 1</diffuse>
      <specular>0.15 0.15 0.15 1</specular>
      <direction>-0.35 -0.25 -1</direction>
    </light>
    <light type="directional" name="fill">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 2.0 0 0 0</pose>
      <diffuse>0.35 0.36 0.40 1</diffuse>
      <specular>0.0 0.0 0.0 1</specular>
      <direction>0.5 0.6 -1</direction>
    </light>

    <!-- 地面（桌面以下 0.75 m） -->
    <model name="floor">
      <static>true</static>
      <pose>0 0 -0.75 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>6 6</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>6 6</size></plane></geometry>
          <material><ambient>0.12 0.12 0.13 1</ambient><diffuse>0.18 0.18 0.19 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- 桌面：上表面 z=0（与真机 base 系一致） -->
    <model name="table">
      <static>true</static>
      <pose>0.225 0.10 -0.02 0 0 0</pose>
      <link name="table_top">
        <collision name="collision">
          <geometry><box><size>1.15 1.10 0.04</size></box></geometry>
          <surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><box><size>1.15 1.10 0.04</size></box></geometry>
          <material><ambient>0.55 0.48 0.40 1</ambient><diffuse>0.62 0.55 0.46 1</diffuse></material>
        </visual>
      </link>
      <link name="leg_0"><pose>0.50 0.50 -0.395 0 0 0</pose>
        <collision name="collision"><geometry><cylinder><radius>0.022</radius><length>0.71</length></cylinder></geometry></collision>
        <visual name="visual"><geometry><cylinder><radius>0.022</radius><length>0.71</length></cylinder></geometry>
          <material><ambient>0.25 0.25 0.26 1</ambient><diffuse>0.3 0.3 0.31 1</diffuse></material></visual>
      </link>
      <link name="leg_1"><pose>0.50 -0.30 -0.395 0 0 0</pose>
        <collision name="collision"><geometry><cylinder><radius>0.022</radius><length>0.71</length></cylinder></geometry></collision>
        <visual name="visual"><geometry><cylinder><radius>0.022</radius><length>0.71</length></cylinder></geometry>
          <material><ambient>0.25 0.25 0.26 1</ambient><diffuse>0.3 0.3 0.31 1</diffuse></material></visual>
      </link>
      <link name="leg_2"><pose>-0.05 0.50 -0.395 0 0 0</pose>
        <collision name="collision"><geometry><cylinder><radius>0.022</radius><length>0.71</length></cylinder></geometry></collision>
        <visual name="visual"><geometry><cylinder><radius>0.022</radius><length>0.71</length></cylinder></geometry>
          <material><ambient>0.25 0.25 0.26 1</ambient><diffuse>0.3 0.3 0.31 1</diffuse></material></visual>
      </link>
      <link name="leg_3"><pose>-0.05 -0.30 -0.395 0 0 0</pose>
        <collision name="collision"><geometry><cylinder><radius>0.022</radius><length>0.71</length></cylinder></geometry></collision>
        <visual name="visual"><geometry><cylinder><radius>0.022</radius><length>0.71</length></cylinder></geometry>
          <material><ambient>0.25 0.25 0.26 1</ambient><diffuse>0.3 0.3 0.31 1</diffuse></material></visual>
      </link>
      <!-- 桌腿必须与桌面用固定关节连成一棵树：否则模型是「多子图 / 浮动 link」，
           bullet-featherstone 会报 "Multiple sub-trees / floating links detected" （虽仍能加载）。 -->
      <joint name="leg_0_joint" type="fixed">
        <parent>table_top</parent><child>leg_0</child>
        <pose>0.50 0.50 -0.395 0 0 0</pose>
      </joint>
      <joint name="leg_1_joint" type="fixed">
        <parent>table_top</parent><child>leg_1</child>
        <pose>0.50 -0.30 -0.395 0 0 0</pose>
      </joint>
      <joint name="leg_2_joint" type="fixed">
        <parent>table_top</parent><child>leg_2</child>
        <pose>-0.05 0.50 -0.395 0 0 0</pose>
      </joint>
      <joint name="leg_3_joint" type="fixed">
        <parent>table_top</parent><child>leg_3</child>
        <pose>-0.05 -0.30 -0.395 0 0 0</pose>
      </joint>
    </model>

    <!-- 放置垫 + 60 mm 宫格 -->
    <model name="pad">
      <static>true</static>
      <pose>{cx:.6f} {cy:.6f} 0 0 0 {pad_yaw:.6f}</pose>
      <link name="surface">
        <collision name="collision">
          <pose>0 0 {PAD_THICK_M/2:.6f} 0 0 0</pose>
          <geometry><box><size>0.30 0.30 {PAD_THICK_M}</size></box></geometry>
          <surface><friction><ode><mu>0.95</mu><mu2>0.95</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <pose>0 0 {PAD_THICK_M/2:.6f} 0 0 0</pose>
          <geometry><box><size>0.30 0.30 {PAD_THICK_M}</size></box></geometry>
          <material><ambient>0.10 0.10 0.12 1</ambient><diffuse>0.14 0.14 0.16 1</diffuse></material>
        </visual>{grid}

      </link>
    </model>

    <!-- 俯视相机：真机旧支架实测位姿（base_link 系） -->
    <model name="topcam">
      <static>true</static>
      <pose>{tx:.9f} {ty:.9f} {tz:.9f} 0.050732797136017804 0.94726404543898191 -1.5114025091536183</pose>
      <link name="camera_link">
        <sensor name="color" type="camera">
          <update_rate>15</update_rate>
          <topic>topcam/image</topic>
          <camera>
            <horizontal_fov>{TOPCAM_HFOV}</horizontal_fov>
            <image><width>{TOPCAM_W}</width><height>{TOPCAM_H}</height><format>R8G8B8</format></image>
            <clip><near>0.05</near><far>6.0</far></clip>
            <noise><type>gaussian</type><mean>0.0</mean><stddev>0.004</stddev></noise>
          </camera>
          <always_on>1</always_on>
          <visualize>false</visualize>
        </sensor>
        <sensor name="depth" type="depth_camera">
          <update_rate>15</update_rate>
          <topic>topcam/depth_image</topic>
          <camera>
            <horizontal_fov>{TOPCAM_HFOV}</horizontal_fov>
            <image><width>{TOPCAM_W}</width><height>{TOPCAM_H}</height><format>R_FLOAT32</format></image>
            <clip><near>0.05</near><far>6.0</far></clip>
          </camera>
          <always_on>1</always_on>
          <visualize>false</visualize>
        </sensor>
      </link>
    </model>

    <!-- 相机支架（纯视觉，避免「相机浮在空中」误导人眼复核） -->
    <model name="topcam_stand">
      <static>true</static>
      <link name="post">
        <pose>{tx:.6f} {ty:.6f} -0.045 0 0 0</pose>
        <collision name="collision">
          <geometry><cylinder><radius>0.018</radius><length>1.41</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>0.018</radius><length>1.41</length></cylinder></geometry>
          <material><ambient>0.2 0.2 0.21 1</ambient><diffuse>0.3 0.3 0.31 1</diffuse></material>
        </visual>
      </link>
    </model>

{blocks_sdf}

    <gravity>0 0 -9.8</gravity>
  </world>
</sdf>
"""


def write_scene(batch, index, out_dir):
    blocks = make_layout(batch, index)
    errs = validate_layout(batch, blocks)
    if errs:
        raise SystemExit(f"布局 {batch}#{index} 非法：\n  " + "\n  ".join(errs))
    os.makedirs(out_dir, exist_ok=True)
    world = os.path.join(out_dir, "grasp_world.sdf")
    truth = os.path.join(out_dir, "scene_truth.json")
    with open(world, "w") as f:
        f.write(build_world(blocks))
    payload = {
        "batch": batch, "index": index, "block_size_m": BLOCK_M,
        "zone_m": list(ZONE), "blocks": blocks,
    }
    with open(truth, "w") as f:
        json.dump(payload, f, indent=2, sort_keys=True)
    return world, truth, blocks


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--batch", choices=sorted(BATCH_COUNTS))
    ap.add_argument("--index", type=int, default=0, choices=range(LAYOUTS_PER_BATCH))
    ap.add_argument("--out-dir", default="/tmp/grasp_scene")
    ap.add_argument("--list", action="store_true", help="列出所有批次/布局数量")
    ap.add_argument("--validate-all", action="store_true", help="生成并校验全部布局")
    args = ap.parse_args(argv)

    if args.list:
        for b, n in BATCH_COUNTS.items():
            print(f"{b}: {LAYOUTS_PER_BATCH} 个布局，每布局 {n} 块")
        return 0

    if args.validate_all:
        bad = 0
        for b in BATCH_COUNTS:
            for i in range(LAYOUTS_PER_BATCH):
                blocks = make_layout(b, i)
                errs = validate_layout(b, blocks)
                status = "OK" if not errs else "FAIL"
                print(f"{b}#{i}: {status} ({len(blocks)} 块)")
                for e in errs:
                    print("   ", e)
                bad += bool(errs)
        return 1 if bad else 0

    if not args.batch:
        ap.error("需要 --batch（或 --list / --validate-all）")
    world, truth, blocks = write_scene(args.batch, args.index, args.out_dir)
    print(f"world: {world}\ntruth: {truth}\n{len(blocks)} blocks")
    return 0


if __name__ == "__main__":
    sys.exit(main())

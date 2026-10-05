#!/usr/bin/env python3
"""
方块自动标注（YOLO bbox）
=========================
把「已知 3D 位姿的方块」投影成图像里的 2D 框，写出 YOLO 标注 txt。

**这个脚本的数学必须是 `perception_3d/projector_node.py` 的严格逆运算。**

为什么这句话是本文件的核心，而不是一句口号
------------------------------------------------
训练标签和在线推理用的必须是**同一套投影数学**。推理链是

    (u,v) + depth --[camera_info fx,fy,cx,cy]--> 相机光学系 (X_c,Y_c,Z_c)
                  --[tf2 target_frame←camera_frame]--> base_link

训练链就是把它逐字倒过来。两边一旦不一致，模型学到的是**另一套几何**，
而症状是「检测框看着挺好、3D 坐标系统性偏一点」—— 最难查的一类偏差，
因为没有任何一步会报错。

本仓库的**上一版**就是不一致的：那个 ``world_to_pixel()`` 里**没有任何
旋转**，写死成「相机垂直向下、图像 u ∥ 世界 x、图像 v ∥ 世界 y」：

    Xc = wx-cam_x ; Yc = wy-cam_y ; Zc = cam_z-wz
    u = fx*Xc/Zc + cx ; v = fy*Yc/Zc + cy

而仿真相机的真实光学约定（`arm_grasp_sim/launch/sim.launch.py` 已用实跑
数据反证）是「光心 x+ → world −y、光心 y+ → world −x、光心 z+ → world −z」。
代进去可得：真 u ∝ (wy−cam_y)/(cam_z−wz)，**正好是旧代码的 v**；
        真 v ∝ (wx−cam_x)/(cam_z−wz)，**正好是旧代码的 u**。
—— u/v 对调再叠一次关于 (cx+cy)/2 的点反演。对单个小方块的近正方形包围盒
「对调」几乎看不出来，但方块的**位置**会整体错位几十到两百像素。
仿真里之所以一直没炸，是因为方阵对称 + 模型是从图像内容学的，但这属于
**碰巧**，不是正确。真机的手眼外参必然带任意旋转，旧写法连表达都表达不了
（meta 的 camera 字典只有 position，**根本没有姿态字段**）。

两条后端
--------
``--backend tf``（推荐，**精确**）
    base_link 下的方块 → tf2 查到相机光学系 → 针孔投影。
    旋转交给 tf2，内参用真实的 camera_info，与在线推理严格互逆。
    真机的手眼标定结果（W2 的静态 TF）就是这里的输入，**一处外参两处用**。

``--backend homography``（**不依赖手眼外参**）
    台面上放一块 ArUco/ChArUco 板 → 得到「台面平面 → 像素」的单应矩阵 H，
    把方块的台面足迹投到像素。优点是**五分钟标完、不用动机械臂**，而且有
    一个可检验的重投影误差。

    为什么这个后端值得存在：几何标注依赖可信的手眼外参，而手眼标定若走
    easy_handeye 需要**真实运动**（高风险）。单应把「先采数据集」与「先标定」
    这个顺序耦合整个绕开。

    **两种精度档位**（由 homography.json 里有没有 ``H_top`` 决定）：

    - ``H`` + ``H_top``（**精确**）：H 是台面 (z=0) 的单应，H_top 是**把标定板
      垫高一个方块边长**后标出的单应（z=h）。凸体的轮廓 = 其全部顶点投影的
      凸包，而方块顶点只落在底面和顶面这两个平面上 —— 所以
      「底面足迹 ∪ 顶面足迹」的包围盒就是**真正的轮廓**，不是近似。
      代价是标定时要垫一次垫块（一块 40 mm 的方块本身就行）。
    - 只有 ``H``（**近似**）：把方块当零厚度平面图形，漏掉顶面视差。
      误差≈ (方块高度 / 相机到台面高度) × 离画面中心的径向距离 r。取
      h=40 mm、相机 0.9 m 高即 ≈ 4.4%·r：画面中心几乎为 0，r=250 px 处约
      **11 px**，画面角落（r≈400 px）约 **18 px**。而方块本身才 32 px ——
      边缘处是 ~1/3 的尺寸误差，**训练标签偏粗**，脚本会明确告警。

    两条后端仍可互为独立的第二意见：同帧跑两遍、对得上就放心。

用法
----
    # A) 真机：手眼外参 + 真实内参（推荐）
    python3 scripts/auto_label_grid.py --backend tf \
        --images block_dataset/train/images --out block_dataset/train/labels \
        --blocks real_blocks.json --camera-info camera_info.json \
        --extrinsics hand_eye.json --draw

    # B) 真机：只用一块标定板，不动机械臂
    python3 scripts/auto_label_grid.py --backend homography \
        --images block_dataset/train/images --out block_dataset/train/labels \
        --blocks real_blocks.json --homography aruco_h.json --draw

    # C) 仿真：直接读 generate_grid_world.py 产出的 meta
    python3 scripts/auto_label_grid.py --sim-meta \
        ros2_ws/src/yolo_bringup/worlds/block_grid_world_meta.json

输入文件格式
------------
blocks.json        : [{"x":0.44,"y":0.24,"z":0.02,"size":0.04}, ...]
                     坐标所在的系由 ``--extrinsics`` 里的 ``target_frame`` 命名
                     （**没有** ``--frame`` 开关 —— tf 后端=base_link；
                     homography 后端=标定板所在的台面坐标系，z 忽略）。
camera_info.json   : {"fx":554.4,"fy":554.4,"cx":320,"cy":240,
                      "width":640,"height":480}
extrinsics.json    : {"target_frame":"base_link",
                      "camera_frame":"camera_color_optical_frame",
                      "translation":[x,y,z],
                      "rotation":[qx,qy,qz,qw]}
                     —— 语义 = tf2 的 lookup_transform(target, camera)，
                     即「把 camera_frame 里的点变到 target_frame」。
homography.json    : {"H":[[h11,h12,h13],[h21,h22,h23],[h31,h32,h33]],
                      "H_top":[...],          # 可选，见下
                      "width":640,"height":480}
                     H 把台面坐标 (X,Y) 映到像素 (u,v)，齐次、归一化到 H[2][2]=1。
                     H_top = 把标定板**垫高 `--block-size` 那么多**之后重标一次
                     得到的单应（垫块就用一块方块本身）。给了它，包围盒取
                     「底面 ∪ 顶面」= 真实轮廓；不给则走零厚度近似并在
                     describe 里告警。注意平面坐标 (x,y) 指的是方块**中心**，
                     z 不参与 —— 高度是靠 H / H_top 两张单应分别表达的。

像素约定（沿用 projector_node 的 D7 告警）
-----------------------------------------
``/detections`` 走的是**像素**坐标，不是归一化。本脚本内部一律用像素，
只在**写文件的最后一步**归一化。别把这两者混起来 —— 历史上真有过一份
发归一化坐标的实现，投影会静默算错。
"""

import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from block_classes import describe, parse_classes                      # noqa: E402

PROJECT_ROOT = Path(__file__).resolve().parent.parent

# 仿真相机：来自 worlds/block_grid_world.sdf（generate_grid_world.py 生成）
#   <pose>0.5 0.0 1.5 0 1.57 0</pose>
# 这里存的是**光学系**朝向，不是 Gazebo 视觉模型朝向 —— 两者差一个固定的
# 轴重排，合成后才是下面这个合法旋转（det=+1）。推导与实测反证见
# arm_grasp_sim/launch/sim.launch.py 的 CAMERA_Q* 注释。
SIM_CAMERA_Q = (0.7071067811865476, -0.7071067811865476, 0.0, 0.0)
SIM_TARGET_FRAME = "world"
SIM_CAMERA_FRAME = "overhead_camera/camera_link/color_camera"
DEFAULT_SIM_META = (PROJECT_ROOT / "ros2_ws" / "src" / "yolo_bringup"
                    / "worlds" / "block_grid_world_meta.json")
DEFAULT_IMAGE_DIR = PROJECT_ROOT / "block_dataset" / "train" / "images"
DEFAULT_OUT_DIR = PROJECT_ROOT / "block_dataset" / "train" / "labels"


# ======================================================================
# 纯数学：这一节就是「projector_node 的逆」。改这里等于改训练几何。
# ======================================================================
def quat_to_matrix(q):
    """四元数 (x,y,z,w) → 3×3 旋转矩阵。"""
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        raise ValueError("四元数全零")
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def project_pinhole(points_cam, fx, fy, cx, cy):
    """相机光学系点 → 像素。

    与 ``projector_node`` 的 Step 3 逐字互逆::

        # 正向（推理）
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
        # 反向（本函数）
        u = fx * X_c / Z_c + cx
        v = fy * Y_c / Z_c + cy

    返回 ``(像素点列表, 被丢弃点数)``。**Z_c <= 0 的点必须丢弃** —— 它们在
    相机背后，除出来的 u/v 会翻到另一侧，是个静默的错框来源。
    """
    out, dropped = [], 0
    for p in points_cam:
        if p[2] <= 0.0:
            dropped += 1
            continue
        out.append((fx * p[0] / p[2] + cx, fy * p[1] / p[2] + cy))
    return out, dropped


def bbox_from_pixels(pixels, width, height):
    """像素点集 → 归一化 YOLO bbox ``(xc, yc, w, h)``，或 None。

    裁剪到图像边界内；裁剪后退化（宽或高 ≤ 0）就返回 None —— 那说明方块
    整个在画面外，不该写一条无意义的标签。
    """
    if not pixels:
        return None
    us = [p[0] for p in pixels]
    vs = [p[1] for p in pixels]
    u0 = max(0.0, min(us))
    u1 = min(float(width - 1), max(us))
    v0 = max(0.0, min(vs))
    v1 = min(float(height - 1), max(vs))
    if u1 <= u0 or v1 <= v0:
        return None
    return (((u0 + u1) / 2.0) / width, ((v0 + v1) / 2.0) / height,
            (u1 - u0) / width, (v1 - v0) / height)


def box_corners(cx, cy, cz, size):
    """3D 包围盒的 8 个角点。"""
    h = size / 2.0
    return [(cx + dx, cy + dy, cz + dz)
            for dx in (-h, h) for dy in (-h, h) for dz in (-h, h)]


def footprint_corners(cx, cy, size):
    """台面足迹的 4 个角点（homography 后端用；z 视为 0）。"""
    h = size / 2.0
    return [(cx - h, cy - h), (cx + h, cy - h), (cx + h, cy + h), (cx - h, cy + h)]


# ======================================================================
# 两条后端
# ======================================================================
class TfProjector:
    """base_link 下的 3D 点 → 像素。tf2 那条链的逆运算。"""

    def __init__(self, info, extr):
        self.fx = float(info["fx"])
        self.fy = float(info["fy"])
        self.cx = float(info["cx"])
        self.cy = float(info["cy"])
        self.width = int(info["width"])
        self.height = int(info["height"])
        self.target_frame = extr.get("target_frame", "base_link")
        self.camera_frame = extr.get("camera_frame", "camera_color_optical_frame")
        # 语义 = tf2 lookup_transform(target_frame, camera_frame)
        # → 把 camera_frame 里的点变到 target_frame。
        # 我们要的是**逆方向**（target 里的点 → camera），所以下面用 R^T 与 -R^T t。
        self.R = quat_to_matrix(extr["rotation"])
        self.t = np.array([float(v) for v in extr["translation"]], dtype=float)
        # 相机光心在 target 系的位置**就是 t 本身**：p_target = R·p_cam + t，
        # 取 p_cam = 0 即得。⚠️ 别写成 R^T·(−t) —— 那是「原点在相机系里的坐标」，
        # 数值上恰好也是个像样的三位数，却把 x/y 换了位。这一行是现场核对
        # 手眼外参时第一眼要看的数，错在这里最难发现。
        self.camera_in_target = self.t

    def to_camera(self, p_target):
        """target 系点 → 相机光学系。等价于 tf2 反查，无需再查一次 TF。"""
        return self.R.T @ (np.asarray(p_target, dtype=float) - self.t)

    def project_box(self, center, size):
        pts = [self.to_camera(c) for c in box_corners(*center, size)]
        pixels, dropped = project_pinhole(pts, self.fx, self.fy, self.cx, self.cy)
        if dropped:
            # 只要有一个角点在相机背后，这个框就不可信：裁剪出来的包围盒
            # 会横跨画面，看着像个巨大的目标。宁可不标。
            return None, dropped
        return bbox_from_pixels(pixels, self.width, self.height), 0

    def describe(self):
        return (f"[tf] {self.target_frame} → {self.camera_frame}  "
                f"fx={self.fx:.1f} fy={self.fy:.1f} cx={self.cx:.0f} cy={self.cy:.0f} "
                f"{self.width}×{self.height}\n"
                f"     相机光心在 {self.target_frame} 系 = "
                f"{np.round(self.camera_in_target, 4).tolist()}")


class HomographyProjector:
    """台面平面坐标 → 像素。不依赖手眼外参。

    ``H``     : 台面 (z=0) 的单应
    ``H_top`` : 可选。**垫高一个方块边长**后标出的单应 (z=h)。给了它，
                底面 ∪ 顶面 = 方块的**真实轮廓**（见模块 docstring）；
                不给则退化成零厚度近似，并在 :meth:`describe` 里告警。
    """

    def __init__(self, hdoc):
        self.H = self._norm(hdoc["H"], "H")
        self.H_top = self._norm(hdoc["H_top"], "H_top") if "H_top" in hdoc else None
        self.width = int(hdoc["width"])
        self.height = int(hdoc["height"])

    @staticmethod
    def _norm(raw, name):
        H = np.array(raw, dtype=float)
        if H.shape != (3, 3):
            raise ValueError(f"{name} 必须是 3×3，现在是 {H.shape}")
        if abs(H[2, 2]) < 1e-12:
            raise ValueError(f"{name}[2][2] ≈ 0，没法归一化；标定结果是不是填错了？")
        return H / H[2, 2]

    def project_point(self, x, y, H=None):
        v = (H if H is not None else self.H) @ np.array([x, y, 1.0])
        if abs(v[2]) < 1e-12:
            return None
        return (v[0] / v[2], v[1] / v[2])

    def project_box(self, center, size):
        pixels = []
        for x, y in footprint_corners(center[0], center[1], size):
            p = self.project_point(x, y)
            if p is None:
                return None, 1
            pixels.append(p)
        if self.H_top is not None:
            # 顶面在同样的 (x,y) 足迹上，但走 z=size 那张单应。
            for x, y in footprint_corners(center[0], center[1], size):
                p = self.project_point(x, y, H=self.H_top)
                if p is None:
                    return None, 1
                pixels.append(p)
        return bbox_from_pixels(pixels, self.width, self.height), 0

    def describe(self):
        lines = [f"[homography] 台面平面 → 像素  {self.width}×{self.height}",
                 f"     H     = {np.round(self.H, 6).tolist()}"]
        if self.H_top is None:
            lines += [
                "     H_top = （未提供）",
                "⚠️ 零厚度近似档位：漏掉顶面视差，画面边缘误差可达 ~12 px"
                "（方块本身才 32 px）。要精确就补 H_top，或改用 --backend tf。",
            ]
        else:
            lines.append(f"     H_top = {np.round(self.H_top, 6).tolist()}"
                         "   → 底面∪顶面，**精确轮廓**")
        return "\n".join(lines)


# ======================================================================
# 组装输入
# ======================================================================
def sim_inputs(meta_path):
    """仿真：从 generate_grid_world.py 的 meta 造出 tf 后端所需的三份输入。

    为什么让仿真也走 tf 后端：这样**仿真与真机用同一段数学**。仿真的相机
    位姿是已知常量（不是标定出来的），把它写成外参即可，于是仿真变成这条
    数学的一次实跑验证 —— 而不是一条只有仿真才走的旁路。

    **多类别**：类别从 meta 里逐块带出来（`class_id`），不再由命令行统一指定。
    旧版把 `shape` 和 `color` 都丢掉了，于是只能写单一类别号 —— 那是单类
    数据集时代的写法。这里保留 `class_name` 只为报错时能说人话。
    """
    meta = json.load(open(meta_path))
    cam = meta["camera"]
    fx = (cam["width"] / 2.0) / math.tan(cam["horizontal_fov"] / 2.0)
    info = {"fx": fx, "fy": fx,          # 仿真正方形像素
            "cx": cam["width"] / 2.0, "cy": cam["height"] / 2.0,
            "width": cam["width"], "height": cam["height"]}
    extr = {"target_frame": SIM_TARGET_FRAME,
            "camera_frame": SIM_CAMERA_FRAME,
            "translation": list(cam["position"]),
            "rotation": list(SIM_CAMERA_Q)}
    # ⚠️ meta 里没有 `class_id` 时**不退回 0**，直接报错。
    # 退回 0 的后果是整批标签全变成第 0 类，而训练照样跑完、loss 照样降 ——
    # 最后一个「多类别」模型其实只会认一种颜色，且没有任何一步会报错。
    classes = meta.get("classes")
    blocks = []
    for b in meta["blocks"]:
        if "class_id" not in b:
            raise SystemExit(
                "meta 里方块 %d 没有 class_id —— 这份 meta 是旧版 "
                "generate_grid_world.py 生成的（那个版本只写 color/shape，"
                "不记类别）。请用新版重新生成世界：\n"
                "    python3 scripts/generate_grid_world.py --classes ... "
                "--out-dir <目录>\n"
                "（不要手工补一个类别号：类别号必须与 build_dataset.py "
                "--class-names 的顺序一致，猜错不会报错。）" % b.get("id", -1))
        blocks.append({"x": b["x"], "y": b["y"], "z": b["z"], "size": 0.04,
                       "class_id": b["class_id"],
                       "class_name": b.get("class_name", "?")})
    if classes:
        print(f"[INFO] meta 的类别表：{describe(classes)}")
    return info, extr, blocks


def load_json(path, what):
    p = Path(path)
    if not p.exists():
        sys.exit(f"找不到{what}：{p}")
    try:
        return json.load(open(p))
    except json.JSONDecodeError as e:
        sys.exit(f"{what} 不是合法 JSON（{p}）：{e}")


def load_blocks(path, default_size):
    """读手工写的 blocks 文件（真机路径，没有 meta 可依）。

    类别来自每个方块自己的 `class_id`，缺失时**留给调用方补 `--class-id`** ——
    这里不填默认值，因为「这一块到底属于哪一类」只有写这个文件的人知道。
    """
    doc = load_json(path, "blocks 文件")
    if not isinstance(doc, list):
        sys.exit("blocks 文件必须是一个**列表**：[{\"x\":..,\"y\":..,\"z\":..}, ...]")
    out = []
    for i, b in enumerate(doc):
        try:
            item = {"x": float(b["x"]), "y": float(b["y"]),
                    "z": float(b.get("z", 0.0)),
                    "size": float(b.get("size", default_size))}
        except (KeyError, TypeError, ValueError) as e:
            sys.exit(f"blocks[{i}] 读不出 x/y/z：{b!r}（{e}）")
        if "class_id" in b:
            try:
                item["class_id"] = int(b["class_id"])
            except (TypeError, ValueError) as e:
                sys.exit(f"blocks[{i}] 的 class_id 不是整数：{b['class_id']!r}（{e}）")
            item["class_name"] = str(b.get("class_name", item["class_id"]))
        out.append(item)
    if not out:
        sys.exit("blocks 文件是空的，没什么可标。")
    return out


# ======================================================================
# 主流程
# ======================================================================
def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description="方块自动标注：projector_node 投影链的严格逆运算",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--backend", choices=("tf", "homography"), default="tf",
                    help="tf = 手眼外参+内参（精确）；homography = 台面单应（无需外参）")
    ap.add_argument("--images", type=Path, default=DEFAULT_IMAGE_DIR,
                    help=f"图像目录（默认 {DEFAULT_IMAGE_DIR}）")
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT_DIR,
                    help=f"标签输出目录（默认 {DEFAULT_OUT_DIR}）")
    ap.add_argument("--pattern", default="*.jpg",
                    help="图像文件名通配（默认 *.jpg）")
    ap.add_argument("--blocks", help="方块位姿 JSON（见模块 docstring 的格式）")
    ap.add_argument("--camera-info", help="内参 JSON（tf 后端必需）")
    ap.add_argument("--extrinsics", help="手眼外参 JSON（tf 后端必需）")
    ap.add_argument("--homography", help="台面单应 JSON（homography 后端必需）")
    ap.add_argument("--sim-meta", nargs="?", const=str(DEFAULT_SIM_META),
                    help="仿真：读 generate_grid_world.py 的 meta，"
                         "自动填好内参/外参/方块（走 tf 后端）")
    ap.add_argument("--block-size", type=float, default=0.04,
                    help="方块边长（m，默认 0.04）")
    ap.add_argument("--class-id", type=int, default=None,
                    help="**兜底**类别号：只用于 blocks 文件里没写 class_id 的方块"
                         "（仿真的 --sim-meta 路径不走这里，类别逐块来自 meta）。"
                         "不传且又有方块缺类别号 → 直接报错，不猜 0。")
    ap.add_argument("--draw", action="store_true",
                    help="另外在 --draw-out 写出画了框的图，供人工复核"
                         "（计划要求自动标注当**提案**、人工过一遍）")
    ap.add_argument("--draw-out", type=Path, default=None,
                    help="复核图输出目录（默认 <out>/../label_check）")
    ap.add_argument("--dry-run", action="store_true", help="只打印，不写文件")
    return ap.parse_args(argv)


def build_projector(args):
    if args.sim_meta:
        info, extr, blocks = sim_inputs(args.sim_meta)
        print(f"[INFO] 仿真 meta：{args.sim_meta}")
        print(f"       方块 {len(blocks)} 个，相机世界坐标 {extr['translation']}")
        return TfProjector(info, extr), blocks
    if not args.blocks:
        sys.exit(f"--backend {args.backend} 需要 --blocks（或用 --sim-meta）")
    blocks = load_blocks(args.blocks, args.block_size)
    if args.backend == "tf":
        if not args.camera_info or not args.extrinsics:
            sys.exit("--backend tf 需要 --camera-info 与 --extrinsics")
        return TfProjector(load_json(args.camera_info, "内参文件"),
                           load_json(args.extrinsics, "外参文件")), blocks
    if not args.homography:
        sys.exit("--backend homography 需要 --homography")
    return HomographyProjector(load_json(args.homography, "单应文件")), blocks


def resolve_class_ids(blocks, fallback):
    """给每个方块定下写进标签的类别号；缺类别号又没兜底就**报错退出**。

    ⚠️ 这里曾经是「统一写 `--class-id`，默认 0」。那个默认值的后果不是
    「单类数据集」而是**静默的错误标签**：多类别的世界被整批标成第 0 类，
    训练照样收敛，只是类别信息全丢了。
    """
    missing = [b for b in blocks if "class_id" not in b]
    if missing:
        if fallback is None:
            sys.exit(
                "有 %d 个方块没有类别号，而 --class-id 也没传 —— 不猜。\n"
                "  仿真的话用 --sim-meta（类别逐块来自 meta）；\n"
                "  手工 blocks 文件的话，给每个方块加 \"class_id\"，"
                "或传 --class-id 把**全部**方块标成同一类。"
                % len(missing))
        for b in missing:
            b["class_id"] = fallback
            b["class_name"] = str(fallback)
    return blocks


def main(argv=None):
    args = parse_args(argv)
    proj, blocks = build_projector(args)
    blocks = resolve_class_ids(blocks, args.class_id)
    print(proj.describe())

    if not args.images.is_dir():
        sys.exit(f"图像目录不存在：{args.images}")
    images = sorted(args.images.glob(args.pattern))
    if not images:
        sys.exit(f"{args.images} 里没有匹配 {args.pattern} 的图像")

    if not args.dry_run:
        args.out.mkdir(parents=True, exist_ok=True)
    draw_dir = None
    if args.draw and not args.dry_run:
        draw_dir = args.draw_out or (args.out.parent / "label_check")
        draw_dir.mkdir(parents=True, exist_ok=True)

    total_boxes, total_skipped, bad = 0, 0, []
    per_class = {}                       # 类别号 -> 写出的框数
    for img in images:
        lines, skipped = [], 0
        for b in blocks:
            bbox, drop = proj.project_box((b["x"], b["y"], b["z"]), b["size"])
            if bbox is None:
                skipped += 1
                continue
            cid = b["class_id"]
            lines.append(f"{cid} " + " ".join(f"{v:.6f}" for v in bbox))
            per_class[cid] = per_class.get(cid, 0) + 1
        total_boxes += len(lines)
        total_skipped += skipped
        if skipped:
            bad.append((img.name, skipped))

        if not args.dry_run:
            (args.out / (img.stem + ".txt")).write_text(
                "\n".join(lines) + ("\n" if lines else ""))
        if draw_dir is not None:
            draw_overlay(img, proj, blocks, draw_dir / (img.stem + "_check.png"))

    print(f"\n图像 {len(images)} 张 → 框 {total_boxes} 个"
          f"（平均 {total_boxes/max(1,len(images)):.1f} 个/张，"
          f"方块共 {len(blocks)} 个）")

    # 按类别报框数。**这是「类别表与标签对得上」的第一道闸**：
    # 类别号不连续（缺号）说明类别表与写出的标签不是同一套；
    # 某个类别一个框都没有，则混淆矩阵里那一行会全零 —— 看着像「召回 0」，
    # 实际是「这批数据里根本没有这一类」，两者要分清楚。
    if per_class:
        print("各类别框数：" + "  ".join(
            f"{c}={per_class[c]}" for c in sorted(per_class)))
        gaps = [c for c in range(max(per_class) + 1) if c not in per_class]
        if gaps:
            print(f"⚠️ 类别号 {gaps} 一个框都没有（类别号应当连续 0..{max(per_class)}）"
                  f" —— 要么这批方块里确实缺这几类，要么类别表与生成世界时不一致。")
    if total_skipped:
        # 不静默：画面外/相机背后的方块数必须报出来，否则「少标了几个」
        # 会被当成正常，而漏标会直接变成训练里的假阴性。
        print(f"⚠️ 跳过 {total_skipped} 个（在画面外或相机背后）"
              f"，涉及 {len(bad)} 张图：")
        for name, n in bad[:10]:
            print(f"     {name}: 跳过 {n} 个")
        if len(bad) > 10:
            print(f"     …另有 {len(bad)-10} 张")
    if args.dry_run:
        print("（--dry-run，未写任何文件）")
    else:
        print(f"标签已写出到 {args.out}")
        if draw_dir is not None:
            print(f"复核图已写出到 {draw_dir} —— **请人工过一遍**，"
                  "自动标注按计划只当提案（真机方块不会精确落在命令位置上）。")
    return 0


def draw_overlay(img_path, proj, blocks, out_path):
    """把预测框画到图上，供人工复核。PIL 是可选依赖。"""
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        return
    W = getattr(proj, "width", None)
    H = getattr(proj, "height", None)
    if W is None or H is None:
        return
    im = Image.open(img_path).convert("RGB")
    d = ImageDraw.Draw(im)
    for b in blocks:
        bbox, _ = proj.project_box((b["x"], b["y"], b["z"]), b["size"])
        if bbox is None:
            continue
        xc, yc, bw, bh = bbox
        x0, y0 = (xc - bw / 2) * W, (yc - bh / 2) * H
        d.rectangle([x0, y0, (xc + bw / 2) * W, (yc + bh / 2) * H],
                    outline=(40, 220, 40), width=2)
        # 把类别号写在框上。复核图存在的意义就是**人工看一遍**，
        # 而多类别下「框对不对」之外还多了一问「类别号对不对」——
        # 不写出来这张图就只能回答前一半。
        d.text((x0, max(0, y0 - 11)), f"{b['class_id']}",
               fill=(255, 255, 0))
    im.save(out_path)


if __name__ == "__main__":
    raise SystemExit(main())

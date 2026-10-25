#!/usr/bin/env python3
"""pc_top（节点）与 `--est pc`（离线评估）在**真实帧**上必须逐点相同。

为什么需要这条
==============
点云抓取点的数学现在**只有一份**（``perception_3d/pc_grasp.py``），但它的
两条调用路径包着不同的外壳：

  · 节点：``projector_node._pc_batch_state()`` 建云（TF 从 /tf 来）、
    ``_pc_top_pose()`` 拟合局部桌面并装成 PoseStamped；
  · 离线：``eval_baseline.eval_frame()`` 建云（TF 从记录的 tf.json 来）、
    ``estimate_grasp_pc()`` 补板系坐标。

外壳里任何一个环节错位（TF 方向、深度单位、像素 offset、ROI 里 x/y 反了），
症状都是**几毫米的静默偏差** —— 离线指标依旧漂亮，上板才偏。所以这里把
离线验证过的那 10 帧真实画面当成**金标准**，逐条检测比对两条路径的结果。

判据
====
  · 每个检测：两边同为 None，或同为点且三维差 < 1e-6 m（同一份数学，
    只允许浮点末位差别）；
  · 整批点云：节点建的云与离线的云逐元素相同。

用法
====
    source /opt/ros/humble/setup.bash
    python3 scripts/test_pc_top_consistency.py [帧目录 ...]

不给目录时用 ``~/benchmark/2026-10-baseline/frames/wrist/rep-*``；找不到
就 [SKIP]（基准帧在仓库外、不进公开仓，所以换台机器跑必然缺）。
退出码 0 = 全通过（或跳过），1 = 有失败项。
"""

import glob
import os
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent
PERCEPTION_SRC = PROJECT_ROOT / "ros2_ws" / "src" / "perception_3d" / "perception_3d"
sys.path.insert(0, str(PROJECT_ROOT / "scripts"))
sys.path.insert(0, str(PERCEPTION_SRC))

try:
    import rclpy
except ImportError as e:
    print(f"[SKIP] 未找到 ROS2 Python 环境（{e}）。")
    print("       请先 source /opt/ros/humble/setup.bash 再运行本脚本。")
    sys.exit(0)

import eval_baseline as eb                                   # noqa: E402
import pc_grasp                                               # noqa: E402
from projector_node import Perception3D                       # noqa: E402
from builtin_interfaces.msg import Time                       # noqa: E402

DEFAULT_GLOB = str(Path.home() / "benchmark" / "2026-10-baseline"
                   / "frames" / "wrist" / "rep-*")
MAX_DIFF_M = 1e-6

_failed = []


def check(name, cond, detail=''):
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}"
          + (f"  {detail}" if detail else ''))
    if not cond:
        _failed.append(name)


def patch_intrinsics(node, K):
    """把帧里记录的内参原样注入节点 —— 两边必须用**同一组** K，否则比的
    不是外壳而是内参来源。"""
    node.camera_model.fx = lambda: float(K[0, 0])
    node.camera_model.fy = lambda: float(K[1, 1])
    node.camera_model.cx = lambda: float(K[0, 2])
    node.camera_model.cy = lambda: float(K[1, 2])
    node.camera_info_received = True


def fake_transform(entry):
    """tf.json 的一条 → 有 .transform.rotation/.translation 的对象。

    节点从 /tf 拿的是 TransformStamped，这里只造它用到的字段。
    """
    rot, tr = entry["rotation"], entry["translation"]
    return SimpleNamespace(transform=SimpleNamespace(
        rotation=SimpleNamespace(x=rot["x"], y=rot["y"], z=rot["z"],
                                 w=rot["w"]),
        translation=SimpleNamespace(x=tr["x"], y=tr["y"], z=tr["z"])))


def compare_frame(node, frame_dir):
    f = eb.load_frame(frame_dir)
    K = f["K"]
    patch_intrinsics(node, K)

    depth_m = f["depth"].astype(np.float32) / 1000.0      # 节点入口的口径
    entry = f["tf"]["ok"]["base_link:camera_color_optical_frame"]
    transform = fake_transform(entry)

    # 节点侧：彩色帧入缓冲 → _pc_batch_state（与回调里的顺序一致）
    # ⚠️ 先清空：所有帧的时间戳都用 0.0，_nearest_color 的 min() 在并列时取
    # **缓冲区里第一个**，于是第二帧起会配到上一帧的彩色图 —— 症状是每帧
    # 都偏 0.1–0.5 mm，看起来像「外壳有 bug」。这是测试台自己的问题。
    node.color_buffer.clear()
    node.color_buffer.append((0.0, f["img"]))
    state = node._pc_batch_state(depth_m, transform, 0.0)
    if state is None:
        raise SystemExit(f'{frame_dir}: 节点侧 _pc_batch_state 返回 None '
                         f'（彩色/深度/桌面有一步过不去）')

    # 离线侧：同一帧的云与板位姿。
    # ⚠️ 这里按**与节点相同的 float32 口径**建云，而不是 `eb.point_cloud(f)`
    # （后者用 float64 除）。节点的深度缓冲是 float32（省内存），两边 dtype
    # 不同时 RANSAC 会在边缘 ROI 上偶发换一个平面 —— 实测最坏 0.5 mm，还会
    # 翻掉一个检测的接受/拒绝。那是 dtype 的差，不是外壳的差；外壳测试必须
    # 把它排除掉，否则判据只能放到 1 mm，等于什么也没钉住。
    depth_f32 = f["depth"].astype(np.float32) / 1000.0
    R_bc, t_bc = eb._tf_matrix(entry)
    P_off, valid_off = pc_grasp.base_cloud(depth_f32.astype(np.float64),
                                           K, R_bc, t_bc)
    pose = eb.board_pose(f["img"], K, f["D"], eb.make_board(eb.SQUARE_MM_DEFAULT))
    if pose is None:
        raise SystemExit(f'{frame_dir}: 板没解出来，无法做板系对照')
    f["_cloud"] = (P_off, valid_off)
    f["_board_pose"] = (pose["R"], pose["t"])
    gp_off = eb._global_table_plane(P_off, valid_off)

    cloud_diff = float(np.abs(state["P"] - P_off).max())
    det_rows = []
    worst = 0.0
    for det in f["detections"]["detections"]:
        score = float((det.get("best") or {}).get("score", 0.0))
        pose_node = node._pc_top_pose(
            det["cx"], det["cy"], det["size_x"], det["size_y"], score, state,
            Time(sec=0, nanosec=0))
        box = (int(round(det["cx"] - det["size_x"] / 2)),
               int(round(det["cx"] + det["size_x"] / 2)),
               int(round(det["cy"] - det["size_y"] / 2)),
               int(round(det["cy"] + det["size_y"] / 2)))
        plane = eb._local_table_plane(P_off, valid_off, box, global_plane=gp_off)
        e_off = eb.estimate_grasp_pc(f, det, plane)

        if (pose_node is None) != (e_off is None):
            det_rows.append((det, pose_node, e_off, None))
            continue
        if pose_node is None:
            det_rows.append((det, None, None, 0.0))
            continue
        pn = np.array([pose_node.pose.position.x, pose_node.pose.position.y,
                       pose_node.pose.position.z])
        d = float(np.linalg.norm(pn - e_off["p_base"]))
        worst = max(worst, d)
        det_rows.append((det, pose_node, e_off, d))

    n_both = sum(1 for r in det_rows if r[1] is not None and r[2] is not None)
    n_none = sum(1 for r in det_rows if r[1] is None and r[2] is None)
    n_mismatch = sum(1 for r in det_rows if (r[1] is None) != (r[2] is None))

    name = os.path.basename(os.path.normpath(frame_dir))
    check(f'{name}: 整批点云两边相同', cloud_diff <= MAX_DIFF_M,
          f'最大差 {cloud_diff:.3e} m')
    check(f'{name}: 接受/拒绝的集合一致', n_mismatch == 0,
          f'{len(det_rows)} 检测 → 两边都有 {n_both}，两边都拒 {n_none}，'
          f'只见一边 {n_mismatch}')
    check(f'{name}: 逐点三维差 < {MAX_DIFF_M:g} m', worst <= MAX_DIFF_M,
          f'最大 {worst:.3e} m')
    return n_both


def main(argv):
    dirs = argv[1:] or sorted(glob.glob(DEFAULT_GLOB))
    if not dirs:
        print(f"[SKIP] 没找到基准帧：{DEFAULT_GLOB}")
        print("       基准帧在仓库外（公开仓不存真实画面），换台机器必然缺；"
              "请在记录过基准帧的机器上跑，或显式给帧目录。")
        return 0
    missing = [d for d in dirs if not os.path.isdir(d)]
    if missing:
        print(f'[SKIP] 不是目录：{missing[:3]}')
        return 0

    rclpy.init(args=['--ros-args', '-p', 'grasp_point_mode:=pc_top'])
    try:
        node = Perception3D()
        print("\n--- 参数接线 ---")
        check('grasp_point_mode 由参数面读到 pc_top',
              node.grasp_point_mode == 'pc_top')
        check('pc_top 时订阅了彩色图', node.color_sub is not None)
        # 分数闸门调到 0：这条测的是**几何外壳**，不是分数策略；上闸门会让
        # 两边都拒掉同样的低分检测，「集合一致」就变成一句空话。
        node.pc_min_score = 0.0

        print("\n--- 逐帧：节点 vs 离线 ---")
        total = 0
        for d in dirs:
            total += compare_frame(node, d)
        print(f'\n  两边一致的抓取点合计：{total}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print("\n" + "=" * 66)
    if _failed:
        print(f'❌ {len(_failed)} 项没过：')
        for n in _failed:
            print(f'   - {n}')
        return 1
    print(f'✅ 全部通过（{len(dirs)} 帧）')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))

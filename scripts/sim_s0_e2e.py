#!/usr/bin/env python3
"""E1 · S0 端到端：指定 → 抓取 → 放置 → 评分（**仿真，离线**）。

判据原文（第一百四十九轮定稿）：
    「S0 端到端：指定 → 抓取 → 放置 → 评分，跑 3 次结果可复现。」

一次 run 做这些事（全部在仿真里，真机一次都不碰）：

1. 起 ``arm_grasp_sim/full.launch.py``（``headless`` + 固定布局 ``scene:=S0:0``
   + ``skip_detector:=true`` —— 后者让 ``/targets_3d`` 上没有别人，本脚本发的
   那一个目标就是唯一输入）。
2. 等规划器打出「GraspPlanner 就绪」且场景真值文件已生成。
3. 俯视相机抓一帧 → ``graspnet_candidates.py`` 出候选 → ``stage4_specify.py``
   把「紫色 中间」翻成一个 (x, y, 顶面) 毫米坐标。
4. 只发这一个目标到 ``/targets_3d``，同时按 ~1 Hz 采**指定块**的 Gazebo 真值
   位姿（这是「它真的被抬起来了」的独立证据，不是听规划器自报）。
5. ``/grasp_planner/telemetry`` 报 ``grasped>=1`` 就停发（规划器已经在执行，
   继续发只会让它放下后再抓一次）。等 ``place_placed>=1`` 且 ``busy==false``。
6. 读全部方块的真值位姿，**用几何评分**（见 ``score_run``），写 ``run.json``。
7. 整组退出（``setsid`` 起的进程组一刀切），清掉残留 ROS 节点。

为什么评分不听规划器的自报
==========================
``telemetry`` 的 ``place_placed`` / ``grasped`` 是状态机的**主张**，不是世界
的事实；一个 attach 丢了、碰撞体没进场景的故障，恰恰会让主张与世界分家。
所以 ``pass`` 只看 Gazebo 真值五条，遥测单独记一份作对照（``planner_claim``）。
这正是 E1 失败注入要防的「假绿灯」。

为什么三次必须真的换一次仿真
============================
``full.launch.py`` 每条命令只起一套 Gazebo。同一套里重复跑，方块已经在垫上了，
第二次的「指定」就没有意义。所以 ``--runs 3`` 是起三次、每次从零布局开始。
布局由 ``gen_scene.py --batch S0 --index 0`` 用固定种子生成 ⇒ 三次的初始世界
逐块相同（``scene_truth.json`` 可直接比对）。

失败注入
========
``--inject`` 在**发布目标之前**改一处系统行为，用来证明评分器会红：

    frame_swap   把 (x, y) 对调再发（错坐标系）⇒ 臂去空桌上抓空气
    no_close     夹爪 close 目标写成 open 值（夹爪没夹，≈ attach 丢失）
                 ⇒ 空夹被 ``sim_gripper_empty_threshold`` 判出来，重试到超时

其余两类（删碰撞体、改错阈值）不在这条路上做，理由与离线覆盖见
``test_sim_s0_e2e.py`` 的头注。

用法（必须先 source ROS 与工作区，见 README）
----
    python3 scripts/sim_s0_e2e.py --out /tmp/e1 --runs 3
    python3 scripts/sim_s0_e2e.py --out /tmp/e1-red --runs 1 --inject frame_swap
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import move_block as mb  # noqa: E402  gz 位姿读取的唯一实现（含 10% 假失败重试）

VENV_PY = os.path.expanduser("~/robotics/venvs/graspnet/bin/python")
SIM_YAML = os.path.join(
    REPO, "ros2_ws", "src", "arm_grasp_sim", "config", "grasp_planner_sim.yaml")
READY_RE = re.compile(r"GraspPlanner 就绪")

# --------------------------------------------------------------------------
# 纯计算：评分与复现比对（可被 test_sim_s0_e2e.py 直接 import，不需要 ROS）
# --------------------------------------------------------------------------

def _dist_xy(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def nearest_slot(xy, slots):
    """最近的放置槽位 → (下标, 距离米)。slots 是 [[x, y], ...]。"""
    best = min(range(len(slots)), key=lambda i: _dist_xy(xy, slots[i]))
    return best, _dist_xy(xy, slots[best])


def score_run(pre, post, target_id, slots, lift_peak_z=None, *,
              move_min_m=0.050, others_max_m=0.005, slot_tol_m=0.020,
              lift_min_m=0.080, z_floor_m=0.005):
    """一次 run 的几何评分。全部输入都是 Gazebo 真值（米）。

    参数
        pre          {块 id: (x, y, z)} 运行前
        post         {块 id: (x, y, z)} 运行后
        target_id    被指定的块
        slots        [[x, y], ...] 规划器允许的放置槽位
        lift_peak_z  运行中采到的指定块 z 峰值（None = 没采到）

    返回带 ``pass`` 与逐条判据的 dict；``pass`` 是五条的与。
    """
    ids = sorted(pre)
    missing = [i for i in ids if i not in post]
    if missing:
        raise ValueError(f"运行后读不到这些块的真值: {missing}")

    tgt = pre[target_id]
    post_tgt = post[target_id]
    slot_i, slot_d = nearest_slot(post_tgt[:2], slots)

    checks = {}
    checks["target_moved"] = {
        "ok": _dist_xy(tgt, post_tgt) > move_min_m,
        "detail": f"位移 {1000 * _dist_xy(tgt, post_tgt):.1f} mm（> {1000 * move_min_m:.0f}）",
    }
    lifted = (lift_peak_z is not None
              and (lift_peak_z - tgt[2]) > lift_min_m)
    checks["target_lifted"] = {
        "ok": bool(lifted),
        "detail": ("未采到 z 峰值" if lift_peak_z is None else
                   f"z 峰值 {1000 * lift_peak_z:.1f} mm − 初值 {1000 * tgt[2]:.1f} mm"
                   f" = {1000 * (lift_peak_z - tgt[2]):.1f} mm"
                   f"（> {1000 * lift_min_m:.0f}）"),
    }
    checks["target_in_slot"] = {
        "ok": slot_d < slot_tol_m,
        "detail": f"落点距槽位 {slot_i} {1000 * slot_d:.1f} mm（< {1000 * slot_tol_m:.0f}）",
    }
    others = {i: _dist_xy(pre[i], post[i]) for i in ids if i != target_id}
    worst_i = max(others, key=others.get)
    checks["others_unmoved"] = {
        "ok": others[worst_i] <= others_max_m,
        "detail": (f"最大位移 {1000 * others[worst_i]:.1f} mm @ 块 {worst_i}"
                   f"（≤ {1000 * others_max_m:.0f}）"),
    }
    low = {i: post[i][2] for i in ids if post[i][2] <= z_floor_m}
    checks["all_above_table"] = {
        "ok": not low,
        "detail": ("全部 z > 5 mm" if not low else
                   "低于台面: " + ", ".join(
                       f"块 {i} z={1000 * z:.1f} mm" for i, z in low.items())),
    }
    return {
        "pass": all(c["ok"] for c in checks.values()),
        "checks": checks,
        "slot_index": slot_i,
        "slot_dist_mm": 1000 * slot_d,
        "target_block": target_id,
        "post_xy": list(post_tgt[:2]),
    }


def compare_runs(runs, *, max_spread_m=0.010):
    """三次 run 的可复现判据。runs 是 score_run 的返回值列表（顺序 = 运行序）。

    「可复现」= 同一块被指定、落在同一槽位、三次都 pass、落点两两最大差
    ≤ 10 mm。最后一条是**比分更严**的一条：三次都「pass」也可能各自落在
    槽位容差的两端（相差 40 mm），那不算复现。
    """
    ids = {r["target_block"] for r in runs}
    slot_ids = {r["slot_index"] for r in runs}
    pts = [r["post_xy"] for r in runs]
    spread = 0.0
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            spread = max(spread, _dist_xy(pts[i], pts[j]))
    checks = {
        "same_target": {"ok": len(ids) == 1,
                        "detail": f"目标块 {sorted(ids)}"},
        "same_slot": {"ok": len(slot_ids) == 1,
                      "detail": f"槽位 {sorted(slot_ids)}"},
        "all_pass": {"ok": all(r["pass"] for r in runs),
                     "detail": f"{sum(r['pass'] for r in runs)}/{len(runs)} pass"},
        "landing_spread": {"ok": len(pts) >= 2 and spread <= max_spread_m,
                           "detail": f"落点两两最大差 {1000 * spread:.1f} mm"
                                     f"（≤ {1000 * max_spread_m:.0f}）"},
    }
    return {"pass": all(c["ok"] for c in checks.values()), "checks": checks,
            "spread_mm": 1000 * spread}


# --------------------------------------------------------------------------
# 环境：真值、槽位、启动 / 收工
# --------------------------------------------------------------------------

def load_truth(path):
    with open(path) as f:
        return json.load(f)


def load_slots(yaml_path=SIM_YAML):
    import yaml
    with open(yaml_path) as f:
        doc = yaml.safe_load(f)
    raw = doc["grasp_planner"]["ros__parameters"]["place_slots"]
    return [[float(raw[i]), float(raw[i + 1])] for i in range(0, len(raw), 2)]


def load_place_z(yaml_path=SIM_YAML):
    import yaml
    with open(yaml_path) as f:
        doc = yaml.safe_load(f)
    return float(doc["grasp_planner"]["ros__parameters"]["place_z"])


def write_params_variant(dst, **overrides):
    """复制仿真 yaml 并改几个键 —— 注入用。

    ⚠️ 只能改**不在 launch 覆盖字典里**的键（gripper_backend / ik_group /
    tool0_to_fingertip / place_release_lift / table_scene_enabled /
    carry_objects 那七个），否则 launch 里那份后叠的会赢，改了等于没改
    （grasp_planner_sim.yaml 头注列了名单）。
    """
    import yaml
    with open(SIM_YAML) as f:
        doc = yaml.safe_load(f)
    p = doc["grasp_planner"]["ros__parameters"]
    p.update(overrides)
    with open(dst, "w") as f:
        yaml.safe_dump(doc, f, allow_unicode=True)
    return dst


def launch(outdir, scene, params_file=None, extra_args=()):
    env = dict(os.environ)
    env.setdefault("GZ_IP", "127.0.0.1")
    env["RCUTILS_LOGGING_BUFFERED_STREAM"] = "1"
    log_path = os.path.join(outdir, "launch.log")
    log = open(log_path, "w")
    cmd = ["ros2", "launch", "arm_grasp_sim", "full.launch.py",
           "headless:=true", f"scene:={scene}", "skip_detector:=true", *extra_args]
    if params_file:
        cmd.append(f"planner_params_file:={params_file}")
    proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True, env=env)
    return proc, log, log_path


def kill_group(proc, log=None):
    """整组退出 + 清残留。

    ``launch`` 用 ``start_new_session`` 起，所以它的每一个子进程（gz、bridge、
    move_group、规划器…）都在同一个 pgid 里，一刀切得干净。之后按可执行文件名
    兜一遍 KILL —— 只清 ROS/仿真基础设施，不碰别的交互进程。
    """
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            break
        try:
            proc.wait(timeout=12)
            break
        except subprocess.TimeoutExpired:
            continue
    for pat in ("/opt/ros/humble/lib/tf2_ros/static_transform_publisher",
                "/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher",
                "/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge",
                "/opt/ros/humble/lib/moveit_ros_move_group/move_group",
                "camera_adapter.py", "gz sim"):
        subprocess.run(["pkill", "-KILL", "-f", pat],
                       capture_output=True, check=False)
    if log:
        log.close()


def _tail(path, n=40):
    try:
        with open(path, errors="replace") as f:
            return "".join(f.readlines()[-n:])
    except FileNotFoundError:
        return ""


# --------------------------------------------------------------------------
# 一次 run
# --------------------------------------------------------------------------

class _Node:
    """发布目标 + 订阅遥测（transient_local，否则锁存对我们无效）。"""

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy,
                               QoSProfile, ReliabilityPolicy)
        from diagnostic_msgs.msg import DiagnosticArray
        from vision_msgs.msg import Detection3DArray

        self._rclpy = rclpy
        rclpy.init()
        self.n = Node("sim_s0_e2e")
        self.pub = self.n.create_publisher(Detection3DArray, "/targets_3d", 10)
        qos = QoSProfile(
            depth=1, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.telem = None
        self.n.create_subscription(DiagnosticArray, "/grasp_planner/telemetry",
                                   self._on_telem, qos)

    def _on_telem(self, msg):
        # 规划器只有一个 DiagnosticStatus；取第一条即可（多条的语义没定义，
        # 而这个话题的契约写在 grasp_planner/telemetry.py 头注）。
        if msg.status:
            self.telem = {kv.key: kv.value for kv in msg.status[0].values}

    def pump(self, seconds=0.2):
        end = time.time() + seconds
        while time.time() < end:
            self._rclpy.spin_once(self.n, timeout_sec=0.05)

    def publish_target(self, xyz, label):
        from vision_msgs.msg import Detection3D, Detection3DArray
        from vision_msgs.msg import ObjectHypothesisWithPose
        msg = Detection3DArray()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.n.get_clock().now().to_msg()
        d = Detection3D()
        d.header = msg.header
        d.bbox.center.position.x = xyz[0]
        d.bbox.center.position.y = xyz[1]
        d.bbox.center.position.z = xyz[2]
        h = ObjectHypothesisWithPose()
        h.hypothesis.class_id = label
        h.hypothesis.score = 0.9
        h.pose.pose.position.x = xyz[0]
        h.pose.pose.position.y = xyz[1]
        h.pose.pose.position.z = xyz[2]
        d.results.append(h)
        msg.detections.append(d)
        self.pub.publish(msg)

    def shutdown(self):
        try:
            self.n.destroy_node()
            self._rclpy.shutdown()
        except Exception:
            pass


def _run_py(script, *args, timeout=300):
    r = subprocess.run([sys.executable, os.path.join(HERE, script), *args],
                       capture_output=True, text=True, timeout=timeout)
    if r.returncode != 0:
        raise RuntimeError(
            f"{script} 退出码 {r.returncode}\n--- stdout ---\n{r.stdout}"
            f"\n--- stderr ---\n{r.stderr}")
    return r.stdout


def parse_specify_output(text):
    """``stage4_specify.py`` 的 stdout 是「解析: …」+「结果: {json}」两行。

    不能整段 ``json.loads`` —— 第一行是给人看的解析树。取 ``结果:`` 之后
    的部分；格式变了要**响亮地**失败，不能返回一个空 dict。
    """
    marker = "结果:"
    i = text.find(marker)
    if i < 0:
        raise ValueError(f"stage4_specify 输出里没有 {marker!r}：\n{text[:500]}")
    return json.loads(text[i + len(marker):])


def one_run(outdir, scene, command, inject, *, motion_timeout, lift_log):
    """跑一次完整流程 → score dict（含 ``post_xy``）。失败抛异常。"""
    os.makedirs(outdir, exist_ok=True)
    batch, index = scene.split(":")
    truth_path = f"/tmp/grasp_scene_{batch}_{index}/scene_truth.json"
    if os.path.exists(truth_path):
        os.remove(truth_path)          # 防止读到上一轮的旧真值

    params = None
    if inject == "no_close":
        params = write_params_variant(
            os.path.join(outdir, "planner_params_inject.yaml"),
            sim_gripper_close=0.0475)

    proc, log, log_path = launch(outdir, scene, params)
    node = None
    try:
        # ---- 1. 等规划器就绪（日志）+ 真值文件 ----
        t0 = time.time()
        while time.time() - t0 < 240:
            if READY_RE.search(_tail(log_path, 400)) and os.path.exists(truth_path):
                break
            if proc.poll() is not None:
                raise RuntimeError(
                    f"launch 提前退出 rc={proc.returncode}\n{_tail(log_path, 80)}")
            time.sleep(2.0)
        else:
            raise RuntimeError(f"240 s 内没就绪\n{_tail(log_path, 80)}")

        truth = load_truth(truth_path)
        ids = [b["id"] for b in truth["blocks"]]
        pre = {}
        for i in ids:
            p = mb.read_pose(f"block_{i}", tries=6)
            if p is None:
                raise RuntimeError(f"读不到 block_{i} 的初始位姿")
            pre[i] = p

        # ---- 2. 俯视抓帧 → 候选 → 指定 ----
        frame = os.path.join(outdir, "pre")
        cap_log = ""
        for attempt in range(3):
            r = subprocess.run(
                [sys.executable, os.path.join(HERE, "sim_capture_frame.py"),
                 frame, "--camera", "topcam", "--truth", truth_path],
                capture_output=True, text=True, timeout=180)
            cap_log += r.stdout + r.stderr
            # 判据是「这一帧落盘了」，不是某个下游文件（candidates.json 是
            # 下一步生成的）。抓帧脚本自己会报 rc。
            if r.returncode == 0 and os.path.exists(
                    os.path.join(frame, "depth.png")):
                break
            shutil.rmtree(frame, ignore_errors=True)
            time.sleep(5)
        else:
            raise RuntimeError(f"抓帧失败\n{cap_log[-2000:]}")

        cand_json = os.path.join(frame, "candidates.json")
        r = subprocess.run(
            [VENV_PY, os.path.join(HERE, "graspnet_candidates.py"), frame,
             "--json", cand_json, "--no-infer"],
            capture_output=True, text=True, timeout=600)
        if r.returncode != 0:
            raise RuntimeError(f"候选提取失败 rc={r.returncode}\n{r.stderr[-2000:]}")

        spec = parse_specify_output(
            _run_py("stage4_specify.py", frame, command,
                    "--hue-table", "sim"))
        if spec.get("status") != "ok" or not spec.get("block"):
            raise RuntimeError(f"指定失败: {spec}")
        blk = spec["block"]
        xyz_mm = (blk["x"], blk["y"], blk["top"])

        # 指定结果 → 真值里哪一块（最近邻 + 颜色核对）
        near = min(truth["blocks"],
                   key=lambda b: math.hypot(b["x"] * 1000 - xyz_mm[0],
                                            b["y"] * 1000 - xyz_mm[1]))
        det_err_mm = math.hypot(near["x"] * 1000 - xyz_mm[0],
                                near["y"] * 1000 - xyz_mm[1])
        if det_err_mm > 20.0:
            raise RuntimeError(
                f"指定结果离最近的真值块 {det_err_mm:.1f} mm（> 20）—— 测量链本身就不对")
        target_id = near["id"]

        # ---- 3. 发目标 + 采提升峰值 ----
        pub_xyz = (xyz_mm[0] / 1000.0, xyz_mm[1] / 1000.0, xyz_mm[2] / 1000.0)
        if inject == "frame_swap":
            pub_xyz = (pub_xyz[1], pub_xyz[0], pub_xyz[2])
        node = _Node()
        samples = []
        t0 = time.time()
        last_sample = 0.0
        grasped = False
        while time.time() - t0 < motion_timeout:
            node.publish_target(pub_xyz, blk.get("color") or "block")
            node.pump(0.1)
            if time.time() - last_sample >= 1.2:
                p = mb.read_pose(f"block_{target_id}", tries=2)
                if p is not None:
                    samples.append([round(time.time() - t0, 2), *p])
                last_sample = time.time()
            tl = node.telem or {}
            if int(tl.get("grasped", 0) or 0) >= 1:
                grasped = True
                break

        # ---- 4. 等放置完成 ----
        tl = {}
        t1 = time.time()
        while time.time() - t1 < 240:
            node.pump(0.2)
            tl = node.telem or {}
            if (int(tl.get("place_placed", 0) or 0) >= 1
                    and str(tl.get("busy", "true")).lower() == "false"):
                break
        node.pump(2.0)           # 落稳
        tl = node.telem or dict(tl)
        state = tl.get("state")
        place_placed = int(tl.get("place_placed", 0) or 0)

        # ---- 5. 读运行后真值 → 评分 ----
        post = {}
        for i in ids:
            p = mb.read_pose(f"block_{i}", tries=6)
            if p is None:
                raise RuntimeError(f"读不到 block_{i} 的运行后位姿")
            post[i] = p
        lift_peak = max((s[3] for s in samples), default=None)
        if lift_log:
            with open(lift_log, "a") as f:
                for row in samples:
                    f.write(json.dumps(row) + "\n")

        res = score_run(pre, post, target_id, load_slots(), lift_peak)
        res.update({
            "target_block": target_id,
            "target_color": near["color"],
            "spec": spec,
            "det_err_mm": det_err_mm,
            "published_xyz_m": list(pub_xyz),
            "pre": {str(k): list(v) for k, v in pre.items()},
            "post": {str(k): list(v) for k, v in post.items()},
            "lift_samples": samples,
            "lift_peak_z": lift_peak,
            "inject": inject,
            "telem_state": state,
            "telem_place_placed": place_placed,
            "planner_claim": {"grasped_seen": grasped, "state": state,
                              "place_placed": place_placed},
            "post_xy": list(post[target_id][:2]),
            "elapsed_s": round(time.time() - t0, 1),
        })
        return res
    finally:
        if node is not None:
            node.shutdown()
        kill_group(proc, log)


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", required=True, help="输出目录（每次 run 一个子目录）")
    ap.add_argument("--scene", default="S0:0", help="固定布局，默认 S0:0")
    ap.add_argument("--command", default="purple center",
                    help="自然语言指定（交给 stage4_specify.py）")
    ap.add_argument("--runs", type=int, default=3, help="跑几次（每次新起仿真）")
    ap.add_argument("--inject", default="none",
                    choices=("none", "frame_swap", "no_close"),
                    help="失败注入（见文件头）")
    ap.add_argument("--motion-timeout", type=float, default=360.0,
                    help="从发目标到 grasped>=1 的壁钟上限（秒）")
    args = ap.parse_args()

    if args.inject != "none" and args.runs != 1:
        ap.error("注入 run 只跑一次（--runs 1）：它本来就不会抓成")

    os.makedirs(args.out, exist_ok=True)
    runs = []
    for k in range(1, args.runs + 1):
        d = os.path.join(args.out, f"run-{k}")
        print(f"\n=== run {k}/{args.runs}  scene={args.scene}  inject={args.inject} ===")
        try:
            r = one_run(d, args.scene, args.command, args.inject,
                        motion_timeout=args.motion_timeout,
                        lift_log=os.path.join(d, "lift.jsonl"))
        except Exception as exc:                       # noqa: BLE001
            print(f"run {k} 异常：{exc}")
            r = {"pass": False, "error": str(exc), "target_block": None,
                 "slot_index": None, "post_xy": [0, 0]}
        with open(os.path.join(d, "run.json"), "w") as f:
            json.dump(r, f, ensure_ascii=False, indent=1)
        runs.append(r)
        for name, c in (r.get("checks") or {}).items():
            print(f"  [{'PASS' if c['ok'] else 'FAIL'}] {name}: {c['detail']}")
        print(f"  → {'PASS' if r.get('pass') else 'FAIL'}"
              f"  （遥测: {r.get('planner_claim')}）")

    summary = {"scene": args.scene, "command": args.command,
               "inject": args.inject, "runs": runs}
    if args.runs > 1 and all(r.get("checks") for r in runs):
        summary["reproducible"] = compare_runs(runs)
    with open(os.path.join(args.out, "summary.json"), "w") as f:
        json.dump(summary, f, ensure_ascii=False, indent=1)

    print("\n=== 汇总 ===")
    for k, r in enumerate(runs, 1):
        print(f"run {k}: {'PASS' if r.get('pass') else 'FAIL'}"
              f"  目标块={r.get('target_block')} 槽位={r.get('slot_index')}"
              f"  落点距槽位={r.get('slot_dist_mm', float('nan')):.1f} mm")
    if "reproducible" in summary:
        for name, c in summary["reproducible"]["checks"].items():
            print(f"  [{'PASS' if c['ok'] else 'FAIL'}] {name}: {c['detail']}")
        print(f"  可复现：{'PASS' if summary['reproducible']['pass'] else 'FAIL'}")

    ok = all(r.get("pass") for r in runs) and (
        "reproducible" not in summary or summary["reproducible"]["pass"])
    print(f"→ {os.path.join(args.out, 'summary.json')}")
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())

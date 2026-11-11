#!/usr/bin/env python3
"""V1 仿真：**单腕部机位 vs 腕部三机位合并**（感知级，按契约 §11.3）。

问题与口径
==========
契约 V1：单机位够不够？主终点「端到端成功率、遮挡漏块率」。

本工具跑**感知级**那一半（不执行抓放）：

- 在同一个仿真场景里，把腕部相机依次摆到 E9 的三个机位
  （``pick`` 拾取区 / ``free`` 北侧自由区 / ``pad`` 垫面；位姿唯一来源
  ``stage4_wrist_survey.survey_tool0``），每个机位抓一帧;
- 每帧独立跑生产分割 ``graspnet_candidates.find_blocks(split=True)``；
- **单机位**池 = 只看 ``pick`` 那一帧；**三机位**池 = 三帧实例在 base 系里
  按块心去重合并（同一块可能出现多次，保留顶面像素最多的那个）；
- 对每个真值块问「池里有没有一个实例落在它 15 mm 内」= **检出**；
- 指定 top-1 用 E5 同一套命令集（``stage4_e5_eval.gen_commands``，只留真值上
  唯一指向该块的说法），在两种池上跑同一个解析器 ``stage4_specify.select``，
  选中实例离目标块心 ≤15 mm 才算对。

**端到端成功率不在本工具里**：它要跑规划/执行，仿真 RTF≈0.14，且 v0.6 的
腕部 e2e 流水线尚不存在。报告必须把这一半标成「未跑」，不能拿感知级结果冒充。

判据（先定，再看数）
====================
- **遮挡漏块率** = 真值块里「池中 15 mm 内无实例」的比例，pick 机位 vs 三机位
  各报一次；差 = V1 的收益。
- **指定 top-1**：分母 = 全部命令；误选（status=ok 但选错块）单独计数。
- 样本量：契约 §11.4 要求每格 ≥15 个块级尝试；不足只报描述统计。

用法（先 source ROS 与两个工作区；见 ``run_v1_sim.sh``）
========================================================
    python3 scripts/stage4_v1_sim.py --scene S3:0 --scene S3:1 \\
        --out ~/robotics/notes/.../v1-s3 --urdf /tmp/ur5.urdf
    # 栈已经起着（--attach）时不再自己 launch，只摆臂抓帧。

退出码：0 = 全部场景跑完并写出报告；1 = 有场景失败；2 = 输入/环境缺失。
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import time

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import numpy as np                                                # noqa: E402

#: 实例 ↔ 真值块的匹配半径（mm）。块边长 29.5、相邻块心 ≥31 mm ⇒ 15 mm
#: 不会把邻块算成本块。
MATCH_MM = 15.0
#: 跨机位去重半径（mm）。同一个块从不同机位看，块心差通常在几毫米内。
DEDUPE_MM = 20.0

#: E9 的三个腕部机位（与 ``stage4_clear_run.E9_VIEWS`` 同源；这里只写 zone/
#: height，位姿由 ``stage4_wrist_survey.survey_tool0`` 唯一算出来）。
VIEWS = (
    {"name": "pick", "zone": (0.250, 0.575, 0.170, 0.420), "height": 0.500},
    {"name": "free", "zone": (0.280, 0.750, 0.460, 0.620), "height": 0.450},
    {"name": "pad", "zone": (0.310, 0.610, -0.300, 0.000), "height": 0.500},
)
PICK_VIEW = "pick"
#: 真值块参与 V1 的空间范围 = 拾取区（与 ``stage4_specify.ZONE_M`` 一致）。
PICK_ZONE_M = (0.25, 0.575, 0.17, 0.42)


def view_pose(name):
    """机位名 → (tool0 位置 3, 旋转 3×3, 相机光心 3)。位姿唯一来源。"""
    import stage4_wrist_survey as ws
    for v in VIEWS:
        if v["name"] == name:
            return ws.survey_tool0(v["zone"], v["height"])
    raise KeyError(name)


def unwrap_to(q, q_ref):
    """把每个关节角换到离 ``q_ref`` 最近的等价角（±2π）。"""
    q = np.asarray(q, dtype=float)
    q_ref = np.asarray(q_ref, dtype=float)
    return q + 2.0 * np.pi * np.round((q_ref - q) / (2.0 * np.pi))


def usable(insts, zone_m=PICK_ZONE_M):
    """拾取区内、不是低处碎片的实例（与生产路径同一口径）。"""
    import stage4_specify as sp
    x0, x1, y0, y1 = (v * 1000.0 for v in zone_m)
    return [b for b in insts
            if not b.get("low") and x0 <= b["cx"] <= x1 and y0 <= b["cy"] <= y1]


def merge_pools(named_pools, dedupe_mm=DEDUPE_MM):
    """多机位实例合并：块心 20 mm 内视为同一块，保留像素最多的那个。

    ``named_pools`` = [(view_name, [inst, ...]), ...]。返回新列表（idx 重排），
    每个实例带 ``views``（看到它的机位名，升序）与 ``n_view``。
    """
    cand = []
    for view, insts in named_pools:
        for b in insts:
            c = dict(b)
            c["views"] = [view]
            cand.append(c)
    cand.sort(key=lambda b: (-int(b.get("n", 0)), str(b["views"][0])))
    kept = []
    for b in cand:
        hit = None
        for k in kept:
            if np.hypot(b["cx"] - k["cx"], b["cy"] - k["cy"]) <= dedupe_mm:
                hit = k
                break
        if hit is None:
            kept.append(b)
        else:
            hit["views"] = sorted(set(hit["views"]) | {b["views"][0]})
    for i, b in enumerate(kept):
        b["idx"] = i
        b["n_view"] = len(b["views"])
        b["views"] = ",".join(b["views"])
    return kept


def nearest(insts, x_mm, y_mm):
    """池里离 (x, y) 最近的实例与距离；空池返回 (None, inf)。"""
    best, dmin = None, float("inf")
    for b in insts:
        d = float(np.hypot(b["cx"] - x_mm, b["cy"] - y_mm))
        if d < dmin:
            best, dmin = b, d
    return best, dmin


def coverage(truth, insts, zone_m=PICK_ZONE_M, tol_mm=MATCH_MM):
    """真值块 → 是否在池里被检出（**一对一**匹配，块心 tol 内）。

    一对一很重要：叠放的两块块心几乎重合，若各取最近实例，会用一个实例把
    上下两块都记成「检出」—— 下层被压住的那块正好被掩盖。
    """
    x0, x1, y0, y1 = (v * 1000.0 for v in zone_m) if zone_m else (None,) * 4
    blocks = []
    for b in truth["blocks"]:
        x, y = float(b["x"]) * 1000.0, float(b["y"]) * 1000.0
        if zone_m and not (x0 <= x <= x1 and y0 <= y <= y1):
            continue
        blocks.append((int(b["id"]), x, y))
    pairs = []
    for tid, x, y in blocks:
        for b in insts:
            d = float(np.hypot(b["cx"] - x, b["cy"] - y))
            if d <= tol_mm:
                pairs.append((d, tid, int(b["idx"])))
    pairs.sort()
    used_t, used_i, assigned = set(), set(), {}
    for d, tid, idx in pairs:
        if tid in used_t or idx in used_i:
            continue
        used_t.add(tid)
        used_i.add(idx)
        assigned[tid] = (idx, d)
    rows = []
    for tid, x, y in blocks:
        a = assigned.get(tid)
        near, _d = nearest(insts, x, y)
        rows.append({"id": tid, "x": round(x, 1), "y": round(y, 1),
                     "detected": a is not None,
                     "nearest_mm": (None if near is None
                                    else round(_d, 1)),
                     "instance": (None if a is None else a[0])})
    return rows


def specify_score(truth, insts, tol_mm=MATCH_MM):
    """E5 命令集在同一池上的指定 top-1（选中实例离目标块心 ≤tol 才算对）。"""
    import stage4_e5_eval as ev
    import stage4_specify as sp
    cmds, skipped, _recs, disagree = ev.gen_commands(truth)
    counts = {"ok": 0, "ambiguous": 0, "none": 0, "bad": 0}
    correct = misbind = 0
    rows = []
    for c in cmds:
        r = sp.select(insts, c["text"])
        st = r["status"]
        counts[st] = counts.get(st, 0) + 1
        tb = next(b for b in truth["blocks"] if int(b["id"]) == int(c["target"]))
        tx, ty = float(tb["x"]) * 1000.0, float(tb["y"]) * 1000.0
        got = None
        d = None
        if r.get("instance") is not None:
            b = next((x for x in insts if int(x["idx"]) == int(r["instance"])),
                     None)
            if b is not None:
                got = int(b["idx"])
                d = float(np.hypot(b["cx"] - tx, b["cy"] - ty))
        hit = d is not None and d <= tol_mm
        correct += int(hit)
        misbind += int(st == "ok" and not hit)
        rows.append({"text": c["text"], "target": int(c["target"]),
                     "status": st, "instance": got,
                     "dist_mm": (None if d is None else round(d, 1)),
                     "correct": hit})
    return {"n_commands": len(cmds), "n_skipped_not_unique": skipped,
            "n_indep_disagree": len(disagree), "counts": counts,
            "correct": correct, "misbind": misbind,
            "id_rate": (correct / len(cmds)) if cmds else None,
            "commands": rows}


def occluded_ids(truth, tol_mm=MATCH_MM):
    """被上面一块压住的真值块（水平投影重叠 ⇒ 顶面可能被遮）。

    判据是**真值几何**，不是检测结果：另一块 z 更高且块心水平距离 < 15 mm
    （29.5 mm 方块叠放时通常同轴；15 mm 已覆盖 gen_scene 的叠放偏移）。
    """
    occ = set()
    for b in truth["blocks"]:
        x, y = float(b["x"]) * 1000.0, float(b["y"]) * 1000.0
        z = float(b.get("z", 0.0))
        for o in truth["blocks"]:
            if o is b or float(o.get("z", 0.0)) <= z:
                continue
            if np.hypot(float(o["x"]) * 1000.0 - x,
                        float(o["y"]) * 1000.0 - y) < tol_mm:
                occ.add(int(b["id"]))
    return occ


def eval_scene(label, truth, named_pools):
    """一个场景：各机位单独 + 合并池的覆盖与指定；pick 是单机位臂。"""
    occ = occluded_ids(truth)
    out = {"label": label, "batch": truth.get("batch"),
           "occluded_ids": sorted(occ), "n_truth_in_zone": None, "views": {}}
    all_merged = merge_pools(named_pools)
    for view, insts in named_pools:
        pool = usable(insts)
        cov = coverage(truth, pool)
        cov_all = coverage(truth, pool, zone_m=None)
        out["views"][view] = {
            "n_instances": len(pool), "coverage": cov,
            "coverage_world": cov_all,
            "n_detected": sum(1 for r in cov if r["detected"]),
            "n_truth": len(cov),
            "n_detected_world": sum(1 for r in cov_all if r["detected"]),
            "n_truth_world": len(cov_all),
            "occluded_detected": sum(1 for r in cov_all
                                     if r["detected"] and r["id"] in occ),
            "occluded_total": sum(1 for r in cov_all if r["id"] in occ),
            "specify": specify_score(truth, pool),
        }
    pool = usable(all_merged)
    cov = coverage(truth, pool)
    cov_all = coverage(truth, pool, zone_m=None)
    out["merged"] = {
        "n_instances": len(pool), "coverage": cov,
        "coverage_world": cov_all,
        "n_detected": sum(1 for r in cov if r["detected"]), "n_truth": len(cov),
        "n_detected_world": sum(1 for r in cov_all if r["detected"]),
        "n_truth_world": len(cov_all),
        "occluded_detected": sum(1 for r in cov_all
                                 if r["detected"] and r["id"] in occ),
        "occluded_total": sum(1 for r in cov_all if r["id"] in occ),
        "specify": specify_score(truth, pool),
        "n_from_3views": sum(1 for b in pool if int(b.get("n_view", 0)) >= 3),
    }
    out["n_truth_in_zone"] = len(cov)
    single = out["views"][PICK_VIEW]
    out["occluded_miss"] = {
        "single_missing": [r["id"] for r in single["coverage"]
                           if not r["detected"]],
        "merged_missing": [r["id"] for r in cov if not r["detected"]],
        "recovered": [r["id"] for r in single["coverage"]
                      if not r["detected"]
                      and any(m["id"] == r["id"] and m["detected"]
                              for m in cov)],
        "single_missing_world": [r["id"] for r in single["coverage_world"]
                                 if not r["detected"]],
        "merged_missing_world": [r["id"] for r in cov_all
                                 if not r["detected"]],
    }
    return out


# ---------------------------------------------------------------- ROS 驱动
class Arm:
    """/joint_states 读 + JTC 发轨迹（只为摆机位，不抓不放）。"""

    JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
              "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

    def __init__(self):
        import threading

        import rclpy
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from sensor_msgs.msg import JointState

        rclpy.init()
        self.rclpy = rclpy
        self.node = Node("stage4_v1_sim")
        self.q = {}
        self.node.create_subscription(JointState, "/joint_states", self._cb, 50)
        self.cli = ActionClient(
            self.node, FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory")
        self._spin = threading.Thread(target=rclpy.spin, args=(self.node,),
                                      daemon=True)
        self._spin.start()

    def _cb(self, m):
        for n, p in zip(m.name, m.position):
            self.q[n] = float(p)

    def wait_joints(self, timeout=60.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if all(j in self.q for j in self.JOINTS):
                return [self.q[j] for j in self.JOINTS]
            time.sleep(0.2)
        raise RuntimeError("60 s 内没有完整 /joint_states")

    def goto(self, q_target, secs=3.0, timeout=60.0):
        from builtin_interfaces.msg import Duration
        from control_msgs.action import FollowJointTrajectory
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        q0 = self.wait_joints()
        q1 = unwrap_to(q_target, q0)
        traj = JointTrajectory()
        traj.joint_names = self.JOINTS
        p0 = JointTrajectoryPoint()
        p0.positions = q0
        p0.time_from_start = Duration(sec=0)
        p1 = JointTrajectoryPoint()
        p1.positions = [float(v) for v in q1]
        p1.time_from_start = Duration(sec=int(secs))
        traj.points = [p0, p1]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        # 控制器在 launch 后要过一会儿才 active；这段窗口里 action server 已经在，
        # 但 goal 会被「Controller is not running」拒收 —— 重试，不是失败。
        last = None
        t0 = time.time()
        while time.time() - t0 < 90.0:
            if not self.cli.wait_for_server(timeout_sec=5.0):
                time.sleep(1.0)
                continue
            fut = self.cli.send_goal_async(goal)
            t1 = time.time()
            while not fut.done() and time.time() - t1 < 10.0:
                time.sleep(0.05)
            if not fut.done():
                last = "goal 10 s 无回应"
                time.sleep(1.0)
                continue
            gh = fut.result()
            if gh is not None and gh.accepted:
                break
            last = "控制器未 running（拒收）"
            time.sleep(1.0)
        else:
            raise RuntimeError("JTC 90 s 内没接受轨迹：%s" % last)
        rf = gh.get_result_async()
        t0 = time.time()
        while not rf.done() and time.time() - t0 < timeout:
            time.sleep(0.2)
        if not rf.done():
            raise RuntimeError("JTC 轨迹超时")
        status = rf.result().status
        time.sleep(1.0)
        qa = np.array(self.wait_joints())
        err = float(np.max(np.abs(qa - q1)))
        return {"status": int(status), "max_err_rad": round(err, 4),
                "ok": status == 4 and err < 0.05}

    def shutdown(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


SIM_IDLE_TIMEOUT_S = 20.0


def other_sims(pgrep_text):
    """``pgrep -af`` 输出 → 正在跑的 Gazebo 命令行。

    本机所有仿真共用 ``/tmp/grasp_scene_*``，第二台起来后
    ``sim_capture_frame`` 可能抓到**另一场**的相机（2026-11-11 实测：S1 帧里
    7 块、颜色与真值全不符，O1 一度判成 0%）。
    """
    alive = []
    for line in (pgrep_text or "").splitlines():
        if "gz sim" in line and "grasp_world.sdf" in line:
            alive.append(line.strip())
    return alive


def _pgrep_sims():
    try:
        r = subprocess.run(["pgrep", "-af", "gz sim"],
                           capture_output=True, text=True)
    except OSError:
        return ""
    return r.stdout if r.returncode == 0 else ""


def wait_other_sims_gone(timeout=SIM_IDLE_TIMEOUT_S):
    """等残留 Gazebo 退出；返回超时后仍在跑的列表（空 = 干净）。"""
    deadline = time.time() + timeout
    while True:
        alive = other_sims(_pgrep_sims())
        if not alive or time.time() >= deadline:
            return alive
        time.sleep(0.5)


def launch_sim(scene, log_path):
    os.makedirs(os.path.dirname(log_path) or ".", exist_ok=True)
    alive = wait_other_sims_gone()
    if alive:
        raise RuntimeError(
            "本机已有 Gazebo 在跑，抓到的帧可能来自别的场景；"
            "先关掉再跑：\n  " + "\n  ".join(alive))
    # 旧场景目录会让 wait_truth 立刻返回（文件是上一轮留下的），于是在控制器
    # 还没 active 时就发轨迹。先删掉，让 truth 文件只在本次生成时出现。
    batch, idx = scene.split(":")
    import shutil
    shutil.rmtree("/tmp/grasp_scene_%s_%s" % (batch, idx),
                  ignore_errors=True)
    log = open(log_path, "w")
    p = subprocess.Popen(
        ["ros2", "launch", "arm_grasp_sim", "full.launch.py",
         "headless:=true", "skip_planner:=true", "scene:=%s" % scene],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    return p, log


def stop_sim(p):
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        p.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(p.pid), signal.SIGKILL)


def wait_truth(scene, timeout=180.0):
    batch, idx = scene.split(":")
    path = "/tmp/grasp_scene_%s_%s/scene_truth.json" % (batch, idx)
    t0 = time.time()
    while time.time() - t0 < timeout:
        if os.path.exists(path):
            time.sleep(8.0)          # 等相机/TF 稳定
            return path
        time.sleep(2.0)
    return None


def capture(frame_dir, truth_path, timeout):
    cmd = [sys.executable, os.path.join(HERE, "sim_capture_frame.py"),
           frame_dir, "--camera", "wrist", "--timeout", str(timeout),
           "--truth", truth_path]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        return {"ok": False, "error": (r.stderr or r.stdout).strip()[-800:]}
    return {"ok": True}


def run_scene(arm, chain, scene, out_dir, urdf, timeout, views):
    truth_path = wait_truth(scene)
    if truth_path is None:
        return {"ok": False, "error": "等不到 scene_truth.json"}
    with open(truth_path, encoding="utf-8") as f:
        truth = json.load(f)
    named = []
    moves = {}
    for name in views:
        tool0, _cam, R = view_pose(name)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = tool0
        q0 = arm.wait_joints()
        q, _pe, _re, _it = chain.ik(T, q0)
        mv = arm.goto(q)
        moves[name] = mv
        if not mv["ok"]:
            return {"ok": False, "error": "机位 %s 没到位" % name,
                    "moves": moves}
        frame_dir = os.path.join(out_dir, scene.replace(":", "_"), name)
        cap = capture(frame_dir, truth_path, timeout)
        if not cap["ok"]:
            return {"ok": False, "error": "抓帧失败 %s" % name,
                    "moves": moves, "capture": cap}

    import block_classes
    import graspnet_candidates as gc
    named_pools = []
    for name in views:
        d = os.path.join(out_dir, scene.replace(":", "_"), name)
        frame = gc.load_frame(d)
        insts, _tz = gc.find_blocks(frame, split=True,
                                    hue_table=block_classes.SIM_HUE_DEG)
        named_pools.append((name, insts))
    res = eval_scene(scene, truth, named_pools)
    res.update(ok=True, scene=scene, truth_path=truth_path, moves=moves)
    return res


def render(rows):
    out = ["| 场景 | 臂 | 真值(区/全) | 检出(区/全) | 区内漏块 | 遮挡块检出 "
           "| 实例 | 命令 | 指定 top-1 | 误选 |",
           "|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        for arm_name, v in ((PICK_VIEW, r["views"][PICK_VIEW]),
                            ("合并", r["merged"])):
            sp = v["specify"]
            out.append("| %s | %s | %d/%d | %d/%d | %d | %d/%d | %d | %d | %s "
                       "| %d |" % (
                           r["label"], arm_name, v["n_truth"],
                           v["n_truth_world"], v["n_detected"],
                           v["n_detected_world"],
                           v["n_truth"] - v["n_detected"],
                           v["occluded_detected"], v["occluded_total"],
                           v["n_instances"], sp["n_commands"],
                           "—" if sp["id_rate"] is None
                           else "%.1f%%" % (100 * sp["id_rate"]),
                           sp["misbind"]))
    return "\n".join(out)


def finish(args, results, views):
    good = [r for r in results if r.get("ok")]
    if good:
        print("\n## 结果\n")
        print(render(good))
        rec = sum(len(r["occluded_miss"]["recovered"]) for r in good)
        miss_s = sum(len(r["occluded_miss"]["single_missing_world"])
                     for r in good)
        miss_m = sum(len(r["occluded_miss"]["merged_missing_world"])
                     for r in good)
        print("\n单机位漏、三机位补回：%d 块" % rec)
        print("全桌漏块：单机位 %d 块 → 合并 %d 块" % (miss_s, miss_m))
    report = {"kind": "stage4-v1", "scene": args.scene, "views": views,
              "match_mm": MATCH_MM, "dedupe_mm": DEDUPE_MM,
              "results": results,
              "note": "感知级；端到端成功率未跑（无 v0.6 腕部 e2e 流水线）"}
    with open(os.path.join(args.out, "v1.json"), "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=1)
    print("\n→ %s" % os.path.join(args.out, "v1.json"))
    return 0 if good and len(good) == len(results) else 1


def eval_saved(args):
    """已落盘帧重评（不加仿真）。"""
    import block_classes
    import graspnet_candidates as gc
    views = [v.strip() for v in args.views.split(",") if v.strip()]
    results = []
    for scene in args.scene:
        sd = os.path.join(args.out, scene.replace(":", "_"))
        truth, named = None, []
        for name in views:
            d = os.path.join(sd, name)
            if not os.path.isdir(d):
                print("缺帧目录：%s" % d, file=sys.stderr)
                return 2
            if truth is None:
                with open(os.path.join(d, "truth.json"),
                          encoding="utf-8") as f:
                    truth = json.load(f)
            insts, _tz = gc.find_blocks(
                gc.load_frame(d), split=True,
                hue_table=block_classes.SIM_HUE_DEG)
            named.append((name, insts))
        r = eval_scene(scene, truth, named)
        r.update(label=scene, scene=scene, ok=True)
        results.append(r)
        print("\n== %s ==" % scene, flush=True)
    return finish(args, results, views)


def main(argv=None):
    ap = argparse.ArgumentParser(description="V1 仿真：单机位 vs 三机位")
    ap.add_argument("--scene", action="append", required=True,
                    help="BATCH:INDEX，可重复，如 S3:0")
    ap.add_argument("--out", required=True)
    ap.add_argument("--urdf", default="/tmp/ur5.urdf")
    ap.add_argument("--timeout", type=float, default=180.0,
                    help="单帧抓取超时（仿真彩色流很慢，默认 180 s）")
    ap.add_argument("--views", default="pick,free,pad")
    ap.add_argument("--attach", action="store_true",
                    help="栈已起，不自己 launch（同一场景）")
    ap.add_argument("--from-frames", action="store_true",
                    help="不起仿真：直接读 <out>/<scene>/<view>/ 已落盘的帧重评"
                         "（改了统计口径后重算用）")
    args = ap.parse_args(argv)

    if args.from_frames:
        return eval_saved(args)

    views = [v.strip() for v in args.views.split(",") if v.strip()]
    for v in views:
        if v not in [x["name"] for x in VIEWS]:
            ap.error("未知机位 %r" % v)
    if not os.path.exists(args.urdf):
        print("缺 URDF：%s（先起一次仿真，launch 会写 /tmp/ur5.urdf）"
              % args.urdf, file=sys.stderr)
        return 2
    os.makedirs(args.out, exist_ok=True)

    import urkin
    chain = urkin.Chain(args.urdf, root="base_link", tip="tool0")
    arm = Arm()
    results = []
    try:
        for scene in args.scene:
            print("\n== %s ==" % scene, flush=True)
            proc = log = None
            try:
                if not args.attach:
                    proc, log = launch_sim(
                        scene, os.path.join(args.out, scene.replace(":", "_")
                                            + ".launch.log"))
                try:
                    r = run_scene(arm, chain, scene, args.out, args.urdf,
                                  args.timeout, views)
                except Exception as e:                # 单场失败不打断整批
                    r = {"ok": False,
                         "error": "%s: %s" % (type(e).__name__, e)}
            except Exception as e:                    # 起不来也只废这一场
                r = {"ok": False, "error": "%s: %s" % (type(e).__name__, e)}
            finally:
                if proc is not None:
                    stop_sim(proc)
                    log.close()
                    time.sleep(4.0)
            r["label"] = scene
            results.append(r)
            print("  ok=%s %s" % (r.get("ok"), r.get("error", "")), flush=True)
    finally:
        arm.shutdown()

    return finish(args, results, views)


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""一轮精密抓取的编排：**板子抓帧 / 开发机算 / 板子执行**，两者之间只走 SSH + 文件。

分工不这么分不行（不是偏好）：
  · 板上没有 torch / MinkowskiEngine / graspness 权重，也没有外网 ⇒ GraspNet 只能在开发机跑；
  · 开发机 WSL 的**跨进程 DDS 发现是坏的**（同进程 29633 条 / 跨进程 0 条）⇒ 开发机不能当 ROS 节点。
所以板子只做「抓帧」和「执行」，中间所有计算都在开发机，接缝是文件。

一个循环（`--cycle`）的动作顺序：

    [巡检位 pick]  → 抓帧 → GraspNet 候选 → 选块 → 生成计划
        → （--go 才执行）approach → grasp(闭合) → lift → to_slot
          → place_approach → place(张开) → retreat → [巡检位 place]
        → 抓帧 → 量放置误差

`--go` 之前的每一步都是只读的。**执行一次也不自动重试**：任何一条判红就停在原地，
把失败航点与日志留给现场判断。

用法：
  # 0) 准备（推脚本、写场景）
  python3 precision_round.py --run-dir RUN --urdf U --scene RUN/scene.json --push
  # 1) 只到计划为止（不动臂）
  python3 precision_round.py --run-dir RUN --urdf U --cycle
  # 2) 真跑一个循环
  python3 precision_round.py --run-dir RUN --urdf U --cycle --go
  # 3) 回巡检位（单独一步）
  python3 precision_round.py --run-dir RUN --urdf U --survey place --go
"""
import argparse
import hashlib
import json
import os
import shlex
import subprocess
import sys
import time

import numpy as np

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
# 槽位路由是从 ROS 包里借来的**纯函数**（一个文件、不 import rclpy），
# 不复制一份到 scripts/ —— 复制出来的那份迟早和包里那份不一样。
sys.path.insert(0, os.path.join(HERE, "..", "ros2_ws", "src", "grasp_planner"))

from grasp_planner.slot_routing import pick_slot                # noqa: E402

PY_SELF = sys.executable
PY_GRASPNET = os.path.expanduser("~/robotics/venvs/graspnet/bin/python")
BOARD_DIR = "~/precision"
BOARD_SCRATCH = "/tmp/precision"
BOARD_PY_FILES = ("precision_capture.py", "precision_exec.py")
RUN_SH = """#!/bin/bash
# 板上入口：把所有 ROS 环境一次性装好。ROS_DOMAIN_ID=99 是现场值。
# ⚠️ 必须是 bash —— Ubuntu 的 /bin/sh 是 dash，source 不了 setup.bash
#    （报 "Bad substitution" / "builtin: not found"）。
export ROS_DOMAIN_ID=99
. /opt/ros/humble/setup.bash
. "$HOME/yolo_ros_original/ros2_ws/install/setup.bash"
exec python3 "$@"
"""
# 拾取区（米）。y_min 从 0.22 收到 0.25 是**实测**定的：巡检位地面足迹只到
# y=225.0 mm（2026-10-25，survey_footprint.py），0.22 那一角在画面外 15.5 px。
# 板上 yaml 写的 pick_zone 是 y≥0.00 —— 那段相机根本看不见，照抄就会悄悄漏块。
PICK_ZONE = (0.38, 0.62, 0.25, 0.45)     # x_min, x_max, y_min, y_max（米）
MAX_CAND_MM = 12.0


def sh(cmd, quiet=False, check=True):
    if not quiet:
        print("  $ %s" % " ".join(shlex.quote(c) for c in cmd))
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.stdout.strip() and not quiet:
        for line in r.stdout.rstrip().splitlines():
            print("    " + line)
    if r.returncode != 0:
        if r.stderr.strip():
            print("    !! " + r.stderr.strip().replace("\n", "\n    !! "))
        if check:
            raise SystemExit("命令失败（%d）：%s" % (r.returncode, " ".join(cmd)))
    return r


class Board:
    def __init__(self, alias, run_dir):
        self.alias = alias
        self.run_dir = run_dir

    def ssh(self, remote_cmd, quiet=False, check=True):
        return sh(["ssh", self.alias, remote_cmd], quiet=quiet, check=check)

    def push(self):
        """推板上要用的两个脚本 + run.sh，并核对 md5（推没推上去要能证明）。"""
        print("推到板上 %s/" % BOARD_DIR)
        self.ssh("mkdir -p %s/plans" % BOARD_DIR, quiet=True)
        for f in BOARD_PY_FILES:
            src = os.path.join(HERE, f)
            sh(["scp", "-q", src, "%s:%s/" % (self.alias, BOARD_DIR)], quiet=True)
        tmp = os.path.join(self.run_dir, "run.sh")
        open(tmp, "w").write(RUN_SH)
        os.chmod(tmp, 0o755)
        sh(["scp", "-q", tmp, "%s:%s/run.sh" % (self.alias, BOARD_DIR)], quiet=True)
        self.ssh("chmod +x %s/run.sh" % BOARD_DIR, quiet=True)
        for f in BOARD_PY_FILES + ("run.sh",):
            local = hashlib.md5(open(os.path.join(HERE, f) if f != "run.sh" else tmp,
                                     "rb").read()).hexdigest()
            r = self.ssh("md5sum %s/%s" % (BOARD_DIR, f), quiet=True)
            remote = r.stdout.split()[0] if r.stdout.strip() else "?"
            print("  %-22s %s %s" % (f, local[:12],
                                     "一致" if local == remote else "**不一致（%s）**" % remote[:12]))
            if local != remote:
                raise SystemExit("推送后 md5 不一致：%s" % f)

    def joints(self, dest_local):
        """取当前关节角到 dest_local/joint_states.json（只读 /joint_states）。"""
        remote = BOARD_SCRATCH + "/joints"
        self.ssh("mkdir -p %s" % remote, quiet=True)
        self.ssh("%s/run.sh %s/precision_capture.py %s --joints-only --timeout 20"
                 % (BOARD_DIR, BOARD_DIR, remote), quiet=True)
        os.makedirs(dest_local, exist_ok=True)
        sh(["scp", "-q", "%s:%s/joint_states.json" % (self.alias, remote),
            dest_local + "/"], quiet=True)
        return json.load(open(os.path.join(dest_local, "joint_states.json")))

    def capture(self, dest_local, name, timeout=40):
        """板子上抓一帧，拉回 dest_local。板上路径固定，不把开发机的路径传过去。"""
        remote = "%s/frames/%s" % (BOARD_SCRATCH, name)
        self.ssh("mkdir -p %s" % remote, quiet=True)
        self.ssh("%s/run.sh %s/precision_capture.py %s --timeout %s"
                 % (BOARD_DIR, BOARD_DIR, remote, timeout))
        os.makedirs(dest_local, exist_ok=True)
        sh(["scp", "-q", "%s:%s/*" % (self.alias, remote), dest_local + "/"], quiet=True)
        need = ("color.png", "depth.png", "meta.json", "tf.json", "joint_states.json")
        miss = [f for f in need if not os.path.exists(os.path.join(dest_local, f))]
        if miss:
            raise SystemExit("拉回来的帧缺 %s" % miss)

    def exec_plan(self, plan_local, log_local, go, upto=None):
        name = os.path.basename(plan_local)
        remote_plan = "%s/plans/%s" % (BOARD_DIR, name)
        sh(["scp", "-q", plan_local, "%s:%s" % (self.alias, remote_plan)], quiet=True)
        args = "--plan %s" % remote_plan
        if upto:
            args += " --upto %s" % upto
        args += " --go" if go else " --list"
        # 执行器的 JSONL 写在计划旁边；拉回来另存一份到 cycles/NN/
        remote_log = os.path.splitext(remote_plan)[0] + ".jsonl"
        self.ssh("rm -f %s" % remote_log, quiet=True)
        r = self.ssh("%s/run.sh %s/precision_exec.py %s" % (BOARD_DIR, BOARD_DIR, args),
                     check=False)
        if os.path.exists(log_local):
            os.remove(log_local)
        # --list 预演时执行器根本不写日志，没有可拉的东西 —— 别报成"!!"让人以为坏了
        if go:
            sh(["scp", "-q", "%s:%s" % (self.alias, remote_log), log_local],
               quiet=True, check=False)
        return r.returncode


# ---------------------------------------------------------------- 场景与状态
def load_state(path):
    if os.path.exists(path):
        return json.load(open(path))
    return {"slots_mm": [], "slot_classes": [], "used": [], "cycles": [],
            "stop_reason": None, "pose": "unknown"}


def save_state(path, state):
    json.dump(state, open(path, "w"), ensure_ascii=False, indent=1)


def parse_slots(text):
    out = []
    for part in (text or "").split(";"):
        if part.strip():
            x, y = [float(v) for v in part.split(",")]
            out.append([x * 1000 if abs(x) < 10 else x, y * 1000 if abs(y) < 10 else y])
    return out


def pick_next(analysis, state):
    """选一块抓：类别还剩空槽的优先；同类里候选离块心最近的优先。

    不按 GraspNet 分数选 —— 分数没标定过（round 144 的高分落在平板上）。
    """
    x0, x1, y0, y1 = PICK_ZONE
    used, slot_classes = state["used"], state["slot_classes"]

    def room(cls):
        return sum(1 for i, u in enumerate(used)
                   if not u and (not slot_classes[i] or slot_classes[i] == cls))

    cands = []
    for p in analysis["per_block"]:
        if not p["best"] or p["dist_mm"] > MAX_CAND_MM:
            continue
        b = next(x for x in analysis["blocks"] if x["idx"] == p["block_idx"])
        if not (x0 * 1000 <= b["cx"] <= x1 * 1000 and y0 * 1000 <= b["cy"] <= y1 * 1000):
            continue
        r = room(p["class"])
        if r <= 0:
            continue
        cands.append({"p": p, "block": b, "room": r})
    if not cands:
        return None
    cands.sort(key=lambda c: (c["room"], c["p"]["dist_mm"]))
    return cands[0]


# ---------------------------------------------------------------- 步骤
def frame_lighting(frame_dir):
    """彩色帧够不够亮到能判色。返回 (ok, 诊断行)。

    为什么要在候选之前单独查：2026-10-25 20:25 那帧彩色均值 112.6，13 分钟后
    同一台臂同一批块降到 6.4（顶面 RGB 只有 [10,2,2]）—— 顶面像素的 V 全低于
    判色阈 40，于是每块都变成 ``class="unknown"`` 且**票数为 0**。几何（深度是
    结构光打的）照常出块，所以整个链看起来没坏，只是类别全丢；再往后会以
    「类别 unknown 没有空槽」这个名字莫名其妙地收工。
    """
    from PIL import Image               # robot-pose venv 自带
    a = np.asarray(Image.open(os.path.join(frame_dir, "color.png")).convert("RGB"))
    p99 = float(np.percentile(a.max(axis=2), 99))
    ok = p99 >= 60                      # 判色阈是 v≥40 / s≥60，留一档余量
    return ok, "彩色帧 p99 亮度 %.0f（阈 60）" % p99


def do_survey(args, board, state, which):
    tmp = os.path.join(args.run_dir, "tmp")
    os.makedirs(tmp, exist_ok=True)
    board.joints(tmp)
    plan = os.path.join(args.run_dir, "plans", "survey_%s.json" % which)
    os.makedirs(os.path.dirname(plan), exist_ok=True)
    sh([PY_SELF, os.path.join(HERE, "precision_plan.py"), "--survey", which,
        "--urdf", args.urdf, "--from-frame", tmp, "--out", plan])
    rc = board.exec_plan(plan, os.path.join(args.run_dir, "plans",
                                            "survey_%s.jsonl" % which), args.go)
    state["pose"] = "survey_" + which if (args.go and rc == 0) else state.get("pose")
    save_state(args.state, state)
    return rc


def do_cycle(args, board, state):
    n = len(state["cycles"]) + 1
    cdir = os.path.join(args.run_dir, "cycles", "%02d" % n)
    os.makedirs(cdir, exist_ok=True)
    os.makedirs(os.path.join(args.run_dir, "frames"), exist_ok=True)
    if not state["slots_mm"]:
        raise SystemExit("场景里没有槽位 —— 先写 --scene（slots_mm + slot_classes）")

    # 1) 回拾取巡检位（每一步都从「已知位姿」开始，不靠上一步恰好停对）
    rc = do_survey(args, board, state, "pick")
    if rc != 0:
        state["stop_reason"] = "走到拾取巡检位失败（rc=%d）" % rc
        save_state(args.state, state)
        return rc

    # 2) 抓帧
    if args.frame_dir:
        # 复用已抓的帧：**执行的就是刚才给人看过的那份计划**。
        # 不加这条的话，--go 会重新抓帧、重新推理，可能选中另一块 ——
        # 那样「批准的计划」和「真正跑的计划」不是同一个东西。
        fdir = os.path.abspath(args.frame_dir)
        print("  复用帧 %s（不重新抓帧）" % fdir)
    else:
        fdir = os.path.join(args.run_dir, "frames", "pick-%02d" % n)
        board.capture(fdir, "pick-%02d" % n)
    # 没 --go 时臂**根本没去**巡检位，帧来自当前位姿 —— 不能记成 survey_pick。
    if args.go:
        state["pose"] = "survey_pick"

    # 2b) 视野闸门：拾取区四角必须落在画面内（含 40 px 边距）。
    # 阈值不是「按理说看得见」算的 —— 是逐帧把四角投到地面反投影回去算的。
    # 没 --go 时臂不在巡检位，这个判据没有意义，所以只记录、不拦。
    fjson = os.path.join(fdir, "footprint.json")
    rf = sh([PY_SELF, os.path.join(HERE, "survey_footprint.py"),
             "--frame", fdir, "--zone", ",".join("%.3f" % v for v in PICK_ZONE),
             "--json", fjson], check=False)
    foot_ok = None
    if os.path.exists(fjson):
        foot_ok = json.load(open(fjson)).get("ok")
    if args.go and rf.returncode != 0:
        state["stop_reason"] = "拾取区出了画面（survey_footprint rc=%d）" % rf.returncode
        save_state(args.state, state)
        print("\n!! %s —— 调整巡检位或缩小 PICK_ZONE，不发轨迹。" % state["stop_reason"])
        return rf.returncode

    # 2c) 光照闸门：太暗就停下来开灯，别让类别一路变成 unknown 再以别的名义收工
    light_ok, light_note = frame_lighting(fdir)
    print("  " + light_note + ("  ✓" if light_ok else "  ✗ **太暗**"))
    if not light_ok:
        state["stop_reason"] = "彩色帧太暗：%s —— 开灯后重跑本步" % light_note
        save_state(args.state, state)
        print("\n!! %s" % state["stop_reason"])
        return 3

    # 3) 候选
    cand_json = os.path.join(fdir, "candidates.json")
    sh([PY_GRASPNET, os.path.join(HERE, "graspnet_candidates.py"), fdir,
        "--json", cand_json])
    analysis = json.load(open(cand_json))
    choice = pick_next(analysis, state)
    if choice is None:
        state["stop_reason"] = "拾取区没有可抓的块（候选≤%.0f mm 且类别有空槽）" % MAX_CAND_MM
        save_state(args.state, state)
        print("收工：%s" % state["stop_reason"])
        return 0

    p, block = choice["p"], choice["block"]
    cls = p["class"]
    # 4) 槽位
    slot_idx = pick_slot(state["used"], state["slot_classes"], cls)
    if slot_idx is None:
        state["stop_reason"] = "类别 %s 没有空槽%s" % (
            cls, "（unknown 多半是太暗或顶面花了 —— 先看 color.png）"
            if cls == "unknown" else "")
        save_state(args.state, state)
        return 0

    # 5) 计划
    plan = os.path.join(cdir, "pick.json")
    sh([PY_SELF, os.path.join(HERE, "precision_plan.py"), "--pick",
        "--candidates", cand_json, "--block", str(p["block_idx"]),
        "--slot", ",".join("%.4f" % (v / 1000) for v in state["slots_mm"][slot_idx]),
        "--urdf", args.urdf, "--from-frame", fdir, "--out", plan])

    rec = {"n": n, "go": bool(args.go), "frame_pick": os.path.relpath(fdir, args.run_dir),
           "frame_pose": state.get("pose"),
           "block_idx": p["block_idx"], "class": cls,
           "block_xy_mm": [round(block["cx"], 2), round(block["cy"], 2)],
           "cand_dist_mm": p["dist_mm"], "cand_score": p["best"]["score"],
           "cand_width_mm": p["best"]["width_mm"],
           "slot_idx": slot_idx, "plan": os.path.relpath(plan, args.run_dir),
           "foot_ok": foot_ok, "ok": False}
    # 抓帧时臂是不是静止的 —— 不是的话，位置精度那一栏的读数不可信
    fjs = json.load(open(os.path.join(fdir, "joint_states.json")))
    rec["frame_still"] = fjs.get("still")
    rec["frame_p2p_deg"] = fjs.get("joint_p2p_deg")
    rec["frame_gap_ms"] = fjs.get("gap_ms")

    # 6) 执行（--go 才真动）
    log = os.path.join(cdir, "exec.jsonl")
    rc = board.exec_plan(plan, log, args.go)
    rec["exec_rc"] = rc
    rec["exec_log"] = os.path.relpath(log, args.run_dir) if os.path.exists(log) else None

    if not args.go:
        state["cycles"].append(rec)
        state["stop_reason"] = "预演（未加 --go）：计划已生成，未执行"
        save_state(args.state, state)
        print("\n⇒ 预演到此为止。计划：%s" % plan)
        print("   确认后加 --go 重跑本步。")
        return 0
    if rc != 0:
        state["stop_reason"] = "执行判红（rc=%d，见 %s）" % (rc, rec["exec_log"])
        state["cycles"].append(rec)
        save_state(args.state, state)
        print("\n!! 执行判红：%s" % state["stop_reason"])
        return rc

    # 7) 放置巡检帧 + 量误差（臂此刻正停在 survey_place）
    state["pose"] = "survey_place"
    pdir = os.path.join(args.run_dir, "frames", "place-%02d" % n)
    board.capture(pdir, "place-%02d" % n)
    from precision_verify import measure_placements
    meas = measure_placements(pdir, state["slots_mm"])
    json.dump(meas, open(os.path.join(cdir, "place_measure.json"), "w"),
              ensure_ascii=False, indent=1)
    slot_rec = next((s for s in meas["slots"] if s["slot_idx"] == slot_idx), None)
    rec["place_frame"] = os.path.relpath(pdir, args.run_dir)
    rec["place_dist_mm"] = slot_rec["dist_mm"] if slot_rec else None
    rec["place_dx_mm"] = slot_rec["dx_mm"] if slot_rec else None
    rec["place_dy_mm"] = slot_rec["dy_mm"] if slot_rec else None
    rec["placed_class_colour"] = (slot_rec or {}).get("block", {}).get("class") \
        if slot_rec and slot_rec["block"] else None

    from precision_verify import summarize_exec_log
    ls = summarize_exec_log(log)
    rec.update(close_mm=ls["close_mm"], lift_mm=ls["lift_mm"], slipped=ls["slipped"])
    empty = ls["close_mm"] is not None and ls["close_mm"] <= 5.0
    rec["ok"] = bool(ls["traj_ok"] and ls["gate_all_ok"] and not empty
                     and ls["slipped"] is not True and rec["place_dist_mm"] is not None
                     and rec["frame_still"] is not False)
    if rec["ok"]:
        state["used"][slot_idx] = True
    else:
        state["stop_reason"] = ("循环 %d 判红（空夹=%s 滑移=%s 轨迹=%s 闸门=%s 放置=%s）"
                                % (n, empty, ls["slipped"], ls["traj_ok"],
                                   ls["gate_all_ok"], rec["place_dist_mm"]))
    state["cycles"].append(rec)
    if not state["stop_reason"] and all(state["used"]):
        state["stop_reason"] = "槽位用尽（%d 个）" % len(state["used"])
    save_state(args.state, state)

    print("\n循环 %d：%s %s → 槽 %d，候选偏 %.1f mm，放置偏 %s mm，闭合回读 %s mm"
          % (n, cls, rec["block_xy_mm"], slot_idx, rec["cand_dist_mm"],
             rec["place_dist_mm"], rec["close_mm"]))
    return 0 if rec["ok"] else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--urdf", default=None)
    ap.add_argument("--board", default="jetson-hotspot")
    ap.add_argument("--scene", default=None, help="场景 JSON（槽位 + 类别 + 摆放说明）")
    ap.add_argument("--slots", default=None, help="临时给槽位：x,y;x,y（米）")
    ap.add_argument("--slot-classes", default=None, help="逗号分隔，与槽位等长；空串=通配")
    ap.add_argument("--go", action="store_true", help="**真的执行轨迹**")
    ap.add_argument("--push", action="store_true", help="只推脚本并核对 md5")
    ap.add_argument("--survey", choices=["pick", "place"], default=None)
    ap.add_argument("--cycle", action="store_true")
    ap.add_argument("--frame-dir", default=None,
                    help="复用已抓的帧目录（执行的就是预览过的那份计划）")
    args = ap.parse_args()

    os.makedirs(args.run_dir, exist_ok=True)
    args.state = os.path.join(args.run_dir, "round.json")
    state = load_state(args.state)
    if args.scene:
        sc = json.load(open(args.scene))
        state["slots_mm"] = sc.get("slots_mm") or state.get("slots_mm", [])
        state["slot_classes"] = sc.get("slot_classes") or \
            [""] * len(state["slots_mm"])
        state["scene_note"] = sc.get("note")
    if args.slots:
        state["slots_mm"] = parse_slots(args.slots)
        state["slot_classes"] = ([c.strip() for c in args.slot_classes.split(",")]
                                 if args.slot_classes else [""] * len(state["slots_mm"]))
    if state.get("slots_mm") and not state.get("slot_classes"):
        state["slot_classes"] = [""] * len(state["slots_mm"])
    if state.get("slot_classes") and len(state["slot_classes"]) != len(state["slots_mm"]):
        raise SystemExit("slot_classes 与槽位数不等长")
    n_slots = len(state.get("slots_mm", []))
    if len(state.get("used", [])) != n_slots:
        state["used"] = [False] * n_slots
    save_state(args.state, state)

    board = Board(args.board, args.run_dir)
    board.push()

    if args.push:
        print("槽位：%s" % state["slots_mm"])
        print("槽位类别：%s" % state["slot_classes"])
        return 0
    if args.survey:
        return do_survey(args, board, state, args.survey)
    if args.cycle:
        if not args.urdf:
            raise SystemExit("--cycle 要 --urdf")
        return do_cycle(args, board, state)
    ap.error("要 --push / --survey / --cycle 之一")


if __name__ == "__main__":
    sys.exit(main())

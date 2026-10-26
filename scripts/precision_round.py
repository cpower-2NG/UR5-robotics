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

加 `--held-capture` 时执行被切成两段：跑到 lift 停住、抓一帧（块还夹在爪里）、
再从下一条航点接着跑。这一帧本来要给 `measure_held.py` 用 —— 分开「抓的时候就没抓正」
与「抓完到放下之间滑了」。**2026-10-26 实测这条路走不通**：腕部相机偏 tool0 侧面
91 mm、低 24.2 mm、光轴竖直，夹着的块离它只有约 161 mm 竖直，29° 偏轴、落在画面外
（见 `measure_held.py` 的说明）。所以这个开关现在**直接拒绝**，不再白跑一段动作。

`--resume` 是通用的续跑：从本地执行日志里最后一个「到位」的航点之后接着跑；
`--finish` 是只读收尾：不动机器人，只补抓放置巡检帧、量误差、写 state ——
收尾崩了（比如上一次是量夹持帧把进程打死）用它对账，不用重跑一轮。

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
BOARD_PY_FILES = ("precision_capture.py", "precision_exec.py",
                  "gripper_action.py", "precision_grip_check.py")
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
# 夹持帧（在 lift 停一下抓一帧）**作废**：腕部相机看不到夹着的块，理由与实测见
# `measure_held.py` 文件头。留着常量只为了让 --held-capture 的拒绝信息能引用它。
HELD_CAPTURE_WAYPOINT = "lift"


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

    def exec_plan(self, plan_local, log_local, go, upto=None, from_=None,
                  append=False, reset_remote=False):
        """跑一份计划。``upto``/``from_`` 把一条计划切成两段跑（例如抓住块之后
        停一下抓帧看它夹在哪，再继续去槽位）。

        ``append=True`` 表示「这是同一份计划的第二段」：板上 JSONL 不清空，
        拉回来时只取本地还没有的那些行 —— 两段合起来仍是一份完整日志，
        否则第二段会把第一段的闭合/抬升回读覆盖掉，滑移判据就没了。
        ``reset_remote=True`` 只在**续跑**时用：板上的日志里有上一次被闸门
        拒绝的那几行（一条轨迹都没发），留着会把整轮判成「闸门没过」。
        """
        name = os.path.basename(plan_local)
        remote_plan = "%s/plans/%s" % (BOARD_DIR, name)
        sh(["scp", "-q", plan_local, "%s:%s" % (self.alias, remote_plan)], quiet=True)
        args = "--plan %s" % remote_plan
        if upto:
            args += " --upto %s" % upto
        if from_:
            args += " --from %s" % from_
        args += " --go" if go else " --list"
        # 执行器的 JSONL 写在计划旁边；拉回来另存一份到 cycles/NN/
        remote_log = os.path.splitext(remote_plan)[0] + ".jsonl"
        if not append or reset_remote:
            self.ssh("rm -f %s" % remote_log, quiet=True)
        r = self.ssh("%s/run.sh %s/precision_exec.py %s" % (BOARD_DIR, BOARD_DIR, args),
                     check=False)
        # --list 预演时执行器根本不写日志，没有可拉的东西 —— 别报成"!!"让人以为坏了
        if go:
            self._pull_log(remote_log, log_local, append,
                           remote_has_prefix=not reset_remote)
        return r.returncode

    def _pull_log(self, remote_log, log_local, append, remote_has_prefix=True):
        if append and os.path.exists(log_local):
            tmp = log_local + ".part"
            sh(["scp", "-q", "%s:%s" % (self.alias, remote_log), tmp],
               quiet=True, check=False)
            if not os.path.exists(tmp):
                return
            with open(tmp, encoding="utf-8") as f:
                new = f.readlines()
            os.remove(tmp)
            with open(log_local, encoding="utf-8") as f:
                have = len(f.readlines())
            # remote_has_prefix: 板上是「第一段 + 第二段」，只取本地没有的那截；
            # 续跑时板上被清过，只有第二段的行 —— 那就要**全部**追加，
            # 按 have 去切会把第二段自己的前 have 行扔掉。
            tail = new[have:] if remote_has_prefix else new
            if tail:
                with open(log_local, "a", encoding="utf-8") as f:
                    f.writelines(tail)
            return
        if os.path.exists(log_local):
            os.remove(log_local)
        sh(["scp", "-q", "%s:%s" % (self.alias, remote_log), log_local],
           quiet=True, check=False)


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


def next_waypoint(plan, after):
    """计划里 ``after`` 之后的**那一条**航点。

    必须从计划里读，不能把名字写死：`precision_plan.split_jumps` 会在关节跳变
    超过 1 rad 时插中间点（这一轮就插出了 `to_slot_split1`）。
    2026-10-26 第 5 轮把名字写死成 `to_slot`，第二段因此**跳过了**那个插入点，
    闸门基准对不上（34.08°）被拒 —— 拒是对的，但一轮抓放白停了：臂现在夹着块
    悬在 lift。
    """
    names = [w["name"] for w in plan["waypoints"]]
    if after not in names:
        raise SystemExit("计划里没有航点 %r（有 %s）" % (after, names))
    i = names.index(after) + 1
    if i >= len(names):
        raise SystemExit("%r 已是最后一条航点，后面没有可跑的" % after)
    return names[i]


def do_cycle(args, board, state):
    n = len(state["cycles"]) + 1
    # 上一轮的红字不能挂在新一轮头上：人在场、改了东西、重新开一轮，就是新的判决。
    state["stop_reason"] = None
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

    return finish_after_exec(args, board, state, rec, cdir, plan, log, n,
                             slot_idx, cls)


def last_arrived(log_local):
    """本地执行日志里**最后一个「到位」的航点**；没有就返回 None。

    续跑要从这里读，不能写死航点名：计划会插中间航点（`to_slot_split1`），
    写死 `to_slot` 会在闸门那里被判 34° 偏差（2026-10-26 第 5 轮）。
    `arrive` 事件带航点名，是最权威的「臂真的到了」的记录 —— `traj ok` 只说明
    控制器收下了轨迹，`arrive` 才是实测到位。
    """
    last = None
    if log_local and os.path.exists(log_local):
        for line in open(log_local):
            line = line.strip()
            if not line:
                continue
            try:
                r = json.loads(line)
            except ValueError:          # 半截行（scp 中途断掉）不算
                continue
            if r.get("event") == "arrive" and r.get("waypoint"):
                last = r["waypoint"]
    return last


def do_resume(args, board, state):
    """把最后一个循环从「最后一条到位的航点」之后接着跑完。

    什么时候用：一段轨迹没跑成/被打断，臂停在某个航点（例如夹着块停在 lift）。
    这时**不能**重跑整个循环 —— 那会把夹着块的手重新下探到台面。

    板上的日志会先清掉：里面留着上一次被拒的那几行，会把这轮判成「闸门没过」。
    前一段的记录已经在本地 `cycles/NN/exec.jsonl` 里了，不会丢。
    """
    if not state["cycles"]:
        print("!! round.json 里没有可续跑的循环")
        return 2
    rec = state["cycles"].pop()          # 先摘下来，跑完由 finish_after_exec 放回
    old_rc = rec.get("exec_rc")          # 上一次的判决 —— 预演时原样放回，别抹掉
    n = rec["n"]
    cdir = os.path.join(args.run_dir, "cycles", "%02d" % n)
    plan = os.path.join(args.run_dir, rec["plan"])
    log = os.path.join(cdir, "exec.jsonl")
    done = last_arrived(log)
    if done is None:
        print("!! %s 里没有「到位」的航点 —— 不知道臂停在哪，不能瞎续" % log)
        state["cycles"].append(rec)
        return 2
    nxt = next_waypoint(json.load(open(plan)), done)
    rec["resumed_from"] = nxt
    print("续跑循环 %d：最后一个到位的是 %s，接着跑 %s（块 %s，槽 %s）"
          % (n, done, nxt, rec.get("class"), rec.get("slot_idx")))
    rc = board.exec_plan(plan, log, args.go, from_=nxt, append=True,
                         reset_remote=True)
    rec["exec_rc_resume"] = rc
    if not args.go:
        rec["exec_rc"] = old_rc
        state["cycles"].append(rec)
        save_state(args.state, state)
        print("\n⇒ 预演：一条轨迹都没发。真动加 --go。")
        return 0
    if rc != 0:
        rec["exec_rc"] = rc
        state["stop_reason"] = "续跑判红（rc=%d，块可能还夹在爪里）" % rc
        state["cycles"].append(rec)
        save_state(args.state, state)
        print("\n!! 续跑判红：%s" % state["stop_reason"])
        return rc
    rec["exec_rc"] = 0
    state["stop_reason"] = None          # 上一次的红字不算在这一轮头上
    return finish_after_exec(args, board, state, rec, cdir, plan, log, n,
                             rec["slot_idx"], rec["class"])


def check_survey_pose(pdir, plan_path, urdf, tol_deg=2.0):
    """放置巡检帧必须抓在计划的 `survey_place` 位姿上。

    `--finish` 是人工触发的只读收尾：臂要是停在别处（比如中途停下的 lift），
    帧里的块和槽位就不在同一套坐标里，量出来的「放置偏」是假的 —— 宁可不写数。
    """
    wr = next((w for w in json.load(open(plan_path))["waypoints"]
               if w["name"] == "survey_place"), None)
    if wr is None:
        return None
    js = json.load(open(os.path.join(pdir, "joint_states.json")))
    have = dict(zip(js["name"], js["position"]))
    import urkin
    names = urkin.Chain(urdf).move_joints
    dev = max(abs(np.degrees(have[k] - w)) for k, w in zip(names, wr["q"]))
    if dev > tol_deg:
        raise SystemExit(
            "放置巡检帧不是在 survey_place 上抓的（最大差 %.1f°）—— 帧里没有"
            "这一轮的落点。先把臂开回巡检位，再跑 --finish。" % dev)
    return dev


def do_finish(args, board, state):
    """只读收尾：**不动机器人**，只补抓放置巡检帧 + 量误差 + 写 state。

    什么时候用：收尾那一段崩了。2026-10-26 第 5 个循环就是这样 ——
    `measure_held` 抛的是 SystemExit，不属于 Exception，穿过 `except Exception`
    把进程打死：放置帧抓到了、`place_measure.json` 也写了，但 round.json 没更新，
    这一轮仍记成「执行判红」。当时臂正好停在 survey_place，重跑收尾就能对上账。
    """
    if not state["cycles"]:
        print("!! round.json 里没有循环")
        return 2
    if not args.urdf:
        raise SystemExit("--finish 要 --urdf（要核对巡检位姿、算 tool0）")
    rec = state["cycles"].pop()          # 跑完由 finish_after_exec 放回
    n = rec["n"]
    cdir = os.path.join(args.run_dir, "cycles", "%02d" % n)
    plan = os.path.join(args.run_dir, rec["plan"])
    log = os.path.join(cdir, "exec.jsonl")
    print("收尾循环 %d：只抓巡检帧 + 量误差，**一条轨迹都不发**" % n)
    # 上一次的红字说的是「那一段没跑成」；那一段后来续跑成功了，红字就该下账。
    # 这一轮要是真判红，finish_after_exec 会重新写一条。
    state["stop_reason"] = None
    return finish_after_exec(args, board, state, rec, cdir, plan, log, n,
                             rec["slot_idx"], rec["class"])


def finish_after_exec(args, board, state, rec, cdir, plan, log, n, slot_idx, cls):
    """轨迹跑完之后的部分：放置巡检帧 → 量放置误差 → 量夹持帧 → 判红 → 记 state。

    单独抽出来是为了 `--resume`：第 5 轮两段跑的第二段被闸门拒了（我在
    `--from` 上写死了航点名），臂夹着块停在 lift；续跑要跑的正是这一段，
    而它在 `do_cycle` 的后半截。
    """
    # 7) 放置巡检帧 + 量误差（臂此刻正停在 survey_place）
    state["pose"] = "survey_place"
    pdir = os.path.join(args.run_dir, "frames", "place-%02d" % n)
    board.capture(pdir, "place-%02d" % n)
    dev = check_survey_pose(pdir, plan, args.urdf)
    if dev is not None:
        print("  巡检帧位姿核对：与计划 survey_place 最大差 %.2f° → OK" % dev)
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

    # 7b) 夹持帧：块**被抓着的时候**离 tool0 轴多远。和上面的放置偏一起看，
    # 才能分开「抓的时候就没抓正」与「抓完到放下之间滑了」。
    if rec.get("held_frame"):
        try:
            from measure_held import measure as measure_held_frame
            hv = measure_held_frame(os.path.join(args.run_dir, rec["held_frame"]),
                                    args.urdf, cls, plan, verbose=False)
            rec["held"] = hv
            print("  夹持帧：块心相对 tool0 (dx, dy) = (%+.1f, %+.1f) mm%s"
                  % (hv["block_in_tool_mm"][0], hv["block_in_tool_mm"][1],
                     "" if "vs_plan_xy_mm" not in hv
                     else "；计划要求 (%+.1f, %+.1f)，超出 |%.1f| mm"
                     % (hv["plan_expected_xy_mm"][0], hv["plan_expected_xy_mm"][1],
                        hv["vs_plan_norm_mm"])))
        except Exception as exc:                              # noqa: BLE001
            rec["held_error"] = "%s: %s" % (type(exc).__name__, exc)
            print("  !! 夹持帧量不出来：%s（帧仍在 %s）"
                  % (rec["held_error"], rec["held_frame"]))

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
    if rec.get("held"):
        h = rec["held"]["block_in_tool_mm"]
        print("        夹持时块心相对 tool0 (%+.1f, %+.1f) mm —— "
              "这一项在放置之前就量到了" % (h[0], h[1]))
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
    ap.add_argument("--held-capture", action="store_true",
                    help="**已作废**：腕部相机看不到夹着的块（见 measure_held.py），"
                         "这个开关现在直接拒绝，不会让臂白跑一段")
    ap.add_argument("--resume", action="store_true",
                    help="续跑最后一个循环：从本地执行日志里最后一个「到位」的航点"
                         "之后接着跑，然后量放置误差。用于一段轨迹没跑成、臂停在半路")
    ap.add_argument("--finish", action="store_true",
                    help="只读收尾最后一个循环：不动机器人，只补抓放置巡检帧、"
                         "量误差、写 state。收尾那段崩掉之后用它补账")
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
    if args.held_capture:
        raise SystemExit(
            "--held-capture 已作废，没有执行任何动作。\n"
            "  腕部相机看不到夹着的块：它偏 tool0 侧面 91 mm、低 24.2 mm、光轴竖直，\n"
            "  夹着块时块离它只有约 161 mm 竖直（29° 偏轴），整个夹爪都在画面下边框外面。\n"
            "  实测 frames/held-05：全帧最近深度点 327 mm，彩色图里只有台面。\n"
            "  这是刚体关系，换哪个位姿都一样。要分开「没抓正/滑了」得换一台看得见夹爪的相机。")
    if args.resume:
        return do_resume(args, board, state)
    if args.finish:
        return do_finish(args, board, state)
    if args.cycle:
        if not args.urdf:
            raise SystemExit("--cycle 要 --urdf")
        return do_cycle(args, board, state)
    ap.error("要 --push / --survey / --cycle / --resume / --finish 之一")


if __name__ == "__main__":
    sys.exit(main())

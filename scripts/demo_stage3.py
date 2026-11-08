#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""阶段 3 一键演示：**开发机上跑，不依赖 AI**。

它只做编排，不复制任何一条算法：

    1. 只读预检        板上三件套 + JTC + dashboard 节点数 + 夹爪 0x0200
    2. 抓垫位帧        stage2_round.py --capture-pose stage2high（动臂，不动夹爪）
    3. 重解垫位        stage2_pad.py → grid-demo.json
    4. 跑一轮 N 层       stage3_round.py --go（自带拆塔、逐层测量、判分）
    5. 人眼复核        verdict=person_review 时再问一次，然后 stage3_score.py 重判

人在环（AGENTS.md §1 红线 6「有交互终端」档）
────────────────────────────────────────────
· 动臂前必须由人在**交互终端**敲 ``yes``；stdin 不是 TTY 就直接拒绝（fail-closed）。
· 夹爪初始化/开合是**另一件事**的当次授权：本脚本**从不写夹爪寄存器**。
  0x0200 不是 1 就停，把初始化命令原样打给人。
· 任何一步判红立即退出，不续跑、不重试（块可能还在爪里或半放）。

用法
────
    bash scripts/demo_stage3.sh                  # 3 层演示（与六轮验收同口径）
    bash scripts/demo_stage3.sh --frames 2       # 更快：每个视点少抓一帧
    bash scripts/demo_stage3.sh --dry-run        # 离线：不连板、不动臂

``--dry-run`` 用上一轮的拾取帧和关节角把三层计划全生成一遍，只验证
运动学/护栏/参数拼装，不发一条轨迹。
"""
import argparse
import json
import os
import re
import shlex
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
PY = sys.executable

BOARD_ALIAS = (os.environ.get("STAGE3_DEMO_BOARD")
               or os.environ.get("STAGE2_BOARD")
               or "jetson-hotspot")

NOTES = os.path.expanduser("~/robotics/notes/ur5-vision-grasp")
FINAL_RUN = os.path.join(NOTES, "runs", "2026-11-04-stage3-final")
DEFAULT = {
    "urdf": os.path.join(NOTES, "runs", "2026-10-25-precision", "robot.urdf"),
    "scene": os.path.join(FINAL_RUN, "scene-final.json"),
    "zcal": os.path.join(NOTES, "runs", "2026-11-01-stage3-entry",
                         "entry-zcal", "zcal.json"),
    "mat": os.path.join(NOTES, "stage2-mat", "stage2-mat.json"),
    "bias": os.path.join(FINAL_RUN, "bias_estimate.json"),
    "grid": os.path.join(FINAL_RUN, "grid-round13.json"),
    "frame": os.path.join(FINAL_RUN, "round-13", "frame-pick"),
    "joints": os.path.join(FINAL_RUN, "round-13", "joints-stage2pick",
                           "joint_states.json"),
}


# ---------------------------------------------------------------- 记账
class Log:
    """终端输出 + 落一份日志（``<run-dir>/demo.log``）。"""

    def __init__(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.path = path
        self.f = open(path, "a", encoding="utf-8")
        self.say("")
        self.say("===== %s =====" % time.strftime("%Y-%m-%d %H:%M:%S"))

    def write(self, text):
        self.f.write(text)
        self.f.flush()

    def say(self, msg=""):
        print(msg, flush=True)
        self.write(msg + "\n")

    def cmd(self, argv, cwd=None, capture=False, timeout=None):
        self.say("  $ " + shlex.join(argv))
        try:
            if capture:
                p = subprocess.run(argv, cwd=cwd, capture_output=True,
                                   text=True, timeout=timeout)
                out = (p.stdout or "") + (p.stderr or "")
                if out:
                    self.write(out)
                    if not out.endswith("\n"):
                        self.write("\n")
                return p.returncode, out
            p = subprocess.Popen(argv, cwd=cwd, stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, text=True,
                                 bufsize=1)
            for line in p.stdout:
                sys.stdout.write(line)
                sys.stdout.flush()
                self.write(line)
            return p.wait(), ""
        except FileNotFoundError as exc:
            self.say("!! 命令不存在：%s" % exc)
            return 127, ""
        except subprocess.TimeoutExpired:
            self.say("!! 命令超时（%s s）" % timeout)
            return 124, ""

    def close(self):
        self.f.close()


def _grab(pattern, text, cast=str):
    m = re.search(pattern, text, re.M)
    if not m:
        return None
    try:
        return cast(m.group(1))
    except (TypeError, ValueError):
        return None


def _mark(ok, name, detail=""):
    return "  [%s] %s%s" % ("通过" if ok else "不通过", name,
                            ("：" + detail) if detail else "")


# ---------------------------------------------------------------- 只读预检
def preflight(log, label="预检"):
    """板上三件套 / JTC / dashboard 节点数 / 夹爪初始化状态。**全部只读。**"""
    log.say("\n== %s（只读：不发轨迹、不写寄存器）==" % label)
    ok = True

    rc, _ = log.cmd(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8",
                     BOARD_ALIAS, "true"], capture=True, timeout=20)
    if rc != 0:
        log.say(_mark(False, "板子可达", "ssh %s 连不上" % BOARD_ALIAS))
        return False
    log.say(_mark(True, "板子可达", BOARD_ALIAS))

    rc, out = log.cmd(
        ["ssh", "-o", "BatchMode=yes", BOARD_ALIAS,
         "~/precision/run.sh ~/precision/preflight_status.py"],
        capture=True, timeout=90)
    rm = _grab(r"^robot_mode\s*=\s*(-?\d+)", out, int)
    sm = _grab(r"^safety_mode\s*=\s*(-?\d+)", out, int)
    ps = _grab(r"^程序状态\s*=\s*'?([A-Za-z_]+)", out)
    jtc_action = bool(re.search(r"^JTC action\s*=\s*在", out, re.M))
    jtc_state = _grab(r"^JTC 控制器\s*=\s*'?([a-z_]+)", out)
    got = rm is not None and sm is not None and ps is not None
    if not got:
        log.say(_mark(False, "状态查询", "preflight_status.py 没给出完整输出"
                      "（rc=%d）" % rc))
        return False

    ok &= _row(log, rm == 7, "robot_mode",
               "%s（要 7 = RUNNING）" % rm)
    ok &= _row(log, sm == 1, "safety_mode",
               "%s（要 1 = NORMAL）" % sm)
    ok &= _row(log, bool(ps) and ps.upper() == "PLAYING",
               "示教器程序", "%r（要 PLAYING —— 去示教器按播放）" % ps)
    ok &= _row(log, jtc_action, "JTC action server", "在" if jtc_action else "不在")
    ok &= _row(log, jtc_state == "active", "JTC 控制器状态", repr(jtc_state))

    rc, out = log.cmd(
        ["ssh", "-o", "BatchMode=yes", BOARD_ALIAS,
         'pgrep -af "/ur_robot_driver/[d]ashboard_client" | wc -l'],
        capture=True, timeout=20)
    n_dash = _grab(r"^\s*(\d+)\s*$", out, int)
    ok &= _row(log, n_dash == 1, "dashboard_client 节点数",
               "%s（要恰好 1；0 = 栈没起，>1 = 有孤儿抢服务）" % n_dash)

    rc, out = log.cmd(
        ["ssh", "-o", "BatchMode=yes", BOARD_ALIAS,
         "~/precision/run.sh ~/precision/grip_status.py"],
        capture=True, timeout=60)
    st = _grab(r"0x0200 初始化状态\s*=\s*'?(\d+|None)'?", out)
    pos = _grab(r"0x0202 开口回读\s*=\s*'?([-\d.]+|None)'?", out)
    ok &= _row(log, st == "1", "夹爪初始化 0x0200",
               "%s（要 1；0=未初始化，断电就丢）" % st)
    if st != "1":
        log.say("   初始化是夹爪写入，要另一次当次授权（急停就绪、两指行程内"
                "无手无遮挡），本脚本不代跑：")
        log.say("   python3 scripts/ag95_init_close.py --go --close")
    else:
        log.say(_mark(True, "夹爪开口回读 0x0202", "%s mm" % pos))

    return bool(ok)


def _row(log, cond, name, detail):
    log.say(_mark(cond, name, detail))
    return bool(cond)


# ---------------------------------------------------------------- 人在环
def ask_yes(log, prompt):
    """红线 6：动臂前由人在交互终端敲 ``yes``；拿不到就不发。"""
    if not sys.stdin.isatty():
        log.say("!! 当前 stdin 不是交互终端 —— 红线 6 要求由人当场敲 yes。")
        log.say("   请在能输入的终端里直接运行本脚本（不要 nohup / 不要重定向）。")
        return False
    try:
        return input(prompt).strip().lower() == "yes"
    except EOFError:
        return False


def authorized(log, n_layers):
    log.say("\n" + "=" * 62)
    log.say(">>> 需要人动手：确认急停就绪、两指整个行程内无手无遮挡。")
    log.say("    接下来会动臂：先走到 stage2high 抓一帧，再跑 %d 层抓放。"
            % n_layers)
    log.say("    塔位若还立着，机器人会先自己逐层拆回拾取区（不需要人拆）。")
    log.say("    判分通过后，机器人会把新建的塔再逐层拆回拾取区，台面还原。")
    log.say("    夹爪开合在执行器里，属同一次当次授权。")
    log.say("=" * 62)
    if not ask_yes(log, "确认无误请输入 yes 并回车： "):
        log.say("!! 没有拿到当次授权 —— 不发任何运动。")
        return False
    return True


def wait_playing(log, attempts=3):
    for i in range(1, attempts + 1):
        if preflight(log, label="动臂前复核 %d/%d" % (i, attempts)):
            return True
        if i == attempts:
            break
        if not sys.stdin.isatty():
            return False
        try:
            input(">>> 需要人动手：去示教器按 PLAY（程序需 PLAYING），"
                  "完成后回车重试： ")
        except EOFError:
            return False
    log.say("!! 复核没通过 —— fail-closed，不发运动。")
    return False


# ---------------------------------------------------------------- 垫位
def capture_and_solve_grid(args, log, rundir):
    frame_dir = os.path.join(rundir, "pad-frame")
    log.say("\n== 第 1 步：走到 stage2high 抓垫位帧（动臂，不动夹爪）==")
    rc, _ = log.cmd(
        [PY, "-u", os.path.join(HERE, "stage2_round.py"),
         "--run-dir", rundir, "--urdf", args.urdf,
         "--capture-pose", "stage2high",
         "--capture-out", frame_dir, "--go"], cwd=HERE)
    if rc != 0:
        log.say("!! 抓垫位帧失败（rc=%d）—— 不发抓取计划。" % rc)
        return None

    log.say("\n== 第 2 步：重解垫位（只读，不动臂）==")
    grid = os.path.join(rundir, "grid-demo.json")
    rc, _ = log.cmd(
        [PY, "-u", os.path.join(HERE, "stage2_pad.py"),
         "--frame", frame_dir, "--mat", args.mat, "--out", grid], cwd=HERE)
    if rc != 0:
        log.say("!! 垫位判红（rc=%d）—— 不动臂，先查现场/垫子。" % rc)
        return None
    try:
        doc = json.load(open(grid))
        log.say("   垫心 base (%.1f, %.1f) mm，yaw %.3f°"
                % (doc["grid_center_base_mm"][0],
                   doc["grid_center_base_mm"][1],
                   doc["grid_yaw_base_deg"]))
    except (OSError, ValueError, KeyError) as exc:
        log.say("!! 读不到刚写出的 grid：%s" % exc)
        return None
    return grid


# ---------------------------------------------------------------- 一轮
def make_scene(args, rundir):
    """scene-final.json + 本次融合帧数（只改这两个键，别的一字不动）。"""
    try:
        raw = json.load(open(args.scene))
    except (OSError, ValueError) as exc:
        raise SystemExit("读不到场景 %s：%s" % (args.scene, exc))
    raw["pick_frames"] = int(args.frames)
    raw["terminal_frames"] = int(args.frames)
    path = os.path.join(rundir, "demo-scene.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(raw, f, ensure_ascii=False, indent=1)
    return path


def bias_seed(path):
    """``<run>/bias_estimate.json`` → ``"bx,by"``；版本不符/坏文件当没有。"""
    try:
        doc = json.load(open(path))
    except (OSError, ValueError):
        return None
    if doc.get("version") != 3:
        return None
    val = doc.get("bias_mm")
    if not (isinstance(val, (list, tuple)) and len(val) == 2):
        return None
    try:
        return "%.3f,%.3f" % (float(val[0]), float(val[1]))
    except (TypeError, ValueError):
        return None


def round_argv(args, rundir, grid, scene, seed):
    argv = [
        PY, "-u", os.path.join(HERE, "stage3_round.py"),
        "--run-dir", rundir,
        "--n-layers", str(args.n_layers),
        "--group", "open", "--arm", "geometric",
        "--place-target", "previous", "--bias-comp", "fixed",
        "--scene", scene, "--urdf", args.urdf,
        "--grid", grid, "--zcal", args.zcal,
        "--nudge-axes", "yaw", "--nudge-open-mm", "40",
        "--nudge-deadband-yaw-deg", "1.0",
        "--nudge-deadband-xy-mm", "1.0",
        "--nudge-lift-mm", "2.0", "--nudge-max-cycles", "2",
        "--place-open-mm", "50", "--place-clearance-mm", "3",
        "--release-dwell-s", "0.5",
        "--d-mm", "331", "--tower-half-edge", "0.020",
        "--terminal-wait-s", "10",
    ]
    if seed:
        # 负值必须用 `=` 形式，否则 argparse 把它当选项。
        argv.append("--bias-seed=" + seed)
    if args.dry_run:
        argv += ["--frame", args.frame, "--joints", args.joints]
    else:
        argv.append("--go")
    return argv


def teardown_argv(args, rundir, grid, scene, seed):
    """判分之后的收尾动作：同一套轮次参数 + ``--teardown-only``（仍带 --go）。"""
    return round_argv(args, rundir, grid, scene, seed) + ["--teardown-only"]


def teardown_after(args, log, rundir, grid, scene, seed):
    """判分通过后把塔拆回拾取区 —— 一次演示以台面还原结束。"""
    log.say("\n== 第 4 步：拆塔回拾取区（动臂）==")
    rc, _ = log.cmd(teardown_argv(args, rundir, grid, scene, seed), cwd=HERE)
    if rc != 0:
        log.say("!! 拆塔没走完（rc=%d）—— 塔位可能还有块，先看现场；"
                "下次 --go 开工也会先自动拆。" % rc)
    else:
        log.say("拆塔完成：塔位已空，块回拾取区。")
    return rc


def round_dirs(rundir):
    try:
        return {d for d in os.listdir(rundir)
                if re.fullmatch(r"round-\d+", d)}
    except OSError:
        return set()


def show_score(log, cdir):
    """把判分结果压成几行；返回 verdict。"""
    path = os.path.join(cdir, "score.json")
    try:
        res = json.load(open(path))
    except (OSError, ValueError):
        log.say("!! 读不到判分 %s" % path)
        return None
    log.say("\n判分：verdict=%s（%s）" % (res.get("verdict"),
                                       res.get("reason") or "-"))
    for row in res.get("layers") or []:
        log.say("  第 %s 层：%s%s" % (
            row.get("layer"), row.get("status"),
            ("（%s）" % ", ".join(row.get("reasons") or []))
            if row.get("reasons") else ""))
    log.say("  报告：%s" % path)
    return res.get("verdict")


def teardown_decision(verdict, keep_tower):
    """判分通过才拆；--keep-tower 跳过；其它一律留塔给人看。

    只看 ``verdict``，**不看建塔那一步的退出码**：``stage3_round.py`` 遇到
    ``person_review`` 返回 1（不是失败），人工复核后重判 ok 时 rc 仍是 1；
    若把 rc 也当闸门，复核通过的轮次会被误判成「没通过」而不拆。
    """
    if verdict != "ok":
        return "leave"
    return "keep" if keep_tower else "teardown"


def review_if_needed(args, log, cdir):
    """verdict=person_review 时请人看一眼塔，批准后重判。"""
    round_json = os.path.join(cdir, "round.json")
    try:
        doc = json.load(open(round_json))
    except (OSError, ValueError) as exc:
        log.say("!! 读不到 %s：%s" % (round_json, exc))
        return
    if doc.get("human_review"):
        return
    res_path = os.path.join(cdir, "score.json")
    try:
        res = json.load(open(res_path))
    except (OSError, ValueError):
        return
    if res.get("verdict") != "person_review":
        return

    log.say("\n" + "=" * 62)
    log.say(">>> 需要人动手：先看塔。判分停在 person_review（机器判不了）。")
    log.say("    塔立得对、没有倾斜/悬空，就输入 yes 记录人工复核并重判。")
    log.say("=" * 62)
    if not ask_yes(log, "人眼复核通过请输入 yes： "):
        log.say("   没记录人工复核 —— 判分保持 person_review。")
        return
    doc["human_review"] = {"approved": True,
                           "note": "demo_stage3 现场人眼复核"}
    with open(round_json, "w", encoding="utf-8") as f:
        json.dump(doc, f, ensure_ascii=False, indent=1)
    rc, _ = log.cmd(
        [PY, "-u", os.path.join(HERE, "stage3_score.py"),
         "--observations", round_json,
         "--out-report", os.path.join(cdir, "score")], cwd=HERE)
    log.say("重判 rc=%d" % rc)


# ---------------------------------------------------------------- 主流程
def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description="阶段 3 一键演示（开发机；不依赖 AI）")
    ap.add_argument("--n-layers", type=int, default=3)
    ap.add_argument("--frames", type=int, default=3,
                    help="每个视点的融合帧数（默认 3 = 六轮验收口径；"
                         "2 更快、测量噪声略大）")
    ap.add_argument("--run-dir", default=None,
                    help="默认 ~/robotics/notes/ur5-vision-grasp/runs/"
                         "<日期>-stage3-demo")
    ap.add_argument("--board", default=BOARD_ALIAS, help="SSH 别名")
    ap.add_argument("--urdf", default=DEFAULT["urdf"])
    ap.add_argument("--scene", default=DEFAULT["scene"])
    ap.add_argument("--zcal", default=DEFAULT["zcal"])
    ap.add_argument("--mat", default=DEFAULT["mat"])
    ap.add_argument("--bias-estimate", default=DEFAULT["bias"])
    ap.add_argument("--grid", default=None,
                    help="给了就跳过抓垫位帧与重解（只用于复跑/离线）")
    ap.add_argument("--frame", default=DEFAULT["frame"],
                    help="--dry-run 的拾取帧")
    ap.add_argument("--joints", default=DEFAULT["joints"],
                    help="--dry-run 的起点 joint_states.json")
    ap.add_argument("--dry-run", action="store_true",
                    help="离线：不连板、不动臂，只生成三层计划")
    ap.add_argument("--skip-preflight", action="store_true",
                    help="跳过只读预检（仍要人敲 yes）")
    ap.add_argument("--keep-tower", action="store_true",
                    help="判分通过后不自动拆塔（留现场看；默认会拆回拾取区）")
    return ap.parse_args(argv)


def main(argv=None):
    global BOARD_ALIAS
    args = parse_args(argv)
    BOARD_ALIAS = args.board
    # 子进程（stage2_round / stage3_round）自己在 import 时读这个变量。
    os.environ["STAGE2_BOARD"] = BOARD_ALIAS

    for key in ("urdf", "scene", "zcal", "mat", "bias_estimate",
                "grid", "frame", "joints"):
        val = getattr(args, key)
        if val:
            setattr(args, key, os.path.abspath(os.path.expanduser(val)))

    if args.run_dir:
        rundir = os.path.abspath(os.path.expanduser(args.run_dir))
    else:
        suffix = "-dryrun" if args.dry_run else ""
        rundir = os.path.join(NOTES, "runs",
                              time.strftime("%Y-%m-%d") + "-stage3-demo"
                              + suffix)
    os.makedirs(rundir, exist_ok=True)
    log = Log(os.path.join(rundir, "demo.log"))

    log.say("阶段 3 演示")
    log.say("  轮次目录：%s" % rundir)
    log.say("  层数：%d    融合帧数：%d    板子：%s"
            % (args.n_layers, args.frames, BOARD_ALIAS))
    log.say("  本次动作：只读预检 → 抓垫位帧 → 重解垫位 → %d 层抓放 → 判分"
            " → 拆塔" % args.n_layers)
    if args.dry_run:
        log.say("  **离线模式**：不连板、不动臂，只生成计划")

    # ---- 垫位：复用 / 重解
    grid = args.grid
    if args.dry_run:
        grid = grid or DEFAULT["grid"]
        for name, path in (("拾取帧", args.frame), ("起点关节角", args.joints),
                           ("grid", grid)):
            if not os.path.exists(path):
                log.say("!! 离线模式缺%s：%s" % (name, path))
                log.close()
                return 2

    # ---- 只读预检
    if not args.dry_run and not args.skip_preflight:
        if not preflight(log):
            log.say("\n!! 预检没过 —— fail-closed，不动臂。")
            log.close()
            return 2
    elif args.skip_preflight and not args.dry_run:
        log.say("\n⚠️ 已跳过只读预检（--skip-preflight）—— 执行器仍会在板上"
                "核对 PLAYING / JTC / robot_mode。")

    # ---- 当次授权（离线模式不涉及运动）
    if not args.dry_run:
        if not authorized(log, args.n_layers):
            log.close()
            return 2
        if not args.skip_preflight and not wait_playing(log):
            log.close()
            return 2

    # ---- 垫位
    if not args.dry_run and not args.grid:
        grid = capture_and_solve_grid(args, log, rundir)
        if not grid:
            log.say("\n!! 垫位这一步没过 —— 不进入抓放。")
            log.close()
            return 1

    # ---- 一轮
    scene = make_scene(args, rundir)
    seed = bias_seed(args.bias_estimate)
    if seed:
        log.say("\n偏置先验 b̂=(%s) mm（来自 %s）" % (seed, args.bias_estimate))
    else:
        log.say("\n⚠️ 没有可用的 b̂ 先验（version 3）—— --bias-comp fixed "
                "这一轮其实不补。")
    before = round_dirs(rundir)
    argv = round_argv(args, rundir, grid, scene, seed)
    log.say("\n== 第 3 步：%s ==" % ("离线生成 %d 层计划" % args.n_layers
                                     if args.dry_run
                                     else "真跑 %d 层" % args.n_layers))
    rc, _ = log.cmd(argv, cwd=HERE)

    new_dirs = sorted(round_dirs(rundir) - before)
    cdir = (os.path.join(rundir, new_dirs[0]) if new_dirs else None)
    if cdir and not args.dry_run:
        if os.path.exists(os.path.join(cdir, "score.json")):
            review_if_needed(args, log, cdir)
        verdict = show_score(log, cdir)
    else:
        verdict = None
    # 警告放在判分之后：person_review 的 rc=1 不是臂出错，别先说成「判红」。
    if not args.dry_run and rc != 0:
        if verdict == "ok":
            log.say("\n（建塔那段的退出码是 %d，属 person_review；"
                    "人眼复核后重判 ok）" % rc)
        elif verdict == "person_review":
            log.say("\n!! 判分停在 person_review（rc=%d）：这是「机器判不了」，"
                    "不是臂中途出错；塔留在原地。" % rc)
        else:
            log.say("\n!! 本轮判红（rc=%d）。停在原地，不续跑；块可能还在爪里或"
                    "半放，先看示教器。" % rc)

    log.say("\n== 结束 ==")
    td_rc = 0
    decision = teardown_decision(verdict, args.keep_tower)
    if not args.dry_run:
        if decision == "keep":
            log.say("\n== 第 4 步：拆塔 —— 已按 --keep-tower 跳过 ==")
        elif decision == "teardown":
            td_rc = teardown_after(args, log, rundir, grid, scene, seed)
        else:
            log.say("\n== 第 4 步：拆塔 —— 本轮没通过，不拆；"
                    "塔留在原地给人看 ==")
    log.say("  轮次目录：%s" % rundir)
    log.say("  日志：%s" % log.path)
    if args.dry_run:
        log.say("  离线模式：不发轨迹、不判分。")
        log.close()
        return 0 if rc == 0 else rc
    if verdict == "ok":
        log.say("  结论：verdict=ok")
    else:
        log.say("  结论：verdict=%s（rc=%d）" % (verdict, rc))
    if td_rc != 0:
        log.say("  收尾：拆塔没走完（rc=%d）—— 塔位可能还有块" % td_rc)
    elif not args.dry_run and decision == "teardown":
        log.say("  收尾：塔已拆回拾取区")
    log.close()
    return 0 if verdict == "ok" and td_rc == 0 else 1


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""紧急回 HOME —— 板端一键、fail-closed、**当次** STOP→PLAY 授权。

为什么单独一个工具（2026-11-17 用户提）
======================================
轮内异常（空夹 / 别住 / 计划中止 / 编排器崩）会把臂停在任意姿态；
``park_arm.py`` 是**开发机发起**的整轮工具（本机规划 + 推板 + 监听），
意外现场不一定还连着开发机或编排器。这里给一个**板端自足**的紧急入口：

    读当前关节 → 本地闸门规划 → 板上 MoveIt 段级裁决
    → 挂**本次** STOP→PLAY 监听 → precision_exec 执行

红线不变（逐条对照 AGENTS §1）
=============================
· **红线 6**：真机运动逐次授权。本工具自己起
  ``real_motion_authorize.py --listen``，只读观测示教器两步；没有**本次**
  新令牌（``play_at`` 变化且剩余 ≥120 s）⇒ 一条轨迹都不发。
· **红线 1**：不手搓 RTDE，运动只走 ``precision_exec``（JTC）；授权只读
  Dashboard。
· **红线 2**：本工具**不写**夹爪寄存器。
· **安全**：默认只规划 + 打印（不动臂）；``--go`` 才执行。段级裁决默认
  fail-closed（板上 MoveIt 不可达 ⇒ 不执行）；只有显式
  ``--allow-no-validator`` 才退到本地闸门，且大声告警、写进计划元数据。

用法
====
板上（经 run.sh 带 ROS 环境）::

    ~/precision/run.sh ~/stage4-entry/scripts/emergency_home.py        # 计划+裁决
    ~/precision/run.sh ~/stage4-entry/scripts/emergency_home.py --go   # 授权后执行

开发机也可以直接 ssh 调它。``--robot-ip`` 缺省读板上
``~/.config/ur5-vision-grasp/ur-robot-ip``。

退出码：0 成功 / 2 参数或环境 / 3 规划闸门拒 / 4 段级裁决或授权失败 /
5 执行未完成。任何一步失败都**不**继续下一步。
"""
import argparse
import json
import os
import re
import subprocess
import sys
import time

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import home_pose as hp                                          # noqa: E402
import precision_plan as pp                                     # noqa: E402
import urkin                                                    # noqa: E402

EXIT_OK, EXIT_ARGS, EXIT_PLAN, EXIT_GATE, EXIT_EXEC = 0, 2, 3, 4, 5
CFG = os.path.expanduser("~/.config/ur5-vision-grasp")
TOKEN_FILE = os.path.join(CFG, "motion-auth.json")
IP_FILE = os.path.join(CFG, "ur-robot-ip")
#: 令牌剩余下限：执行一条 6 s 级 park 再多留 2 分钟余量（与 park_arm 同口径）。
MIN_REMAINING_S = 120.0


def log(*a):
    print(" ".join(str(x) for x in a), flush=True)


def run(cmd, timeout=None):
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)


def read_joints(work_dir, capture_py):
    """读当前关节 → 按 ``urkin.Chain.move_joints`` 排序（缺一个就停）。"""
    os.makedirs(work_dir, exist_ok=True)
    r = run([sys.executable, capture_py, work_dir, "--joints-only",
             "--timeout", "20"], timeout=40)
    path = os.path.join(work_dir, "joint_states.json")
    if r.returncode != 0 or not os.path.exists(path):
        raise SystemExit("%d 读关节失败：%s"
                         % (EXIT_ARGS, (r.stderr or r.stdout)[-300:]))
    doc = json.load(open(path, encoding="utf-8"))
    names, pos = doc.get("name") or [], doc.get("position") or []
    chain = urkin.Chain(os.path.expanduser("~/precision/robot.urdf"))
    try:
        return [float(pos[names.index(n)]) for n in chain.move_joints], chain
    except ValueError as exc:
        raise SystemExit("%d 关节状态缺关节：%s" % (EXIT_ARGS, exc))


def read_token():
    try:
        doc = json.load(open(TOKEN_FILE, encoding="utf-8"))
    except (OSError, ValueError):
        return None
    return doc


def segment_check(chain, q_start, q_end, allow_no_validator):
    """板上段级裁决。返回 ``True``；不可达且未显式豁免 ⇒ ``SystemExit(4)``。"""
    try:
        import stage4_moveit_validator as mv
        v = mv.make_segment_validator(chain.move_joints, require=True)
        ok, why = v(q_start, q_end)
    except Exception as exc:                                   # noqa: BLE001
        ok, why = None, str(exc)[:160]
    if ok is True:
        log("   段级碰撞裁决（板上 MoveIt，只读）：过")
        return True
    if ok is False:
        log("   !! 段级碰撞裁决拒绝：%s —— 不执行" % why)
        raise SystemExit(EXIT_GATE)
    if not allow_no_validator:
        log("   !! 段级裁决不可达（%s）—— 默认不执行（fail-closed）；"
            "确认要只靠本地闸门，显式加 --allow-no-validator" % why)
        raise SystemExit(EXIT_GATE)
    log("   !!⚠️ 段级裁决不可达（%s）—— **已显式豁免**，只靠本地闸门"
        "（限位/奇异/缠绕/高度/大跳），结果写进计划元数据" % why)
    return False


def wait_for_new_token(old_play, timeout_s):
    """挂本次 STOP→PLAY 监听，直到**新**令牌成立。返回 token 字符串。"""
    ip = None
    for cand in (os.environ.get("UR_ROBOT_IP"),):
        if cand:
            ip = cand.strip()
            break
    if not ip and os.path.exists(IP_FILE):
        ip = open(IP_FILE, encoding="utf-8").read().strip()
    if not ip:
        log("!! %d 没有机器人 IP（--robot-ip / UR_ROBOT_IP / %s）"
            % (EXIT_GATE, IP_FILE))
        raise SystemExit(EXIT_GATE)
    # 上一轮孤儿监听会重复观测同一批按键 ⇒ 先清（分两条命令，pkill 的
    # 模式用字符类避免匹配到自己 —— b3demo 实测的坑）。
    run(["pkill", "-f", "[r]eal_motion_authorize.py --listen"])
    log(">>> 需要人动手：确认臂到 HOME 的路径上无人无遮挡、急停在手；")
    log(">>> 然后在示教器上先按 STOP，看到 STOPPED 后再按 PLAY。")
    listener = subprocess.Popen(
        [sys.executable, os.path.join(HERE, "real_motion_authorize.py"),
         "--listen", "--robot-ip", ip, "--timeout", str(int(timeout_s))])
    deadline = time.time() + float(timeout_s) + 20.0
    try:
        while time.time() < deadline:
            doc = read_token() or {}
            play = str(doc.get("play_at") or "")
            try:
                remain = float(doc.get("expires") or 0) - time.time()
            except (TypeError, ValueError):
                remain = -1.0
            if play and play != old_play and remain >= MIN_REMAINING_S:
                tok = doc.get("token")
                if tok:
                    log("   本次新令牌就绪（按本机时钟还剩 %.0f s）"
                        % remain)
                    return tok
            if listener.poll() is not None:
                break
            time.sleep(2.0)
    finally:
        if listener.poll() is None:
            listener.terminate()
            try:
                listener.wait(timeout=5)
            except subprocess.TimeoutExpired:
                listener.kill()
    log("!! %d %.0f s 内没等到**本次**新 STOP→PLAY —— 不执行（fail-closed）"
        % (EXIT_GATE, timeout_s))
    raise SystemExit(EXIT_GATE)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--go", action="store_true",
                    help="真的执行（缺省只规划 + 裁决，不动臂）")
    ap.add_argument("--robot-ip", default=None)
    ap.add_argument("--timeout", type=float, default=600.0,
                    help="等 STOP→PLAY 的秒数（缺省 600）")
    ap.add_argument("--work-dir", default=os.path.expanduser(
        "~/precision/e9/emergency-home"))
    ap.add_argument("--capture", default=os.path.expanduser(
        "~/precision/precision_capture.py"))
    ap.add_argument("--exec", dest="exec_py", default=os.path.expanduser(
        "~/precision/precision_exec.py"))
    ap.add_argument("--allow-no-validator", action="store_true",
                    help="段级裁决不可达时只靠本地闸门继续（默认不继续）")
    args = ap.parse_args(argv)

    log("1) 读当前关节")
    q_now, chain = read_joints(args.work_dir, args.capture)
    log("   q_now(deg) = %s" % [round(v * 180.0 / 3.141592653589793, 1)
                                for v in q_now])

    log("2) 规划 park（HOME = %s，出处 home_pose.py）" % (hp.HOME_JOINTS_DEG,))
    try:
        plan, _checks = hp.plan_park(chain, q_now)
    except pp.PlanError as exc:
        log("   !! 本地闸门拒绝：%s —— 不写计划、不执行" % exc)
        return EXIT_PLAN
    valid = segment_check(chain, q_now, plan["waypoints"][-1]["q"],
                          args.allow_no_validator)
    plan.setdefault("meta", {})["emergency_home"] = True
    plan["meta"]["collision_validator"] = bool(valid)
    plan["meta"]["allow_no_validator"] = bool(args.allow_no_validator)
    plan["meta"]["q_start"] = [round(float(v), 6) for v in q_now]

    os.makedirs(args.work_dir, exist_ok=True)
    ppath = os.path.join(args.work_dir, "01-home.json")
    with open(ppath, "w", encoding="utf-8") as fh:
        json.dump(plan, fh, ensure_ascii=False, indent=1)
    dur = float(plan["waypoints"][-1].get("duration") or 0.0)
    log("   计划：%d 航点（duration %.1f s）→ %s"
        % (len(plan["waypoints"]), dur, ppath))

    if not args.go:
        log("3) 只规划不执行（未给 --go）。要执行：同命令加 --go，"
            "按提示做 STOP→PLAY。")
        return EXIT_OK

    log("3) 等当次授权（STOP→PLAY 两步只读核验）")
    old = (read_token() or {}).get("play_at") or ""
    tok = wait_for_new_token(old, args.timeout)

    log("4) 执行（precision_exec，一条 park 计划）")
    logdir = os.path.join(args.work_dir, "logs")
    os.makedirs(logdir, exist_ok=True)
    r = run([sys.executable, args.exec_py, "--plan", ppath,
             "--go", "--require-auth", tok, "--log-dir", logdir],
            timeout=max(120.0, dur + 120.0))
    out = (r.stdout or "") + (r.stderr or "")
    for line in out.strip().splitlines()[-8:]:
        log("   | " + line)
    if "ALL DONE" not in out:
        log("!! %d 执行未完成（没有 ALL DONE，rc=%d）" % (EXIT_EXEC, r.returncode))
        return EXIT_EXEC
    log("臂已回 HOME（%s）。" % (hp.HOME_JOINTS_DEG,))
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())

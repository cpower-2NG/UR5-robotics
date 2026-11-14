#!/usr/bin/env python3
"""B3-lite 发起端：轮参数上板 + **脱离发起**（开发机断线不中断整轮）。

设计（2026-11-14 用户拍板）
──────────────────────────
本机对轮内只有四个动词，发起是其中之一，且发生在**任何运动之前**：

    launch（本工具）→ watch（只读/软停）→ archive（归档拉取）

它做两件事，然后本机就可以走人：
1. **参数上板**：expect / pad-spec / facts / urdf / bias 种子 scp 到板上镜像
   路径（md5 核对）。编排器以 ``--where board`` 在板上跑，它读的一切都必须
   在板上。
2. **脱离发起**：一条 SSH 起 ``setsid nohup bash stage4_round_board.sh ...``，
   立即返回。板上轮壳做 flock 单例（D1）+ 自动两步授权监听 + exec 编排器。
   **验收判据（B3）**：发起后 kill 本地 ssh，整轮照跑完成。

安全边界（接口铁律）：launch 不携带任何运动含义——它只是"把这一次轮的参数
放好并按下编排器的启动键"；授权仍由人在示教器上按 STOP→PLAY 产生（板上
监听只读观测）。本工具不能注目标、不能跳闸门、不能续令牌。

用法
────
    python3 scripts/launch_round.py --batch S0 --layout 1 --run 1 \
        --board jetson-hotspot --board-repo '~/stage4-entry' \
        --expect ~/.config/ur5-vision-grasp/preflight-expect-2026-11-14.json \
        --pad-spec ~/.config/ur5-vision-grasp/pad-spec.json \
        --facts <本地 facts-input.json> \
        --urdf <本地 robot.urdf> \
        [--round-dir '~/precision/e9/s0-1-r1'] \
        [--dry-run] [--round-arg '--place-yaw-equiv180'] ...

退出码：0 = 已发起（pid 见输出）；8 = 板上已有轮在跑（flock 拒绝）；
2 = 参数/推送问题。
"""
import argparse
import hashlib
import os
import shlex
import subprocess
import sys
import time

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import precision_round as pr                                     # noqa: E402

EXIT_OK, EXIT_RUNNING, EXIT_BAD = 0, 8, 2


def _clean(token, what):
    if not token or " " in token or "'" in token or '"' in token:
        raise SystemExit("%s 含空白/引号（板上路径一律不带引号直书）：%r"
                         % (what, token))
    return token


def push_file(local, remote, alias):
    """本地 → 板上（md5 核对；语义与 RoundBoard.push 一致，但独立于 Round）。"""
    if not os.path.exists(local):
        raise SystemExit("要推的文件不存在：%s" % local)
    cmd = ["scp"] + pr.ssh_opts() + ["-o", "LogLevel=ERROR", local,
                                     "%s:%s" % (alias, remote)]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        raise SystemExit("scp %s → 失败：%s" % (local, (r.stderr or "")[:200]))
    want = hashlib.md5(open(local, "rb").read()).hexdigest()
    chk = subprocess.run(["ssh"] + pr.ssh_opts() + [alias, "md5sum %s" % remote],
                         capture_output=True, text=True)
    got = (chk.stdout or "").split()[0] if (chk.stdout or "").strip() else "?"
    if got != want:
        raise SystemExit("推送后 md5 不一致：%s（本地 %s / 板上 %s）"
                         % (local, want[:12], got[:12]))
    print("  推送 %-46s md5 %s 一致" % (os.path.basename(local), want[:12]))


def board_inputs(args, round_dir):
    """命令行本地路径 → 板上镜像路径表（推什么、推到哪、编排器读哪份）。"""
    cfg = "~/.config/ur5-vision-grasp"
    pushes = []                       # [(本地, 板上)]
    paths = {"repo": _clean(args.board_repo, "--board-repo"),
             "round_dir": _clean(round_dir, "--round-dir")}
    if args.expect:
        pushes.append((args.expect, "%s/%s" % (cfg, os.path.basename(args.expect))))
        paths["expect"] = pushes[-1][1]
    if args.pad_spec:
        pushes.append((args.pad_spec, "%s/%s" % (cfg, os.path.basename(args.pad_spec))))
        paths["pad_spec"] = pushes[-1][1]
    if args.pad_mat:
        # 2D 标定（解垫用）：编排器的 --pad-mat 缺省是**开发机**路径——板上
        # 没有（b3demo-r1 首跑实测 argparse 当场拒），必须推上来并显式传。
        paths["pad_mat"] = pr.BOARD_DIR + "/stage2-mat.json"
        pushes.append((args.pad_mat, paths["pad_mat"]))
    if args.urdf:
        paths["urdf"] = args.urdf_board or (pr.BOARD_DIR + "/robot.urdf")
        pushes.append((args.urdf, paths["urdf"]))
    if args.facts:
        paths["facts"] = "%s/facts-input.json" % paths["round_dir"]
        pushes.append((args.facts, paths["facts"]))
    if args.bias:
        paths["bias"] = "%s/bias_estimate.json" % (
            os.path.dirname(paths["round_dir"].rstrip("/")))
        pushes.append((args.bias, paths["bias"]))
    return pushes, paths


def orchestrator_args(args, paths):
    """编排器命令行（板上侧）。``--where board`` 是本工具的固定语义。"""
    urdf = paths.get("urdf")
    if not urdf:
        if not args.urdf:
            raise SystemExit("板上轮必须给 --urdf（launch 会推到 "
                             "~/precision/robot.urdf）；本地文件不存在就别发起")
        urdf = _clean(args.urdf_board or (pr.BOARD_DIR + "/robot.urdf"),
                      "--urdf-board")
    argv = ["--batch", args.batch, "--layout", str(args.layout),
            "--run", str(args.run),
            "--out-dir", paths["round_dir"],
            "--urdf", urdf,
            "--board-repo", paths["repo"]]
    for k in ("expect", "pad_spec", "pad_mat", "facts", "bias"):
        if paths.get(k):
            # bias 的编排器旗标是 --bias-estimate（裸 --bias 会歧义命中
            # --bias-comp/--bias-seed，argparse 直接拒——b3demo 实测）
            flag = "--bias-estimate" if k == "bias" else "--" + k.replace("_", "-")
            argv += [flag, paths[k]]
    for extra in args.round_arg or []:
        if " " in extra or "'" in extra or '"' in extra:
            raise SystemExit("--round-arg 必须是单个旗标/值：%r" % extra)
        argv.append(extra)
    if args.dry_run:
        argv.append("--dry-run")
    return argv


def remote_launch_cmd(repo, round_dir, orch_argv):
    """板上发起命令串（setsid 脱离；pid 写进 round.pid；日志进 round.log）。

    路径按板上约定不带空白/引号，故直接拼（shlex 会把 ``~`` 焊死）。
    """
    for t in (repo, round_dir):
        _clean(t, "板上路径")
    script = "%s/scripts/stage4_round_board.sh" % repo
    return ("mkdir -p %(d)s && cd %(r)s && "
            "setsid nohup bash %(s)s %(r)s %(a)s "
            ">> %(d)s/round.log 2>&1 < /dev/null & "
            "echo $! > %(d)s/round.pid; "
            "echo \"LAUNCHED pid=$(cat %(d)s/round.pid)\""
            % {"d": round_dir, "r": repo, "s": script,
               "a": " ".join(orch_argv)})


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--batch", default="S0")
    ap.add_argument("--layout", type=int, required=True)
    ap.add_argument("--run", type=int, default=1)
    ap.add_argument("--board", required=True, help="SSH 别名（~/.ssh/config）")
    ap.add_argument("--board-repo", default="~/stage4-entry")
    ap.add_argument("--round-dir", default=None,
                    help="板上轮目录（缺省 ~/precision/e9/<batch-l-run>）")
    ap.add_argument("--expect", default=None, help="本地 expect JSON（推上板）")
    ap.add_argument("--pad-spec", default=None, help="本地 pad-spec（推上板）")
    ap.add_argument("--pad-mat", default=None,
                    help="本地 2D 标定（解垫用；缺省开发机路径在板上不存在，"
                         "正式轮必推）")
    ap.add_argument("--facts", default=None, help="本地 facts-input（推上板）")
    ap.add_argument("--urdf", default=None, help="本地 robot.urdf（推上板）")
    ap.add_argument("--urdf-board", default=None,
                    help="板上 urdf 路径（缺省 ~/precision/robot.urdf）")
    ap.add_argument("--bias", default=None, help="本地 bias_estimate.json")
    ap.add_argument("--round-arg", action="append", default=[],
                    help="透传给编排器的单个旗标/值，可重复（如 --dry-run）")
    ap.add_argument("--dry-run", action="store_true",
                    help="编排器干跑（不执行、不动臂 ⇒ 不需要令牌）")
    args = ap.parse_args(argv)

    round_dir = args.round_dir or ("~/precision/e9/%s-%d-r%d"
                                   % (args.batch.lower(), args.layout,
                                      args.run))
    pushes, paths = board_inputs(args, round_dir)
    print("发起 %s:%d run%d → 板上 %s"
          % (args.batch, args.layout, args.run, paths["round_dir"]))
    for local, remote in pushes:
        push_file(local, remote, args.board)

    orch = orchestrator_args(args, paths)
    print("  编排器参数：--where board %s" % " ".join(orch))
    cmd = remote_launch_cmd(paths["repo"], paths["round_dir"], orch)
    r = subprocess.run(["ssh"] + pr.ssh_opts() + [args.board, cmd],
                       capture_output=True, text=True)
    out = (r.stdout or "") + (r.stderr or "")
    if r.returncode != 0 or "LAUNCHED" not in out:
        if "已有轮在跑" in out or (r.returncode == 8):
            print("!! 板上已有轮在跑（flock 拒绝）—— 先 watch 确认")
            return EXIT_RUNNING
        print("!! 发起失败（rc=%d）：%s" % (r.returncode, out.strip()[:400]))
        return EXIT_BAD
    for line in out.splitlines():
        if line.startswith("LAUNCHED"):
            print("  " + line)
    # 2 秒探活：轮壳秒退（flock 拒 / 参数拒）当场报出来，不给假 LAUNCHED
    # （b3demo 实测：被拒的轮壳 1 s 内退 8，LAUNCHED 只是"启动键按下去了"）
    time.sleep(2.0)
    chk = subprocess.run(
        ["ssh"] + pr.ssh_opts() + [args.board,
                                   'P=$(cat %s/round.pid 2>/dev/null); '
                                   '[ -n "$P" ] && kill -0 "$P" 2>/dev/null '
                                   '&& echo ALIVE || echo DEAD'
                                   % paths["round_dir"]],
        capture_output=True, text=True)
    if "ALIVE" in (chk.stdout or ""):
        print("  轮壳 2 s 探活：ALIVE。已脱离发起，开发机可以断线；"
              "观察用 watch_round.py --follow")
        return EXIT_OK
    tail = subprocess.run(
        ["ssh"] + pr.ssh_opts() + [args.board,
                                   "tail -n 4 %s/round.log 2>/dev/null"
                                   % paths["round_dir"]],
        capture_output=True, text=True)
    print("!! 轮壳秒退（探活 DEAD）—— round.log 尾部：")
    for ln in (tail.stdout or "").strip().splitlines():
        print("    | " + ln)
    print("  常见原因：flock 拒（已有轮）/ 预检参数缺（pad-mat 等）。")
    return EXIT_BAD


if __name__ == "__main__":
    sys.exit(main())

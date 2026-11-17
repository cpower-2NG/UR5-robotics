#!/usr/bin/env python3
"""把臂归位 HOME（开发机发起，板上执行；与 setup-view 同一套授权模式）。

流程（默认**只规划与验证，不动臂**）：
1. 板上取当前关节（capture --joints-only）；
2. 开发机用 `home_pose.plan_park` 出计划 JSON v2（限位/奇异/缠绕/高度护栏/
   大跳插点全闸门；板上 MoveIt 可达时再过段级裁决）；
3. 推计划与执行脚本上板（md5 核对）；
4. **要当次 STOP→PLAY**（红线 6；本工具自动挂 600 s 授权监听，等两步观测
   完成即执行——用户在示教器按键，本工具只读核验）。

用法（开发机）::

    python3 scripts/park_arm.py --board jetson-hotspot            # 规划+挂监听+等授权+执行
    python3 scripts/park_arm.py --board jetson-hotspot --no-go    # 只规划与验证

HOME 位姿唯一出处 = ``scripts/home_pose.py``（"烛台姿"，见其模块文档）。
"""
import argparse
import hashlib
import json
import os
import subprocess
import sys
import time

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import precision_round as pr                                     # noqa: E402
import home_pose as hp                                           # noqa: E402
import precision_plan as pp                                      # noqa: E402
import urkin                                                     # noqa: E402

EXIT_OK, EXIT_BAD = 0, 2


def ssh(alias, cmd):
    return subprocess.run(["ssh"] + pr.ssh_opts() + [alias, cmd],
                          capture_output=True, text=True)


def capture_joints(alias, remote_dir, local_dir):
    ssh(alias, "mkdir -p %s" % remote_dir)
    r = ssh(alias, "%s/run.sh %s/precision_capture.py %s --joints-only "
            "--timeout 20" % (pr.BOARD_DIR, pr.BOARD_DIR, remote_dir))
    if r.returncode != 0:
        raise SystemExit("取关节失败：%s" % (r.stderr or r.stdout)[:200])
    os.makedirs(local_dir, exist_ok=True)
    subprocess.run(["scp"] + pr.ssh_opts() + ["-o", "LogLevel=ERROR",
                    "%s:%s/joint_states.json" % (alias, remote_dir),
                    os.path.join(local_dir, "joint_states.json")],
                   check=True, capture_output=True)
    doc = json.load(open(os.path.join(local_dir, "joint_states.json"),
                         encoding="utf-8"))
    names, pos = doc["name"], doc["position"]
    return [float(pos[names.index(n)]) for n in
            urkin.Chain(os.path.expanduser(
                "~/robotics/notes/ur5-vision-grasp/runs/2026-10-25-precision/"
                "robot.urdf")).move_joints]


def push(local, remote, alias):
    want = hashlib.md5(open(local, "rb").read()).hexdigest()
    subprocess.run(["scp"] + pr.ssh_opts() + ["-o", "LogLevel=ERROR", local,
                    "%s:%s" % (alias, remote)], check=True,
                   capture_output=True)
    chk = ssh(alias, "md5sum %s" % remote)
    got = (chk.stdout or "").split()[0] if (chk.stdout or "").strip() else "?"
    if got != want:
        raise SystemExit("推送 md5 不一致：%s" % local)
    print("  推送 %s md5 %s 一致" % (os.path.basename(remote), want[:12]))


def validate_segment_on_board(alias, repo, q_start, q_end):
    """段级碰撞裁决**在板上做**（move_group 同机；rclpy 只在板上有）。

    只读：/check_state_validity 采样查询，不发轨迹、不写场景。
    返回 ``(True/False/None, 理由)``；None = 板上裁决不可达。
    """
    import base64
    payload = base64.b64encode(json.dumps(
        {"q_start": list(q_start), "q_end": list(q_end)}).encode()).decode()
    script = (
        "export ROS_DOMAIN_ID=99; . /opt/ros/humble/setup.bash; "
        ". %s/ros2_ws/install/setup.bash 2>/dev/null; "
        "python3 - <<'PY'\n"
        "import sys, json, base64, os\n"
        "sys.path.insert(0, os.path.expanduser('%s/scripts'))\n"
        "d = json.loads(base64.b64decode('%s').decode())\n"
        "import stage4_moveit_validator as mv\n"
        "import urkin\n"
        "ch = urkin.Chain(os.path.expanduser('~/precision/robot.urdf'))\n"
        "v = mv.make_segment_validator(ch.move_joints, require=True)\n"
        "ok, why = v(d['q_start'], d['q_end'])\n"
        "print('VALID', ok, why)\n"
        "PY" % (repo, repo, payload))
    r = ssh(alias, script)
    out = r.stdout or ""
    if "VALID True" in out:
        return True, ""
    if "VALID False" in out:
        return False, out.split("VALID False", 1)[1].strip()[:120]
    return None, (out + r.stderr).strip()[:160]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--board", default="jetson-hotspot")
    ap.add_argument("--board-repo", default="~/stage4-entry")
    ap.add_argument("--urdf", default=os.path.expanduser(
        "~/robotics/notes/ur5-vision-grasp/runs/2026-10-25-precision/robot.urdf"))
    ap.add_argument("--work-dir", default=os.path.expanduser(
        "~/robotics/notes/ur5-vision-grasp/runs/park"))
    ap.add_argument("--board-dir", default="$HOME/precision/plans/park")
    ap.add_argument("--no-go", action="store_true",
                    help="只规划与验证（推上板，但不挂监听不执行）")
    args = ap.parse_args(argv)

    os.makedirs(args.work_dir, exist_ok=True)
    remote_dir = "$HOME/precision/e9/park/joints-now"
    print("1) 取当前关节")
    q_start = capture_joints(args.board, remote_dir,
                             os.path.join(args.work_dir, "joints-now"))

    print("2) 规划 park（HOME = %s，出处 home_pose.py）" % (hp.HOME_JOINTS_DEG,))
    chain = urkin.Chain(args.urdf)
    plan, _ = hp.plan_park(chain, q_start)
    vok, vwhy = None, ""
    for attempt in (1, 2, 3):        # 首查可因规划场景懒加载慢一拍（实测）
        vok, vwhy = validate_segment_on_board(args.board, args.board_repo,
                                              q_start,
                                              plan["waypoints"][-1]["q"])
        if vok is not None:
            break
        print("   裁决第 %d 次不应答，重试…" % attempt)
        time.sleep(3)
    if vok is True:
        print("   段级碰撞裁决（板上 MoveIt，只读）：过")
        plan["meta"]["collision_validator"] = True
    elif vok is False:
        print("   !! 段级碰撞裁决**拒绝**：%s —— 不写计划" % vwhy)
        return EXIT_BAD
    else:
        print("   !! 段级裁决不可达（%s）—— 只靠本地闸门，**不执行**，仅推上板"
              % vwhy)
        if not args.no_go:
            return EXIT_BAD
    ppath = os.path.join(args.work_dir, "01-home.json")
    json.dump(plan, open(ppath, "w", encoding="utf-8"),
              ensure_ascii=False, indent=1)
    print("   计划：%d 航点（duration %.1f s）"
          % (len(plan["waypoints"]), plan["waypoints"][-1]["duration"]))

    print("3) 推上板")
    r = ssh(args.board, "mkdir -p $HOME/precision/plans/park "
            "$HOME/precision/logs/park "
            "$HOME/precision/e9/park/exec")
    sh_path = os.path.join(args.work_dir, "park.sh")
    sh = "#!/bin/bash\n# park_arm: 回 HOME（任何 rc≠0 立即停）\nset -u\n" \
         'TOKEN="$1"\n[ -n "${TOKEN:-}" ] || { echo "用法: $0 <token>"; exit 2; }\n' \
         "~/precision/run.sh ~/precision/real_motion_authorize.py " \
         "--check-token \"$TOKEN\" --min-remaining 120 || " \
         "{ echo '!!!!! STOP: 令牌剩余不足'; exit 9; }\n" \
         "cd %s && ~/precision/run.sh scripts/precision_exec.py " \
         "--plan %s/01-home.json --log-dir %s --go --require-auth \"$TOKEN\"\n" \
         % (args.board_repo, args.board_dir, "$HOME/precision/logs/park")
    open(sh_path, "w", encoding="utf-8").write(sh)
    os.chmod(sh_path, 0o755)
    push(ppath, "$HOME/precision/plans/park/01-home.json", args.board)
    push(sh_path, "$HOME/precision/plans/park/park.sh", args.board)

    if args.no_go:
        print("只规划不执行（--no-go）。执行时重跑不带 --no-go。")
        return EXIT_OK

    print("4) 挂 600 s 授权监听 —— 请在示教器按 STOP，看到 STOPPED 后按 PLAY")
    # 记旧握手的 play_at：红线 6「上一次的授权不覆盖这一次」——只认**本次**
    # 新写的令牌（play_at 变化 + 未过期）。
    old = ssh(args.board, 'python3 -c \'import json,os;'
              'd=json.load(open(os.path.expanduser("~/.config/ur5-vision-grasp/'
              'motion-auth.json")));print(d.get("play_at",""))\' 2>/dev/null')
    old_play = (old.stdout or "").strip()
    # ⚠️ pkill 与启动**必须分两条 ssh**：同一个 cmdline 里既有 pkill 模式又有
    # 真实启动命令时，pkill -f 会匹配到自己的 shell 把自己杀掉（b3demo 实测：
    # 监听根本没起，log 文件都不存在，用户白按两步）。
    ssh(args.board, "pkill -f '[r]eal_motion_authorize.py --listen' 2>/dev/null; "
        "true")
    time.sleep(1)
    ssh(args.board, "setsid nohup python3 %s/scripts/real_motion_authorize.py "
        "--listen --robot-ip $(cat ~/.config/ur5-vision-grasp/ur-robot-ip) "
        "--timeout 600 > /tmp/park_listen.log 2>&1 < /dev/null &"
        % args.board_repo)
    time.sleep(2)
    deadline = time.time() + 620
    while time.time() < deadline:
        # ⚠️ $HOME 不能写在传给远端 shell 的单引号里（不展开 ⇒ open 字面
        # "$HOME" 路径必然 FileNotFoundError——b3demo 实测循环瞎等的根因）；
        # python 内用 expanduser。
        c = ssh(args.board, 'python3 -c \'import json,os,time;'
                'd=json.load(open(os.path.expanduser("~/.config/'
                'ur5-vision-grasp/motion-auth.json")));'
                'print(d.get("play_at",""), int(float(d.get("expires",0))'
                '-time.time()))\'')
        parts = (c.stdout or "").split()
        # 只认**本次**新令牌：play_at 变化 + 未过期。**不要**再写上界——旧
        # 600 s TTL 时代的 ``<= 600`` 会在 v0.8 的 900 s TTL 下把刚按出来的
        # 新令牌（剩 ~899 s）当成「太新」拒掉，白等倒计时到 600 s（2026-11-17
        # 实测：用户按了 PLAY 却以为没生效）。
        if len(parts) == 2 and parts[0] and parts[0] != old_play \
                and parts[1].lstrip("-").isdigit() and 0 < int(parts[1]):
            print("   本次新令牌就绪（剩 %s s）—— 执行" % parts[1])
            break
        time.sleep(3)
    else:
        print("!! 600 s 内没等到**本次**新握手 —— 不执行（fail-closed）")
        return EXIT_BAD
    e = ssh(args.board, "bash $HOME/precision/plans/park/park.sh "
            "$(python3 -c 'import json;print(json.load(open(\"$HOME/.config/"
            "ur5-vision-grasp/motion-auth.json\"))[\"token\"])')")
    out = e.stdout or ""
    if "ALL DONE" not in out:
        # 一次偶发的空输出（2026-11-15 首跑实测：同令牌手动补跑即成功）
        # ⇒ 同授权窗口内重试一次；令牌守卫（--min-remaining）照旧把关。
        print("   执行无 ALL DONE（stdout %d B）—— 同授权窗口重试一次"
              % len(out))
        e = ssh(args.board, "bash $HOME/precision/plans/park/park.sh "
                "$(python3 -c 'import json;print(json.load(open(\"$HOME/"
                ".config/ur5-vision-grasp/motion-auth.json\"))[\"token\"])')")
        out = e.stdout or ""
    print(out[-500:])
    if "ALL DONE" in out:
        print("臂已回 HOME（%s）。" % (hp.HOME_JOINTS_DEG,))
        return EXIT_OK
    print("!! 执行未完成（没有 ALL DONE）")
    return EXIT_BAD


if __name__ == "__main__":
    sys.exit(main())

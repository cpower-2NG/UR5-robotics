#!/usr/bin/env python3
"""B3-lite 工具（launch_round / watch_round / stage4_round_board.sh）的离线自检。

钉住四件事（2026-11-14 用户拍板的接口铁律）：
1. **launch 只是放参数 + 按启动键**：拼出的编排器命令固定 ``--where board``、
   路径都指板上、参数含空白/引号当场拒（板上路径约定不带引号）；
2. **发起是脱离的**：远端命令串必须 setsid nohup + round.pid 落盘
   （验收判据 B3：发起后 kill 本地 ssh，整轮照跑）；
3. **watch 的唯一指令是减权**：软停载荷 requested=true；kill 门在臂动过之后
   一律拒绝；
4. **板上轮壳**：flock 单例 + 授权监听 + exec（pid 不变 = 锁持有者 = kill 目标）。

不连板子、不动臂；SSH 相关只测命令串构造与输出解析（纯函数）。
跑法：``python3 scripts/test_b3_lite_tools.py``；退出码 0 = 全通过。
"""
import argparse
import json
import os
import subprocess
import sys
import tempfile

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import launch_round as lr                                        # noqa: E402
import watch_round as wr                                         # noqa: E402

PASS = FAIL = 0
FAILS = []


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print("  PASS  %s" % name)
    else:
        FAIL += 1
        FAILS.append(name)
        print("  FAIL  %s   ← %s" % (name, detail))


def ns(**kw):
    d = {"batch": "S0", "layout": 1, "run": 1, "board": "jetson-hotspot",
         "board_repo": "~/stage4-entry", "round_dir": None, "expect": None,
         "pad_spec": None, "pad_mat": None, "facts": None, "urdf": None,
         "urdf_board": None, "bias": None, "round_arg": [], "dry_run": False}
    d.update(kw)
    return argparse.Namespace(**d)


# ---------------------------------------------------------------- launch
print('--- A. launch：路径表 / 编排器参数 / 发起命令串 ---')
with tempfile.TemporaryDirectory() as td:
    urdf = os.path.join(td, "robot.urdf")
    open(urdf, "w").write("<robot/>")
    expect = os.path.join(td, "preflight-expect.json")
    open(expect, "w").write("{}")
    mat = os.path.join(td, "stage2-mat.json")
    open(mat, "w").write("{}")
    a = ns(expect=expect, pad_mat=mat, facts=None, urdf=urdf,
           round_arg=["--place-yaw-equiv180"], dry_run=True)
    pushes, paths = lr.board_inputs(a, "~/precision/e9/s0-1-r1")
    check("expect 推到板上 ~/.config/ur5-vision-grasp/（同名）",
          ("~/".join(["", "config/ur5-vision-grasp/preflight-expect.json"])
           in [p[1] for p in pushes] or
           any(p[1].endswith("/preflight-expect.json") for p in pushes)),
          repr(pushes))
    check("urdf 推到 ~/precision/robot.urdf（缺省位）",
          paths.get("urdf") == "~/precision/robot.urdf", repr(paths))
    check("pad-mat 推到 ~/precision/stage2-mat.json（--pad-mat 缺省是开发机"
          "路径，板上没有——b3demo-r1 首跑实测当场拒）",
          paths.get("pad_mat") == "~/precision/stage2-mat.json"
          and any(p[1] == "~/precision/stage2-mat.json" for p in pushes),
          repr(paths))
    check("路径表带 repo 与 round_dir",
          paths["repo"] == "~/stage4-entry"
          and paths["round_dir"] == "~/precision/e9/s0-1-r1", repr(paths))

    argv = lr.orchestrator_args(a, paths)
    check("编排器参数不含 --where（轮壳固定加 --where board，section D 钉住）",
          "--where" not in argv, repr(argv))
    check("pad-mat 显式传给编排器（板上解析不再吃开发机缺省）",
          argv[argv.index("--pad-mat") + 1] == "~/precision/stage2-mat.json",
          repr(argv))
    check("--out-dir/--urdf/--expect 都指板上路径",
          argv[argv.index("--out-dir") + 1] == "~/precision/e9/s0-1-r1"
          and argv[argv.index("--urdf") + 1] == "~/precision/robot.urdf",
          repr(argv))
    check("--round-arg 透传 + --dry-run 追加",
          "--place-yaw-equiv180" in argv and "--dry-run" in argv, repr(argv))

    a_bad = ns(round_arg=["--cells 8,6"])
    try:
        lr.orchestrator_args(a_bad, paths)
    except SystemExit as e:
        check("--round-arg 带空格 ⇒ 当场拒（板上路径/参数不带引号约定）",
              "单个旗标" in str(e), str(e))
    else:
        check("--round-arg 带空格 ⇒ 当场拒", False, "居然接受了")

    cmd = lr.remote_launch_cmd("~/stage4-entry", "~/precision/e9/s0-1-r1",
                               ["--batch", "S0"])
    check("发起命令串 setsid nohup（断线不中断）",
          "setsid nohup" in cmd and "</dev/null" in cmd.replace(" ", ""), cmd)
    check("发起命令串写 round.pid（kill/巡检的锚）",
          "round.pid" in cmd and "$!" in cmd, cmd)
    check("发起命令串用板上轮壳 + exec 语义的 repo 入参",
          "stage4_round_board.sh ~/stage4-entry" in cmd, cmd)
    try:
        lr.remote_launch_cmd("~/sta ge", "~/x", [])
    except SystemExit as e:
        check("发起路径含空白 ⇒ 当场拒", "含空白" in str(e), str(e))
    else:
        check("发起路径含空白 ⇒ 当场拒", False, "居然接受了")

    # 完整 argv 端到端拼一次（不碰 SSH）：--dry-run 干跑口径
    full = lr.remote_launch_cmd(paths["repo"], paths["round_dir"],
                                lr.orchestrator_args(a, paths))
    check("端到端：走板上轮壳（它固定 --where board）+ 干跑旗标在（干跑不要令牌）",
          "stage4_round_board.sh" in full and "--dry-run" in full, full)

# ---------------------------------------------------------------- watch
print('--- B. watch：状态解析 / 渲染 / kill 门 ---')
SEP = wr.SEP
# 样例镜像 status_cmd 的真实输出：**先标记、后内容**；且 cat 的文件末尾
# 无换行时，标记粘在内容最后一行（b3demo-r10 实测的 `]-----8<----- log`）。
sample = (
    SEP + ' phases\n'
    '[\n {"phase": "preflight", "rc": 0},\n {"phase": "views", "rc": 0},\n'
    ' {"phase": "authorize", "rc": 0}\n]'
    + SEP + ' log\n'
    '62==============================================\n'
    '>>> 需要人动手：示教器先按 STOP，看到 STOPPED 后再按 PLAY。\n'
    '授权成立：按板上时钟令牌还剩 590 秒'
    + SEP + ' done\n'
    + SEP + ' pid\n'
    '4321\n'
    + SEP + ' alive\n'
    'ALIVE\n'
    + SEP + ' token\n'
    '543\n')
st = wr.parse_status(sample)
check("解析：phases 列表、done、pid、存活、令牌剩余",
      [p["phase"] for p in st["phases"]] == ["preflight", "views", "authorize"]
      and st["done"] is False and st["pid"] == "4321"
      and st["alive"] is True
      and st["token_remaining"] == 543, repr(st))
check("解析：粘行标记不丢内容（phases 的 ] 与 log 的末行都完整）",
      '"authorize"' in json.dumps(st["phases"])
      and "令牌还剩 590 秒" in st["log"], repr(st))
text = wr.render(st)
check("渲染：需要人动手行加星标（提示转达给现场的人）",
      "★" in text and "按 STOP" in text, text)
check("渲染：令牌剩余秒数可见", "令牌剩余：~543 s" in text, text)
check("渲染：跑着的进程可见（pid 锚）", "跑着（pid 4321）" in text, text)

st_done = wr.parse_status('[]\n' + SEP + ' phases\n' + SEP + ' log\n'
                          + SEP + ' done\n'
                          '{"round": "x"}\n' + SEP + ' pid\n'
                          'nopid\n' + SEP + ' alive\n'
                          'DEAD\n' + SEP + ' token\n'
                          'notoken\n')
check("解析：done 在 / 无 pid / 无令牌不炸",
      st_done["done"] is True and st_done["pid"] is None
      and st_done["token_remaining"] is None, repr(st_done))

ok, why = wr.decision_kill([{"phase": "preflight"}, {"phase": "views"}],
                           False, True)
check("kill 门：还没进观察段且活着 ⇒ 允许", ok, why)
ok, why = wr.decision_kill([], False, True)
check("kill 门：刚发起（无 phases）⇒ 允许", ok, why)
ok, why = wr.decision_kill([], False, False)
check("kill 门：进程已不在 ⇒ 拒绝（没有 kill 的对象）", not ok, why)
for ph in ("observe", "execute", "score", "archive"):
    ok, why = wr.decision_kill([{"phase": "preflight"}, {"phase": ph}],
                               False, True)
    check("kill 门：进了 %s 段 ⇒ 拒绝（臂动过/在动，留孤儿进程才是危险）" % ph,
          not ok and ph in why, why)
ok, why = wr.decision_kill([], True, True)
check("kill 门：round-done 在 ⇒ 拒绝（不需要 kill）", not ok, why)

print('--- C. 软停载荷：requested=true（减权语义钉住） ---')
payload = json.dumps({"requested": True, "reason": "现场按停", "at": 1.0,
                      "by": "watch_round"}, ensure_ascii=False)
doc = json.loads(payload)
check("软停载荷只有 requested/reason/at/by（没有「继续」这种键）",
      doc["requested"] is True and set(doc) == {"requested", "reason", "at",
                                                "by"}, repr(doc))

# ---------------------------------------------------------------- 轮壳
print('--- D. 板上轮壳 stage4_round_board.sh：源码级钉住 ---')
sh = os.path.join(HERE, "stage4_round_board.sh")
src = open(sh, encoding="utf-8").read()
r = subprocess.run(["bash", "-n", sh], capture_output=True, text=True)
check("bash -n 语法过", r.returncode == 0, r.stderr[:200])
check("flock 单例在场（D1：并发发起立即失败）",
      "flock -n 9" in src and "exit 8" in src, "")
check("授权监听 = real_motion_authorize --listen（只读观测，从不发 play/stop）",
      "--listen" in src and "real_motion_authorize.py" in src, "")
check("没有 IP 也不静默：监听不起要说明 fail-closed",
      "fail-closed" in src, "")
check("末尾 exec 编排器（pid 不变 = 锁持有者 = round.pid = kill 目标）"
      " 且固定 --where board",
      "exec python3 scripts/stage4_round_run.py --where board" in src, "")
check("轮壳自报 pid 到 --out-dir/round.pid（launch 侧 $! 经多层 ssh/setsid"
      " 偶发为空——b3demo-r1 首跑实测）",
      "--out-dir" in src and "round.pid" in src, "")

print('\n%d PASS  %d FAIL' % (PASS, FAIL))
if FAILS:
    print('失败项：')
    for f in FAILS:
        print('  - %s' % f)
sys.exit(0 if FAIL == 0 else 1)

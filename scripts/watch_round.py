#!/usr/bin/env python3
"""B3-lite 观察端（开发机）：只读轮询 + **软停**（唯一指令，只能减权）。

设计（2026-11-14 用户拍板）
──────────────────────────
编排器在板上跑（launch_round.py 发起）；本工具是开发机的眼睛和紧急刹车：

· **只读轮询**（缺省）：一次 SSH 取 phases.json / round.log 尾部 / 令牌剩余
  秒数 / round-done 标记，格式化打印。``--follow`` 每 2 s 一次（SSH 连接复用）。
  不开任何网络服务——这个项目的接缝就是 SSH+文件，状态轮询不需要低延迟通道。
· **--stop <理由>**（唯一指令，减权）：板上写 ``~/.config/ur5-vision-grasp/
  stop-request.json``；``precision_exec`` 在下一个**计划边界**停下（正在走的
  那条轨迹会走完，单航点 ≤12 s）⇒ 本轮 void（stop_requested），不算抓取失败。
  「立刻停」只有示教器急停——执行段本来就必须人在场。
  文件是一次性的：编排器 execute 起手会清残留；没有「继续」通道。
· **--kill-before-observe**：仅当轮还没进入观察段（臂未动）时允许杀掉编排器
  进程；execute 已开始则 refuse——中途 kill 会留孤儿 bash 子进程，那才是危险。

安全边界：本工具**造不出令牌、改不了目标、跳不了闸门**；能做的只有
"看"和"让轮早停"。

用法
────
    python3 scripts/watch_round.py --board jetson-hotspot \
        --round-dir '~/precision/e9/s0-1-r1' --follow
    python3 scripts/watch_round.py ... --stop '现场按停'
    python3 scripts/watch_round.py ... --kill-before-observe
"""
import argparse
import base64
import json
import os
import subprocess
import sys
import time

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import precision_round as pr                                     # noqa: E402

EXIT_OK, EXIT_BAD, EXIT_REFUSED = 0, 2, 3
SEP = "-----8<-----"
#: 软停请求在板上落的位置（与 rma.STOP_REQUEST_FILE 同一处；这里需要**字面**
#: 路径给远端 shell，取 rma 的常量展开前形态）
STOP_REQUEST_REMOTE = "~/.config/ur5-vision-grasp/stop-request.json"
TOKEN_REMOTE = "~/.config/ur5-vision-grasp/motion-auth.json"


def ssh(alias, cmd):
    return subprocess.run(["ssh"] + pr.ssh_opts() + [alias, cmd],
                          capture_output=True, text=True)


def status_cmd(round_dir, tail_lines):
    """一次取齐五样：phases / 日志尾部 / done 标记 / pid 存活 / 令牌剩余秒数。

    每段**先打标再取内容**——解析端只认「分隔符之后」的行（2026-11-14 自检
    抓过：phases 排在第一个分隔符之前，整段被解析器丢弃）。
    done 只在归档段写（干跑轮到不了）⇒ 「轮结束」的判定是 done **或**
    pid 不存活（alive 段）。
    """
    return (
        "echo '%(s)s phases'; cat %(d)s/phases.json 2>/dev/null; "
        "echo '%(s)s log'; tail -n %(n)s %(d)s/round.log 2>/dev/null; "
        "echo '%(s)s done'; cat %(d)s/round-done.json 2>/dev/null; "
        "echo '%(s)s pid'; cat %(d)s/round.pid 2>/dev/null "
        "|| cut -d' ' -f1 $HOME/precision/round_board.holder 2>/dev/null "
        "|| echo nopid; "
        "echo '%(s)s alive'; "
        'P=$(cat %(d)s/round.pid 2>/dev/null); '
        '[ -n "$P" ] || P=$(cut -d" " -f1 $HOME/precision/round_board.holder '
        '2>/dev/null); '
        'if [ -n "$P" ] && kill -0 "$P" 2>/dev/null; then echo ALIVE; '
        "else echo DEAD; fi; "
        "echo '%(s)s token'; "
        'A="$HOME/.config/ur5-vision-grasp/motion-auth.json"; '
        'if [ -f "$A" ]; then python3 -c \'import json,time,sys;'
        "d=json.load(open(sys.argv[1]));"
        'print(int(float(d.get("expires",0))-time.time()))\' "$A" 2>/dev/null'
        " || echo tokERR; else echo notoken; fi"
        % {"d": round_dir, "s": SEP, "n": int(tail_lines)})


def parse_status(text):
    """SSH 输出 → dict（phases/list、log/str、done/bool、pid、alive、token）。

    标记可能**粘在上一段内容的行尾**（cat 的文件末尾无换行时，`echo` 的
    标记与最后一个字符同行，b3demo-r10 实测：`]-----8<----- log`）——
    所以按「行内含 SEP」切分，标记前的残行还给上一段。
    """
    out = {}
    cur = None

    def _add(part):
        if cur is not None and part:
            out[cur].append(part)

    for line in text.splitlines():
        if SEP in line:
            before, _, after = line.partition(SEP)
            _add(before)
            cur = after.strip()
            out[cur] = out.get(cur, [])
            continue
        _add(line)
    parts = {k: "\n".join(v).strip() for k, v in out.items()}
    phases = []
    try:
        raw = json.loads(parts.get("phases") or "[]")
        phases = raw if isinstance(raw, list) else []
    except ValueError:
        phases = []
    pid = parts.get("pid", "nopid").strip()
    tok = parts.get("token", "notoken").strip()
    rem = int(tok) if tok.lstrip("-").isdigit() else None
    alive = parts.get("alive", "DEAD").strip() == "ALIVE"
    return {"phases": phases, "log": parts.get("log", ""),
            "done": bool(parts.get("done", "").strip()),
            "pid": None if pid == "nopid" else pid,
            "alive": alive,
            "token_remaining": rem}


def render(st):
    """状态 → 人看的几行。``需要人动手`` 行加星标（提示要转达给现场的人）。"""
    lines = []
    if st["phases"]:
        last = st["phases"][-1]
        lines.append("阶段：%s（rc=%s%s）" % (
            last.get("phase"), last.get("rc"),
            "  void" if last.get("void") else ""))
        lines.append("  已跑：%s" % " → ".join(
            p.get("phase", "?") for p in st["phases"]))
    else:
        lines.append("阶段：（phases.json 还没落盘——刚发起？）")
    lines.append("进程：%s" % ("跑着（pid %s）" % st["pid"]
                              if st.get("alive") and st["pid"]
                              else "不在（已结束/未发起）"))
    if st["token_remaining"] is not None:
        lines.append("令牌剩余：~%d s" % st["token_remaining"])
    lines.append("round-done：%s" % ("在" if st["done"] else "还没"))
    for ln in st["log"].splitlines():
        mark = "  ★ " if ("需要人动手" in ln or "STOP" in ln) else "  | "
        lines.append(mark + ln)
    if not st["log"]:
        lines.append("  |（round.log 还没内容）")
    return "\n".join(lines)


def decision_kill(phases, done, alive=None):
    """``--kill-before-observe`` 的门：轮还没动臂且还活着才许杀。返回 (ok, 理由)。"""
    if done:
        return False, "轮已结束（round-done 在），不需要 kill"
    if alive is False:
        return False, "编排器进程已经不在（轮已结束/未发起）—— 没 kill 的对象"
    names = [p.get("phase") for p in phases]
    for moved in ("observe", "execute", "score", "archive"):
        if moved in names:
            return False, ("轮已进入 %s 段（臂动过/在动）—— 不许远程 kill"
                           "（会留孤儿进程）；用示教器急停或 --stop 软停" % moved)
    return True, "尚未进入观察段（%s）—— 可以杀" % (names or ["刚发起"])


def do_stop(alias, reason):
    payload = json.dumps({"requested": True, "reason": reason,
                          "at": time.time(), "by": "watch_round"},
                         ensure_ascii=False)
    b64 = base64.b64encode(payload.encode("utf-8")).decode("ascii")
    cmd = ("mkdir -p $(dirname %s) && echo %s | base64 -d > %s && "
           "echo STOP_WRITTEN" % (STOP_REQUEST_REMOTE, b64,
                                  STOP_REQUEST_REMOTE))
    r = ssh(alias, cmd)
    if "STOP_WRITTEN" in (r.stdout or ""):
        print("软停请求已写（计划边界生效；正在走的轨迹会走完，≤十几秒）。")
        print("立刻停 = 示教器急停（人在现场）。")
        return EXIT_OK
    print("!! 软停写入失败：%s" % ((r.stderr or r.stdout or "")[:200]))
    return EXIT_BAD


def do_kill_before_observe(alias, round_dir):
    r = ssh(alias, status_cmd(round_dir, 1))
    st = parse_status(r.stdout or "")
    ok, why = decision_kill(st["phases"], st["done"], st.get("alive"))
    if not ok:
        print("!! 拒绝：%s" % why)
        return EXIT_REFUSED
    if not st["pid"]:
        print("!! 拒绝：找不到 round.pid（轮没在跑？）")
        return EXIT_REFUSED
    k = ssh(alias, "kill %s 2>/dev/null; sleep 1; "
            "kill -0 %s 2>/dev/null && echo ALIVE || echo DEAD" %
            (st["pid"], st["pid"]))
    if "DEAD" in (k.stdout or ""):
        print("编排器（pid %s）已停（轮未动臂，安全）。holder 残留清掉："
              "下一轮可直接发起。" % st["pid"])
        ssh(alias, "rm -f $HOME/precision/round_board.holder")
        return EXIT_OK
    print("!! kill 后进程仍活着（%s）—— 上板看 round.log" % st["pid"])
    return EXIT_BAD


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--board", required=True)
    ap.add_argument("--round-dir", required=True)
    ap.add_argument("--follow", action="store_true")
    ap.add_argument("--interval", type=float, default=2.0)
    ap.add_argument("--tail", type=int, default=12)
    ap.add_argument("--stop", default=None, metavar="理由",
                    help="写软停请求（唯一指令，只能让轮早停）")
    ap.add_argument("--kill-before-observe", action="store_true",
                    help="仅当轮未进入观察段时允许杀编排器（臂未动）")
    args = ap.parse_args(argv)

    if args.stop is not None:
        return do_stop(args.board, args.stop)
    if args.kill_before_observe:
        return do_kill_before_observe(args.board, args.round_dir)

    while True:
        r = ssh(args.board, status_cmd(args.round_dir, args.tail))
        st = parse_status(r.stdout or "")
        print("\n".join(render(st).splitlines()))
        # done 只在归档段写（干跑轮到不了）⇒ 进程不在也视为结束
        if st["done"] or (st["pid"] is not None and not st["alive"]):
            print("轮已结束%s。" % ("（round-done 在，归档可拉）" if st["done"]
                                    else "（进程不在；干跑/中断的轮无 done 标记）"))
            return EXIT_OK
        if not args.follow:
            return EXIT_OK
        try:
            time.sleep(max(1.0, float(args.interval)))
        except KeyboardInterrupt:
            return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())

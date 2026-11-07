#!/usr/bin/env python3
"""``precision_round.Board`` 的真机运动授权接线自检（不连板、不动臂）。

钉住一件事：``exec_plan(go=True)`` **必须**带 ``--require-auth``，且令牌只能来自
板上握手文件 —— 读不到/过期/缺两步记录，就在本地抛错、**一条命令都不发**。
``go=False``（预演）不读令牌、也不带 ``--require-auth``。

为什么要专门测：``precision_exec.py`` 自己也会拒 ``--go`` 不带令牌，但那时
计划已经 scp 上板、ssh 已发出去，日志里留下一条"被拒"，现场很容易误读成
闸门/轨迹的问题。fail-closed 要发生在**发命令之前**。

跑法：``python3 scripts/test_precision_round.py``；退出码 0 = 全通过。
"""

import json
import os
import sys
import tempfile
import time

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import precision_round as pr                                    # noqa: E402

N_PASS = N_FAIL = 0
FAILS = []


def check(name, cond, detail=''):
    global N_PASS, N_FAIL
    if cond:
        N_PASS += 1
        print('  PASS  %s' % name)
    else:
        N_FAIL += 1
        FAILS.append(name)
        print('  FAIL  %s%s' % (name, ('\n        ' + detail) if detail else ''))


class _Res:
    def __init__(self, stdout='', rc=0):
        self.stdout = stdout
        self.stderr = ''
        self.returncode = rc


class FakeBoard(pr.Board):
    """把 ssh/scp 全换成内存：只记录发了什么，不碰网络。"""

    def __init__(self, token_doc=None, board_now=1_000_000.0):
        super().__init__('fake', '/tmp/fake-run')
        self.token_doc = token_doc
        self.board_now = board_now
        self.cmds = []          # (kind, text)

    def ssh(self, remote_cmd, quiet=False, check=True):
        self.cmds.append(('ssh', remote_cmd))
        if 'date +%s' in remote_cmd:
            doc = '' if self.token_doc is None else json.dumps(self.token_doc)
            return _Res(stdout='%s---\n%r\n' % (doc, self.board_now))
        return _Res()

    def _pull_log(self, *a, **kw):
        self.cmds.append(('pull', ''))


def _run(board, go=True):
    plan = os.path.join(tempfile.gettempdir(), 'fake-plan.json')
    open(plan, 'w').write('{}')
    old_sh = pr.sh
    pr.sh = lambda *a, **k: _Res()
    try:
        return board.exec_plan(plan, '/tmp/fake-log.jsonl', go)
    finally:
        pr.sh = old_sh


def _exec_cmds(board):
    return [c for kind, c in board.cmds if 'precision_exec.py' in c]


def _raises(fn):
    try:
        fn()
    except SystemExit as e:
        return str(e)
    return None


now = 1_000_000.0
good = {'token': 'a' * 32, 'expires': now + 300, 'stop_at': now - 100,
        'play_at': now - 90}

print('--- A. go=True：带令牌 ---')
b = FakeBoard(good, now)
rc = _run(b, go=True)
ex = _exec_cmds(b)
check('发了一条执行命令', len(ex) == 1, repr(ex))
check('命令带 --go', ex and ' --go' in ex[0], repr(ex))
check('命令带 --require-auth 和该令牌',
      ex and ('--require-auth ' + 'a' * 32) in ex[0], repr(ex))

print('--- B. go=False：预演不发令牌 ---')
b = FakeBoard(None, now)          # 板上根本没有令牌文件
b.cmds.clear()
rc = _run(b, go=False)
ex = _exec_cmds(b)
check('预演不读令牌文件', not any('date +%s' in c for _, c in b.cmds),
      repr(b.cmds))
check('预演带 --list、不带 --require-auth',
      ex and ' --list' in ex[0] and '--require-auth' not in ex[0], repr(ex))

print('--- C. 失败一律 fail-closed：一条命令都不发 ---')
for name, doc, want in [
        ('没有令牌文件', None, '没有运动授权令牌'),
        ('令牌已过期', dict(good, expires=now - 1), '已过期'),
        ('缺 play_at', dict(good, play_at=None), '两步记录'),
        ('缺 stop_at', dict(good, stop_at=None), '两步记录'),
]:
    b = FakeBoard(doc, now)
    msg = _raises(lambda b=b: _run(b, go=True))
    check('%s ⇒ 抛错' % name, msg is not None and want in msg, repr(msg))
    check('%s ⇒ 没发执行命令' % name, not _exec_cmds(b), repr(b.cmds))

print('--- D. 过期判定用板上时钟 ---')
# 开发机时间与板子差 1 小时：令牌按**板子**时钟还剩 300 秒，必须放行
b = FakeBoard(good, now)
pc = time.time
time.time = lambda: now + 3600        # 本机"快一小时"
try:
    rc = _run(b, go=True)
finally:
    time.time = pc
check('本机快 1 小时也不误判过期', len(_exec_cmds(b)) == 1, repr(b.cmds))

print()
print('通过 %d，失败 %d' % (N_PASS, N_FAIL))
if FAILS:
    print('失败项：')
    for f in FAILS:
        print('  - %s' % f)
sys.exit(0 if N_FAIL == 0 else 1)

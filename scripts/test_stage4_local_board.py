#!/usr/bin/env python3
"""B1：LocalBoard（板上本机适配器）文件/命令原语的单元测试。

不碰真板子：全部在临时目录里验证与 RoundBoard 同名方法的**本地语义**
（push 的 md5 核对 / clear 幂等 / pull_dir 内容拷贝 / token_state 的
fail-closed / 与 RoundBoard 共享 wait_token 端口）。
"""
import hashlib
import json
import os
import sys
import tempfile

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import stage4_round_run as rr

PASS = FAIL = 0


def check(name, cond, detail=''):
    global PASS, FAIL
    if ok_label(cond):
        PASS += 1
        print('  PASS  %s' % name)
    else:
        FAIL += 1
        print('  FAIL  %s  ← %s' % (name, detail))


def ok_label(cond):
    return bool(cond)


def test_file_primitives():
    tmp = tempfile.mkdtemp(prefix='localboard-')
    board = rr.LocalBoard(board_dir=os.path.join(tmp, 'board'))
    local = os.path.join(tmp, 'plan.json')
    with open(local, 'w') as f:
        f.write('{"v": 2}')

    board.push(local, os.path.join(tmp, 'board', 'plan', 'plan.json'),
               'plan.json')
    pushed = os.path.join(tmp, 'board', 'plan', 'plan.json')
    check('push 复制并自动建目录', os.path.exists(pushed))
    check('push 后 md5 两端一致',
          hashlib.md5(open(local, 'rb').read()).hexdigest()
          == hashlib.md5(open(pushed, 'rb').read()).hexdigest())

    board.push_many([(local, os.path.join(tmp, 'board', 'p2.json'))])
    check('push_many 批量', os.path.exists(os.path.join(tmp, 'board',
                                                       'p2.json')))

    board.mkdir(os.path.join(tmp, 'board', 'exec'))
    check('mkdir 幂等建目录', os.path.isdir(os.path.join(tmp, 'board', 'exec')))

    with open(pushed, 'a') as f:
        f.write('残留')
    board.clear(os.path.join(tmp, 'board', 'plan'))
    check('clear 删目录（防旧证据混轮）',
          not os.path.exists(os.path.join(tmp, 'board', 'plan')))
    board.clear(os.path.join(tmp, 'board', 'plan'))
    check('clear 幂等（不存在也不炸）', True)

    board.pull_file(pushed if os.path.exists(pushed) else local,
                    os.path.join(tmp, 'back.json'))
    check('pull_file 拉回', os.path.exists(os.path.join(tmp, 'back.json')))

    os.makedirs(os.path.join(tmp, 'board', 'burst'), exist_ok=True)
    for i in range(3):
        with open(os.path.join(tmp, 'board', 'burst', 'color_%03d.png' % i),
                  'w') as f:
            f.write('x')
    dst = os.path.join(tmp, 'local-burst')
    board.pull_dir(os.path.join(tmp, 'board', 'burst'), dst)
    n = len([f for f in os.listdir(dst) if f.startswith('color_')])
    check('pull_dir 拉目录内容（3 帧）', n == 3, n)
    board.pull_dir(os.path.join(tmp, 'board', 'burst'), dst)
    check('pull_dir 重复拉不炸（dirs_exist_ok）',
          len([f for f in os.listdir(dst) if f.startswith('color_')]) == 3)

    r = board.ssh('printf hello')
    check('ssh 原语 = 本地 bash 执行', r.returncode == 0
          and (r.stdout or '').strip() == 'hello', repr(r.stdout))
    r = board.ssh('exit 3')
    check('本地命令 rc 透传（不静默）', r.returncode == 3)


def test_token_state_fail_closed():
    tmp = tempfile.mkdtemp(prefix='localboard-tok-')
    board = rr.LocalBoard(board_dir=tmp)
    tok, rem, why = board.token_state()
    check('没有令牌文件 ⇒ (None, None, 原因)', tok is None and rem is None
          and why, why)
    # 伪造令牌文件（真值由 real_motion_authorize 写；这里只测读端语义）
    import precision_round as pr
    path = os.path.expanduser(pr.AUTH_TOKEN_FILE)
    backup = None
    if os.path.exists(path):
        backup = open(path, 'rb').read()
    os.makedirs(os.path.dirname(os.path.expanduser(pr.AUTH_TOKEN_FILE)),
                exist_ok=True)
    try:
        with open(path, 'w') as f:
            json.dump({'token': 't', 'stop_at': 1, 'play_at': 2,
                       'expires': 0}, f)
        tok, rem, why = board.token_state()
        check('过期令牌 ⇒ None + 过期原因', tok is None and '过期' in why, why)
        import time
        with open(path, 'w') as f:
            json.dump({'token': 't', 'stop_at': 1, 'play_at': 2,
                       'expires': time.time() + 300}, f)
        tok, rem, why = board.token_state()
        check('有效令牌 ⇒ (tok, rem>0, "")',
              tok == 't' and rem > 0 and why == '', (tok, rem, why))
    finally:
        if backup is not None:
            with open(path, 'wb') as f:
                f.write(backup)
        else:
            os.remove(path)


def test_port_shared():
    check('RoundBoard 与 LocalBoard 共享同一端口基类',
          issubclass(rr.RoundBoard, rr.BoardPort)
          and issubclass(rr.LocalBoard, rr.BoardPort))
    check('wait_token 定义在端口上（两适配器共用同一份）',
          rr.BoardPort.wait_token is not None
          and 'wait_token' not in rr.RoundBoard.__dict__
          and 'wait_token' not in rr.LocalBoard.__dict__)
    for m in ('ssh', 'push', 'push_many', 'mkdir', 'clear', 'pull_file',
              'pull_dir', 'joints', 'preflight', 'token_state', 'wait_token',
              'run_script', 'analyze', 'probe'):
        check('端口方法齐全：%s' % m,
              m in rr.LocalBoard.__dict__ or m in rr.BoardPort.__dict__)


if __name__ == '__main__':
    print('[B1] LocalBoard 单元测试')
    test_file_primitives()
    test_token_state_fail_closed()
    test_port_shared()
    print('\n%d PASS, %d FAIL' % (PASS, FAIL))
    sys.exit(1 if FAIL else 0)

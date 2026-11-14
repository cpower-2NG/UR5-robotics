#!/usr/bin/env python3
"""真机运动授权（``real_motion_authorize.py``）的纯函数自检：不连控制柜、不动臂。

钉住四件事：

1. **只读**：``query()`` 只发 ``robotmode`` / ``programState`` / ``safetystatus``
   三条查询；任何 play/stop 字样都不得出现在写入流里；
2. **次序**：没有 STOP 记录、STOP 过期、只看到 PLAYING 但 robotmode 不是 RUNNING
   ⇒ 都不写令牌（fail-closed）；
3. **令牌**：错令牌/过期/缺 STOP-PLAY 记录都不成立；正常令牌报出握手秒数；
4. **闸门接到执行器**：``precision_exec.py --go`` 不带 ``--require-auth`` 时
   在加载计划**之前**就退出 2。

另做三次变异/边界核对，证明判据不是空转：STOP 窗口收到 0 秒 ⇒ 刚写的记录也拒；
令牌过期一秒 ⇒ 拒；PLAYING 但 robotmode 停着 ⇒ 不写令牌。

跑法：``python3 scripts/test_real_motion_authorize.py``；退出码 0 = 全通过。
"""

import contextlib
import io
import json
import os
import sys
import tempfile
import time

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import real_motion_authorize as rma                             # noqa: E402
import precision_exec as pe                                     # noqa: E402

IP = '<ur-controller-ip>'   # 占位符：本测试从不真的连控制柜

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


def quiet(fn, *a, **kw):
    """跑一个会打印提示的函数，把输出吞掉（测试输出只看 PASS/FAIL）。"""
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc = fn(*a, **kw)
    return rc, buf.getvalue()


def obs(program='PLAYING', robotmode='RUNNING', safety='NORMAL'):
    return {'program': program, 'robotmode': robotmode, 'safety': safety,
            'raw': {'programState': 'Program state: %s x.urp' % program,
                    'robotmode': 'Robotmode: %s' % robotmode,
                    'safetystatus': 'Safetystatus: %s' % safety}}


class Seq:
    """按调用序返回观测；用完之后一直重复最后一条。"""

    def __init__(self, items):
        self.items = list(items)
        self.i = 0

    def __call__(self, ip, timeout=5.0):
        it = self.items[min(self.i, len(self.items) - 1)]
        self.i += 1
        return it


# =====================================================================
print('--- A. 只读查询：写入流里只有三条查询 ---')


class FakeFile:
    def __init__(self, conn):
        self.conn = conn

    def readline(self):
        return self.conn.replies.pop(0) if self.conn.replies else ''

    def write(self, s):
        self.conn.written.append(s.strip())

    def flush(self):
        pass


class FakeConn:
    def __init__(self, replies):
        self.replies = list(replies)
        self.written = []
        self.closed = False

    def makefile(self, mode):
        return FakeFile(self)

    def close(self):
        self.closed = True


conn = FakeConn(['Connected.\n', 'Robotmode: RUNNING\n',
                 'Program state: PLAYING x.urp\n', 'Safetystatus: NORMAL\n'])
_cc = rma.socket.create_connection
rma.socket.create_connection = lambda addr, timeout=None: conn
try:
    raw = rma.query('IP')
finally:
    rma.socket.create_connection = _cc
check('query 返回三条回复', raw is not None and set(raw) ==
      {'robotmode', 'programState', 'safetystatus'}, repr(raw))
check('写入流正好是三条只读查询',
      conn.written == ['robotmode', 'programState', 'safetystatus'],
      repr(conn.written))
check('写入流没有 play/stop/power',
      not any(w.lower() in ('play', 'stop', 'power_on', 'brake_release')
              for w in conn.written), repr(conn.written))
check('连接已关闭', conn.closed)


def boom(addr, timeout=None):
    raise OSError('refused')


_cc = rma.socket.create_connection
rma.socket.create_connection = boom
try:
    none = rma.query('IP')
finally:
    rma.socket.create_connection = _cc
check('连不上 ⇒ None（不猜状态）', none is None)

check('_state 去掉前缀', rma._state('Program state: PLAYING x.urp') == 'PLAYING')
check('_state 容忍无前缀', rma._state('STOPPED x.urp') == 'STOPPED')
check('_state 空 ⇒ 空串', rma._state('') == '')
check('_norm 机器人模式', rma._norm('Robotmode: RUNNING') == 'RUNNING')
check('_norm 安全状态', rma._norm('Safetystatus: NORMAL') == 'NORMAL')

# =====================================================================
print('--- B. do_stop：看到 STOPPED 才写记录 ---')
_ob = rma.observe
with tempfile.TemporaryDirectory() as td:
    rma.observe = Seq([obs('PLAYING'), obs('STOPPED')])
    rc, out = quiet(rma.do_stop, 'IP', td, 2.0, 0.2)
    check('PLAYING→STOPPED ⇒ 退出 0', rc == 0, 'rc=%r' % rc)
    p = os.path.join(td, rma.STOP_FILE)
    rec = json.load(open(p, encoding='utf-8')) if os.path.exists(p) else {}
    check('写了 STOP 记录（带 nonce/时间/观测）',
          bool(rec.get('nonce')) and bool(rec.get('at')) and
          rec.get('program') == 'STOPPED' and rec.get('observed'),
          repr(rec))
    check('提示里先 STOP 再 PLAY',
          '按 STOP' in out and '按 PLAY' in out, repr(out))

with tempfile.TemporaryDirectory() as td:
    rma.observe = Seq([obs('PLAYING')])
    rc, out = quiet(rma.do_stop, 'IP', td, 0.3, 0.2)
    check('一直 PLAYING ⇒ 退出 2', rc == 2, 'rc=%r' % rc)
    check('超时不写 STOP 记录',
          not os.path.exists(os.path.join(td, rma.STOP_FILE)))
    check('提示写明不授权', 'fail-closed' in out, repr(out))
rma.observe = _ob

# =====================================================================
print('--- C. do_play：次序、状态、令牌 ---')
_ob = rma.observe


def write_stop(td, age_s=0.0):
    os.makedirs(td, exist_ok=True)
    with open(os.path.join(td, rma.STOP_FILE), 'w', encoding='utf-8') as fh:
        json.dump({'nonce': 'n1', 'at': time.time() - age_s, 'program': 'STOPPED',
                   'observed': {'programState': 'STOPPED'}}, fh)


with tempfile.TemporaryDirectory() as td:
    rc, out = quiet(rma.do_play, 'IP', td, 0.3, 0.2)
    check('没有 STOP 记录 ⇒ 退出 2', rc == 2, 'rc=%r' % rc)
    check('没有 STOP 记录 ⇒ 不写令牌',
          not os.path.exists(os.path.join(td, rma.TOKEN_FILE)), repr(out))

with tempfile.TemporaryDirectory() as td:
    write_stop(td, age_s=10 ** 4)
    rc, out = quiet(rma.do_play, 'IP', td, 0.3, 0.2)
    check('STOP 记录太旧 ⇒ 退出 2', rc == 2, 'rc=%r' % rc)
    check('STOP 记录太旧 ⇒ 不写令牌',
          not os.path.exists(os.path.join(td, rma.TOKEN_FILE)))

with tempfile.TemporaryDirectory() as td:
    write_stop(td, age_s=1.0)
    rc, out = quiet(rma.do_play, 'IP', td, 0.3, 0.2, stop_window=0.0)
    check('变异：STOP 窗口 0 秒 ⇒ 1 秒前的记录也拒（窗口真的被查）',
          rc == 2, 'rc=%r' % rc)

with tempfile.TemporaryDirectory() as td:
    write_stop(td, age_s=1.0)
    # 程序在 PLAYING，但 robotmode 不是 RUNNING ⇒ 不许写令牌
    rma.observe = Seq([obs('PLAYING', robotmode='STOPPED')])
    rc, out = quiet(rma.do_play, 'IP', td, 0.3, 0.2)
    check('PLAYING 但 robotmode 停着 ⇒ 退出 2', rc == 2, 'rc=%r' % rc)
    check('PLAYING 但 robotmode 停着 ⇒ 不写令牌',
          not os.path.exists(os.path.join(td, rma.TOKEN_FILE)), repr(out))

with tempfile.TemporaryDirectory() as td:
    write_stop(td, age_s=1.0)
    rma.observe = Seq([obs('PLAYING', robotmode='STOPPED'),
                       obs('PLAYING', robotmode='RUNNING')])
    rc, out = quiet(rma.do_play, 'IP', td, 2.0, 0.2)
    check('迟到的 RUNNING ⇒ 退出 0', rc == 0, 'rc=%r' % rc)
    p = os.path.join(td, rma.TOKEN_FILE)
    doc = json.load(open(p, encoding='utf-8')) if os.path.exists(p) else {}
    check('令牌文件内容完整',
          doc.get('kind') == 'stage4-motion-authorization' and
          len(doc.get('token', '')) == 32 and doc.get('stop_nonce') == 'n1' and
          doc.get('play_at') and doc.get('expires') > doc.get('play_at'),
          repr(doc))
    check('令牌自述「从未发送 play/stop」', '从未发送' in doc.get('note', ''),
          repr(doc.get('note')))
    tok = doc.get('token')

    ok, why, age = rma.check_token(tok, auth_dir=td, now=doc['play_at'] + 5)
    check('check_token 正常 ⇒ ok', ok and abs(age - 5.0) < 1e-6, why)

    ok, why, _ = rma.check_token('0' * 32, auth_dir=td, now=doc['play_at'] + 5)
    check('错令牌 ⇒ 不成立', not ok and '不匹配' in why, why)

    ok, why, _ = rma.check_token(tok, auth_dir=td, now=doc['expires'] + 1)
    check('变异：过期一秒 ⇒ 不成立', not ok and '过期' in why, why)

    # token_remaining：整轮编排器的 R1/R2 闸门与执行脚本首行守卫都读它。
    rem, why = rma.token_remaining(tok, auth_dir=td, now=doc['play_at'] + 5)
    check('token_remaining: 正常 ⇒ TTL−5 秒',
          rem is not None and abs(rem - (rma.TOKEN_TTL_S - 5.0)) < 1e-6,
          repr((rem, why)))
    rem, why = rma.token_remaining(tok, auth_dir=td, now=doc['expires'] - 30)
    check('token_remaining: 恰好剩 30 秒', rem is not None and abs(rem - 30) < 1e-6,
          repr((rem, why)))
    rem, why = rma.token_remaining(tok, auth_dir=td, now=doc['expires'] + 1)
    check('token_remaining: 过期一秒 ⇒ None + 原因',
          rem is None and '过期' in why, repr((rem, why)))
    rem, why = rma.token_remaining('0' * 32, auth_dir=td, now=doc['play_at'] + 5)
    check('token_remaining: 错令牌 ⇒ None', rem is None and '不匹配' in why,
          repr((rem, why)))

    # CLI 的 --min-remaining：剩余不够就在**发执行之前**退 2。
    rc0, out0 = quiet(rma.main, ['--check-token', tok, '--auth-dir', td,
                                 '--min-remaining', '500'])
    check('CLI --min-remaining 500（剩 ≈600）⇒ 0', rc0 == 0, repr(out0))
    rc2, out2 = quiet(rma.main, ['--check-token', tok, '--auth-dir', td,
                                 '--min-remaining', '700'])
    check('CLI --min-remaining 700 ⇒ 2（不让它跑到一半过期）', rc2 == 2,
          repr(out2))
    check('不足时提示写清还剩多少', '只剩' in out2 and '700' in out2, repr(out2))

    doc2 = dict(doc)
    doc2.pop('stop_at')
    json.dump(doc2, open(p, 'w', encoding='utf-8'))
    ok, why, _ = rma.check_token(tok, auth_dir=td, now=doc['play_at'] + 5)
    check('缺 STOP 记录字段 ⇒ 不成立', not ok and '缺少' in why, why)

with tempfile.TemporaryDirectory() as td:
    ok, why, _ = rma.check_token('x' * 32, auth_dir=td)
    check('没有令牌文件 ⇒ 不成立（fail-closed）', not ok and '没有授权令牌' in why,
          why)
rma.observe = _ob

# =====================================================================
print('--- C2. do_listen：自动顺序两步（板上轮内用，B3-lite） ---')
_ob = rma.observe

with tempfile.TemporaryDirectory() as td:
    # 顺序语义与手动两步一致：先看到 STOPPED、再看到 PLAYING+RUNNING+NORMAL
    rma.observe = Seq([obs('PLAYING'), obs('STOPPED'),
                       obs('PLAYING', robotmode='STOPPED'),
                       obs('PLAYING', robotmode='RUNNING')])
    rc, out = quiet(rma.do_listen, 'IP', td, 2.0, 0.2)
    check('listen：PLAYING→STOPPED→(非 RUNNING)→RUNNING ⇒ 退出 0', rc == 0,
          'rc=%r out=%r' % (rc, out[-200:]))
    doc = {}
    p = os.path.join(td, rma.TOKEN_FILE)
    if os.path.exists(p):
        doc = json.load(open(p, encoding='utf-8'))
    check('listen：令牌带 STOP/PLAY 两步记录',
          doc.get('stop_at') and doc.get('play_at')
          and doc.get('stop_nonce'), repr(doc))
    check('listen：令牌自述从未发送 play/stop',
          '从未发送' in doc.get('note', ''), repr(doc.get('note')))
    check('listen：提示包含完整握手指引',
          '按 STOP' in out and '按 PLAY' in out and '急停' in out, repr(out))

with tempfile.TemporaryDirectory() as td:
    # 第 1 步超时 ⇒ 不写令牌（fail-closed，与 do_stop 同判据）
    rma.observe = Seq([obs('PLAYING')])
    rc, out = quiet(rma.do_listen, 'IP', td, 0.3, 0.2)
    check('listen：一直 PLAYING（没按 STOP）⇒ 退出 2', rc == 2, 'rc=%r' % rc)
    check('listen：不写令牌',
          not os.path.exists(os.path.join(td, rma.TOKEN_FILE)))

with tempfile.TemporaryDirectory() as td:
    # 第 2 步状态不全 ⇒ 不写令牌
    rma.observe = Seq([obs('STOPPED'), obs('PLAYING', robotmode='STOPPED')])
    rc, out = quiet(rma.do_listen, 'IP', td, 0.3, 0.2)
    check('listen：PLAYING 但 robotmode 停着 ⇒ 退出 2', rc == 2, 'rc=%r' % rc)
    check('listen：不写令牌',
          not os.path.exists(os.path.join(td, rma.TOKEN_FILE)))

with tempfile.TemporaryDirectory() as td:
    # 变异：STOP 窗口 0 秒 ⇒ 刚记的第 1 步也被拒（窗口真的被查）
    rma.observe = Seq([obs('STOPPED'), obs('PLAYING', robotmode='RUNNING')])
    rc, out = quiet(rma.do_listen, 'IP', td, 2.0, 0.2, stop_window=0.0)
    check('listen：变异 STOP 窗口 0 秒 ⇒ 拒（rc 2，不写令牌）', rc == 2, 'rc=%r' % rc)
    check('listen：不写令牌',
          not os.path.exists(os.path.join(td, rma.TOKEN_FILE)))

# CLI 接线：--listen 进互斥组、走到 do_listen
_rmai = rma.observe
with tempfile.TemporaryDirectory() as td:
    rma.observe = Seq([obs('STOPPED'), obs('PLAYING', robotmode='RUNNING')])
    rc, out = quiet(rma.main, ['--listen', '--auth-dir', td,
                               '--timeout', '2.0', '--interval', '0.2',
                               '--robot-ip', 'IP'])
    check('CLI --listen ⇒ 退出 0 且令牌落盘', rc == 0 and
          os.path.exists(os.path.join(td, rma.TOKEN_FILE)), repr(out[-200:]))
rma.observe = _ob

# =====================================================================
print('--- C3. stop_requested：软停请求的读端（只能减权，无「继续」通道） ---')
with tempfile.TemporaryDirectory() as td:
    asked, doc = rma.stop_requested(os.path.join(td, 'nope.json'))
    check('文件不在 ⇒ 不在（False，空内容）', asked is False and doc == {},
          repr((asked, doc)))
    p = os.path.join(td, 'stop-request.json')
    with open(p, 'w', encoding='utf-8') as fh:
        json.dump({'requested': True, 'reason': '现场按停', 'at': 123.0}, fh)
    asked, doc = rma.stop_requested(p)
    check('requested=true ⇒ 在场，内容原样带出', asked is True and
          doc.get('reason') == '现场按停', repr(doc))
    with open(p, 'w', encoding='utf-8') as fh:
        fh.write('{坏 JSON')
    asked, doc = rma.stop_requested(p)
    check('坏 JSON ⇒ 当作不在（不猜）', asked is False and doc == {},
          repr((asked, doc)))

# =====================================================================
print('--- D. 闸门接到 precision_exec：--go 没令牌就一条也不发 ---')
_argv = sys.argv
try:
    sys.argv = ['precision_exec.py', '--plan', '/no/such/plan.json', '--go']
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc = pe.main()
    out = buf.getvalue()
finally:
    sys.argv = _argv
check('--go 无 --require-auth ⇒ 退出 2', rc == 2, 'rc=%r' % rc)
check('并在加载计划之前就退出（没读文件）',
      '--require-auth' in out and '不做任何动作' not in out, repr(out))
check('提示给出握手步骤', 'real_motion_authorize.py --step stop' in out, repr(out))

# =====================================================================
print()
print('通过 %d，失败 %d' % (N_PASS, N_FAIL))
if FAILS:
    print('失败项：')
    for f in FAILS:
        print('  - %s' % f)
sys.exit(0 if N_FAIL == 0 else 1)

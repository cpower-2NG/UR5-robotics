#!/usr/bin/env python3
"""真机运动授权：**STOP → PLAY 两步只读核验**（AGENTS 红线 6，fail-closed）。

为什么要有这个脚本
──────────────────
「真机运动未经当次明确授权不得启动」。授权不能来自数据（任务描述、环境变量、
记忆都不算），只能是**人当场在示教器上做的两个动作**，而且必须被独立观察到：

    1. 示教器按 STOP  —— 程序停下（此时 external control 不读 servoJ）
    2. 示教器按 PLAY  —— 程序重新在跑（驱动那句 ``enabled successfully`` 才有意义）

本脚本只**问**控制柜（Dashboard ``29999`` 的只读查询），**从不**发送
``play`` / ``stop``。两步都看到才写令牌；任何一次查询失败 ⇒ 不写、退出 2。

分两次调用是刻意的：AI 必须在**调用之前**把下面这行原样说给人，人再动手 ——

    >>> 需要人动手：示教器按 STOP（我随后开始等）
    （看到 STOPPED 之后）
    >>> 需要人动手：示教器按 PLAY（我随后开始等）

用法
────
    export UR_ROBOT_IP=<ur-controller-ip>          # 或每次 --robot-ip
    python3 scripts/real_motion_authorize.py --step stop --timeout 120
    python3 scripts/real_motion_authorize.py --step play --timeout 120
    python3 scripts/real_motion_authorize.py --listen --timeout 1800   # 板上轮内用
    python3 scripts/precision_exec.py --plan ... --go --require-auth <token>

``--listen``（B3-lite，2026-11-14）：编排器上板后，两步握手不能再依赖有人在
开发机记得敲两条命令 —— 同一个只读观测循环里**自动顺序**做两步（看到 STOPPED
记第 1 步 → 看到 PLAYING+RUNNING+NORMAL 写令牌）。观测语义、写盘内容、
fail-closed 判据与 ``--step stop/--step play`` 逐条一致；``wait_token`` 的
fresh_min 核验照旧是独立的一道。它仍然**从不发送 play/stop**。

令牌写在 ``~/.config/ur5-vision-grasp/motion-auth.json``（仓外），**10 分钟**过期。
上一次的授权不覆盖下一次：每个动臂段落都要重新握手。

退出码：0 = 看到并写盘；2 = 超时/数据缺失/次序不对（都不授权）。
"""

import argparse
import json
import os
import secrets
import socket
import sys
import time
import datetime

DEFAULT_AUTH_DIR = os.path.join(os.path.expanduser('~'), '.config',
                                'ur5-vision-grasp')
STOP_FILE = 'motion-auth-stop.json'
TOKEN_FILE = 'motion-auth.json'
STOP_REQUEST_FILE = os.path.join(DEFAULT_AUTH_DIR, 'stop-request.json')
TOKEN_TTL_S = 600.0        # 令牌有效期：10 分钟
STOP_WINDOW_S = 600.0      # PLAY 必须在 STOP 之后这么久以内
DASHBOARD_PORT = 29999


# ---------------------------------------------------------------- 只读查询
def query(robot_ip, timeout=5.0):
    """**只读**问控制柜。返回 ``{命令: 回复}``；连不上/超时返回 ``None``。

    只发 ``robotmode`` / ``programState`` / ``safetystatus`` 三条查询。
    ⚠️ 绝不发 ``play`` / ``stop`` / ``power_on`` 之类 —— 那是人的动作。
    """
    try:
        s = socket.create_connection((robot_ip, DASHBOARD_PORT), timeout=timeout)
    except OSError:
        return None
    try:
        f = s.makefile('rw')
        f.readline()                      # 问候行，不拿来判状态
        out = {}
        for cmd in ('robotmode', 'programState', 'safetystatus'):
            f.write(cmd + '\n')
            f.flush()
            out[cmd] = f.readline().strip()
        return out
    except OSError:
        return None
    finally:
        s.close()


def _state(raw):
    """``"Program state: PLAYING x.urp"`` / ``"PLAYING x.urp"`` → ``PLAYING``。"""
    if not raw:
        return ''
    txt = str(raw)
    if ':' in txt:
        txt = txt.split(':', 1)[1]
    return txt.strip().split()[0].upper() if txt.strip() else ''


def _norm(reply):
    """Dashboard 回复 → 判定用的短状态（RUNNING / NORMAL / PLAYING …）。"""
    if not reply:
        return ''
    txt = str(reply)
    # robotmode / safetystatus 的回复形如 "Robotmode: RUNNING"
    if ':' in txt:
        txt = txt.split(':', 1)[1]
    return txt.strip().split()[0].upper() if txt.strip() else ''


def observe(robot_ip, timeout=5.0):
    """一次观测 → ``{'program','robotmode','safety','raw'}``；失败返回 None。"""
    raw = query(robot_ip, timeout=timeout)
    if raw is None:
        return None
    return {
        'program': _state(raw.get('programState')),
        'robotmode': _norm(raw.get('robotmode')),
        'safety': _norm(raw.get('safetystatus')),
        'raw': raw,
    }


def _poll(robot_ip, want, timeout, interval, extra=None):
    """轮询直到 ``want(obs)`` 为真；返回 (obs, 失败原因或 None)。"""
    deadline = time.time() + float(timeout)
    last = None
    while time.time() < deadline:
        obs = observe(robot_ip)
        if obs is not None:
            last = obs
            if want(obs) and (extra is None or extra(obs)):
                return obs, None
        time.sleep(max(0.2, float(interval)))
    if last is None:
        return None, '控制柜 %s:%d 一次都没答复（Dashboard 时通时不通）' % (
            robot_ip, DASHBOARD_PORT)
    return last, '超时 %.0f s：最后看到 program=%s robotmode=%s safety=%s' % (
        float(timeout), last['program'], last['robotmode'], last['safety'])


# ---------------------------------------------------------------- 两步
def _record_stop(obs, auth_dir):
    """第 1 步的落盘（do_stop 与 do_listen 共用）。返回记录。"""
    os.makedirs(auth_dir, exist_ok=True)
    rec = {'nonce': secrets.token_hex(8), 'at': time.time(),
           'at_iso': datetime.datetime.now().isoformat(timespec='seconds'),
           'observed': obs['raw'], 'program': obs['program']}
    with open(os.path.join(auth_dir, STOP_FILE), 'w', encoding='utf-8') as fh:
        json.dump(rec, fh, ensure_ascii=False, indent=1)
    return rec


def _write_token(obs, stop_rec, auth_dir):
    """第 2 步的落盘（do_play 与 do_listen 共用）。返回令牌。"""
    now = time.time()
    token = secrets.token_hex(16)
    doc = {
        'kind': 'stage4-motion-authorization',
        'token': token,
        'stop_nonce': stop_rec.get('nonce'),
        'stop_at': stop_rec.get('at'),
        'play_at': now,
        'play_at_iso': datetime.datetime.now().isoformat(timespec='seconds'),
        'expires': now + TOKEN_TTL_S,
        'observed_stop': stop_rec.get('observed'),
        'observed_play': obs['raw'],
        'note': 'STOP→PLAY 两步都是脚本从 Dashboard 只读查询里**看到**的；'
                '脚本从未发送 play/stop',
    }
    os.makedirs(auth_dir, exist_ok=True)
    with open(os.path.join(auth_dir, TOKEN_FILE), 'w', encoding='utf-8') as fh:
        json.dump(doc, fh, ensure_ascii=False, indent=1)
    return token


def do_stop(robot_ip, auth_dir, timeout, interval):
    print('>>> 需要人动手：示教器按 STOP（我随后开始等）')
    obs, why = _poll(robot_ip, lambda o: o['program'] == 'STOPPED',
                     timeout, interval)
    if why:
        print('!! 没看到 STOPPED：%s' % why)
        print('   ⇒ 不写任何授权（fail-closed）')
        return 2
    rec = _record_stop(obs, auth_dir)
    print('看到 STOPPED（robotmode=%s）⇒ 记下第 1 步' % obs['robotmode'])
    print('>>> 需要人动手：示教器按 PLAY（我随后开始等）')
    return 0


def do_play(robot_ip, auth_dir, timeout, interval, stop_window=STOP_WINDOW_S):
    stop_path = os.path.join(auth_dir, STOP_FILE)
    try:
        stop_rec = json.load(open(stop_path, encoding='utf-8'))
    except (OSError, ValueError):
        print('!! 没有第 1 步的记录 %s —— 先跑 --step stop' % stop_path)
        return 2
    age = time.time() - float(stop_rec.get('at', 0))
    if age > stop_window:
        print('!! 第 1 步（STOP）已过去 %.0f 秒（> %.0f）—— 重新握手'
              % (age, stop_window))
        return 2

    print('>>> 需要人动手：示教器按 PLAY（我随后开始等）')
    obs, why = _poll(
        robot_ip,
        lambda o: o['program'] == 'PLAYING',
        timeout, interval,
        extra=lambda o: o['robotmode'] == 'RUNNING' and o['safety'] == 'NORMAL')
    if why:
        print('!! 没看到 PLAYING+RUNNING+NORMAL：%s' % why)
        print('   ⇒ 不写令牌（fail-closed）')
        return 2
    token = _write_token(obs, stop_rec, auth_dir)
    print('看到 PLAYING（robotmode=%s，safety=%s）⇒ 两步握手成立' % (
        obs['robotmode'], obs['safety']))
    print('令牌：%s   （%.0f 秒内有效）' % (token, TOKEN_TTL_S))
    return 0


def do_listen(robot_ip, auth_dir, timeout, interval,
              stop_window=STOP_WINDOW_S):
    """自动顺序两步（板上轮内用，B3-lite）。

    与 ``--step stop/--step play`` 的人工接力的唯一区别是「谁记得喊下一步」：
    这里是同一个观测循环按次序自动推进，**观测门、窗口门、fail-closed 判据
    逐条相同**。两步超时/窗口超了 ⇒ 不写令牌退出 2 —— 编排器的 wait_token
    等不到新鲜令牌自然停轮。
    """
    print('>>> 需要人动手：确认两指整个行程内无手无遮挡、急停在手，')
    print('>>> 然后示教器先按 STOP，看到 STOPPED 后再按 PLAY。')
    obs, why = _poll(robot_ip, lambda o: o['program'] == 'STOPPED',
                     timeout, interval)
    if why:
        print('!! [listen] 没看到 STOPPED：%s —— 不写任何授权（fail-closed）'
              % why)
        return 2
    rec = _record_stop(obs, auth_dir)
    print('看到 STOPPED（robotmode=%s）⇒ 记下第 1 步' % obs['robotmode'])
    print('>>> 需要人动手：示教器按 PLAY（我随后开始等）')
    obs2, why2 = _poll(
        robot_ip,
        lambda o: o['program'] == 'PLAYING',
        timeout, interval,
        extra=lambda o: o['robotmode'] == 'RUNNING' and o['safety'] == 'NORMAL')
    if why2:
        print('!! [listen] 没看到 PLAYING+RUNNING+NORMAL：%s —— 不写令牌'
              '（fail-closed）' % why2)
        return 2
    age = time.time() - float(rec.get('at', 0))
    if age > stop_window:
        print('!! [listen] 第 1 步（STOP）已过去 %.0f 秒（> %.0f）—— 重新握手'
              % (age, stop_window))
        return 2
    token = _write_token(obs2, rec, auth_dir)
    print('看到 PLAYING（robotmode=%s，safety=%s）⇒ 两步握手成立' % (
        obs2['robotmode'], obs2['safety']))
    print('令牌：%s   （%.0f 秒内有效）' % (token, TOKEN_TTL_S))
    return 0


# ---------------------------------------------------------------- 令牌校验
def token_path(auth_dir=None):
    return os.path.join(auth_dir or DEFAULT_AUTH_DIR, TOKEN_FILE)


def check_token(token, auth_dir=None, now=None):
    """令牌是否成立。返回 ``(ok, 原因, 距离握手秒数或 None)``。只读，不改文件。"""
    now = time.time() if now is None else float(now)
    try:
        doc = json.load(open(token_path(auth_dir), encoding='utf-8'))
    except (OSError, ValueError):
        return False, '没有授权令牌（先做 STOP→PLAY 两步握手）', None
    if doc.get('token') != token:
        return False, '令牌不匹配（令牌来自另一次握手）', None
    exp = float(doc.get('expires') or 0)
    if now > exp:
        return False, '令牌已过期（超过 %.0f 秒）' % TOKEN_TTL_S, None
    if not doc.get('play_at') or not doc.get('stop_at'):
        return False, '令牌缺少 STOP/PLAY 两步记录', None
    return True, '', now - float(doc['play_at'])


def token_remaining(token, auth_dir=None, now=None):
    """令牌**还剩多少秒**（用板/本机当前时间判）。返回 ``(seconds, 原因)``。

    与 ``check_token`` 的分工：它只回答「成不成立」；整轮编排器要回答的是
    「够不够跑完剩下的段」（R1/R2）与执行脚本首行的 ``--min-remaining`` 守卫，
    所以这里把剩余秒数直接给出来。令牌不成立时 ``seconds=None``、``原因`` 说明
    是哪一种（没有 / 不匹配 / 过期 / 缺两步记录）。只读，不改文件。
    """
    now = time.time() if now is None else float(now)
    try:
        doc = json.load(open(token_path(auth_dir), encoding='utf-8'))
    except (OSError, ValueError):
        return None, '没有授权令牌（先做 STOP→PLAY 两步握手）'
    if doc.get('token') != token:
        return None, '令牌不匹配（令牌来自另一次握手）'
    if not doc.get('play_at') or not doc.get('stop_at'):
        return None, '令牌缺少 STOP/PLAY 两步记录'
    rem = float(doc.get('expires') or 0) - now
    if rem <= 0:
        return None, '令牌已过期（超过 %.0f 秒）' % TOKEN_TTL_S
    return rem, ''


def stop_requested(path=None):
    """软停请求（``watch_round.py --stop`` 写入）是否在场。``(bool, 内容)``。

    B3-lite 接口铁律的落点：本机对轮内**唯一**的指令只能「减权」——这个文件
    让本轮在下一个计划边界提前停，除此之外**没有任何效果**；不存在对应的
    「继续 / 加权」通道。内容只做日志（谁在何时因何停）。
    """
    p = path or STOP_REQUEST_FILE
    try:
        with open(p, encoding='utf-8') as fh:
            doc = json.load(fh)
    except (OSError, ValueError):
        return False, {}
    return bool(doc.get('requested')), doc


# ---------------------------------------------------------------- CLI
def main(argv=None):
    ap = argparse.ArgumentParser(
        description='真机运动授权：STOP→PLAY 两步只读核验（从不发送 play/stop）')
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--step', choices=('stop', 'play'))
    g.add_argument('--listen', action='store_true',
                   help='板上轮内用：自动顺序两步（看到 STOPPED → 看到 '
                        'PLAYING+RUNNING+NORMAL 才写令牌；观测语义与手动两步'
                        '逐条一致，仍然从不发送 play/stop）')
    g.add_argument('--check-token', default=None,
                   help='只校验令牌（不动控制柜）')
    ap.add_argument('--min-remaining', type=float, default=0.0,
                    help='配合 --check-token：剩余秒数低于它就算不成立（退出 2）。'
                         '执行脚本首行用它，别让令牌在跑到一半时过期')
    ap.add_argument('--robot-ip', default=os.environ.get('UR_ROBOT_IP'),
                    help='控制柜 IP（默认环境变量 UR_ROBOT_IP）')
    ap.add_argument('--timeout', type=float, default=120.0)
    ap.add_argument('--interval', type=float, default=0.5)
    ap.add_argument('--auth-dir', default=DEFAULT_AUTH_DIR)
    args = ap.parse_args(argv)

    if args.check_token:
        rem, why = token_remaining(args.check_token, args.auth_dir)
        if rem is None:
            print('不成立：%s' % why)
            return 2
        if rem < float(args.min_remaining):
            print('不成立：令牌只剩 %.0f 秒（要求 ≥ %.0f 秒）—— 现在发会让它'
                  '跑到一半过期' % (rem, args.min_remaining))
            return 2
        print('OK：握手在 %.0f 秒前完成，剩 %.0f 秒'
              % (TOKEN_TTL_S - rem, rem))
        return 0

    if not args.robot_ip:
        print('!! 缺 --robot-ip（环境变量 UR_ROBOT_IP 也没设）')
        return 2
    if args.step == 'stop':
        return do_stop(args.robot_ip, args.auth_dir, args.timeout, args.interval)
    if args.listen:
        return do_listen(args.robot_ip, args.auth_dir, args.timeout,
                         args.interval)
    return do_play(args.robot_ip, args.auth_dir, args.timeout, args.interval)


if __name__ == '__main__':
    sys.exit(main())

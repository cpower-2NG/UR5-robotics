#!/usr/bin/env python3
"""``stage4_preflight_expect.py`` 自检（离线，不碰板子 / 不碰臂）。

钉住三件事：
1. 期望值里的 ``tree`` 两项由**本地检出**算出，且用的是与板上预检同一份代码
   （``PREFLIGHT_TREE_CODE``），不是另写一份口径；
2. 身份字段（``serial`` / ``stack`` / 备注）**原样继承**旧基线 —— 重算树哈希
   不该顺手把「认亲」判据改宽；
3. 坏输入（旧基线没 serial / 目录空了 / 文件不存在）必须非 0，且不写半成品。

跑法：``python3 scripts/test_stage4_preflight_expect.py``；退出码 0 = 全通过。
"""

import hashlib
import json
import os
import subprocess
import sys
import tempfile

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import stage4_round_run as rr                                    # noqa: E402

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


def _run(repo, old, out):
    return subprocess.run(
        [sys.executable, os.path.join(HERE, 'stage4_preflight_expect.py'),
         '--repo', repo, '--from', old, '--out', out],
        capture_output=True, text=True)


def _local_scan(repo):
    """直接跑共享代码串，作为「同一口径」的对照。"""
    ns = {'hashlib': hashlib, 'os': os}
    exec(rr.PREFLIGHT_TREE_CODE, ns)
    return ns['scan_tree'](repo)


OLD = {'serial': 'SN-TEST', 'wrist_usb3': True,
       'stack': {'ur_robot_driver': True, 'move_group': True},
       'note': 'baseline'}

print('--- A. 正常生成 ---')
with tempfile.TemporaryDirectory() as tt:
    repo = os.path.join(tt, 'repo')       # out 必须写在 repo **外**，否则自己进哈希
    os.makedirs(os.path.join(repo, 'scripts'))
    for name, body in (('a.py', 'x = 1\n'), ('b.py', 'y = 2\n')):
        with open(os.path.join(repo, 'scripts', name), 'w') as f:
            f.write(body)
    old = os.path.join(tt, 'old.json')
    out = os.path.join(tt, 'new.json')
    with open(old, 'w') as f:
        json.dump(OLD, f)

    r = _run(repo, old, out)
    check('rc=0', r.returncode == 0, r.stderr[-400:])
    got = json.load(open(out))
    check('serial 原样继承', got.get('serial') == 'SN-TEST', repr(got))
    check('stack 原样继承', got.get('stack') == OLD['stack'], repr(got))
    check('note 原样继承', got.get('note') == 'baseline', repr(got))
    check('tree_files = 2', got.get('tree_files') == 2,
          repr(got.get('tree_files')))
    ref = _local_scan(repo)
    check('tree_sha256 与板上预检同一份代码算出的一致',
          got.get('tree_sha256') == ref['sha256'],
          '%r vs %r' % (got.get('tree_sha256'), ref['sha256']))
    check('共享代码串确实在 PREFLIGHT_PY 里（不是第二份实现）',
          rr.PREFLIGHT_TREE_CODE in rr.PREFLIGHT_PY)

    # 改一个字节 ⇒ 哈希必须变
    with open(os.path.join(repo, 'scripts', 'a.py'), 'w') as f:
        f.write('x = 2\n')
    _run(repo, old, out)
    got2 = json.load(open(out))
    check('源码变一个字节 ⇒ 哈希变',
          got2['tree_sha256'] != got['tree_sha256'], repr(got2))

    # .pyc / 构建产物 / 备份目录不进哈希
    os.makedirs(os.path.join(repo, 'scripts', '__pycache__'))
    with open(os.path.join(repo, 'scripts', '__pycache__', 'a.pyc'), 'wb') as f:
        f.write(b'\x00junk')
    os.makedirs(os.path.join(repo, 'ros2_ws', 'build', 'pkg'))
    with open(os.path.join(repo, 'ros2_ws', 'build', 'pkg', 'o.o'), 'wb') as f:
        f.write(b'obj')
    os.makedirs(os.path.join(repo, 'scripts.bak-20260101'))
    with open(os.path.join(repo, 'scripts.bak-20260101', 'z.py'), 'w') as f:
        f.write('z = 3\n')
    _run(repo, old, out)
    got3 = json.load(open(out))
    check('构建产物 / 备份目录不进哈希（否则每次部署假红）',
          got3['tree_sha256'] == got2['tree_sha256']
          and got3['tree_files'] == got2['tree_files'], repr(got3))

    # worktree 里 `.git` 是文件（gitdir 指针），不是目录 —— 板上是普通目录复制，
    # 漏跳它就永远差一个文件（2026-11-12 实测本地 370 / 板上 369）。
    with open(os.path.join(repo, '.git'), 'w') as f:
        f.write('gitdir: /somewhere/.git/worktrees/x\n')
    _run(repo, old, out)
    got4 = json.load(open(out))
    check('.git 文件不进哈希（本地 worktree 与板上目录才可比）',
          got4['tree_sha256'] == got3['tree_sha256']
          and got4['tree_files'] == got3['tree_files'], repr(got4))

print('--- B. 坏输入 fail-closed ---')
with tempfile.TemporaryDirectory() as tt:
    old_no_serial = os.path.join(tt, 'old-no-serial.json')
    with open(old_no_serial, 'w') as f:
        json.dump({'stack': {}}, f)
    out = os.path.join(tt, 'new.json')
    r = _run(tt, old_no_serial, out)
    check('旧基线缺 serial ⇒ 非 0', r.returncode != 0, repr(r.returncode))
    check('旧基线缺 serial ⇒ 不写文件', not os.path.exists(out), repr(out))

    old = os.path.join(tt, 'old.json')
    with open(old, 'w') as f:
        json.dump(OLD, f)
    empty = os.path.join(tt, 'empty')
    os.makedirs(empty)
    r = _run(empty, old, out)
    check('本地树 0 个文件 ⇒ 非 0（--repo 指错）', r.returncode != 0,
          repr(r.returncode))
    check('本地树 0 个文件 ⇒ 不写文件', not os.path.exists(out), repr(out))

    r = _run(tt, os.path.join(tt, 'nope.json'), out)
    check('--from 不存在 ⇒ rc 2', r.returncode == 2, repr(r.returncode))

print()
print('通过 %d，失败 %d' % (N_PASS, N_FAIL))
if FAILS:
    print('失败项：')
    for f in FAILS:
        print('  - %s' % f)
sys.exit(0 if N_FAIL == 0 else 1)

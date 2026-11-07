#!/usr/bin/env python3
"""`wrist_tf_publish.py` 的纯函数自检（**不需要 ROS、不需要真机**）。

只测「文件能不能用」那一半：帧名、有限数、四元数模长，以及 `--verify` 用到的
逐项比对。**真正的 TF 通不通不在这里测** —— 那要一台活着的驱动 + 相机，
是 `--verify` 存在的理由本身。

为什么这几条值得测：它们全是**错了不会报错**的那种 —— 拿错标定文件（发到
光学帧的旧版本）会让 `camera_link` 有两个父节点；NaN 会被 TF 静默发出去；
非单位四元数一归一化就**改掉了**标定值。

跑法：
    python3 scripts/test_wrist_tf_publish.py
退出码 0 = 全通过。
"""

import json
import math
import os
import sys
import tempfile

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from wrist_tf_publish import (CalibError, DEFAULT_CALIB, WANT_CHILD,  # noqa: E402
                              WANT_PARENT, describe, load_calib,
                              parse_calib, same_transform)

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


def raises(fn, *a, **kw):
    """跑一个**应当抛** `CalibError` 的调用；没抛就把失败原因当返回值。"""
    try:
        fn(*a, **kw)
    except CalibError as exc:
        return True, str(exc)
    except Exception as exc:                       # noqa: BLE001
        return False, '抛的不是 CalibError：%s: %s' % (type(exc).__name__, exc)
    return False, '没有抛异常'


def doc(**over):
    """一份最小可用标定文档；用关键字覆盖任意字段。"""
    d = {
        'parent_frame': WANT_PARENT,
        'child_frame': WANT_CHILD,
        'translation': {'x': 0.1, 'y': -0.2, 'z': 0.3},
        'rotation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
    }
    d.update(over)
    return d


def with_file(contents, fn, *a, **kw):
    """把 ``contents`` 写成临时 JSON 再跑 ``fn(path, ...)``。"""
    fd, path = tempfile.mkstemp(suffix='.json')
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as fh:
            if isinstance(contents, str):
                fh.write(contents)
            else:
                json.dump(contents, fh)
        return fn(path, *a, **kw)
    finally:
        os.unlink(path)


def main():
    print('=' * 70)
    print('  wrist_tf_publish 纯函数自检（无 ROS）')
    print('=' * 70)

    # ---- 1. 仓内那份真文件 ----
    cal = parse_calib(load_calib(DEFAULT_CALIB))
    check('仓内标定文件可解析', cal['parent'] == WANT_PARENT
          and cal['child'] == WANT_CHILD, describe(cal))
    check('仓内四元数已归一', abs(math.sqrt(sum(v * v for v in cal['q'])) - 1.0)
          < 1e-6, repr(cal['q']))
    check('describe 同时写父帧与子帧',
          WANT_PARENT in describe(cal) and WANT_CHILD in describe(cal))

    ok, msg = same_transform(cal['t'], cal['q'], cal['t'], cal['q'])
    check('自己与自己比：一致', ok, msg)

    # ---- 2. 帧名：拿错文件必须拦 ----
    ok, why = raises(parse_calib, doc(parent_frame='camera_color_optical_frame'))
    check('父帧不对 ⇒ 拒（旧光学帧标定）', ok, why)

    ok, why = raises(parse_calib, doc(child_frame='camera_color_optical_frame'))
    check('子帧不对 ⇒ 拒', ok, why)

    ok, why = raises(parse_calib, {k: v for k, v in doc().items()
                                   if k != 'parent_frame'})
    check('缺 parent_frame ⇒ 拒', ok, why)

    # ---- 3. 数值：NaN / inf / 缺项 / 非数 ----
    d = doc()
    d['translation']['x'] = float('nan')
    ok, why = raises(parse_calib, d)
    check('NaN 平移 ⇒ 拒', ok, why)

    d = doc()
    d['rotation']['w'] = float('inf')
    ok, why = raises(parse_calib, d)
    check('inf 旋转 ⇒ 拒', ok, why)

    d = doc()
    del d['translation']['z']
    ok, why = raises(parse_calib, d)
    check('缺 z ⇒ 拒', ok, why)

    d = doc()
    d['translation']['x'] = '0.1'
    ok, why = raises(parse_calib, d)
    check('字符串当数 ⇒ 拒', ok, why)

    d = doc()
    d['translation']['x'] = True
    ok, why = raises(parse_calib, d)
    check('布尔当数 ⇒ 拒', ok, why)

    # ---- 4. 四元数模长 ----
    ok, why = raises(parse_calib, doc(rotation={'x': 0, 'y': 0, 'z': 0, 'w': 0}))
    check('模长 0 ⇒ 拒（不是旋转）', ok, why)

    # 0.9 * 单位四元数 = 模长 0.9，归一化会改掉标定值
    ok, why = raises(parse_calib, doc(rotation={'x': 0, 'y': 0, 'z': 0, 'w': 0.9}))
    check('模长 0.9 ⇒ 拒（不许静默归一化）', ok, why)

    # 1.0005 在容差内：接受（数值来源的末位噪声）
    parse_calib(doc(rotation={'x': 0, 'y': 0, 'z': 0, 'w': 1.0005}))
    check('模长 1.0005（容差内）⇒ 接受', True)

    # ---- 5. 比对：q 与 -q 是同一个旋转 ----
    t = [0.0, 0.0, 0.0]
    q = [0.1, 0.2, 0.3, math.sqrt(1 - 0.14)]
    ok, msg = same_transform(t, q, t, [-v for v in q])
    check('q 整体取负 ⇒ 一致', ok, msg)

    ok, msg = same_transform([0.001, 0, 0], q, t, q, t_tol_mm=0.5)
    check('t 差 1 mm 超 0.5 mm 容差 ⇒ 不一致', not ok, msg)

    ok, msg = same_transform([0.0001, 0, 0], q, t, q, t_tol_mm=0.5)
    check('t 差 0.1 mm 在容差内 ⇒ 一致', ok, msg)

    ok, msg = same_transform(t, [0.2, 0.2, 0.3, q[3]], t, q)
    check('q 差 0.1 ⇒ 不一致', not ok, msg)

    # ---- 6. 文件层：不存在 / 坏 JSON / 顶层不是对象 ----
    ok, why = raises(load_calib, '/nonexistent/wrist.json')
    check('文件不存在 ⇒ 拒', ok, why)

    ok, why = raises(with_file, '{not json', load_calib)
    check('坏 JSON ⇒ 拒', ok, why)

    ok, why = raises(with_file, [1, 2, 3], load_calib)
    check('顶层是数组 ⇒ 拒', ok, why)

    # ---- 汇总 ----
    print('-' * 70)
    if N_FAIL:
        print('  %d 项失败 / 共 %d 项：%s' % (N_FAIL, N_PASS + N_FAIL,
                                            ', '.join(FAILS)))
        return 1
    print('  %d 项全部通过' % N_PASS)
    return 0


if __name__ == '__main__':
    sys.exit(main())

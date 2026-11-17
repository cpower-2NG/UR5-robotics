#!/usr/bin/env python3
"""``emergency_home.py`` 的接线自检（只查源码结构 + 纯逻辑，不连 ROS、不动臂）。

钉住「意外现场不能悄悄放宽」的几处：
  · 缺省**不动臂**（没有 ``--go`` 在授权之前就返回）；
  · ``--go`` 必须有**本次**新令牌（``play_at`` 变化 + 剩余 ≥120 s）——
    红线 6 的 fail-closed 在源码里能看见；
  · 段级裁决默认不可达即停（``--allow-no-validator`` 必须显式）；
  · 执行只走 ``precision_exec --require-auth``，且认 ``ALL DONE``；
  · HOME 唯一出处是 ``home_pose``（不另写一套关节值）。

跑法：python3 scripts/test_emergency_home.py
"""
import ast
import os
import sys

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "emergency_home.py")

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
        print('  FAIL  %s%s' % (name, ('  ' + str(detail)) if detail else ''))


def main():
    src = open(SRC, encoding='utf-8').read()
    tree = ast.parse(src)
    funcs = {n.name for n in ast.walk(tree)
             if isinstance(n, ast.FunctionDef)}
    for fn in ("main", "wait_for_new_token", "segment_check", "read_joints"):
        check('定义 %s()' % fn, fn in funcs)

    # --go 之前必须返回（缺省不动臂）。注意找**调用**而不是函数定义：
    # 定义在 main 之前。
    i_go = src.find("if not args.go:")
    i_wait = src.find("tok = wait_for_new_token(")
    check('缺省（无 --go）在授权/执行之前返回',
          0 <= i_go < i_wait, (i_go, i_wait))
    check('缺省分支返回 EXIT_OK 且提示 --go',
          "return EXIT_OK" in src[i_go:i_wait]
          and "--go" in src[i_go:i_wait])

    # 红线 6：本次新令牌 + 剩余下限
    check('wait_for_new_token：play_at 必须变化',
          "play != old_play" in src)
    check('wait_for_new_token：剩余 ≥ MIN_REMAINING_S',
          "remain >= MIN_REMAINING_S" in src)
    check('MIN_REMAINING_S = 120（与 park_arm 同口径）',
          "MIN_REMAINING_S = 120.0" in src)
    check('授权监听 = real_motion_authorize --listen',
          "real_motion_authorize.py" in src and '"--listen"' in src)

    # 段级裁决 fail-closed + 显式豁免
    check('段级裁决不可达默认 SystemExit(EXIT_GATE)',
          "if not allow_no_validator:" in src
          and "raise SystemExit(EXIT_GATE)" in src)
    check('豁免时大声告警并写进计划元数据',
          "已显式豁免" in src and "allow_no_validator" in src)

    # 执行只走 precision_exec + require-auth + ALL DONE
    check('执行走 precision_exec --go --require-auth',
          '"--require-auth"' in src and '"--go"' in src
          and "precision_exec.py" in src)
    check('执行认 ALL DONE（没有即非 0 退出）',
          "ALL DONE" in src and "return EXIT_EXEC" in src)

    # HOME 唯一出处
    check('HOME 用 home_pose.plan_park（不另写关节值）',
          "hp.plan_park(" in src and "HOME_JOINTS_DEG" in src)
    check('源码里不出现裸 HOME 度数表（唯一出处）',
          "HOME_JOINTS_DEG = " not in src)

    # 旧监听先清（b3demo 的坑）
    check('启动前清孤儿监听（字符类防自匹配）',
          '"[r]eal_motion_authorize.py --listen"' in src)

    print('\n通过 %d，失败 %d' % (N_PASS, N_FAIL))
    if FAILS:
        print('失败项：')
        for f in FAILS:
            print('  - %s' % f)
    return 0 if N_FAIL == 0 else 1


if __name__ == '__main__':
    sys.exit(main())

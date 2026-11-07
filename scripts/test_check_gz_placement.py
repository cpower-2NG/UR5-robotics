#!/usr/bin/env python3
"""check_gz_placement.py 的离线自检（不碰 gz、不碰 ROS）。

为什么必须有
============
`check_gz_placement.py` 的**负对照**是「场景里一块都没放进去 ⇒ 必须报 0/5」。
但一个永远输出 0 的坏检查器也能过负对照 —— 所以这里再喂**已知答案**的合成
位姿，要求它在「真在槽里」时报 1、在「悬空/掉地上/偏太远/摔翻」时报 0，
并逐条覆盖 A4 的整轮判据（一块一格、两两距离、拾取区残留、A4 PASS）。

用法：python3 scripts/test_check_gz_placement.py
"""
from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("cgp", HERE / "check_gz_placement.py")
cgp = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cgp)

# 与 grasp_planner_sim.yaml 的 place_slots 同值；这里只为离线测试提供固定输入，
# 生产代码仍然只从 yaml 读（见 check_gz_placement.py）。
SLOTS = [(0.2942, -0.17724), (0.4242, -0.17724), (0.5542, -0.17724),
         (0.2942, -0.04724), (0.4242, -0.04724), (0.5542, -0.04724)]
TOL_M = 0.040
Z_MIN, Z_MAX = 0.012, 0.023
PREFIX = "block"
MAX_TILT = 15.0
MIN_PAIR = 0.025
PICK_ZONE = (0.25, 0.575, 0.17, 0.42)


def pose(name, x, y, z, tilt_deg=0.0, yaw_deg=0.0):
    """竖直方块 + 可加绕 x 的倾角、绕 z 的偏航（偏航不影响 z 轴）。"""
    # q = qx(tilt) * qz(yaw)，都为小角度，直接乘
    a = math.radians(tilt_deg) / 2
    b = math.radians(yaw_deg) / 2
    qx1, qw1 = math.sin(a), math.cos(a)
    qz2, qw2 = math.sin(b), math.cos(b)
    # 四元数乘法 q1 * q2（q1 绕 x，q2 绕 z）
    qx = qx1 * qw2
    qy = -qx1 * qz2          # x 轴 × z 轴 → -y
    qz = qw1 * qz2
    qw = qw1 * qw2
    return {"name": name, "x": x, "y": y, "z": z,
            "qx": qx, "qy": qy, "qz": qz, "qw": qw}


def run(name, poses, want_placed, want_a4=None, want_conflicts=None,
        want_pair_viol=None, want_leftovers=None):
    r = cgp.classify(poses, SLOTS, TOL_M, Z_MIN, Z_MAX, PREFIX,
                     MAX_TILT, MIN_PAIR, PICK_ZONE)
    ok = r["n_placed"] == want_placed
    if want_a4 is not None:
        ok = ok and r["a4_ok"] == want_a4
    if want_conflicts is not None:
        ok = ok and (len(r["conflicts"]) > 0) == want_conflicts
    if want_pair_viol is not None:
        ok = ok and (len(r["pairwise_violations"]) > 0) == want_pair_viol
    if want_leftovers is not None:
        ok = ok and (len(r["pick_zone_leftovers"]) > 0) == want_leftovers
    print(f"{'OK  ' if ok else 'FAIL'} {name}: 已放置 {r['n_placed']}/"
          f"{r['n_blocks']}，A4={'PASS' if r['a4_ok'] else 'FAIL'}"
          f"（期望放置 {want_placed}）")
    return ok


def main():
    pickup = [pose(f"block_{i}", 0.30 + 0.05 * i, 0.20, 0.01475) for i in range(5)]
    cases = [
        ("全在拾取区（负对照形状）", pickup, 0, False, False, False, True),
        ("一块在槽位 #4", [pose("block_0", 0.4242, -0.04724, 0.01763)]
         + pickup[1:], 1, False, False, False, True),
        ("槽位 xy 但悬空 0.2 m（还在爪里）",
         [pose("block_0", 0.4242, -0.04724, 0.20)], 0, False, None, None, False),
        ("槽位 xy 但掉到地上", [pose("block_0", 0.4242, -0.04724, -0.72)],
         0, False, None, None, False),
        ("偏 35 mm 仍算在槽里（门 40）",
         [pose("block_0", 0.4242 + 0.035, -0.04724, 0.01763)], 1, None, None, None, False),
        ("偏 45 mm 不算",
         [pose("block_0", 0.4242 + 0.045, -0.04724, 0.01763)], 0, None, None, None, False),
        ("摔翻 30° 不算",
         [pose("block_0", 0.4242, -0.04724, 0.01763, tilt_deg=30.0)], 0, None, None, None, False),
        ("偏航 45° 仍算（z 轴没变）",
         [pose("block_0", 0.4242, -0.04724, 0.01763, yaw_deg=45.0)], 1, None, None, None, False),
        ("非 block 前缀不参与",
         [pose("table", 0.4242, -0.04724, 0.01763)], 0, None, None, None, False),
    ]
    oks = [run(*c) for c in cases]

    oks.append(run("同槽两块标冲突",
                   [pose("block_0", 0.4242, -0.04724, 0.01763),
                    pose("block_1", 0.4242, -0.04724, 0.01763)],
                   2, False, True, None, False))

    oks.append(run("两两 20 mm 标叠放",
                   [pose("block_0", 0.4242, -0.04724, 0.01763),
                    pose("block_1", 0.4242, -0.04724, 0.03763)],
                   1, False, False, True, False))

    # 整轮 A4 全过：5 块落 5 个不同槽位
    full = [pose(f"block_{i}", *SLOTS[i], 0.01763) for i in range(5)]
    oks.append(run("5 块 5 槽，A4 应 PASS", full, 5, True, False, False, False))

    # 槽位解析：扁平 → 成对；奇数长度必须拒绝（静默少一个槽位是事故）
    flat = {"place_slots": [0.1, 0.2, 0.3, 0.4]}
    assert cgp.slots_from_params(flat) == [(0.1, 0.2), (0.3, 0.4)]
    try:
        cgp.slots_from_params({"place_slots": [0.1, 0.2, 0.3]})
        print("FAIL 奇数 place_slots 没被拒")
        oks.append(False)
    except RuntimeError:
        print("OK   奇数 place_slots 被拒")
        oks.append(True)

    print("ALL OK" if all(oks) else "SOME FAILED")
    return 0 if all(oks) else 1


if __name__ == "__main__":
    sys.exit(main())

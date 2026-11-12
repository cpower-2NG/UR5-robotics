#!/usr/bin/env python3
"""``precision_capture`` 的配对逻辑自检：按**时间戳**取帧，不取「最新」。

为什么值得单测：抓帧工具写出的 `meta / tf / joint_states` 是「帧、关节角、TF
是同一时刻」这个全链前提的唯一证据。2026-11-12 第 321 轮实测它已经破了 ——
同一组 burst 里 TF↔图像时间差从 +42 ms 线性恶化到 −1793 ms，因为写盘期间不
自旋、每帧都拿订阅队列里最旧的那条。本轮把配对改成「按时间戳挑最近的」，
本文件把这条规则钉住：**改回取最新、把容差放宽、或删掉流鲜度检查，都必须红**。

**不 import rclpy / numpy / PIL** —— 被测的都是纯函数，本机没 ROS 也能跑。

跑法：
    python3 scripts/test_precision_capture.py
"""
import os
import sys
import types

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import precision_capture as pc                                   # noqa: E402

N_PASS = N_FAIL = 0
FAILS = []

MS = 10 ** 6


def check(name, cond, detail=''):
    global N_PASS, N_FAIL
    if cond:
        N_PASS += 1
        print('  PASS  %s' % name)
    else:
        N_FAIL += 1
        FAILS.append(name)
        print('  FAIL  %s%s'
              % (name, '' if detail == '' else '   ← %r' % (detail,)))


def stamp_obj(ns):
    """只要有 header.stamp 的最小声呐的形状（`stamp_ns` 只读这两个字段）。"""
    return types.SimpleNamespace(
        header=types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=ns // 10 ** 9, nanosec=ns % 10 ** 9)))


def tf_obj(name):
    """一个可辨识的假 transform：pick_chain 只把它原样带出来。"""
    t = types.SimpleNamespace(frame_id=name, child_frame_id="c-" + name)
    t.header = types.SimpleNamespace(frame_id=name, child_frame_id="c-" + name)
    t.transform = types.SimpleNamespace(
        translation=types.SimpleNamespace(x=1.0, y=2.0, z=3.0),
        rotation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
    return t


def js_obj(t_ns, pos):
    m = stamp_obj(t_ns)
    m.name = ["a", "b"]
    m.position = list(pos)
    return m


def section_nearest():
    print("\n[1] nearest / match：按时间戳挑，不是「取最新」")
    c = 1000 * MS
    hist = [(c - 33 * MS, "before"), (c, "exact"), (c + 33 * MS, "after")]
    hit = pc.nearest(hist, c)
    check("nearest 给出与图像**同戳**的那条（不是表尾那条）",
          hit[2] == "exact" and hit[0] == 0, hit)
    hit = pc.nearest(hist[:1], c)
    check("只有旧帧时退回最近的一条，并把差量报出来",
          hit[2] == "before" and hit[0] == 33 * MS, hit)
    check("空历史 ⇒ None", pc.nearest([], c) is None)

    hit = pc.match(hist, c, pc.MATCH_TOL_MS, wait_s=0.0)
    check("★ match 在容差内**优先同戳**（旧实现会拿到 +33 ms 的另一帧）",
          hit is not None and hit[2] == "exact", hit)
    hit = pc.match([(c - 100 * MS, "old")], c, pc.MATCH_TOL_MS, wait_s=0.0)
    check("★ 差 100 ms > 容差 40 ms ⇒ 配不上（返回 None），不拿别的帧冒充",
          hit is None, hit)
    hit = pc.match([], c, pc.MATCH_TOL_MS, wait_s=0.0)
    check("历史为空且等不到 ⇒ None", hit is None, hit)
    # 等一个还没到的同戳帧：sleep 是注入的，第 2 次调用时“新帧到达”。
    arrived = [(c - 33 * MS, "before")]
    calls = []

    def sleep(_s):
        calls.append(1)
        if len(calls) == 2:
            arrived.append((c, "exact"))

    hit = pc.match(arrived, c, pc.MATCH_TOL_MS, wait_s=1.0, sleep=sleep)
    check("同戳帧晚到一点：**等**它，而不是立刻退到最近的一条",
          hit is not None and hit[2] == "exact" and len(calls) == 2, hit)


def section_pick_chain():
    print("\n[2] pick_chain：静态取最新、动态按图像时间戳挑，流断了要报")
    keys = ("s1", "d1")
    static = {"s1": tf_obj("s1")}
    c = 10_000 * MS
    dyn = {"d1": [(c - 8 * MS, tf_obj("d-old")),
                  (c, tf_obj("d-exact")),
                  (c + 8 * MS, tf_obj("d-new"))]}

    trans, origin, deltas, why = pc.pick_chain(keys, static, dyn, c, c)
    check("静态帧来自 /tf_static、动态帧来自 /tf",
          trans is not None and origin["s1"] == "tf_static"
          and origin["d1"] == "tf", (origin, why))
    check("★ 动态帧取**与图像同戳**的那条，不是表尾那条",
          trans is not None and trans["d1"].frame_id == "d-exact",
          None if trans is None else trans["d1"].frame_id)
    check("时间差记账为 0 ms（同戳）", deltas == [0.0], deltas)

    # 动态帧比图像旧 1.8 s（第 321 轮的实际数字）⇒ 必须拒绝，而不是照写。
    bad = {"d1": [(c - 1800 * MS, tf_obj("d-stale"))]}
    trans, origin, deltas, why = pc.pick_chain(keys, static, bad, c, c)
    check("★ 动态 TF 比图像旧 1.8 s ⇒ 拒绝（why 点名「配不上」）",
          trans is None and why and "配不上" in why, why)

    # 缺链
    trans, origin, deltas, why = pc.pick_chain(keys, static, {}, c, c)
    check("缺动态帧 ⇒ trans=None 且 why 点名缺哪个键",
          trans is None and "d1" in (why or ""), why)

    # 流断了：最近一帧离“现在”1.2 s（但离图像只有 0 ms，因为图像更旧）
    old_c = 1_000 * MS
    dyn_old = {"d1": [(old_c, tf_obj("d"))]}
    trans, origin, deltas, why = pc.pick_chain(keys, static, dyn_old, old_c,
                                               old_c + 1200 * MS)
    check("★ 动态 TF 流断了（最近一帧已是 1.2 s 前）⇒ 拒绝，不拿旧帧写证据",
          trans is None and why and "流断了" in why, why)

    # 俯视那条链是纯静态的：不应要求任何动态帧。
    t_chain = pc.TOPCAM_CHAIN
    t_static = {k: tf_obj(k) for k in t_chain}
    trans, origin, deltas, why = pc.pick_chain(t_chain, t_static, {}, 0, 0)
    check("俯视链全静态 ⇒ 成功、无动态时间差",
          trans is not None and deltas == [] and why is None, why)
    check("俯视链的 origin 全是 /tf_static",
          trans is not None and set(origin.values()) == {"tf_static"}, origin)


def section_joints():
    print("\n[3] joints_at：把关节角插值到图像时间戳（不是取最近一条）")
    hist = [js_obj(0, [0.0, 1.0]), js_obj(10 * MS, [0.1, 1.1])]
    names, pos, resid, interp = pc.joints_at(hist, 5 * MS)
    check("图像落在两条之间 ⇒ 真插值，取中点",
          interp is True and abs(pos[0] - 0.05) < 1e-9, (pos, interp))
    check("残差 = 到最近端点的距离（5 ms）", abs(resid - 5.0) < 1e-6, resid)
    names, pos, resid, interp = pc.joints_at(hist, 30 * MS)
    check("图像晚于整段历史 ⇒ 退化成最近一条并标 interp=False",
          interp is False and abs(pos[0] - 0.1) < 1e-9, (pos, interp))
    check("历史为空 ⇒ None", pc.joints_at([], 0) is None)


def main():
    print("precision_capture 配对逻辑自检（纯函数，不需要 ROS）")
    section_nearest()
    section_pick_chain()
    section_joints()
    print("\n" + "=" * 72)
    print("PASS %d   FAIL %d" % (N_PASS, N_FAIL))
    if FAILS:
        print("失败项：")
        for f in FAILS:
            print("  - %s" % f)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

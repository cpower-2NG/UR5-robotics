#!/usr/bin/env python3
"""E5 评估器自检：命令生成 / 独立复核 / 判分三块。

为什么值得单测「独立复核」：它是**防评估器自己骗自己**的那道闸 —— 命令集是拿
解析器生成的，如果判分也只用解析器，两边同源就会一起错。复核必须能在命令指向
错块时喊 False，否则它是个装饰。

跑法：
    python3 scripts/test_stage4_e5_eval.py
"""
import inspect
import os
import sys

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import stage4_e5_eval as ev                                   # noqa: E402
import stage4_specify as sp                                   # noqa: E402

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
        print('  FAIL  %s%s' % (name, '   ← %s' % detail if detail else ''))


def tb(color, tid, x_mm, y_mm, top_mm=29.5, yaw=0.0, size_m=0.0295):
    """真值块：**米**，与 truth.json 同单位同字段。"""
    return {"color": color, "id": tid, "layer": 0, "size": size_m,
            "top": top_mm / 1000.0, "x": x_mm / 1000.0, "y": y_mm / 1000.0,
            "yaw": yaw, "z": 0.0}


def truth(blocks):
    return {"batch": "t", "block_size_m": 0.0295, "blocks": blocks}


def _texts(cmds, target):
    return [c["text"] for c in cmds if c["target"] == target]


# ------------------------------------------------------------------ 命令生成
def section_gen():
    print('\n[1] 命令生成：只留真值上唯一指向该块的说法')
    t = truth([tb("red", 0, 300, 200), tb("red", 1, 300, 260),
               tb("cyan", 2, 450, 330)])
    cmds, skipped, _recs, dis = ev.gen_commands(t)

    check("无独立复核分歧", not dis, str(dis))
    check("右红（y 小）拿到 'red right'", "red right" in _texts(cmds, 0),
          str(_texts(cmds, 0)))
    check("右红拿不到 'red left'", "red left" not in _texts(cmds, 0))
    check("左红（y 大）拿到 'red left'", "red left" in _texts(cmds, 1))
    check("左红拿不到 'red right'", "red right" not in _texts(cmds, 1))
    check("两块 x 相同 ⇒ 谁都不生成 'red front'",
          not any("red front" == c for c in _texts(cmds, 0) + _texts(cmds, 1)))
    check("'at' 命令按真值坐标生成", "at 300,200" in _texts(cmds, 0)
          and "at 300,260" in _texts(cmds, 1))
    check("单实例青色：方位/限定词全收", {"cyan left", "cyan center",
                                   "cyan nearest", "cyan largest"}
          <= set(_texts(cmds, 2)))
    check("跳过项被计数而不是静默丢", skipped > 0)

    # 关系：两块红都在青的左边时，「更左的那块」应当是唯一解
    t2 = truth([tb("red", 0, 300, 300), tb("red", 1, 300, 290),
                tb("cyan", 2, 300, 200)])
    cmds2, _s2, _r2, dis2 = ev.gen_commands(t2)
    check("关系命令零分歧（真值米 → 毫米的换算没错），否则报错",
          not dis2, str(dis2))


# ------------------------------------------------------------------ 独立复核
def section_indep():
    print('\n[2] 独立复核：指错块必须 False，判不了必须 None')
    blocks = [tb("red", 0, 300, 200), tb("red", 1, 300, 260),
              tb("cyan", 2, 450, 330)]
    t = truth(blocks)
    right, left = blocks[0], blocks[1]

    check("red right 指向右红 → True", ev._indep_unique("red right", right, t))
    check("red right 指向左红 → False（指错了）",
          ev._indep_unique("red right", left, t) is False)
    check("red left 指向左红 → True", ev._indep_unique("red left", left, t))
    check("red left 指向右红 → False",
          ev._indep_unique("red left", right, t) is False)
    check("red front 两块并排 → 目标也不是唯一 → False",
          ev._indep_unique("red front", right, t) is False)

    # 关系取「最稳的一对」：间隔 100 与 90（差 10 ≥ 5）⇒ 只有前者成立
    t3 = truth([tb("red", 0, 300, 300), tb("red", 1, 300, 290),
                tb("cyan", 2, 300, 200)])
    check("red left-of cyan 指向间隔最大的红 → True",
          ev._indep_unique("red left-of cyan", t3["blocks"][0], t3))
    check("red left-of cyan 指向间隔次大的红 → False（差 10 mm 已分高下）",
          ev._indep_unique("red left-of cyan", t3["blocks"][1], t3) is False)

    # 间隔差 3 mm < 5 ⇒ 解析器会判歧义、命令根本不会进集合；复核也必须 False
    t4 = truth([tb("red", 0, 300, 300), tb("red", 1, 300, 297),
                tb("cyan", 2, 300, 200)])
    check("间隔差 3 mm ⇒ 两块都不算唯一",
          ev._indep_unique("red left-of cyan", t4["blocks"][0], t4) is False
          and ev._indep_unique("red left-of cyan", t4["blocks"][1], t4) is False)

    check("on/below 真值判不了 → None",
          ev._indep_unique("red on cyan", right, t) is None)
    check("largest 同尺寸判不了 → None",
          ev._indep_unique("red largest", right, t) is None)
    check("at 恒 True", ev._indep_unique("at 300,200", right, t) is True)

    # 单实例颜色：方位/限定词都恒成立
    check("单实例 cyan left → True",
          ev._indep_unique("cyan left", t["blocks"][2], t))
    check("裸色 cyan → True（只有一个）",
          ev._indep_unique("cyan", t["blocks"][2], t))

    # 变异：把平手阈值清零，3 mm 的差就不再是平手 ⇒ 两块都要变 True
    old = sp.TIE_MM
    try:
        sp.TIE_MM = 0.0
        a = ev._indep_unique("red left-of cyan", t4["blocks"][0], t4)
        b = ev._indep_unique("red left-of cyan", t4["blocks"][1], t4)
        check("变异「平手阈值清零」→ 3 mm 差被当成唯一（复核跟着变）",
              a is True and b is False)
    finally:
        sp.TIE_MM = old

    # 变异：把毫米换算写错（乘 1 而不是 1000）⇒ 关系门槛全不成立 ⇒ 复核 False
    src = inspect.getsource(ev)
    assert 'def Y(b): return b["y"] * 1000.0' in src
    ns = {'__file__': ev.__file__, '__name__': 'mut'}
    exec(compile(src.replace('def Y(b): return b["y"] * 1000.0',
                             'def Y(b): return b["y"]'),
                 '<mut>', 'exec'), ns)
    check("变异「真值不换算成毫米」→ 关系复核不再认命（守的是单位）",
          ns["_indep_unique"]("red left-of cyan", t3["blocks"][0], t3) is False)


# ------------------------------------------------------------------ 判分
def _res(**kw):
    base = {"label": "x", "counts": {"ok": 3}, "n_truth": 2, "n_matched": 2,
            "n_false_pos": 0, "n_commands": 3, "id_rate": 1.0,
            "auto_top1_rate": 1.0, "wrong_ok": 0,
            "n_indep_disagree": 0, "indep_disagree": [],
            "geom": [{"truth_id": 0, "found": True, "dxy_mm": 0.5,
                      "dtop_mm": 0.5, "dyaw_deg": 0.5},
                     {"truth_id": 1, "found": True, "dxy_mm": 0.6,
                      "dtop_mm": 0.6, "dyaw_deg": 0.6}]}
    base.update(kw)
    return base


def section_judge():
    print('\n[3] 判分：四条硬门各自能单独把批次判红')
    ok, fails = ev.judge_batch(_res())
    check("全绿 → 通过", ok and not fails, str(fails))

    ok, fails = ev.judge_batch(_res(id_rate=0.8))
    check("指定 ID 80% < 90% → 红", not ok and any("指定 ID" in f for f in fails))

    ok, fails = ev.judge_batch(_res(wrong_ok=1))
    check("误配 1 次 → 红", not ok and any("误配" in f for f in fails))

    g = _res()["geom"]
    g[1]["found"] = False
    ok, fails = ev.judge_batch(_res(geom=g))
    check("漏检 1 块 → 红", not ok and any("漏检" in f for f in fails))

    g = _res()["geom"]
    g[0]["dxy_mm"] = 2.5
    ok, fails = ev.judge_batch(_res(geom=g))
    check("中心误差 2.5 mm > 2 → 红", not ok and any("dxy_mm" in f for f in fails))

    ok, fails = ev.judge_batch(_res(n_indep_disagree=2))
    check("独立复核有分歧 → 红（评估器自身可疑）",
          not ok and any("独立复核" in f for f in fails))


def main():
    print('=' * 68)
    section_gen()
    section_indep()
    section_judge()
    print('\n%d PASS  %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('未通过：' + '; '.join(FAILS))
    return 1 if N_FAIL else 0


if __name__ == '__main__':
    sys.exit(main())

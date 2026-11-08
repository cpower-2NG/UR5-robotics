#!/usr/bin/env python3
"""阶段 4 指定通道的自检：语法 / 谓词语义 / fail-closed 三条分开钉。

为什么要有「歧义必须报 ambiguous」这一节：契约 §6 明说「0 匹配不猜、差距小要人
确认」。把一次猜错判成成功，比报错更糟 —— 所以每个「挑一个」的动作都要有
可解释的间隔，间隔不够就必须交人。

跑法：
    python3 scripts/test_stage4_specify.py
"""
import copy
import inspect
import os
import sys

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

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


def block(idx, color, x, y, top=30.0, ext=(29.0, 28.8), n=800, px=0.93,
          low=False):
    return {"idx": idx, "class": color, "cx": x, "cy": y, "top": top,
            "yaw_rect": 3.0, "n": n, "px_area_mm2": px,
            "ext_rect": list(ext), "low": low, "class_share": 1.0}


# ------------------------------------------------------------------ 语法
def section_grammar():
    print('\n[1] 语法：合法写法都要认，语法外必须报错')
    ok = {
        "red left": {"kind": "position", "color": "red", "position": "left",
                     "qualifier": None},
        "RED RIGHT": {"kind": "position", "color": "red", "position": "right",
                      "qualifier": None},
        "  red   center ": {"kind": "position", "color": "red",
                            "position": "center", "qualifier": None},
        "red left-of green": {"kind": "relation", "color": "red",
                              "relation": "left-of", "other": "green",
                              "qualifier": None},
        "red left of green": {"kind": "relation", "color": "red",
                              "relation": "left-of", "other": "green",
                              "qualifier": None},
        "red_left_of_green": {"kind": "relation", "color": "red",
                              "relation": "left-of", "other": "green",
                              "qualifier": None},
        "red in-front-of cyan": {"kind": "relation", "color": "red",
                                 "relation": "front-of", "other": "cyan",
                                 "qualifier": None},
        "red on-top-of cyan": {"kind": "relation", "color": "red",
                               "relation": "on", "other": "cyan",
                               "qualifier": None},
        "red above cyan": {"kind": "relation", "color": "red",
                           "relation": "on", "other": "cyan",
                           "qualifier": None},
        "red under cyan": {"kind": "relation", "color": "red",
                           "relation": "below", "other": "cyan",
                           "qualifier": None},
        "red behind cyan": {"kind": "relation", "color": "red",
                            "relation": "back-of", "other": "cyan",
                            "qualifier": None},
        "red top": {"kind": "stack", "color": "red", "where": "top",
                    "qualifier": None},
        "red bottom": {"kind": "stack", "color": "red", "where": "bottom",
                       "qualifier": None},
        "red nearest": {"kind": "color", "color": "red", "qualifier": "nearest"},
        "red largest": {"kind": "color", "color": "red",
                        "qualifier": "largest"},
        "red most-visible": {"kind": "color", "color": "red",
                             "qualifier": "most_visible"},
        "red most visible": {"kind": "color", "color": "red",
                             "qualifier": "most_visible"},
        "red left nearest": {"kind": "position", "color": "red",
                             "position": "left", "qualifier": "nearest"},
        "at 450,300": {"kind": "at", "x": 450.0, "y": 300.0},
        "at 450 300": {"kind": "at", "x": 450.0, "y": 300.0},
        "at -12.5, 300": {"kind": "at", "x": -12.5, "y": 300.0},
    }
    for text, want in ok.items():
        try:
            got = sp.parse_command(text)
        except sp.CommandError as e:
            check("解析 %r" % text, False, '误报语法错：%s' % e)
            continue
        bad = {k: (want[k], got.get(k)) for k in want if got.get(k) != want[k]}
        check("解析 %r" % text, not bad, str(bad))

    bad_cases = [
        ("", "空命令"),
        ("red middle", "看不懂"),
        ("red up", "看不懂"),
        ("red left-of", "缺少另一个颜色"),
        ("red left green", "不认识的关系词"),   # 关系必须写 left-of
        ("red on top", "不是颜色"),
        ("greenish left", "不认识的颜色"),
        ("red nearest extra", "不认识的关系词"),
        ("at", "两个数"),
        ("at a,b", "不是数字"),
        ("at 1,2,3", "两个数"),
    ]
    for text, needle in bad_cases:
        try:
            sp.parse_command(text)
            check("拒绝 %r" % text, False, '居然解析成功了')
        except sp.CommandError as e:
            check("拒绝 %r（理由含「%s」）" % (text, needle), needle in str(e),
                  '实际：%s' % e)

    # 词表 = 整张调色板：契约里的 `yellow on blue` 必须能解析（本批没有蓝块
    # 由 select 回 none，不是语法错）。blue 不在默认 7 类里。
    try:
        c = sp.parse_command("yellow on blue")
        check('契约示例 "yellow on blue" 能解析（blue 是调色板备选色）',
              c["other"] == "blue")
    except sp.CommandError as e:
        check('契约示例 "yellow on blue" 能解析（blue 是调色板备选色）',
              False, str(e))


# ------------------------------------------------------------------ 选择
def _scene():
    return [
        block(0, "red", 300.0, 200.0),
        block(1, "red", 300.0, 260.0),      # 同色，沿 y 分开 ⇒ left/right
        block(2, "cyan", 450.0, 330.0),
        block(3, "tan", 380.0, 380.0),
        block(4, "purple", 520.0, 210.0),
    ]


def section_selection():
    print('\n[2] 选择：方位 / 关系 / 限定词 / 歧义')
    sc = _scene()

    def sel(text, blocks=None):
        return sp.select(blocks if blocks is not None else sc, text)

    r = sel("red left")
    check("red left → y 大的那块（+y = 左）", r["status"] == "ok"
          and r["instance"] == 1, r)
    r = sel("red right")
    check("red right → y 小的那块", r["status"] == "ok"
          and r["instance"] == 0, r)
    r = sel("red front")
    check("同 y 的两块 front/back 判歧义（不是乱选）",
          r["status"] == "ambiguous", r)
    r = sel("red")
    check("裸颜色 + 多实例 → ambiguous，给候选", r["status"] == "ambiguous"
          and len(r["candidates"]) == 2, r)
    r = sel("red nearest")
    check("red nearest 用离 base 原点的距离", r["status"] == "ok"
          and r["instance"] == 0, r)

    r = sel("cyan left")
    check("单实例颜色：方位恒成立", r["status"] == "ok" and r["instance"] == 2, r)
    r = sel("blue left")
    check("本批没有的颜色 → none（不是 bad）", r["status"] == "none", r)
    r = sel("red left-of cyan")
    check("red left-of cyan：两块红都在青的右边 ⇒ none", r["status"] == "none", r)
    r = sel("red right-of cyan")
    check("red right-of cyan 有多对满足 ⇒ 取间隔最大的一对",
          r["status"] == "ok" and r["instance"] == 0, r)

    # center：只有一个同色实例时恒成立；多个时离形心最近
    r = sel("red center")
    check("red center 两块对称 ⇒ ambiguous", r["status"] == "ambiguous", r)

    # at
    r = sel("at 301,261")
    check("at 精确落在红块上", r["status"] == "ok" and r["instance"] == 1, r)
    equal = [block(0, "red", 300.0, 195.0), block(1, "cyan", 300.0, 225.0)]
    r = sel("at 300,210", equal)
    check("at 两块之间等距 ⇒ ambiguous", r["status"] == "ambiguous", r)
    r = sel("at 100,100")
    check("at 太远 ⇒ none（不硬选最近的）", r["status"] == "none", r)

    # 区外 / low 块不参与
    out = sc + [block(5, "red", 900.0, 900.0), block(6, "red", 310.0, 240.0,
                                                    low=True)]
    r = sel("cyan left", out)
    check("区外与 low 块不参与选择", r["status"] == "ok" and r["instance"] == 2, r)
    r = sel("red", out)
    check("拾取区外同色块不制造歧义", r["status"] == "ambiguous"
          and len(r["candidates"]) == 2, r)

    # 限定词
    big = [block(0, "red", 320.0, 200.0, ext=(29.0, 29.0), n=900),
           block(1, "red", 400.0, 300.0, ext=(22.0, 20.0), n=500)]
    r = sel("red largest", big)
    check("largest 按外接面积", r["status"] == "ok" and r["instance"] == 0, r)
    r = sel("red most visible", big)
    check("most_visible 按可见比例（n·px/块面积）", r["status"] == "ok"
          and r["instance"] == 0, r)
    tie = [block(0, "red", 320.0, 200.0, n=800, px=0.93),
           block(1, "red", 400.0, 300.0, n=800, px=0.93)]
    r = sel("red largest", tie)
    check("largest 完全一样 ⇒ ambiguous", r["status"] == "ambiguous", r)

    # 关系间隔门槛：差 3 mm < REL_MARGIN_MM=5 ⇒ 不算 left-of
    near = [block(0, "red", 300.0, 200.0), block(1, "cyan", 300.0, 203.0)]
    r = sel("red left-of cyan", near)
    check("间隔 3 mm 不算 left-of（门槛 5 mm）", r["status"] == "none", r)

    # 层叠：单俯视下 on/below 只在顶高差≈一个块边时成立
    stack = [block(0, "red", 300.0, 200.0, top=60.0),
             block(1, "cyan", 302.0, 202.0, top=30.0)]
    r = sel("red on cyan", stack)
    check("red on cyan：顶高差 30 mm 且水平贴近", r["status"] == "ok"
          and r["instance"] == 0, r)
    r = sel("cyan below red", stack)
    check("cyan below red 对称成立", r["status"] == "ok" and r["instance"] == 1, r)
    side = [block(0, "red", 300.0, 200.0, top=60.0),
            block(1, "cyan", 360.0, 200.0, top=30.0)]
    r = sel("red on cyan", side)
    check("水平隔 60 mm 不算 on", r["status"] == "none", r)

    # 回显字段齐全
    r = sel("cyan left")
    b = r.get("block") or {}
    check("回显含 id/颜色/base 中心/顶高/yaw/可见比例",
          set(("id", "color", "x", "y", "top", "yaw", "visible")) <= set(b), b)


# ------------------------------------------------------------------ 变异
def _mutate(old, new):
    src = inspect.getsource(sp)
    assert old in src, '变异锚点没找到：%r' % old
    ns = {'__file__': sp.__file__, '__name__': 'mut'}
    exec(compile(src.replace(old, new), '<mut>', 'exec'), ns)
    return ns


def section_mutation():
    print('\n[3] 变异：把语义改坏，对应的断言必须变红')
    sc = _scene()

    m = _mutate('scored = [(b, b["cy"]) for b in same]',
                'scored = [(b, -b["cy"]) for b in same]')
    r = m["select"](sc, "red left")
    check('变异「left 改取最小 y」→ red left 变成另一块',
          r["instance"] != 1, r)

    m = _mutate("REL_MARGIN_MM = 5.0", "REL_MARGIN_MM = 0.0")
    near = [block(0, "red", 300.0, 200.5), block(1, "cyan", 300.0, 200.0)]
    r = m["select"](near, "red left-of cyan")
    check('变异「关系间隔门槛降到 0」→ 0.5 mm 也判 left-of',
          r["status"] == "ok", r)

    m = _mutate("TIE_MM = 5.0", "TIE_MM = 0.0")
    r = m["select"](sc, "red center")
    check('变异「平手阈值降到 0」→ 本该歧义的 center 变成 ok',
          r["status"] == "ok", r)

    m = _mutate('return [b for b in blocks if not b.get("low") '
                'and _in_zone(b, zone_m)]',
                'return [b for b in blocks if True]')
    bad = copy.copy(sc) + [block(9, "red", 900.0, 900.0)]
    r = m["select"](bad, "red")
    check('变异「不过滤区外块」→ 区外红块制造额外候选',
          len(r["candidates"]) == 3, r)


def main():
    print('=' * 68)
    section_grammar()
    section_selection()
    section_mutation()
    print('\n%d PASS  %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('未通过：' + '; '.join(FAILS))
    return 1 if N_FAIL else 0


if __name__ == '__main__':
    sys.exit(main())

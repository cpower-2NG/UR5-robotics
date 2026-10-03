#!/usr/bin/env python3
"""`plot_table_envelope.py` 的纯函数自检（**不需要 matplotlib、不需要 ROS**）。

只测不画图的那一半：参数怎么解析、格心怎么变格边、JSON 两种表怎么摊平、
字体怎么挑、以及一条**结构性**的：`import` 这个模块**不许把 matplotlib 拉进来**
（拉进来就意味着有人把 import 挪到文件顶层了，那样本机/板上没装 matplotlib
 的环境会连 `--help` 都用不了）。

**真正的作图不在这里测** —— 一张 PNG 对不对，得人看（就像这张图存在的理由）。
这里只保证「喂给 matplotlib 的东西是对的」。

跑法：
    python3 scripts/test_plot_table_envelope.py
退出码 0 = 全通过。
"""

import os
import subprocess
import sys

# ⚠️ 必须在 import 被测模块**之前**设。理由是一次实测（第八十四轮）：给
# `plot_table_envelope.py` 做变异测试时，把一行改成了**等长的**另一行，
# 而写入落在**同一秒**里 —— CPython 的 `.pyc` 失效判据是 (mtime, size)，
# 两个都没变，于是它**继续用旧的字节码**，变异被静默吞掉，自检报「全过」。
# 也就是说：**那时红/绿都不代表源码**。关掉写字节码，这类假绿就不可能发生。
# （这条不是洁癖：一个报绿的变异测试比没有变异测试更坏。）
sys.dont_write_bytecode = True

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plot_table_envelope import (                                 # noqa: E402
    CJK_CANDIDATES, MISSING, SYM_COLOR, SYM_LABEL_EN, SYM_LABEL_ZH, SYM_ORDER,
    codes, edges, parse_slot, parse_zone, pick_font, validate, worst_point)

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


def raises(fn, *a):
    try:
        fn(*a)
    except ValueError:
        return True
    except Exception:
        return False
    return False


# ----------------------------------------------------------------------
# A. parse_zone：**x 界在前**，这一条最容易写反
# ----------------------------------------------------------------------
print("\n--- A. parse_zone ---")
nm, box = parse_zone("现状:0.30,0.90,-0.18,0.16")
check("名字与四个界都读出来", nm == "现状" and box == (0.30, 0.90, -0.18, 0.16),
      f"got {nm!r} {box!r}")
check("顺序是 xmin,xmax,ymin,ymax —— x 的跨度是 0.60、y 的是 0.34",
      abs((box[1] - box[0]) - 0.60) < 1e-12 and abs((box[3] - box[2]) - 0.34) < 1e-12,
      f"got {box!r}")
check("负数界能读", parse_zone("a:-0.5,0.5,-0.5,0.5")[1] == (-0.5, 0.5, -0.5, 0.5))
check("名字按**第一个**冒号切，后面的冒号留在数里 ⇒ 解析失败（名字不能含冒号）",
      raises(parse_zone, "a:b:0,1,2,3"))
check("但下划线名字正常", parse_zone("第1步_现状:0,1,2,3")[0] == "第1步_现状")
check("没有冒号 ⇒ 报错（不猜）", raises(parse_zone, "0.1,0.2,0.3,0.4"))
check("三个数 ⇒ 报错", raises(parse_zone, "a:0,1,2"))
check("下界比上界大 ⇒ 报错（写反了要出声，不能画出一个负宽度的框）",
      raises(parse_zone, "a:0.9,0.3,-0.1,0.1"))
check("非数字 ⇒ 报错", raises(parse_zone, "a:1,2,3,x"))


# ----------------------------------------------------------------------
# B. parse_slot
# ----------------------------------------------------------------------
print("\n--- B. parse_slot ---")
check("两个数", parse_slot("0.44,+0.24") == (0.44, 0.24))
check("负数", parse_slot("-0.44,-0.24") == (-0.44, -0.24))
check("一个数 ⇒ 报错", raises(parse_slot, "0.44"))
check("三个数 ⇒ 报错", raises(parse_slot, "0.44,0.24,0.1"))


# ----------------------------------------------------------------------
# C. edges：格心 → 格边
# ----------------------------------------------------------------------
print("\n--- C. edges ---")
e = edges([0.0, 0.1, 0.2])
check("等距时边就是中点，首尾各外推半格",
      [round(v, 6) for v in e] == [-0.05, 0.05, 0.15, 0.25], f"got {e!r}")
check("边数 = 格数 + 1（pcolormesh 要的就是这个）", len(edges([0, 1, 2, 3])) == 5)
check("单格也能出两条边（不返回空 —— 返回空会让 matplotlib 报一句难懂的错）",
      len(edges([0.5])) == 2, f"got {edges([0.5])!r}")
check("空列表 ⇒ 空", edges([]) == [])
e2 = edges([0.0, 0.05, 0.09])
check("**末格距**与前几格不同时，边取各自中点（不假设等距）",
      abs(e2[2] - 0.07) < 1e-12, f"got {e2!r}")


# ----------------------------------------------------------------------
# D. codes：两张表（worst / 单层）都摊得开，且坐标解析对
# ----------------------------------------------------------------------
print("\n--- D. codes ---")
DATA = {
    "xs": [0.0, 0.1], "ys": [-0.1, 0.0],
    "cells": {"0.0000,-0.1000": "X", "0.1000,-0.1000": ". ",
              "0.0000,0.0000": "s", "0.1000,0.0000": "~"},
    "per_level": {"grasp_descend": {"0.0000,-0.1000": ".",
                                    "0.1000,-0.1000": ".",
                                    "0.0000,0.0000": ".",
                                    "0.1000,0.0000": "x"}},
}
xs, ys, g = codes(DATA, "worst")
check("xs/ys 原样取回", xs == [0.0, 0.1] and ys == [-0.1, 0.0])
check("grid 是 [y][x]、**y 升序**（作图要的朝向）",
      len(g) == 2 and len(g[0]) == 2, f"got {g!r}")
check("worst 取的是 cells", g[0][0] == "X" and g[1][0] == "s", f"got {g!r}")
_, _, g2 = codes(DATA, "grasp_descend")
check("单层取的是 per_level[那层]（和 worst 不同 ⇒ 没串台）",
      g2[1][1] == "x" and g2[1][0] == ".", f"got {g2!r}")
try:
    codes(DATA, "no_such_level")
    check("不存在的层 ⇒ KeyError（由 main 先拦，这里只钉住它不静默给空图）", False)
except KeyError:
    check("不存在的层 ⇒ KeyError（由 main 先拦，这里只钉住它不静默给空图）", True)

# ⚠️ 这一条是**真的踩到过**的坑，不是假想（第八十四轮）：
# `axis(0.15, 0.95, 0.05)` 给出的 x 里有 `0.30000000000000004` 这种，
# 而键按 %.4f 写成 "0.3000"。第一版 `codes()` 拿 float 去配 float，
# 于是 **17 列里静默漏掉 6 列**，画出来是一片「无数据」，**而且自检是绿的**
# —— 因为那份样本用的是 `round(x, 2)` 的整齐数，正好躲开了。
# 下面这条样本专门放一个**不能往返**的 x，钉住「按格式查」这件事。
print("\n--- D2. 键格式必须按 %.4f 配对（不许拿 float 硬配）---")
RAW = 0.15 + 3 * 0.05                       # = 0.30000000000000004
check("样本本身确实不能往返（否则这条测试是空转）",
      RAW != round(RAW, 4) and f"{RAW:.4f}" == "0.3000", f"got {RAW!r}")
DATA2 = {"xs": [RAW], "ys": [0.0], "cells": {"0.3000,0.0000": "X"},
         "per_level": {"grasp_descend": {"0.3000,0.0000": "X"}}}
_, _, g3 = codes(DATA2, "worst")
check("不能往返的 x 也能取到数据（不是 MISSING）", g3[0][0] == "X", f"got {g3!r}")
check("取不到时是 MISSING 而**不是** SAFE（宁可画成 ? 也不许当成安全）",
      codes({"xs": [0.1], "ys": [0.0], "cells": {}, "per_level": {"k": {}}},
            "worst")[2][0][0] == MISSING)


# ----------------------------------------------------------------------
# D3. validate：**契约**校验。这一段是踩出来的，不是想出来的
# ----------------------------------------------------------------------
# 实测：第一版 draw() 按两元组解包 `worst_at`，而包线写的是
# **三元组** `[x, y, 层级名]` —— 在板上跑起来报 `too many values to unpack`，
# 报的话跟「哪个键、哪一侧错」毫无关系。这 44 条自检当时**一条都没红**，
# 因为它们只覆盖不画图的那一半。validate 就是为了让这类错**在入口就说清楚**。
print("\n--- D3. validate（JSON 契约）---")

# 这份样本**照抄包线脚本真写出来的键名与形状**（含 worst_at 的第三项）。
GOOD = {
    "xs": [0.15, 0.20], "ys": [-0.10, 0.00],
    "cells": {"0.2000,0.1000": "X"}, "per_level": {"grasp_descend": {}},
    "levels": {"grasp_descend": 0.2042}, "table_z": 0.0, "floor": 0.02,
    "guard_margin": 0.02, "worst_link_z": -0.0944,
    "worst_at": [0.20, 0.10000000000000009, "grasp_descend"],
    # 第八十六轮实测算出来的那块（152 格）—— 照抄真值，不自己编。
    "max_rect_safe": [0.40, 0.75, -0.45, 0.45, 152],
}
check("照抄真实形状的样本要通过", validate(GOOD) == [], f"got {validate(GOOD)!r}")
check("★ 三元组 worst_at 不再被当成两元组（这就是当时炸掉的那条）",
      validate(GOOD) == [])
check("两元组 worst_at 也接受（旧 JSON）",
      validate({**GOOD, "worst_at": [0.2, 0.1]}) == [])
check("worst_at = null 接受（没有坏格）",
      validate({**GOOD, "worst_at": None}) == [])
check("四元组 worst_at ⇒ 报出来（不是留给 unpack 去炸）",
      len(validate({**GOOD, "worst_at": [0.2, 0.1, "grasp", 7]})) == 1)
check("worst_at 前两项不是数字 ⇒ 报出来",
      len(validate({**GOOD, "worst_at": ["a", "b"]})) == 1)
check("有 worst_at 但没有 worst_link_z ⇒ 报出来",
      len(validate({k: v for k, v in GOOD.items() if k != "worst_link_z"})) == 1)
check("缺 table_z ⇒ 报出来", any("table_z" in b for b in
                                validate({k: v for k, v in GOOD.items()
                                          if k != "table_z"})))
check("xs 是空列表 ⇒ 报出来（画不出空网格）",
      any("xs" in b for b in validate({**GOOD, "xs": []})))
check("per_level 不是对象 ⇒ 报出来",
      any("per_level" in b for b in validate({**GOOD, "per_level": []})))
check("顶层不是对象 ⇒ 一句话报掉，不抛异常", validate([1, 2]) == ["顶层不是对象"])
check("**一次把所有问题都说出来**（不是修一个才发现下一个）",
      len(validate({})) >= 6, f"got {len(validate({}))}")
check("合法的空 cells 不报（那只是这一层没数据，不是 JSON 坏）",
      validate({**GOOD, "cells": {}}) == [])
# max_rect_safe：与 worst_at 同一个坑 —— 解包前不验长度，错误就出现在 draw() 深处。
check("没有 max_rect_safe 也接受（旧 JSON 没这个键）",
      validate({k: v for k, v in GOOD.items() if k != "max_rect_safe"}) == [])
check("max_rect_safe = null 接受（没算出全安全矩形，是**合法结论**不是坏数据）",
      validate({**GOOD, "max_rect_safe": None}) == [],
      f"got {validate({**GOOD, 'max_rect_safe': None})!r}")
check("★ 四元组 max_rect_safe ⇒ 报出来（这就是 worst_at 那次炸掉的同一个形状）",
      len(validate({**GOOD, "max_rect_safe": [0.4, 0.75, -0.45, 0.45]})) == 1,
      f"got {validate({**GOOD, 'max_rect_safe': [0.4, 0.75, -0.45, 0.45]})!r}")
check("max_rect_safe 里混进字符串 ⇒ 报出来",
      len(validate({**GOOD, "max_rect_safe": [0.4, "0.75", -0.45, 0.45, 152]})) == 1)
check("上下界反了 ⇒ 报出来（画出来是个负宽度的框，什么也看不见）",
      len(validate({**GOOD, "max_rect_safe": [0.75, 0.40, -0.45, 0.45, 152]})) == 1)

# worst_point：**当初炸掉的就是这一处**。以前它在 draw() 里，纯函数自检够不着，
# 所以 44 条全绿也照样漏。提出来之后这条契约才有断言。
print("\n--- D4. worst_point（当初 `too many values to unpack` 的那一处）---")
wx, wy, wl = worst_point(GOOD)
# ⚠️ 别写成 `== (0.2, 0.1, ...)` —— 真实 JSON 里的 y 是
# `0.10000000000000009`（浮点累加噪声），精确相等会假红。这正是包线要按
# `%.4f` 写键的那个同一个来源。
check("三元组解出 x/y/层级名（不是抛异常）",
      abs(wx - 0.2) < 1e-9 and abs(wy - 0.1) < 1e-9 and wl == "grasp_descend",
      f"got {(wx, wy, wl)!r}")
check("两元组也解得出，层级名是 None",
      worst_point({**GOOD, "worst_at": [0.2, 0.1]}) == (0.2, 0.1, None))
check("没有 worst_at ⇒ None（不是抛异常）", worst_point({}) is None)
check("worst_at = null ⇒ None", worst_point({**GOOD, "worst_at": None}) is None)
check("x/y 是 float（不是原样透传的字符串或 numpy 之类）",
      all(isinstance(v, float) for v in worst_point(GOOD)[:2]))


# ----------------------------------------------------------------------
# E. pick_font：按优先级挑，挑不到返回 None（⇒ 退英文）
# ----------------------------------------------------------------------
print("\n--- E. pick_font ---")
check("一个都没有 ⇒ None（不是随便返回一个名字）", pick_font(set()) is None)
check("只有低优先级的也认（WenQuanYi 排在 Noto 后面）",
      pick_font({"WenQuanYi Zen Hei"}) == "WenQuanYi Zen Hei")
check("同时有多个 ⇒ 取**列表里靠前**那个",
      pick_font({"WenQuanYi Zen Hei", "Noto Sans CJK SC"}) == "Noto Sans CJK SC")
check("列表外的字体不认（避免把 Helvetica 当 CJK 用）",
      pick_font({"Helvetica", "DejaVu Sans"}) is None)
check("候选表里没有重复项", len(set(CJK_CANDIDATES)) == len(CJK_CANDIDATES))


# ----------------------------------------------------------------------
# F. 颜色/文案表：**每个符号都得有**，缺一个 KeyError 会画到一半才炸
# ----------------------------------------------------------------------
print("\n--- F. 符号表完整性 ---")
for s in SYM_ORDER:
    check(f"{s!r} 有颜色", s in SYM_COLOR, f"缺 {s!r}")
check("MISSING 在 SYM_ORDER 里（否则图例会漏掉「无数据」）", MISSING in SYM_ORDER)
check("MISSING 排在最后（图例是「坏 → 好」，无数据不属于任何一档）",
      SYM_ORDER[-1] == MISSING)
check("中文文案每个符号都有", all(s in SYM_LABEL_ZH for s in SYM_ORDER))
check("英文文案每个符号都有（缺字体时走它）",
      all(s in SYM_LABEL_EN for s in SYM_ORDER))
check("含 {band} 的文案在两张表里是**同一条**（否则中英图例会不一样长）",
      sorted(k for k, v in SYM_LABEL_ZH.items() if "{band}" in v)
      == sorted(k for k, v in SYM_LABEL_EN.items() if "{band}" in v))
check("两条近限文案都能 format（少一个大括号会在画图时才炸）",
      all(v.format(band=0.05) for v in (SYM_LABEL_ZH["~"], SYM_LABEL_EN["~"])))


# ----------------------------------------------------------------------
# G. 结构性：import 这个模块**不许**把 matplotlib 拉进来
# ----------------------------------------------------------------------
# 这不是洁癖：本机 WSL 有 matplotlib 但没有中文字体，板上两者都有，
# 而将来可能有个环境两者都没有。`import` 就拉 matplotlib 意味着
# 连 `--help` 和本自检都跑不了。谁把 import 挪到顶层，这条会红。
print("\n--- G. import 不拉 matplotlib ---")
code = ("import sys; sys.path.insert(0, %r); import plot_table_envelope; "
        "print('LOADED' if 'matplotlib' in sys.modules else 'CLEAN')"
        % os.path.dirname(os.path.abspath(__file__)))
r = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
check("子进程里 import 之后 matplotlib **不在** sys.modules",
      r.stdout.strip().endswith("CLEAN"),
      f"stdout={r.stdout.strip()!r} stderr={r.stderr.strip()[-200:]!r}")


print("\n通过 %d，失败 %d" % (N_PASS, N_FAIL))
if FAILS:
    print("失败项：")
    for f in FAILS:
        print("  - %s" % f)
sys.exit(1 if N_FAIL else 0)

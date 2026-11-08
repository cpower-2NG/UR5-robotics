#!/usr/bin/env python3
"""E15 预算汇总器的纯函数自检：**不需要 GPU、不需要测量文件**。

钉住四件事，每件都对应一次会发生的误判：

1. 冷帧必须被剔掉 —— 板上同一帧首帧 12.5 s、稳态 0.06 s，混在一起取中位会把
   「启动代价」算成「每帧延迟」；
2. 超预算必须判红 —— 否则「有预算」只是一句话；
3. **未测不是通过** —— 板上没有 MinkowskiEngine，GraspNet 那行是 unknown；
4. 板载统一内存只计 RSS，不加显存 —— 两者相加会把 7.6 GB 的板算成 16 GB。

跑法：``python3 scripts/test_stage4_budget.py``；退出码 0 = 全通过。
"""
import importlib
import os
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import stage4_budget as bg                                       # noqa: E402

N_PASS = N_FAIL = 0
FAILS = []


def check(name, cond, detail=''):
    global N_PASS, N_FAIL
    if cond:
        N_PASS += 1
        print("  ok   %s" % name)
    else:
        N_FAIL += 1
        FAILS.append(name)
        print("  FAIL %s %s" % (name, detail))


def test_steady():
    print("[稳态口径]")
    got = bg._steady([12574.2, 114.3, 109.6, 116.7])
    check("冷帧被剔除", got["n_cold_dropped"] == 1, got)
    check("稳态取暖帧中位", 109 <= got["latency_ms"] <= 117, got)
    check("冷帧值单独记下", got["cold_ms"] == 12574.2, got)

    got2 = bg._steady([57.3, 57.6, 59.0])
    check("全暖帧时一颗都不剔", got2["n_cold_dropped"] == 0, got2)
    check("空输入不崩", bg._steady([])["latency_ms"] is None)


def test_judge():
    print("[判定]")
    ok = {"net": "RNGNet", "host": "board", "status": "已测", "latency_ms": 57.6,
          "vram_mb": 195, "rss_mb": 1347}
    check("锁频 57.6 ms 在预算内", bg.judge(ok)[0] == "pass", bg.judge(ok))

    bad = dict(ok, latency_ms=114.3)
    verdict, over = bg.judge(bad)
    check("默认调频 114.3 ms 判红", verdict == "fail", over)
    check("超限项指名 latency_ms", over and "latency_ms" in over[0], over)

    fat = dict(ok, rss_mb=4000)
    check("RSS 超板内存一半判红", bg.judge(fat)[0] == "fail", bg.judge(fat))

    unknown = {"net": "GraspNet", "host": "board", "status": "未测"}
    check("未测是 unknown 不是 pass", bg.judge(unknown)[0] == "unknown")


def test_budgets():
    print("[预算口径]")
    b = bg.budgets("board")
    check("板有延迟硬线 100 ms", b["latency_ms"] == 100.0, b)
    check("板不计显存（统一内存）", b["vram_mb"] is None, b)
    check("板 RSS 预算是总内存一半",
          abs(b["rss_mb"] - bg.BOARD_RAM_MB / 2) < 1, b)
    check("锁频行沿用板的同一条门",
          bg.budgets("board-pinned") == b)
    d = bg.budgets("dev")
    check("本机不设延迟硬线", d["latency_ms"] is None, d)


def test_render():
    print("[渲染]")
    rows = [{"net": "RNGNet", "host": "board", "status": "已测",
             "latency_ms": 114.3, "vram_mb": 195, "rss_mb": 1310,
             "n_cold_dropped": 1},
            {"net": "GraspNet", "host": "board", "status": "未测"}]
    out = bg.render(rows)
    check("红行带 ❌", "❌" in out, out)
    check("未测行带「未测」", "未测" in out, out)
    check("主机显示成中文标签", "板" in out and "board" not in out)


def test_mutation_guard():
    """把比较改成恒假必须让判定测试失效——否则上面的「判红」是空转。"""
    print("[变异检查]")
    src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "stage4_budget.py"), encoding="utf-8").read()
    mutated = src.replace("if val is not None and lim is not None and val > lim:",
                          "if False:")
    check("比较语句在源码里可定位（变异前提）", mutated != src)
    ns = {"__name__": "mut"}
    exec(compile(mutated, "mutated", "exec"), ns)                # noqa: S102
    row = {"net": "RNGNet", "host": "board", "status": "已测",
           "latency_ms": 114.3, "vram_mb": 195, "rss_mb": 1310}
    check("去掉比较后 114.3 ms 会被误判为通过（证明原测试有牙）",
          ns["judge"](row)[0] == "pass")
    importlib.reload(bg)


def main():
    test_steady()
    test_judge()
    test_budgets()
    test_render()
    test_mutation_guard()
    print("\n%d 项通过，%d 项失败" % (N_PASS, N_FAIL))
    if FAILS:
        print("失败：" + ", ".join(FAILS))
    return 0 if N_FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())

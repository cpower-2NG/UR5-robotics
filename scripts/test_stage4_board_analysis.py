#!/usr/bin/env python3
"""``stage4_board_analysis`` 自检：板上分析的三个子命令参数装配。

为什么值得单测：它是「本机不承担运行时任务」这条定调的执行点 —— 一轮的分析
（解垫 / 真值 / 彩色）全在板上跑、只回传 JSON。参数装配错了不会报错，只会
**量错位置或量错帧**（例如 --xy 把毫米当米），判分却照常出数。

**不 import rclpy**；被封装的两个脚本本机也能跑，但这里只替换它们的 ``main``
来检查装配，不真算图。

跑法：
    python3 scripts/test_stage4_board_analysis.py
"""
import json
import os
import sys
import tempfile

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import stage4_board_analysis as ba                               # noqa: E402
import stage4_clear_run as cr                                    # noqa: E402

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
        print('  FAIL  %s%s'
              % (name, '' if detail == '' else '   ← %r' % (detail,)))


def capture(obj, attr='main'):
    """把 ``obj.attr`` 换成记录调用的桩；返回 (记录列表, 还原函数)。"""
    calls = []
    old = getattr(obj, attr)

    def fake(argv):
        calls.append(list(argv))
        return 0
    setattr(obj, attr, fake)
    return calls, lambda: setattr(obj, attr, old)


# ---- pad：把 CLI 参数原样交给 stage4_pad_from_burst
calls, undo = capture(ba.pfb)
try:
    rc = ba.main(["pad", "--burst", "/b/burst", "--mat", "/b/m.json",
                  "--out", "/b/grid.json", "--min-frames", "20",
                  "--max-spread-mm", "2.0", "--max-spread-deg", "0.5"])
finally:
    undo()
argv = calls[0] if calls else []
check("pad：rc 0", rc == 0)
check("pad：burst 是位置参数、--out/--mat 都传下去",
      argv[:1] == ["/b/burst"] and "--out" in argv and "--mat" in argv,
      argv)
check("pad：帧数与离散阈值传下去",
      argv[argv.index("--min-frames") + 1] == "20" and
      argv[argv.index("--max-spread-mm") + 1] == "2.0", argv)

# ---- pad：--mat 是必需项（板子没有本仓外的默认 mat）
try:
    ba.main(["pad", "--burst", "/b/burst", "--out", "/b/grid.json"])
    check("pad：缺 --mat ⇒ argparse 拒绝（不许用板上的陈旧默认值）", False)
except SystemExit as e:
    check("pad：缺 --mat ⇒ argparse 拒绝（不许用板上的陈旧默认值）",
          e.code not in (0, None))

# ---- truth：共识 + 区 + allow-empty + jobs
calls, undo = capture(ba.tf)
try:
    rc = ba.main(["truth", "--burst", "/b/wrist-post/free", "--out", "/b/t.json",
                  "--zone", "0.1,0.5,0.0,0.4", "--allow-empty", "--jobs", "6",
                  "--min-frames", "5"])
finally:
    undo()
argv = calls[0] if calls else []
check("truth：rc 0 且走 --consensus", rc == 0 and "--consensus" in argv, argv)
check("truth：区 / 允许空 / 进程数都传下去",
      argv[argv.index("--zone") + 1] == "0.1,0.5,0.0,0.4" and
      "--allow-empty" in argv and argv[argv.index("--jobs") + 1] == "6", argv)

calls, undo = capture(ba.tf)
try:
    ba.main(["truth", "--burst", "/b/wrist-pre/pick", "--out", "/b/t.json"])
finally:
    undo()
check("truth：不给 --allow-empty 时 argv 里没有它",
      "--allow-empty" not in (calls[0] if calls else ["--allow-empty"]),
      calls)

# ---- colour：毫米进、米出（这个换算是判分位置对不对的唯一环节）
seen = {}
old = cr.measure_target_colour
cr.measure_target_colour = (
    lambda burst, xy, top_z, **kw: (seen.update(
        burst=burst, xy=xy, top_z=top_z, kw=kw) or
        {"ok": False, "n_frames": 0, "n_missed": 5}))
try:
    with tempfile.TemporaryDirectory() as td:
        out = os.path.join(td, "c.json")
        rc = ba.main(["colour", "--burst", "/b/wrist-post/pad",
                      "--out", out, "--xy", "471.94,-157.6",
                      "--top-z", "0.0295", "--hue", "30.0",
                      "--expect-class", "orange", "--search-mm", "25"])
        doc = json.load(open(out))
finally:
    cr.measure_target_colour = old
check("colour：xy 毫米 → 米（471.94 mm ⇒ 0.47194 m）",
      abs(seen.get("xy", (0, 0))[0] - 0.47194) < 1e-9 and
      abs(seen.get("xy", (0, 0))[1] + 0.1576) < 1e-9, seen.get("xy"))
check("colour：顶面高度 / 窗 / 色相 / 期望类别都传给量测",
      abs(seen.get("top_z", 0) - 0.0295) < 1e-12 and
      seen.get("kw", {}).get("search_mm") == 25.0 and
      seen.get("kw", {}).get("expect_class") == "orange" and
      seen.get("kw", {}).get("expect_hue_deg") == 30.0, seen)
check("colour：ok=False 也是 rc 0（「窗里没看见」是证据，不是失败）",
      rc == 0 and doc.get("ok") is False, (rc, doc))
check("colour：JSON 里带 search_mm / top_z（复盘要能看出来开的哪个窗）",
      doc.get("search_mm") == 25.0 and abs(doc.get("top_z") - 0.0295) < 1e-12,
      doc)

# ---- colour：--xy 写错 ⇒ 2（不是 0 也不是崩）
rc = ba.main(["colour", "--burst", "/b/x", "--out", "/tmp/x.json",
              "--xy", "471.94", "--top-z", "0.03"])
check("colour：--xy 少了逗号 ⇒ rc 2", rc == ba.EXIT_BAD_ARGS, rc)

# ---- colour：相机名写错 ⇒ 2（choices 挪到了 cmd_colour 里，别让它静默用缺省）
rc = ba.main(["colour", "--burst", "/b/x", "--out", "/tmp/x.json",
              "--xy", "1,2", "--top-z", "0.03", "--camera", "nose"])
check("colour：相机名不在 COLOUR_CAMERAS 里 ⇒ rc 2", rc == ba.EXIT_BAD_ARGS, rc)

print()
print('通过 %d，失败 %d' % (N_PASS, N_FAIL))
if FAILS:
    print('失败项：')
    for f in FAILS:
        print('  - %s' % f)
sys.exit(1 if N_FAIL else 0)

#!/usr/bin/env python3
"""board_a1.py 的判分自检 —— 纯离线，不碰 ROS、不碰臂、不碰串口。

为什么值得单独测
----------------
`board_a1.py` 会在**真机上把臂开动起来**，跑一轮要几分钟、还要人摆件。
它的判分要是错了，代价不是「测试红一下」，而是**误判整轮实验**：
  · 判松了 → 4 块 / 4 槽那种拿不到判据 2 的跑法会被报成「A1 通过」；
  · 判紧了 → 真跑通了却被报成失败，人再摆一次件、再动一次臂。
所以这里用**真机日志原文**当夹具（`FIX_REAL_4of4`，2026-10-05 那一轮），
另加合成夹具覆盖另外两条收工路径与两种残缺日志。

变异检查（本仓库的约定：削掉闸门必须立刻变红）
--------------------------------------------
第 4 节把 `board_a1.py` 的源码**真的改坏**，重新导入，断言行为确实翻面。
只断言"改坏了会红" 是自说自话 —— 这里断言的是**改坏之后判分结果与改坏前不同**，
从而证明上面那几条断言是**吃劲的**，不是恰好为真。

用法:
    python3 scripts/test_board_a1.py
退出码 0 = 全通过，1 = 有失败项，2 = 环境不满足。
"""

import sys

sys.dont_write_bytecode = True   # 防 .pyc 假绿（仓库约定）

import importlib.util            # noqa: E402
import os                        # noqa: E402
import tempfile                  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import board_a1                  # noqa: E402

_failed = []


def check(name, cond, detail=''):
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}" + (f"  {detail}" if detail else ''))
    if not cond:
        _failed.append(name)


# --------------------------------------------------------------------- 夹具
# 前缀形状保持与真机一致（`[planner_node-15] [INFO] [ts] [grasp_planner]: `），
# 免得正则只在这一种形状上恰好成立。第一行的夹爪 repr 里带设备路径，已略去。

_P = "[planner_node-15] [INFO] [{}] [grasp_planner]: "

FIX_REAL_4of4 = [
    "[planner_node-15] [INFO] [1791188125.276551303] [grasp_planner]: GraspPlanner 就绪 | 夹爪后端=ag95 (Ag95Gripper(port=<略>, baudrate=115200, slave_id=1, stroke=95.0mm, force=50N, 真实写入, open_is_high=True))",
    _P.format("1791188127.278922022") + "初始化：先张开夹爪，再回 HOME ...",
    "[planner_node-15] [ERROR] [1791188133.068015069] [grasp_planner]: 接近轨迹失败：goal_status=ABORTED error_code=-4",
    _P.format("1791188148.097282029") + "返回 HOME ...",
    _P.format("1791188152.905610953") + "已就绪，等待下一次抓取",
    _P.format("1791188152.911259045") + "候选 4 个 ['block', 'block', 'block', 'block'] | 已放置 0/4 | 已抓 0 个 | 连续未放置 2/10",
    _P.format("1791188164.318160757") + "已夹住物体（回读位置=25.0800）—— 抬起",
    _P.format("1791188168.699231487") + "抬起成功，已记录 base_link=(0.463, 0.180, 0.028)（累计 1 个）",
    _P.format("1791188173.019799704") + "已到 HOME —— 去放置位[0] (0.450, -0.250)",
    _P.format("1791188181.357099868") + "已放置 1/4 个",
    _P.format("1791188190.579324837") + "候选 3 个 ['block', 'block', 'block'] | 已放置 1/4 | 已抓 1 个 | 连续未放置 0/10",
    _P.format("1791188219.081521230") + "已放置 2/4 个",
    _P.format("1791188228.211967777") + "候选 2 个 ['block', 'block'] | 已放置 2/4 | 已抓 2 个 | 连续未放置 0/10",
    _P.format("1791188256.609758296") + "已放置 3/4 个",
    _P.format("1791188265.758266900") + "候选 1 个 ['block'] | 已放置 3/4 | 已抓 3 个 | 连续未放置 0/10",
    _P.format("1791188294.091602967") + "已放置 4/4 个",
    _P.format("1791188303.280199743") + "待命中 0s/20s：还没有可抓的目标",
    _P.format("1791188323.283336534") + "收工：空闲 20s 没有发起过任何一次抓取尝试（上限 idle_finish_timeout=20s） —— 抓取区已空 / 候选都已抓过 / 分数都不够",
    _P.format("1791188323.283889275") + "  已成功放置 4 个 / 槽位共 4 个；已抓取记账 4 个",
    _P.format("1791188323.284473186") + "  **不再接受新目标**。节点仍在运行，Ctrl-C 退出。",
]

# 期望通过的那一轮：抓取区 5 块 / 槽位 4 个。与真机夹具只差
# 「候选数多一个」和「收工语不同」—— 这正是判据 2 要区分的东西。
FIX_OK_5blocks = [
    _P.format("1791188152.905610953") + "已就绪，等待下一次抓取",
    _P.format("1791188152.911259045") + "候选 5 个 ['block', 'block', 'block', 'block', 'block'] | 已放置 0/4 | 已抓 0 个 | 连续未放置 0/10",
    _P.format("1791188181.357099868") + "已放置 1/4 个",
    _P.format("1791188190.579324837") + "候选 4 个 ['block', 'block', 'block', 'block'] | 已放置 1/4 | 已抓 1 个 | 连续未放置 0/10",
    _P.format("1791188219.081521230") + "已放置 2/4 个",
    _P.format("1791188228.211967777") + "候选 3 个 ['block', 'block', 'block'] | 已放置 2/4 | 已抓 2 个 | 连续未放置 0/10",
    _P.format("1791188256.609758296") + "已放置 3/4 个",
    _P.format("1791188265.758266900") + "候选 2 个 ['block', 'block'] | 已放置 3/4 | 已抓 3 个 | 连续未放置 0/10",
    _P.format("1791188294.091602967") + "已放置 4/4 个",
    _P.format("1791188296.100000000") + "候选 1 个 ['block'] | 已放置 4/4 | 已抓 4 个 | 连续未放置 0/10",
    _P.format("1791188296.200000000") + "收工：放置槽位已用尽（4 个）—— 没有地方可放了",
    _P.format("1791188296.300000000") + "  已成功放置 4 个 / 槽位共 4 个；已抓取记账 4 个",
]

FIX_MAXATTEMPTS = [
    _P.format("1791188294.091602967") + "已放置 2/4 个",
    _P.format("1791188296.100000000") + "收工：连续 10 次尝试都没有产生新的成功放置（上限 10）—— 判定为卡住",
    _P.format("1791188296.300000000") + "  已成功放置 2 个 / 槽位共 4 个；已抓取记账 6 个",
]

FIX_TRUNCATED = FIX_REAL_4of4[:16]        # 到「已放置 4/4 个」就断了，没有收工行

# 诱饵：正文里**引用**了收工语（注释/日志里出现），但不是「收工：」那一行。
# 谁要是把判据从「匹配 `收工：` 那一行」放松成「整份日志里找子串」，
# 这条就会把残缺日志判成通过。
DECOY = _P.format("1791188296.400000000") + \
    '  # ③ 槽位闸门：self._finish("放置槽位已用尽（4 个）—— 没有地方可放了")'


def merge_globals(a, b):
    return a + b


# ------------------------------------------------------- 1. 真机那一轮（4/4）
def section_real():
    print("\n[1] 真机 2026-10-05 那一轮（4 块 / 4 槽）—— **判据 2 不该过**")
    v = board_a1.grade(FIX_REAL_4of4, 4)
    check("判据 1 过：已放置 4/4", v["placed_ok"], f"placed={v['placed']}")
    # ↓ 这几条就是变异检查的着力点：把判据 2 从 ok 里摘掉，这条立刻红。
    check("判据 2 不过（收工是 idle_finish_timeout，不是槽位用尽）",
          v["slots_exhausted_ok"] is False)
    check("总结论不过是 FAIL（**不是** A1 通过）", v["ok"] is False)
    check("收工原因认出来了", v["finish"] is not None and "空闲 20s" in v["finish"])
    check("收工自述解出来了", v["summary"] == (4, 4, 4), f"summary={v['summary']}")
    check("槽位数与期望一致时不该报警", v["reported_slots"] is None)

    # 判据 1 必须是 == 而不是 >=：同一份日志按 3 槽判，就不该算「放满」
    check("槽位期望改成 3 时判据 1 必须不过（防 >= 写法）",
          board_a1.grade(FIX_REAL_4of4, 3)["placed_ok"] is False)

    # 取的是**最后**一条「已放置 X/N 个」，不是第一条
    check("多条进度行取最后一条（防取首条）", v["placed"] == (4, 4), f"placed={v['placed']}")


# -------------------------------------------------- 2. 期望通过的那一轮（5 块）
def section_ok():
    print("\n[2] 抓取区 5 块 / 槽位 4 个 —— 两条判据都该过")
    v = board_a1.grade(FIX_OK_5blocks, 4)
    check("判据 1 过", v["placed_ok"], f"placed={v['placed']}")
    check("判据 2 过（放置槽位已用尽）", v["slots_exhausted_ok"] is True)
    check("总结论是 PASS", v["ok"] is True)
    check("从收工语里解出槽位数=4", v["reported_slots"] == 4)
    check("槽位数相符时不报警", v["reported_slots"] == 4)
    check("候选数轨迹被记录下来", len(v["cands"]) >= 4, f"cands={v['cands']}")
    # 同一份日志按 5 槽判：只放了 4/5，判据 1 必须不过
    check("槽位期望改成 5 时判据 1 必须不过",
          board_a1.grade(FIX_OK_5blocks, 5)["ok"] is False)


# ------------------------------------------------------ 3. 另两条路径 + 残缺
def section_others():
    print("\n[3] 卡住路径 / 残缺日志 / 诱饵")
    v = board_a1.grade(FIX_MAXATTEMPTS, 4)
    check("max_attempts 路径：判据 2 不过", v["slots_exhausted_ok"] is False)
    check("max_attempts 路径：总结论 FAIL", v["ok"] is False)

    v = board_a1.grade(FIX_TRUNCATED, 4)
    check("没有收工行时 finish 为 None", v["finish"] is None)
    check("没有收工行时判据 2 不过", v["slots_exhausted_ok"] is False)
    check("没有收工行时总结论 FAIL", v["ok"] is False)

    v = board_a1.grade(FIX_TRUNCATED + [DECOY], 4)
    check("正文里引用收工语**不算**通过（防整行子串匹配）",
          v["slots_exhausted_ok"] is False and v["ok"] is False)

    v = board_a1.grade([], 4)
    check("空日志：不炸，且判 FAIL", v["ok"] is False and v["placed"] == (0, 4))

    # explain() 在每种输入下都要能说话（现场看的就是它）
    for name, lines in (("真机", FIX_REAL_4of4), ("通过", FIX_OK_5blocks),
                        ("卡住", FIX_MAXATTEMPTS), ("残缺", FIX_TRUNCATED),
                        ("空", [])):
        try:
            txt = board_a1.explain(board_a1.grade(lines, 4), 4)
            ok = isinstance(txt, str) and len(txt) > 20
        except Exception as exc:                       # noqa: BLE001
            ok, txt = False, f"{type(exc).__name__}: {exc}"
        check(f"explain() 对「{name}」日志能出话", ok)


# ------------------------------------------------------------ 4. 真·变异检查
def load_mutant(src_text, tag):
    """把源码写进临时目录后导入，返回**变异体**模块。"""
    d = tempfile.mkdtemp(prefix=f"a1mut_{tag}_")
    path = os.path.join(d, "board_a1_mutant.py")
    with open(path, "w") as fh:
        fh.write(src_text)
    spec = importlib.util.spec_from_file_location(f"board_a1_mutant_{tag}", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def section_mutation():
    print("\n[4] 变异检查：把判分逻辑改坏，结论必须**真的翻面**")
    src_path = os.path.join(HERE, "board_a1.py")
    if not os.path.exists(src_path):
        check("找得到 board_a1.py", False, src_path)
        return
    src = open(src_path, encoding="utf-8").read()

    # M1：把判据 2 从 ok 里摘掉 —— 4/4 那轮就会被误报成「A1 通过」
    old = '"ok": bool(full and slots_gone),'
    new = '"ok": bool(full),'
    if old not in src:
        check("M1 找得到要改的那一行", False, old)
    else:
        m = load_mutant(src.replace(old, new, 1), "m1")
        base = board_a1.grade(FIX_REAL_4of4, 4)["ok"]
        got = m.grade(FIX_REAL_4of4, 4)["ok"]
        check("M1 摘掉判据 2 ⇒ 真机那轮被判成通过（与改前不同）",
              base is False and got is True, f"改前={base} 改后={got}")

    # M2：判据 2 从「匹配收工行」放松成「整份日志找子串」—— 诱饵会骗过它
    old = 'slots_gone = bool(finish) and "放置槽位已用尽" in finish'
    new = 'slots_gone = "放置槽位已用尽" in "\\n".join(lines)'
    if old not in src:
        check("M2 找得到要改的那一行", False, old)
    else:
        m = load_mutant(src.replace(old, new, 1), "m2")
        probe = FIX_TRUNCATED + [DECOY]
        base = board_a1.grade(probe, 4)["slots_exhausted_ok"]
        got = m.grade(probe, 4)["slots_exhausted_ok"]
        check("M2 放松成整份日志找子串 ⇒ 被诱饵骗过（与改前不同）",
              base is False and got is True, f"改前={base} 改后={got}")

    # M3：进度行取首条而不是末条 —— 真机那轮会变成 0/4
    old = 'last_placed = placed[-1] if placed else (0, slots)'
    new = 'last_placed = placed[0] if placed else (0, slots)'
    if old not in src:
        check("M3 找得到要改的那一行", False, old)
    else:
        m = load_mutant(src.replace(old, new, 1), "m3")
        base = board_a1.grade(FIX_REAL_4of4, 4)["placed"]
        got = m.grade(FIX_REAL_4of4, 4)["placed"]
        # ⚠️ 变异体给的是 (1, 4) 而不是 (0, 4)：夹具里第一条匹配
        #    `已放置 X/N 个` 的行是「已放置 1/4 个」——候选数那几行写的是
        #    `已放置 0/4`（**没有「个」**），本来就不该被进度正则吃进来。
        check("M3 取首条进度行 ⇒ 真机那轮变成 1/4（与改前不同）",
              base == (4, 4) and got == (1, 4), f"改前={base} 改后={got}")


# --------------------------------------------------- 5. 起栈命令真的带上了关键参数
def section_launch():
    """`build_cmd` 拼错一个参数不会报错，只会**静默跑成另一回事** ——
    比如漏了 `start_delay`，起栈时规划器的启动 HOME 就会撞上状态容差
    （实测开局 `连续未放置 2/10`），而这一轮照样能跑完 4/4，
    于是「少了参数」被掩盖成「跑得好好的」。所以这里逐项钉住。"""
    import argparse

    print("\n[5] 起栈命令：关键参数逐项钉住（漏一个不会报错，只会静默跑成另一回事）")

    def cmd(**kw):
        # ⚠️ 用 RFC 5737 文档保留地址，**不要**改成 10./192.168. 开头的
        #    —— 那是真·私网段，脱敏自检会（正确地）报红。
        a = argparse.Namespace(robot_ip="203.0.113.7",
                               params="/tmp/p.yaml",
                               start_delay=25.0)
        for k, v in kw.items():
            setattr(a, k, v)
        return board_a1.build_cmd(a)

    base = cmd()
    check("起栈命令默认带 start_delay:=25（驱动先连、规划器后起）",
          "start_delay:=25" in base, f"得到 {[x for x in base if 'start_delay' in x]}")
    check("--start-delay 0 能关掉（传下去的是 0，不是被省略）",
          "start_delay:=0" in cmd(start_delay=0.0))
    check("start_delay 用 :g 格式化（12.5 不会变 12.500000）",
          "start_delay:=12.5" in cmd(start_delay=12.5))
    check("carry_objects:=true 在（否则只走位、不夹）",
          "carry_objects:=true" in base)
    check("--robot-ip 真的拼进去了", "robot_ip:=203.0.113.7" in base)
    check("七个相机外参全在（少一个就是静默错标定）",
          all(f"{k}:={v}" in base for k, v in board_a1.CAM.items())
          and len(board_a1.CAM) == 7,
          f"CAM 有 {len(board_a1.CAM)} 项")


def main():
    print("=" * 64)
    print("  board_a1 判分自检（纯离线：不碰 ROS / 臂 / 串口）")
    print("=" * 64)
    section_real()
    section_ok()
    section_others()
    section_mutation()
    section_launch()
    print("\n" + "=" * 64)
    if _failed:
        print(f"  FAIL：{len(_failed)} 项没过")
        for f in _failed:
            print("    ✗ " + f)
        return 1
    print("  全通过：判分逻辑对四条收工路径 + 两种残缺日志都判对了，")
    print("  且三处变异都让结论真的翻面（断言是吃劲的，不是恰好为真）。")
    return 0


if __name__ == "__main__":
    sys.exit(main())

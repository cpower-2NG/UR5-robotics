#!/usr/bin/env python3
"""V4 分析：**移遮挡策略** —— 贪心顺序 vs 最小遮挡集（契约 §11.3，仿真 S4）。

输入是 ``stage4_clear_probe.py`` 的探针报告（只算不动臂）。每条 attempt 已经带：

- ``clearing_steps``：贪心策略实际要挪的块（顺序 + 从哪到哪）；
- ``initial_failed_rows``：**第一次**评测时失败候选的逐障碍物间隙
  ``per_other``（米）；间隙 ≤0 = 接触 = 这个候选是被那一块挡住的。

两个量
======
- **贪心步数** = ``len(clearing_steps)``。
- **最小遮挡集（下界）** = 必须挪的块数下界：
  1. **叠放依赖**：从 ``clearing_steps`` 里 ``from`` 高度 > 0.03 m 的步骤
     （层 1 的块压在某块上；不动它就抓不了下面那块）—— 这些块任何策略都得挪；
  2. **侧向遮挡**：对每个失败候选，取 ``per_other`` 里间隙 ≤0 的块集合；
     最小的非空集合就是「只挪这几块就可能让某个候选变合法」的下界
     （**是下界，不是可行解**：挪走它们后候选仍可能被臂/桌面挡住，报告里
     另计 ``blocked_arm`` 之类不可由移块解除的失败）。
- **全桌合法率** = attempt 的 ``terminal.ok`` ∧ 目标块在格内且合法。

为什么用下界而不是再实现一套「最小集策略」：S4 现有 45 个 attempt 里，
`clearing_steps` 全部 ≤1，且 5 个非零步骤都是**叠放依赖**（被挪块 from z≈44 mm）。
也就是说这批场景里根本没有「多个侧向遮挡块、需要选集合」的决策点 ——
两种策略的结果必然相同。先用数据证明这一点，再决定要不要为 V4 造带侧向
遮挡的场景；不拿一个无决策点的对比冒充实验结论。

用法：
    python3 scripts/stage4_v4_occlusion.py --report <e9-sim-S4-v2.json> \\
        [--report ...] [--out v4.json]
退出码 0 恒（分析型工具）；输入缺失返回 2。
"""
import argparse
import json
import os
import sys

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

#: 层 1 及以上的块，from z 至少这么高（m）。层 0 块心 z≈14.75 mm、层 1≈44 mm。
STACK_Z_M = 0.030


def classify_step(step):
    """一步移遮挡：叠放依赖（被挪块本身在高处）还是侧向清障。"""
    z = float(step["from"][2])
    return "stack" if z > STACK_Z_M else "lateral"


def lateral_lower_bound(failed_rows, exempt=()):
    """侧向遮挡的最小集合大小下界；没有可归因的侧向失败 → 0。

    ``exempt``：已被算作堆叠步的块 —— 那些块无论侧不侧向都得搬走，
    在阻塞集合里再数一遍就是**重复计数**（2026-11-11 实测：S4:1 目标 1
    的阻塞集合 {0, 8} 里 8 正是堆叠依赖，下界算成 3，贪心只用 2 步）。
    """
    ex = {str(k) for k in exempt}
    best = None
    for r in failed_rows:
        per = r.get("per_other") or {}
        blockers = {str(k) for k, v in per.items()
                    if v is not None and float(v) <= 0.0} - ex
        if not blockers:
            continue
        best = len(blockers) if best is None else min(best, len(blockers))
    return 0 if best is None else best


def analyse_attempt(a):
    steps = a.get("clearing_steps") or []
    kinds = [classify_step(s) for s in steps]
    n_stack = sum(1 for k in kinds if k == "stack")
    n_lat = sum(1 for k in kinds if k == "lateral")
    stack_blocks = [s.get("move") for s, k in zip(steps, kinds)
                    if k == "stack"]
    # 一开始就可抓 ⇒ 侧向遮挡不是必需的（失败候选只是没被选中）；
    # 只有整块不可抓时，才需要为某个候选清掉它的侧向遮挡集合。
    # 堆叠步搬走的块在阻塞集合里扣掉，否则同一次物理搬运被数两遍。
    lb_lat = (0 if a.get("initial_b_ok")
              else lateral_lower_bound(a.get("initial_failed_rows") or [],
                                       exempt=stack_blocks))
    terminal = a.get("terminal") or {}
    return {
        "id": a.get("id"),
        "greedy_steps": len(steps),
        "n_stack_steps": n_stack,
        "n_lateral_steps": n_lat,
        "lower_bound": n_stack + lb_lat,
        "step_kinds": kinds,
        "moved": [s.get("move") for s in steps],
        "initial_b_ok": a.get("initial_b_ok"),
        "initial_failed": [r.get("verdict")
                           for r in (a.get("initial_failed_rows") or [])],
        "terminal_ok": bool(terminal.get("ok")),
        "target_legal": bool(a.get("ok")),
        "reason": a.get("reason"),
    }


def analyse_report(path):
    with open(path, encoding="utf-8") as f:
        d = json.load(f)
    rows = []
    for sc in d.get("scenes", []):
        for a in sc.get("attempts", []):
            r = analyse_attempt(a)
            r.update(batch=sc.get("batch"), index=sc.get("index"),
                     truth=sc.get("truth"))
            rows.append(r)
    n = len(rows)
    total_g = sum(r["greedy_steps"] for r in rows)
    total_lb = sum(r["lower_bound"] for r in rows)
    lat_points = sum(1 for r in rows if r["n_lateral_steps"] > 0)
    multi = sum(1 for r in rows if r["greedy_steps"] > 1)
    return {
        "report": path, "n": n,
        "greedy_steps_total": total_g,
        "lower_bound_total": total_lb,
        "steps_hist": {str(k): sum(1 for r in rows if r["greedy_steps"] == k)
                       for k in sorted({r["greedy_steps"] for r in rows})},
        "n_multi_step": multi,
        "n_lateral_decision_points": lat_points,
        "n_terminal_ok": sum(1 for r in rows if r["terminal_ok"]),
        "n_target_legal": sum(1 for r in rows if r["target_legal"]),
        "whole_table_legal_rate": ((sum(1 for r in rows if r["target_legal"])
                                    / n) if n else None),
        "attempts": rows,
    }


def render(reps):
    out = ["| 报告 | 尝试 | 贪心总步数 | 下界总步数 | 多步尝试 | 侧向决策点 "
           "| 全桌合法率 |", "|---|---|---|---|---|---|---|"]
    for r in reps:
        out.append("| %s | %d | %d | %d | %d | %d | %s |" % (
            os.path.basename(r["report"]), r["n"],
            r["greedy_steps_total"], r["lower_bound_total"],
            r["n_multi_step"], r["n_lateral_decision_points"],
            "—" if r["whole_table_legal_rate"] is None
            else "%.1f%%" % (100 * r["whole_table_legal_rate"])))
    return "\n".join(out)


def main(argv=None):
    ap = argparse.ArgumentParser(description="V4：移遮挡策略分析")
    ap.add_argument("--report", action="append", required=True)
    ap.add_argument("--out", default=None)
    args = ap.parse_args(argv)
    reps = []
    for p in args.report:
        if not os.path.exists(p):
            print("缺报告：%s" % p, file=sys.stderr)
            return 2
        reps.append(analyse_report(p))
    print(render(reps))
    for r in reps:
        h = r["steps_hist"]
        print("\n%s：步数分布 %s；叠放 %d 步 / 侧向 %d 步"
              % (os.path.basename(r["report"]), h,
                 sum(a["n_stack_steps"] for a in r["attempts"]),
                 sum(a["n_lateral_steps"] for a in r["attempts"])))
        if not r["n_lateral_decision_points"]:
            print("  ⇒ 没有侧向遮挡决策点：贪心与最小集在本数据上**不可区分**，"
                  "下界 = 贪心。")
        else:
            gap = r["greedy_steps_total"] - r["lower_bound_total"]
            print("  ⇒ 贪心比下界多 %d 步（下界不可行性未核验，见模块注释）"
                  % gap)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump({"kind": "stage4-v4", "stack_z_m": STACK_Z_M,
                       "reports": reps}, f, ensure_ascii=False, indent=1)
        print("\n→ %s" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())

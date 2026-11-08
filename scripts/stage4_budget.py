#!/usr/bin/env python3
"""E15 资源预算汇总：把各网的测量文件归一成**一张带阈值的表**。

为什么要有阈值而不仅是「记个数」：入口判据 E15 要求「峰值显存/内存/延迟
**有预算**」——没有预算就没有红灯，下次换个权重也不会有人发现。

阈值从哪来（每条都可查，不是拍的）：
- **延迟 · 板**：100 ms/帧 —— 直接沿用 E6 的板载门（`入口实验.md` E6 行）。
  板是部署目标，所以只有板上的延迟有硬线。**判定看「板(锁频)」那一行**（2026-11-08
  用户拍板：`sudo jetson_clocks` 是起栈前必做步）；默认调频那行留着作对照，
  它红不红不影响判据。
- **延迟 · 本机**：不设硬线。开发机不是部署目标（AGENTS §4：推理在本机、
  抓帧/执行在板上），只记录基线，防止静默退化。
- **内存 · 板**：峰值 RSS ≤ 板总内存的 50%。Orin 的「显存」是统一内存，
  计入 RSS，**不重复相加**。留一半给 ROS 2、相机缓冲、驱动。
- **内存 · 本机**：峰值显存 ≤ GPU 总量的 50%；RSS 同样 ≤ 系统内存 50%。

口径与测量脚本一致：网络只加载一次，**首帧算预热、不计入稳态**（板上首帧
含 CUDA 初始化与 kernel 选型，实测 12.5 s vs 稳态 0.11 s）。板上 RNGNet 的
测量文件里冷帧会显著离群，按「> 5× 中位」剔除并单独记数。

用法：
  python3 scripts/stage4_budget.py --dev runs/<run>/e20 --board runs/<run>/e20/board
退出码：0 全在预算内 / 1 有超预算项 / 2 输入缺失到无法判。
"""
import argparse
import glob
import json
import os
import statistics
import sys

# 机器容量：查法见台账对应轮；改机器就要改这里
BOARD_RAM_MB = 7619
DEV_GPU_MB = 6138
DEV_RAM_MB = 15 * 1024

LATENCY_BUDGET_MS_BOARD = 100.0      # E6 的板载门，逐字沿用
MEM_FRACTION = 0.5

# 每行：网络名、主机、测量文件（相对给定目录）、缺文件时算不算「未测」
MEASUREMENTS = [
    ("YOLO", "dev", "frame-s0/yolo_bench.json"),
    ("GraspNet", "dev", "graspnet_summary.json"),
    ("RNGNet", "dev", "rng-steady/e20-frame-s0b.json"),
    ("YOLO", "board", "yolo_bench.json"),
    ("RNGNet", "board", "rng-steady4/*.json"),
    # 同一门、另一种调度条件：GPU 锁最高频（jetson_clocks）。默认调频下板载只有
    # 306 MHz 起调、短帧跑不到满频，两者必须分开记，否则「过没过」取决于测法。
    ("RNGNet", "board-pinned", "rng-pinned/*.json"),
    ("GraspNet", "board", None),      # 板上没有 MinkowskiEngine ⇒ 结构性未测
]
HOST_LABEL = {"dev": "本机", "board": "板", "board-pinned": "板(锁频)"}


def _load(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def _steady(ms, vram=None, rss=None):
    """稳态统计：冷帧（>5× 中位）剔掉并计数。

    首帧含 CUDA 初始化与 cuDNN kernel 选型，实测同一帧在板上 12.5 s → 0.06 s，
    笼统取中位会把冷启动的代价藏进「延迟」里，所以单列出来。
    """
    ms = [float(m) for m in ms]
    if not ms:
        return {"latency_ms": None, "vram_mb": vram, "rss_mb": rss,
                "n_frames": 0, "n_cold_dropped": 0}
    med = statistics.median(ms)
    warm = [m for m in ms if m <= 5 * med] if med > 0 else ms
    return {"latency_ms": round(statistics.median(warm), 1) if warm else None,
            "vram_mb": vram or None, "rss_mb": rss or None,
            "n_frames": len(warm), "n_cold_dropped": len(ms) - len(warm),
            "cold_ms": round(max(ms), 1) if len(warm) < len(ms) else None}


def _rng_from_reports(recs):
    return _steady([r["infer_ms"] for r in recs],
                   max((r.get("vram_peak_mb") or 0) for r in recs),
                   max((r.get("rss_peak_mb") or 0) for r in recs))


def _rng_from_one(rec):
    return _steady([rec.get("infer_ms", 0)], rec.get("vram_peak_mb"),
                   rec.get("rss_peak_mb"))


def _graspnet(summary):
    ms = [t for f in summary.get("frames", []) for t in f.get("infer_ms", [])]
    # 汇总文件不带 RSS（只有单帧 graspnet.json 才有），这一格留空而不是猜 0
    return _steady(ms, max((f.get("vram_peak_mb") or 0)
                           for f in summary.get("frames", [])))


def _yolo(rec):
    return {"latency_ms": rec.get("median_ms"),
            "vram_mb": rec.get("vram_peak_mb"),
            "rss_mb": rec.get("rss_peak_mb"),
            "n_frames": len(rec.get("infer_ms") or []) or None,
            "n_cold_dropped": 1 if rec.get("warmup_ms") else 0}


def collect(dev_dir, board_dir, pinned_dir=None):
    rows = []
    for net, host, rel in MEASUREMENTS:
        row = {"net": net, "host": host, "status": "未测", "latency_ms": None,
               "vram_mb": None, "rss_mb": None, "source": rel}
        if rel is None:
            row["note"] = "板上缺 MinkowskiEngine（无外网，需交叉编译）"
            rows.append(row)
            continue
        base = {"dev": dev_dir, "board": board_dir,
                "board-pinned": pinned_dir or board_dir}[host]
        paths = sorted(glob.glob(os.path.join(base, rel)))
        if not paths:
            row["note"] = "测量文件不在"
            rows.append(row)
            continue
        recs = [_load(p) for p in paths]
        if net == "RNGNet":
            got = _rng_from_one(recs[0]) if len(recs) == 1 \
                else _rng_from_reports(recs)
        elif net == "GraspNet":
            got = _graspnet(recs[0])
        else:
            got = _yolo(recs[0])
        row.update(got)
        row["status"] = "已测"
        rows.append(row)
    return rows


def budgets(host):
    """该主机的预算三元组；None 表示该项不设硬线。"""
    if host in ("board", "board-pinned"):
        return {"latency_ms": LATENCY_BUDGET_MS_BOARD,
                "vram_mb": None,                      # 统一内存，计入 RSS
                "rss_mb": BOARD_RAM_MB * MEM_FRACTION}
    return {"latency_ms": None,
            "vram_mb": DEV_GPU_MB * MEM_FRACTION,
            "rss_mb": DEV_RAM_MB * MEM_FRACTION}


def judge(row):
    """返回 (verdict, 超限项列表)。未测 ⇒ unknown，不是 pass。"""
    if row["status"] != "已测":
        return "unknown", []
    b = budgets(row["host"])
    over = []
    for k in ("latency_ms", "vram_mb", "rss_mb"):
        val, lim = row.get(k), b.get(k)
        if val is not None and lim is not None and val > lim:
            over.append("%s %.1f > %.0f" % (k, val, lim))
    return ("fail" if over else "pass"), over


def _fmt(key, row, spec):
    v = row.get(key)
    return "—" if v is None else spec % v


def render(rows):
    out = []
    hdr = ("| 网络 | 主机 | 稳态延迟 ms | 峰值显存 MB | 峰值 RSS MB | 预算 | 判定 |"
           "\n|---|---|---|---|---|---|---|")
    out.append(hdr)
    for r in rows:
        verdict, over = judge(r)
        b = budgets(r["host"])
        lim = []
        if b["latency_ms"]:
            lim.append("延迟 ≤%.0f ms" % b["latency_ms"])
        if b["vram_mb"]:
            lim.append("显存 ≤%.0f MB" % b["vram_mb"])
        if b["rss_mb"]:
            lim.append("RSS ≤%.0f MB" % b["rss_mb"])
        mark = {"pass": "✅ 内", "fail": "❌ 超（%s）" % "；".join(over),
                "unknown": "— 未测"
                }[verdict]
        if r.get("n_cold_dropped"):
            mark += "（剔冷帧 %d）" % r["n_cold_dropped"]
        out.append("| %s | %s | %s | %s | %s | %s | %s |" % (
            r["net"], HOST_LABEL[r["host"]],
            _fmt("latency_ms", r, "%.1f"),
            _fmt("vram_mb", r, "%.0f"),
            _fmt("rss_mb", r, "%.0f"),
            "；".join(lim) or "—", mark))
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description="E15 资源预算汇总")
    ap.add_argument("--dev", required=True, help="本机测量目录")
    ap.add_argument("--board", required=True, help="板端测量目录")
    ap.add_argument("--board-pinned", default=None,
                    help="板端锁频(jetson_clocks)复测目录，可缺")
    ap.add_argument("--json", default=None, help="同时写出机器可读结果")
    args = ap.parse_args()

    rows = collect(args.dev, args.board, args.board_pinned)
    print(render(rows))
    verdicts = [judge(r)[0] for r in rows]
    if args.json:
        rec = {"kind": "stage4-budget", "rows": rows,
               "budgets": {h: budgets(h) for h in HOST_LABEL},
               "verdicts": verdicts}
        os.makedirs(os.path.dirname(os.path.abspath(args.json)) or ".",
                    exist_ok=True)
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(rec, f, ensure_ascii=False, indent=1)
        print("\n→ %s" % args.json)
    if "fail" in verdicts:
        return 1
    if "unknown" in verdicts:
        print("\n注意：有 %d 项未测（unknown 不是通过）。" % verdicts.count("unknown"))
    return 0


if __name__ == "__main__":
    sys.exit(main())

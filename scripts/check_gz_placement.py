#!/usr/bin/env python3
"""用 Gazebo 真值核对「块真的放进槽位了吗」——阶段 4 验收 A4 的只读判据器。

为什么需要它
============
planner 的 `已放置 N/M` 是**自报**的：`_on_place_released` 里张开夹爪后立刻
`_place_placed += 1`（planner_node.py 的放置回调），中间没有任何物理核对。
夹爪张得太快、块没放稳、块被邻块推走，计数照样涨。本脚本读 gz 世界的
**物理真值**做独立旁证 —— 与「仿真抓取要用 gz 真值核验」同一条原则。

只读
====
只 `gz topic -e` 订阅一条 dynamic_pose 消息，不 spawn、不 set_pose、
不发任何 ROS 指令、不写任何文件（`--json` 只打印到 stdout）。

槽位来源
========
`grasp_planner_sim.yaml` 的 `place_slots`（扁平 [x0,y0,x1,y1,...]）、
`place_z`、`table_min_z`、`pick_zone`。脚本里**不写死**任何槽位坐标；
默认路径相对本仓，可用 `--yaml` 指向运行中的那份。

A4 判据（计划 §19.4，全部由真值判）
==================================
一个块算「已放置」要同时满足：

  1. 名字以 `--name-prefix`（默认 `block`）开头；
  2. 到最近槽位中心的 xy 距离 ≤ `--tol-mm`（默认 40 mm，槽距 130 mm）；
  3. 中心 z ∈ [`--z-min-mm`, `--z-max-mm`]（默认 [12, 23] mm）——排除
     「还在爪里（悬空）」与「掉到地上」；注意桌面上未抓的块 z=14.75 mm
     也落在这个带里，**判别项是槽位 xy，不是 z**；
  4. 块自身 z 轴与世界 z 夹角 ≤ `--max-tilt-deg`（默认 15°）——排除摔翻。

整轮 A4 额外要求（`--require-a4` 时判）：

  5. 每块落到的最近槽位互不相同（一块一格）；
  6. 两两块心 3D 距离 ≥ `--min-pair-mm`（默认 25 mm）——排除叠放；
  7. 拾取区（`pick_zone` 的 xy）里没有块残留。

输出
====
每块一行（位置 + 倾角 + 判定），末尾 `已放置 N/M`（M = 场景里检出的块数）
与 `A4: PASS/FAIL`。返回码：0 = 读成功且所有指定的期望都满足；
1 = `--expect-placed` 不符或 `--require-a4` 未过；2 = 读 gz / 参数失败。

用法
====
    python3 scripts/check_gz_placement.py
    python3 scripts/check_gz_placement.py --expect-placed 0      # 负对照
    python3 scripts/check_gz_placement.py --expect-placed 5 --require-a4
"""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_YAML = (REPO_ROOT / "ros2_ws" / "src" / "arm_grasp_sim"
                / "config" / "grasp_planner_sim.yaml")


# ── gz 读取 ────────────────────────────────────────────────────────────────
def find_dynamic_pose_topic(timeout: float) -> str:
    """在 `gz topic -l` 里找唯一的 /world/*/dynamic_pose/info。"""
    try:
        out = subprocess.run(["gz", "topic", "-l"], capture_output=True,
                             text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise RuntimeError(f"跑不了 `gz topic -l`：{exc}") from exc
    if out.returncode != 0:
        raise RuntimeError(f"`gz topic -l` 失败：{out.stderr.strip()}")
    cands = [ln.strip() for ln in out.stdout.splitlines()
             if ln.strip().startswith("/world/")
             and ln.strip().endswith("/dynamic_pose/info")]
    if len(cands) == 1:
        return cands[0]
    if not cands:
        raise RuntimeError(
            "没找到 /world/*/dynamic_pose/info —— 仿真栈起来了吗？"
            "（判活看 `gz topic -l`，不要看 action_ready）")
    raise RuntimeError(
        "找到多个 dynamic_pose 话题，请用 --topic 指定：\n  "
        + "\n  ".join(cands))


def read_pose_text(topic: str, timeout: float) -> str:
    try:
        out = subprocess.run(["gz", "topic", "-e", "-t", topic, "-n", "1"],
                             capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise RuntimeError(f"读 {topic} 超时/失败：{exc}") from exc
    if out.returncode != 0:
        raise RuntimeError(f"`gz topic -e` 失败：{out.stderr.strip()}")
    return out.stdout


def parse_poses(text: str) -> list[dict]:
    """解析 gz 的 text-proto，返回 [{name,x,y,z,qx,qy,qz,qw}, ...]。

    只认 `pose { name ... position { x y z } orientation { x y z w } }`；
    header 用括号栈跳过。不引 protobuf，避免把脚本绑到 gz 的 Python 环境。
    """
    poses: list[dict] = []
    cur: dict | None = None
    sub: str | None = None
    for raw in text.splitlines():
        line = raw.strip()
        if not line:
            continue
        if line == "pose {":
            cur = {}
            sub = None
            continue
        if cur is None:
            continue
        if line.endswith("{"):
            sub = line[:-1].strip()
            continue
        if line == "}":
            if sub is not None:
                sub = None
            else:                      # pose 块结束
                if {"name", "x", "y", "z"} <= cur.keys():
                    poses.append(cur)
                cur = None
            continue
        key, _, val = line.partition(":")
        key, val = key.strip(), val.strip()
        if sub == "position":
            cur[key] = float(val)
        elif sub == "orientation":
            cur["q" + key] = float(val)
        elif sub is None and key == "name":
            cur["name"] = val.strip('"')
    return poses


# ── yaml ──────────────────────────────────────────────────────────────────
def load_planner_params(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    try:
        return data["grasp_planner"]["ros__parameters"]
    except (KeyError, TypeError) as exc:
        raise RuntimeError(
            f"{path} 里没有 grasp_planner.ros__parameters —— 是不是拿错文件了？"
        ) from exc


def slots_from_params(params: dict) -> list[tuple[float, float]]:
    flat = [float(v) for v in params.get("place_slots") or []]
    if not flat:
        raise RuntimeError("place_slots 是空的 —— 没有槽位可判")
    if len(flat) % 2:
        raise RuntimeError(
            f"place_slots 有 {len(flat)} 个值（奇数）—— 必须成对 [x0,y0,...]")
    return list(zip(flat[0::2], flat[1::2]))


def pick_zone_from_params(params: dict) -> tuple[float, float, float, float] | None:
    pz = params.get("pick_zone")
    if not pz or len(pz) < 4:
        return None
    return (float(pz[0]), float(pz[1]), float(pz[2]), float(pz[3]))


# ── 判定 ──────────────────────────────────────────────────────────────────
def tilt_deg(p: dict) -> float | None:
    """块自身 z 轴与世界 z 的夹角（度）。四元数缺失时返回 None。"""
    keys = ("qx", "qy", "qz", "qw")
    if not all(k in p for k in keys):
        return None
    qx, qy, qz, qw = (p[k] for k in keys)
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n == 0:
        return None
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    # 旋转矩阵 (3,3) = 1 - 2(qx² + qy²)
    r33 = 1.0 - 2.0 * (qx * qx + qy * qy)
    r33 = max(-1.0, min(1.0, r33))
    return math.degrees(math.acos(r33))


def classify(poses: list[dict], slots: list[tuple[float, float]],
             tol_m: float, z_min: float, z_max: float, prefix: str,
             max_tilt_deg: float = 15.0, min_pair_m: float = 0.025,
             pick_zone: tuple[float, float, float, float] | None = None) -> dict:
    blocks = [p for p in poses if p["name"].startswith(prefix)]
    rows = []
    by_slot: dict[int, list[str]] = {}
    placed_pts: list[tuple[str, float, float, float]] = []
    for p in sorted(blocks, key=lambda b: b["name"]):
        dists = [math.hypot(p["x"] - sx, p["y"] - sy) for sx, sy in slots]
        idx = min(range(len(dists)), key=dists.__getitem__)
        d = dists[idx]
        in_xy = d <= tol_m
        in_z = z_min <= p["z"] <= z_max
        tilt = tilt_deg(p)
        in_tilt = tilt is not None and tilt <= max_tilt_deg
        placed = in_xy and in_z and in_tilt
        if placed:
            by_slot.setdefault(idx, []).append(p["name"])
            placed_pts.append((p["name"], p["x"], p["y"], p["z"]))
        if not in_xy:
            reason = f"最近槽位 #{idx} 距 {d * 1000:.1f} mm"
        elif not in_z:
            reason = (f"xy 在槽位 #{idx}，但 z={p['z'] * 1000:.1f} mm 不在 "
                      f"[{z_min * 1000:.0f}, {z_max * 1000:.0f}]")
        elif not in_tilt:
            reason = (f"槽位 #{idx}，但倾角 "
                      f"{'?' if tilt is None else f'{tilt:.1f}°'} > {max_tilt_deg}°")
        else:
            reason = f"槽位 #{idx}"
        rows.append({
            "name": p["name"],
            "x_mm": round(p["x"] * 1000, 1),
            "y_mm": round(p["y"] * 1000, 1),
            "z_mm": round(p["z"] * 1000, 1),
            "tilt_deg": None if tilt is None else round(tilt, 1),
            "nearest_slot": idx,
            "dist_mm": round(d * 1000, 1),
            "placed": placed,
            "reason": reason,
        })

    conflicts = {i: names for i, names in by_slot.items() if len(names) > 1}

    # 两两块心 3D 距离（全部块；A4 要求 ≥ min_pair）
    pair_viol = []
    for i in range(len(blocks)):
        for j in range(i + 1, len(blocks)):
            a, b = blocks[i], blocks[j]
            d3 = math.dist((a["x"], a["y"], a["z"]),
                           (b["x"], b["y"], b["z"]))
            if d3 < min_pair_m:
                pair_viol.append({
                    "a": a["name"], "b": b["name"],
                    "dist_mm": round(d3 * 1000, 1)})

    # 拾取区残留（xy ∈ pick_zone，且 z 在台面附近）
    leftovers = []
    if pick_zone:
        x0, x1, y0, y1 = pick_zone
        for p in blocks:
            if x0 <= p["x"] <= x1 and y0 <= p["y"] <= y1 and p["z"] <= 0.06:
                leftovers.append(p["name"])

    n_placed = sum(1 for r in rows if r["placed"])
    a4_ok = (bool(blocks) and n_placed == len(blocks)
             and not conflicts and not pair_viol and not leftovers)
    return {
        "n_blocks": len(blocks),
        "n_placed": n_placed,
        "n_slots": len(slots),
        "slots_used": sorted(by_slot),
        "conflicts": conflicts,
        "pairwise_violations": pair_viol,
        "pick_zone_leftovers": leftovers,
        "a4_ok": a4_ok,
        "blocks": rows,
    }


# ── main ──────────────────────────────────────────────────────────────────
def main() -> int:
    ap = argparse.ArgumentParser(
        description="只读：用 gz 真值核对块是否真的在放置槽位里（A4）")
    ap.add_argument("--yaml", type=Path, default=DEFAULT_YAML,
                    help=f"planner 参数文件（默认 {DEFAULT_YAML}）")
    ap.add_argument("--topic", default=None,
                    help="dynamic_pose 话题；默认自动找唯一的 /world/*/dynamic_pose/info")
    ap.add_argument("--tol-mm", type=float, default=40.0,
                    help="xy 判据：到最近槽位中心的最大距离（默认 40 mm）")
    ap.add_argument("--z-min-mm", type=float, default=12.0,
                    help="块心 z 下限（默认 12 mm）")
    ap.add_argument("--z-max-mm", type=float, default=23.0,
                    help="块心 z 上限（默认 23 mm）")
    ap.add_argument("--max-tilt-deg", type=float, default=15.0,
                    help="块 z 轴与世界 z 的最大夹角（默认 15°）")
    ap.add_argument("--min-pair-mm", type=float, default=25.0,
                    help="两两块心最小 3D 距离（默认 25 mm）")
    ap.add_argument("--name-prefix", default="block",
                    help="块模型名前缀（默认 block）")
    ap.add_argument("--timeout", type=float, default=15.0,
                    help="gz 命令超时秒数（默认 15）")
    ap.add_argument("--expect-placed", type=int, default=None,
                    help="断言已放置数；不符则返回 1（负对照用 --expect-placed 0）")
    ap.add_argument("--require-a4", action="store_true",
                    help="要求整轮 A4 通过（全放、一块一格、无叠放、拾取区无残留）")
    ap.add_argument("--json", action="store_true", help="输出 JSON（只打印，不写文件）")
    args = ap.parse_args()

    try:
        params = load_planner_params(args.yaml)
        slots = slots_from_params(params)
        pick_zone = pick_zone_from_params(params)
        topic = args.topic or find_dynamic_pose_topic(args.timeout)
        poses = parse_poses(read_pose_text(topic, args.timeout))
    except RuntimeError as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 2

    result = classify(poses, slots, args.tol_mm / 1000.0,
                      args.z_min_mm / 1000.0, args.z_max_mm / 1000.0,
                      args.name_prefix, args.max_tilt_deg,
                      args.min_pair_mm / 1000.0, pick_zone)
    result["topic"] = topic
    result["yaml"] = str(args.yaml)
    result["place_z_mm"] = round(float(params.get("place_z", 0.0324)) * 1000, 1)

    if args.json:
        print(json.dumps(result, ensure_ascii=False, indent=2))
    else:
        print(f"话题: {topic}")
        print(f"槽位: {result['n_slots']} 个（place_slots ← {args.yaml}）")
        for i, (sx, sy) in enumerate(slots):
            print(f"  #{i}  ({sx * 1000:8.1f}, {sy * 1000:8.1f}) mm")
        if pick_zone:
            print(f"拾取区 xy: [{pick_zone[0] * 1000:.0f}, {pick_zone[1] * 1000:.0f}]"
                  f" × [{pick_zone[2] * 1000:.0f}, {pick_zone[3] * 1000:.0f}] mm")
        print(f"块: {result['n_blocks']} 个（前缀 {args.name_prefix!r}）")
        for r in result["blocks"]:
            mark = "已放置" if r["placed"] else "未放置"
            tilt = "?" if r["tilt_deg"] is None else f"{r['tilt_deg']:.1f}°"
            print(f"  {r['name']:<12} ({r['x_mm']:8.1f}, {r['y_mm']:8.1f}, "
                  f"{r['z_mm']:7.1f}) mm  倾角 {tilt:>6}  {mark}  [{r['reason']}]")
        for idx, names in result["conflicts"].items():
            print(f"  ⚠ 槽位 #{idx} 有 {len(names)} 块：{', '.join(names)}")
        for v in result["pairwise_violations"]:
            print(f"  ⚠ 两两过近：{v['a']} ↔ {v['b']} = {v['dist_mm']} mm")
        if result["pick_zone_leftovers"]:
            print(f"  ⚠ 拾取区残留：{', '.join(result['pick_zone_leftovers'])}")
        print(f"槽位占用: {len(result['slots_used'])}/{result['n_slots']}")
        print(f"已放置 {result['n_placed']}/{result['n_blocks']}")

        # A4 分项
        n = result["n_blocks"]
        checks = [
            ("全部块都在槽位里", result["n_placed"] == n),
            ("一块一格（无冲突）", not result["conflicts"]),
            (f"两两块心 ≥ {args.min_pair_mm:.0f} mm", not result["pairwise_violations"]),
            ("拾取区无残留", not result["pick_zone_leftovers"]),
        ]
        if args.require_a4:
            for label, ok in checks:
                print(f"  {'✅' if ok else '❌'} {label}")
        print(f"A4: {'PASS' if result['a4_ok'] else 'FAIL'}")

    rc = 0
    if args.expect_placed is not None and result["n_placed"] != args.expect_placed:
        print(f"[FAIL] 期望已放置 {args.expect_placed}，实测 {result['n_placed']}",
              file=sys.stderr)
        rc = 1
    if args.require_a4 and not result["a4_ok"]:
        print("[FAIL] A4 判据未全过", file=sys.stderr)
        rc = 1
    return rc


if __name__ == "__main__":
    sys.exit(main())

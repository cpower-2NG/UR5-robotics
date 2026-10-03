#!/usr/bin/env python3
"""把「URDF 用标称运动学、机器人用出厂标定」这件事，换算成一个毫米数。

背景（见 docs/RESTRUCTURE_PLAN.md 第八十一轮 §四、第八十二轮）：
    UR 驱动启动时会拿 URDF 里的 ``kinematics/hash`` 去和机器人通过主接口
    报上来的哈希比对。不等就报 ERROR：

        The calibration parameters of the connected robot don't match the ones
        from the given kinematics config file. ... critical inaccuracies of tcp positions.

    这句话只说了「有偏差」，没说**多大**。这个脚本负责把它量化。

做法：
    同一个 URDF（``ur_description`` 的 xacro），只换 ``kinematics_params`` 那一份
    yaml，生成两份 URDF；然后在同一组关节角下各做一次正运动学，比较 ``tool0``。
    关节角组随机采样，报告位置/姿态偏差的最大值、均值、p95。

    两份模型用**同一个** URDF 解析器和同一个 ``tool0`` 约定，所以任何约定误差在
    「差值」里自动抵消 —— 这就是这里只信差值、不信绝对值的原因。

用法：
    python3 scripts/ur_calibration_delta.py <nominal.urdf> <calibrated.urdf>
    # 可选：--n 2000 --seed 0 --json out.json

生成那两份 URDF 的办法（板上，已装 ur_description）：
    xacro $(ros2 pkg prefix ur_description)/share/ur_description/urdf/ur.urdf.xacro \\
        ur_type:=ur5 name:=ur5 \\
        kinematics_params:=<yaml> joint_limit_params:=... physical_params:=... \\
        visual_params:=... > out.urdf
    —— 更省事的办法见 scripts/make_ur_calibration_urdfs.sh。

本脚本**不需要** ROS、不需要 numpy 以外的东西，也不连机器人。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np


# ---------------------------------------------------------------- URDF 解析


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF 约定：R = Rz(yaw) @ Ry(pitch) @ Rx(roll)。"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


def _transform(xyz, rpy) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = _rpy_to_matrix(*rpy)
    t[:3, 3] = xyz
    return t


def _axis_angle(axis, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n == 0:
        raise ValueError("revolute joint with zero axis")
    axis = axis / n
    k = np.array(
        [
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ]
    )
    r = np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)
    t = np.eye(4)
    t[:3, :3] = r
    return t


class UrdfChain:
    """base_link -> tip 的关节链，外加每个关节的 origin 原始数值。"""

    def __init__(self, path: str, base: str, tip: str):
        root = ET.parse(path).getroot()
        self.joints: dict[str, ET.Element] = {}
        parent_of: dict[str, str] = {}
        for j in root.findall("joint"):
            name = j.get("name")
            self.joints[name] = j
            parent_of[j.find("child").get("link")] = j.find("parent").get("link")

        # 从 tip 往上走到 base，再反过来
        chain: list[str] = []
        link = tip
        while link != base:
            if link not in parent_of:
                raise ValueError(f"{path}: 从 {tip} 往上游找不到 {base}（走到 {link} 断了）")
            jname = None
            for name, j in self.joints.items():
                if j.find("child").get("link") == link:
                    jname = name
                    break
            chain.append(jname)
            link = parent_of[link]
        chain.reverse()
        self.chain = chain

    def movable(self) -> list[str]:
        """链上真正会动的关节（revolute/continuous），顺序与 self.chain 一致。"""
        out = []
        for name in self.chain:
            if self.joints[name].get("type") in ("revolute", "continuous"):
                out.append(name)
        return out

    def origin_table(self) -> dict[str, tuple[tuple[float, ...], tuple[float, ...]]]:
        out = {}
        for name in self.chain:
            o = self.joints[name].find("origin")
            xyz = tuple(float(v) for v in (o.get("xyz", "0 0 0")).split())
            rpy = tuple(float(v) for v in (o.get("rpy", "0 0 0")).split())
            out[name] = (xyz, rpy)
        return out

    def fk(self, joint_angles: dict[str, float]) -> np.ndarray:
        t = np.eye(4)
        for name in self.chain:
            j = self.joints[name]
            o = j.find("origin")
            xyz = [float(v) for v in (o.get("xyz", "0 0 0")).split()]
            rpy = [float(v) for v in (o.get("rpy", "0 0 0")).split()]
            t = t @ _transform(xyz, rpy)
            jtype = j.get("type")
            if jtype in ("revolute", "continuous"):
                ax_el = j.find("axis")
                axis = [float(v) for v in (ax_el.get("xyz") if ax_el is not None else "1 0 0").split()]
                t = t @ _axis_angle(axis, joint_angles[name])
        return t


# ---------------------------------------------------------------- 主流程


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("nominal", help="用标称运动学生成的 URDF")
    ap.add_argument("calibrated", help="用出厂标定生成的 URDF")
    ap.add_argument("--base", default="base_link")
    ap.add_argument("--tip", default="tool0")
    ap.add_argument("--n", type=int, default=2000, help="随机采样的关节角组数（默认 2000）")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--json", help="把结果也写一份 JSON 到这个路径")
    args = ap.parse_args()

    a = UrdfChain(args.nominal, args.base, args.tip)
    b = UrdfChain(args.calibrated, args.base, args.tip)

    if a.chain != b.chain:
        print("两份 URDF 的关节链不同，没法比较：", file=sys.stderr)
        print("  nominal   :", a.chain, file=sys.stderr)
        print("  calibrated:", b.chain, file=sys.stderr)
        return 2

    mov_a, mov_b = a.movable(), b.movable()
    if mov_a != mov_b:
        print("可动关节集合不同：", mov_a, mov_b, file=sys.stderr)
        return 2

    # ---- 1. 逐关节 origin 的差值（这是「标定到底改了什么」的原始形态）
    ta, tb = a.origin_table(), b.origin_table()
    print("=" * 78)
    print("每个关节 origin 的差值（标定 − 标称）")
    print("=" * 78)
    print(f"{'joint':<22}{'Δx[mm]':>10}{'Δy[mm]':>10}{'Δz[mm]':>10}"
          f"{'Δroll[°]':>11}{'Δpitch[°]':>11}{'Δyaw[°]':>11}")
    worst_lin = 0.0
    worst_ang = 0.0
    for name in a.chain:
        (xa, ra), (xb, rb) = ta[name], tb[name]
        dl = [(xb[i] - xa[i]) * 1000.0 for i in range(3)]
        da = [math.degrees(xb[i] - xa[i]) for i in range(3)]
        worst_lin = max(worst_lin, max(abs(v) for v in dl))
        worst_ang = max(worst_ang, max(abs(v) for v in da))
        print(f"{name:<22}{dl[0]:>10.4f}{dl[1]:>10.4f}{dl[2]:>10.4f}"
              f"{da[0]:>11.5f}{da[1]:>11.5f}{da[2]:>11.5f}")
    print()
    print(f"单关节 origin 最大偏差：位置 {worst_lin:.4f} mm，角度 {worst_ang:.5f}°")

    # ---- 2. 在同一组关节角下比 tool0
    rng = np.random.default_rng(args.seed)
    n = args.n
    qs = rng.uniform(-math.pi, math.pi, size=(n, len(mov_a)))
    pos_dev = np.empty(n)
    rot_dev = np.empty(n)
    abs_nominal = np.empty((n, 3))
    for i in range(n):
        q = {name: float(qs[i, k]) for k, name in enumerate(mov_a)}
        Ta, Tb = a.fk(q), b.fk(q)
        pos_dev[i] = np.linalg.norm(Tb[:3, 3] - Ta[:3, 3]) * 1000.0
        cosang = (np.trace(Ta[:3, :3].T @ Tb[:3, :3]) - 1.0) / 2.0
        rot_dev[i] = math.degrees(math.acos(max(-1.0, min(1.0, cosang))))
        abs_nominal[i] = Ta[:3, 3]

    print()
    print("=" * 78)
    print(f"同一关节角下 {args.tip} 的偏差（{n} 组随机关节角，±180° 均匀采样）")
    print("=" * 78)
    for label, arr in (("位置 [mm]", pos_dev), ("姿态 [°]", rot_dev)):
        print(f"{label:<12} 均值 {arr.mean():8.4f}  中位 {np.median(arr):8.4f}  "
              f"p95 {np.percentile(arr, 95):8.4f}  最大 {arr.max():8.4f}")

    # 姿态偏差几乎恒定（标定的旋转部分是个小常量），位置偏差随臂展放大 ——
    # 所以「最大」比「均值」更该拿来做判据。
    print()
    print("结论口径：抓取判据要用 **最大**，不要用均值 ——")
    print("  位置偏差 ≈ 各连杆长度误差的矢量和，随关节角变化，工作区边缘最大。")

    # ---- 3. 绝对值的自检（只印出来给人对，不作判据）
    t0 = a.fk({name: 0.0 for name in mov_a})
    print()
    print(f"自检：标称模型在 q=0 时 {args.tip} 位置 = "
          f"[{t0[0, 3]:.4f}, {t0[1, 3]:.4f}, {t0[2, 3]:.4f}] m  "
          f"（用来核对解析器没搭错，不做判据）")

    if args.json:
        payload = {
            "tip": args.tip,
            "n": n,
            "seed": args.seed,
            "joints": mov_a,
            "origin_delta": {
                name: {
                    "d_xyz_mm": [(tb[name][0][i] - ta[name][0][i]) * 1000.0 for i in range(3)],
                    "d_rpy_deg": [math.degrees(tb[name][1][i] - ta[name][1][i]) for i in range(3)],
                }
                for name in a.chain
            },
            "tcp_position_dev_mm": {
                "mean": float(pos_dev.mean()),
                "median": float(np.median(pos_dev)),
                "p95": float(np.percentile(pos_dev, 95)),
                "max": float(pos_dev.max()),
            },
            "tcp_rotation_dev_deg": {
                "mean": float(rot_dev.mean()),
                "median": float(np.median(rot_dev)),
                "p95": float(np.percentile(rot_dev, 95)),
                "max": float(rot_dev.max()),
            },
        }
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump(payload, fh, indent=2, ensure_ascii=False)
        print(f"\nJSON 已写到 {args.json}")

    return 0


if __name__ == "__main__":
    sys.exit(main())

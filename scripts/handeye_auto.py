#!/usr/bin/env python3
"""手眼标定**自动采点/求解**：人只管挪臂，这个脚本自己判断并采样本。

它只做三件事（都是调 `easy_handeye2` 已有的服务，不自己算 TF）：

1. 轮询 `/easy_handeye2/calibration/get_current_transforms` —— 拿到**当前**这一拍的
   `base_link←tool0` 和 `相机←靶板`。**任一条查不到就返回空**，这是它的设计，
   也正是我们要的：靶板那一拍不在画面里 ⇒ 这一拍不采。
2. 跟已经采下的样本比：**离得够远**（平移 > `--min-mm` 或 转 > `--min-deg`）才采。
   否则臂没动/动了一点点也会被记成一堆几乎相同的样本 —— 那批样本对求解没有信息，
   却会把方程组的条件数拉坏（[[verify-before-asserting]]：矩阵秩不够时解出的东西看着像真的）。
3. 收满 `--target` 个样本就**停**，然后五种算法各解一遍，报**互相之间的散布**。
   （散布大 ⇒ 样本的信息量不够，该回去再转几个方向，而不是挑一个好看的数交差。）

它**不碰机器人**：不发任何运动命令，只读 TF、只调标定服务。
存盘写在 `~/.ros2/easy_handeye2/`，是板子本地的事。

用法（板上跑，三个终端或后台）：
    # ① 桥（发 相机←靶板）
    python3 scripts/charuco_tf_bridge.py --intrinsics calib/.../wrist_intrinsics.yaml ...
    # ② 服务端（读两条 TF，提供采样/求解服务）
    ros2 run easy_handeye2 handeye_server --ros-args -p name:=wrist_eye_in_hand ...
    # ③ 本脚本
    python3 scripts/handeye_auto.py --target 20
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
# easy_handeye2_msgs 的服务类型在板上才 import 得到；本文件在笔记本上也要能过 pyflakes。
try:
    from easy_handeye2_msgs.srv import ComputeCalibration, SaveCalibration, SetAlgorithm, TakeSample
except ImportError:  # pragma: no cover - 笔记本上没有这个包
    ComputeCalibration = SaveCalibration = SetAlgorithm = TakeSample = None

NS = "/easy_handeye2/calibration"
S_TAKE = f"{NS}/take_sample"
S_CURRENT = f"{NS}/get_current_transforms"
S_LIST = f"{NS}/get_sample_list"
S_COMPUTE = f"{NS}/compute_calibration"
S_SAVE = f"{NS}/save_calibration"
S_ALGO = f"{NS}/set_algorithm"


def quat_to_R(x, y, z, w) -> np.ndarray:
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def tf_vec(t) -> np.ndarray:
    return np.array([t.translation.x, t.translation.y, t.translation.z])


def tf_quat(t):
    return (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)


def tf_rot(t) -> np.ndarray:
    return quat_to_R(*tf_quat(t))


def rot_angle_deg(Ra: np.ndarray, Rb: np.ndarray) -> float:
    c = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


class Sampler(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("handeye_auto")
        self.args = args
        self.cli = {
            "take": self.create_client(TakeSample, S_TAKE),
            "current": self.create_client(TakeSample, S_CURRENT),
            "list": self.create_client(TakeSample, S_LIST),
            "compute": self.create_client(ComputeCalibration, S_COMPUTE),
            "save": self.create_client(SaveCalibration, S_SAVE),
            "algo": self.create_client(SetAlgorithm, S_ALGO),
        }
        self.accepted: list[dict] = []

    def _call(self, key, req, timeout=5.0):
        cli = self.cli[key]
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        fut = cli.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.02)
        return fut.result() if fut.done() else None

    def wait_for_services(self, timeout=180.0) -> bool:
        """等全部服务出现。

        ⚠️ 服务端是**两条 TF 都能查到之后才创建服务**的（`setup_services_and_topics`
        每 2 s 重试）。所以「服务一直没出现」几乎总是意味着**某一条 TF 还没活**，
        而不是这个脚本坏了。
        """
        t0 = time.time()
        last = 0.0
        while time.time() - t0 < timeout:
            missing = [k for k, c in self.cli.items() if not c.service_is_ready()]
            if not missing:
                return True
            if time.time() - last > 10:
                last = time.time()
                print(f"    等 easy_handeye2 服务… 还缺 {missing}"
                      f"（多半是 base_link→tool0 或 相机→靶板 还没活）", flush=True)
            time.sleep(0.3)
        return False

    def current(self) -> dict | None:
        res = self._call("current", TakeSample.Request())
        if res is None or len(res.samples.samples) == 0:
            return None
        s = res.samples.samples[0]
        return {"robot": s.robot, "tracking": s.tracking}

    def take(self) -> bool:
        res = self._call("take", TakeSample.Request())
        return res is not None and len(res.samples.samples) > 0


def novelty(sample: dict, accepted: list[dict], min_mm: float, min_deg: float):
    """返回 (够不够新, 最近的那个样本离多远/多少度)。"""
    if not accepted:
        return True, (float("inf"), float("inf"))
    tr, Rr = tf_vec(sample["robot"]), tf_rot(sample["robot"])
    best = (float("inf"), float("inf"))
    ok = False
    for a in accepted:
        d = float(np.linalg.norm(tr - a["tr"]) * 1000.0)
        ang = rot_angle_deg(Rr, a["Rr"])
        if d < best[0]:
            best = (d, ang)
        if d > min_mm or ang > min_deg:
            ok = True
    return ok, best


def report_diversity(accepted: list[dict]) -> None:
    """样本的姿态够不够散 —— 手眼标定最怕的就是"看着采了 20 个点、其实只有 2 个方向"。

    判据：把每个样本相对第一个的**旋转轴**拿出来看，轴若都挤在一两条线上，
    方程组的秩就不够，解出来的东西会**看起来收敛**但其实是错的。
    """
    if len(accepted) < 3:
        print("    （样本还太少，谈不了散布）")
        return
    R0 = accepted[0]["Rr"]
    axes = []
    for a in accepted[1:]:
        dR = R0.T @ a["Rr"]
        c = max(-1.0, min(1.0, (np.trace(dR) - 1) / 2))
        ang = math.degrees(math.acos(c))
        if ang < 1e-3:
            continue
        w = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]])
        n = np.linalg.norm(w)
        if n > 1e-9:
            axes.append(w / n)
    if len(axes) < 2:
        print("    ⚠️ 相对旋转只有一个方向 —— 手眼方程组秩不够，再换几个方向转")
        return
    A = np.array(axes)
    sv = np.linalg.svd(A, compute_uv=False)
    print(f"    转轴散布：{len(axes)} 个方向，奇异值 {np.round(sv, 3)}")
    if sv[-1] < 0.15:
        print("    ⚠️ 最小的那个奇异值很小 ⇒ 转轴几乎共面/共线，解会不稳，建议再绕别的轴转几个姿态")
    else:
        print("    ✅ 转轴铺开得够（最小奇异值 %.3f）" % sv[-1])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", type=int, default=20, help="收多少个样本就停")
    ap.add_argument("--min-mm", type=float, default=25.0, help="跟已有样本平移差超过这么多毫米才算新姿态")
    ap.add_argument("--min-deg", type=float, default=15.0, help="或转动超过这么多度也算新姿态")
    ap.add_argument("--max-minutes", type=float, default=25.0, help="超时就收摊（0=不限）")
    ap.add_argument("--poll-hz", type=float, default=2.0)
    ap.add_argument("--algorithms", default="Tsai-Lenz,Park,Horaud,Andreff,Daniilidis",
                    help="逗号分隔；五种都解一遍互相印证")
    ap.add_argument("--save", action="store_true", help="解完把标定存盘")
    ap.add_argument("--save-algorithm", default="Tsai-Lenz",
                    help="存盘用哪一种（算法名不会写进文件，所以要显式指定并记下来）")
    args = ap.parse_args()

    rclpy.init()
    node = Sampler(args)
    t_start = time.time()
    try:
        print("等 easy_handeye2 服务…", flush=True)
        if not node.wait_for_services():
            print("❌ 服务一直没出现。先确认 ${S_TAKE} 在不在：\n"
                  "   ros2 service list | grep easy_handeye2\n"
                  "   再确认两条 TF 都查得到：\n"
                  "   ros2 run tf2_ros tf2_echo base_link tool0\n"
                  "   ros2 run tf2_ros tf2_echo camera_color_optical_frame charuco_board", flush=True)
            return 2
        print("✅ 服务都在。开始盯姿态 —— **请开始用手挪臂**（每个姿态停 1~2 秒）。\n", flush=True)

        last_status = 0.0
        # 存活检查用的两个量。**为什么需要**：驱动的 read() 在 RTDE 掉线时仍然
        # `return OK`（`hardware_interface.cpp` 里那一段），于是「关节读数冻住」和
        # 「人没挪臂」在屏幕上是**同一个样子** —— 都是「臂没怎么动」。
        # 不分开报，就会闷头摆 20 个位姿、摆完才发现全白摆。
        last_pose_vec = None
        pose_unchanged_since = None
        warned_frozen = False
        while rclpy.ok() and len(node.accepted) < args.target:
            cur = node.current()
            if cur is None:
                if time.time() - last_status > 5:
                    last_status = time.time()
                    print("    …这一拍没拿到完整位姿（多半是**靶板不在画面里**），"
                          "把臂转回去让板进画面", flush=True)
                time.sleep(1.0 / args.poll_hz)
                continue

            cur_vec = tf_vec(cur["robot"])
            if last_pose_vec is None or float(np.linalg.norm(cur_vec - last_pose_vec)) > 0.0005:
                last_pose_vec = cur_vec
                pose_unchanged_since = time.time()
                warned_frozen = False
            still_s = time.time() - (pose_unchanged_since or time.time())

            ok, (d_mm, ang) = novelty(cur, node.accepted, args.min_mm, args.min_deg)
            if ok:
                if node.take():
                    node.accepted.append({
                        "tr": tf_vec(cur["robot"]),
                        "Rr": tf_rot(cur["robot"]),
                        "t_board": tf_vec(cur["tracking"]),
                    })
                    z = float(np.linalg.norm(tf_vec(cur["tracking"])) * 1000)
                    print(f"  ✅ 第 {len(node.accepted):2d} 个样本："
                          f"法兰 {np.round(tf_vec(cur['robot']) * 1000, 0)} mm，"
                          f"板心距相机 {z:.0f} mm，"
                          f"距上一个样本 {d_mm:.0f} mm / {ang:.1f}°", flush=True)
                    if len(node.accepted) % 5 == 0:
                        report_diversity(node.accepted)
                else:
                    print("    （服务端这一拍没取到 —— TF 在 0.2 s 前那个时刻缺了一角，跳过）", flush=True)
            else:
                if time.time() - last_status > 4:
                    last_status = time.time()
                    print(f"    …臂没怎么动（离最近样本 {d_mm:.0f} mm / {ang:.1f}°）"
                          f"—— 已经 {len(node.accepted)}/{args.target} 个"
                          f"｜当前法兰 {np.round(cur_vec * 1000, 0)} mm"
                          f"，读数 {still_s:.0f}s 没变", flush=True)
                    # 这一句是给「读数冻住」和「人真没动」分家的。
                    # 人正在拖臂却一直没变 ⇒ 是冻住，不是没动。
                    if still_s > 20 and not warned_frozen:
                        warned_frozen = True
                        print("    ⚠️ **角位移读数已经 20 秒没变了。**", flush=True)
                        print("       如果你**正在用手拖臂**，那这不是「你没动」，而是**读数冻住了** ——",
                              flush=True)
                        print("       驱动的 read() 在 RTDE 掉线时仍会 return OK。"
                              "先停下，别闷头摆 20 个位姿。", flush=True)
                        print("       查法：板上另开一个终端 `ros2 topic hz /joint_states`，"
                              "看它还在不在发。", flush=True)
            if args.max_minutes and (time.time() - t_start) > args.max_minutes * 60:
                print(f"\n⏰ 到 {args.max_minutes} 分钟了，收摊。", flush=True)
                break

        print(f"\n收尾：共 {len(node.accepted)} 个样本", flush=True)
        report_diversity(node.accepted)
        if len(node.accepted) < 3:
            print("❌ 样本太少，解不了。", flush=True)
            return 3

        print("\n=== 五种算法各解一遍，看它们互相之间散不散 ===", flush=True)
        results = {}
        for algo in args.algorithms.split(","):
            algo = algo.strip()
            if not algo:
                continue
            # ⚠️ **必须先 set_algorithm**。不切算法的话五次 `compute_calibration`
            #    解的都是服务端当前那一个算法，会印出五个一模一样的数 —— 然后看起来
            #    "五种算法完全一致"，其实只是一种算法说了五遍。
            sa = node._call("algo", SetAlgorithm.Request(new_algorithm=f"OpenCV/{algo}"))
            if sa is None or not sa.success:
                print(f"  {algo:<12} ❌ 切不过去（服务端认不认这个名字？）", flush=True)
                continue
            r = node._call("compute", ComputeCalibration.Request())
            if r is None or not r.valid:
                print(f"  {algo:<12} ❌ 求解失败", flush=True)
                continue
            t = r.calibration.transform
            v = tf_vec(t)
            results[algo] = v
            print(f"  {algo:<12} t = [{v[0] * 1000:+8.2f}, {v[1] * 1000:+8.2f}, {v[2] * 1000:+8.2f}] mm"
                  f"   |t| = {np.linalg.norm(v) * 1000:7.2f} mm", flush=True)

        if len(results) >= 2:
            V = np.array(list(results.values()))
            spread = np.linalg.norm(V - V.mean(axis=0), axis=1) * 1000
            print(f"\n  五种算法之间的散布：最大 {spread.max():.2f} mm"
                  f"（占 |t| 的 {100 * spread.max() / (np.linalg.norm(V.mean(axis=0)) * 1000):.2f}%）")
            if spread.max() > 3.0:
                print("  ⚠️ 散布偏大 ⇒ 样本信息量不够。**别挑好看的挑**，"
                      "回去再绕几个不同方向转几次，重采。")
            else:
                print("  ✅ 算法之间一致 —— 这一份标定可信。")

        if args.save:
            # ⚠️ `save_calibration` 存的是服务端 `last_calibration`，也就是**最后一次
            #    compute 的那一个**；而算法名**不进存盘文件**（HandeyeCalibration 里
            #    只有 parameters + transform）。所以上面循环跑完五种之后直接存，
            #    落盘的是最后一种（Daniilidis），而文件里**看不出**是哪种 ——
            #    回头没人能从文件本身发现这件事。显式切回来再算一次。
            want = args.save_algorithm
            sa = node._call("algo", SetAlgorithm.Request(new_algorithm=f"OpenCV/{want}"))
            if sa is None or not sa.success:
                print(f"\n❌ 想把算法切回 {want} 再存盘，切不过去 —— **不存**（免得存下一个来路不明的）。")
                return 4
            r_cal = node._call("compute", ComputeCalibration.Request())
            if r_cal is None or not r_cal.valid:
                print(f"\n❌ 切回 {want} 之后重新求解失败 —— **不存**。")
                return 5
            r = node._call("save", SaveCalibration.Request())
            ok = r is not None and r.success
            if ok:
                t = tf_vec(r_cal.calibration.transform) if r_cal is not None else None
                print(f"\n存盘：✅ {r.filepath.data}")
                print(f"      用的算法 = **{want}**（文件名里看不出来，记在这里）")
                if t is not None:
                    print(f"      t = {np.round(t * 1000, 2)} mm  |t| = {np.linalg.norm(t) * 1000:.2f} mm")
                if len(results) >= 2:
                    print(f"      对照：五种算法里最远的两个差 "
                          f"{np.linalg.norm(V.max(axis=0) - V.min(axis=0)) * 1000:.2f} mm")
            else:
                print("\n存盘：❌ 失败")
    except (KeyboardInterrupt, ExternalShutdownException):
        print("\n（被打断）", flush=True)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())

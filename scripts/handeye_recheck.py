#!/usr/bin/env python3
"""复核：**存盘的这份手眼标定，现在还算不算数**。

半个月/半年之后回头看「还准不准」，不能靠「文件还在」，也不能靠「时间久了不会变」
—— 要拿**新的**位姿去考它。判据还是那条最强的：

    靶板是钉住/摆住不动的 ⇒ 用存盘的 X 反推「基座→靶板」，**每个位姿都必须落在同一处**。

这条判据不要求知道靶板摆在哪，只要求**复核期间靶板没被挪**。所以靶板随便放，
只要在相机视野里、复核全程不碰它。

⚠️ **只查不存**：全程用 `get_current_transforms`（服务端只查、不写样本表），
   **不调** `take_sample`。所以不会污染已有的样本表，也不用事后清理。

⚠️ **位姿必须够散**：一个位姿必然「通过」（它自己跟自己比）。所以这里要求每个新位姿
   跟已有位姿差 >min-mm **或** >min-deg，否则不收 —— 否则这个复核是**空的**。

⚠️ 跑之前需要：驱动在跑（`base_link→tool0`）、腕部相机在发图、桥在发靶板 TF。
   还得先起 `handeye_publisher`（`publish_on.sh`），否则两棵树是断开的。

用法（板上）：
    ~/handeye_ws/scripts/run_ros.sh scripts/handeye_recheck.py --poses 5
"""
import argparse
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from easy_handeye2_msgs.srv import TakeSample
from easy_handeye2.handeye_calibration import load_calibration

import handeye_auto as ha

NS = "/easy_handeye2/calibration/"


def is_new(sample: dict, accepted: list[dict], min_mm: float, min_deg: float):
    """跟**最近**那个已有位姿差够远就算新 —— 只去重，不逼朝向。

    ⚠️ 这里**故意不用** `handeye_auto.novelty()`：那条还带一个「跟**每一个**已有
    样本都差 >min_deg」的全局闸门。采点时要它是为了逼出真的换朝向；复核时它只会
    让人卡住（半个月前实测卡在 6/20）。复核要的只是「这些位姿不是同一个」，
    否则判据是空的。
    """
    if not accepted:
        return True, (float("inf"), float("inf"))
    tr, Rr = ha.tf_vec(sample["robot"]), ha.tf_rot(sample["robot"])
    best = (float("inf"), float("inf"))
    for a in accepted:
        d = float(np.linalg.norm(tr - a["tr"]) * 1000.0)
        ang = ha.rot_angle_deg(Rr, a["Rr"])
        if d < best[0]:
            best = (d, ang)
    return (best[0] > min_mm) or (best[1] > min_deg), best


class Recheck(Node):
    def __init__(self) -> None:
        super().__init__("handeye_recheck")
        self.cli = self.create_client(TakeSample, NS + "get_current_transforms")
        self.call_error = ""

    def current(self):
        if not self.cli.wait_for_service(timeout_sec=10.0):
            self.call_error = "get_current_transforms 服务不在"
            return None
        fut = self.cli.call_async(TakeSample.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done():
            self.call_error = "服务 5 秒没回"
            return None
        res = fut.result()
        if res is None or len(res.samples.samples) == 0:
            self.call_error = "服务端说这一拍没有数据（靶板不在画面里 / TF 查不到）"
            return None
        self.call_error = ""
        s = res.samples.samples[0]
        return {"robot": s.robot, "tracking": s.tracking}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--name", default="wrist_eih", help="存盘的标定名")
    ap.add_argument("--poses", type=int, default=5, help="复核用几个位姿（≥3 才有意义）")
    ap.add_argument("--min-mm", type=float, default=25.0)
    ap.add_argument("--min-deg", type=float, default=15.0)
    ap.add_argument("--still", type=float, default=0.7, help="每个位姿停稳这么多秒再取")
    ap.add_argument("--gate-mm", type=float, default=5.0, help="靶板落点散开的闸门")
    ap.add_argument("--max-minutes", type=float, default=15.0)
    args = ap.parse_args()

    cal = load_calibration(args.name)
    t0 = cal.transform.translation
    q0 = cal.transform.rotation
    X = ha.T4(ha.tf_vec(t0), ha.tf_rot(cal.transform))
    print(f"存盘的标定：{args.name}")
    print(f"  t = [{t0.x*1000:+.2f}, {t0.y*1000:+.2f}, {t0.z*1000:+.2f}] mm，"
          f"|t| = {np.linalg.norm([t0.x, t0.y, t0.z])*1000:.2f} mm")
    print(f"  q = [{q0.x:+.6f}, {q0.y:+.6f}, {q0.z:+.6f}, {q0.w:+.6f}]\n")

    if args.poses < 3:
        print("❌ 少于 3 个位姿，这个复核证明不了什么")
        return 2

    rclpy.init()
    node = Recheck()
    accepted: list[dict] = []
    last_pose = None
    still_since = time.time()
    t_start = time.time()
    last_status = 0.0

    print(f"请把靶板放进相机视野，然后**挪臂到 {args.poses} 个明显不同的位姿**，")
    print("每个位姿停 1~2 秒。全程**不要碰靶板**。\n")

    while rclpy.ok() and len(accepted) < args.poses:
        if args.max_minutes and time.time() - t_start > args.max_minutes * 60:
            print(f"\n⏰ 到 {args.max_minutes} 分钟了，收摊。")
            break
        cur = node.current()
        if cur is None:
            if time.time() - last_status > 5:
                last_status = time.time()
                print(f"    …拿不到位姿：{node.call_error}", flush=True)
            time.sleep(0.5)
            continue

        rv = ha.tf_vec(cur["robot"])
        if last_pose is None or float(np.linalg.norm(rv - last_pose)) > 0.0005:
            last_pose = rv
            still_since = time.time()
        still = time.time() - still_since

        ok, (d, a) = is_new(cur, accepted, args.min_mm, args.min_deg)
        if ok and still < args.still:
            if time.time() - last_status > 2:
                last_status = time.time()
                print(f"    …位姿够新了，等停稳（{still:.1f}s / {args.still:.1f}s）"
                      f"｜已经 {len(accepted)}/{args.poses}", flush=True)
        elif ok:
            accepted.append({"tr": ha.tf_vec(cur["robot"]), "Rr": ha.tf_rot(cur["robot"]),
                             "t_board": ha.tf_vec(cur["tracking"]),
                             "Rb": ha.tf_rot(cur["tracking"])})
            print(f"  ✅ 第 {len(accepted)}/{args.poses} 个位姿："
                  f"法兰 {np.round(ha.tf_vec(cur['robot'])*1000, 0)} mm，"
                  f"距最近的已有位姿 {d:.0f} mm / {a:.1f}°，停稳 {still:.1f}s", flush=True)
        else:
            if time.time() - last_status > 4:
                last_status = time.time()
                print(f"    …没收：跟最近的已有位姿几乎重合（{d:.0f} mm / {a:.1f}°）"
                      f" —— 请换个**明显不同**的位姿"
                      f"｜已经 {len(accepted)}/{args.poses}", flush=True)
        time.sleep(0.5)

    if len(accepted) < 3:
        print(f"\n❌ 只拿到 {len(accepted)} 个位姿，没法复核")
        node.destroy_node()
        rclpy.shutdown()
        return 3

    print(f"\n用**存盘的 X** 反推「基座→靶板」，看 {len(accepted)} 个位姿落点散不散：")
    dmm, adeg = ha.board_fixed_spread(accepted, X)
    print(f"  散开：{dmm:.2f} mm / {adeg:.2f}°（闸门 {args.gate_mm} mm）")
    ok = dmm <= args.gate_mm
    if ok:
        print("\n✅ 标定仍然成立 —— 相机与法兰的刚性关系没变。")
    else:
        print("\n❌ 不成立 —— 这半个月里相机/支架/法兰很可能被**物理动过**。")
        print("   ⇒ 别用这份外参。重新标定（见 docs/待执行命令.md §1-8）。")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 4


if __name__ == "__main__":
    sys.exit(main())

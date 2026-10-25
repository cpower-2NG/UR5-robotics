#!/usr/bin/env python3
"""一眼抓帧（只订阅，不发任何消息，不动臂）—— 精密抓取一轮的第 1 段。

为什么单独一个脚本而不是让开发机直接订阅话题
--------------------------------------------
开发机上 **跨进程 DDS 发现是坏的**（同进程 29633 条 / 跨进程 0 条），
所以开发机当不了 ROS 节点；板上也没有 GraspNet 的推理环境（无 torch/ME/权重/外网）。
两边的接缝只能是**文件**：板上抓帧 → SSH 取回 → 开发机算 → 计划送回来执行。

与 capture_live.py 的差别（**都是为了不再静默**）
------------------------------------------------
1. `/tf_static` 上有**多个**发布者，各发自己那一组。整条消息覆盖式保存只会剩下
   最后一组，于是 `base_link:tool0`、`tool0:camera_link` 会丢。这里按键累积。
   ⚠️ 12 键的链**跨两个话题**：六个转动的关节在 `/tf`（动态），`flange:tool0`、
   `tool0:camera_link` 与相机内的固定帧在 `/tf_static`。只查 `/tf_static`
   永远缺那六个 —— 2026-10-25 第一次上板就这么栽了一次。
2. 12 键的链**必须到齐**才算抓帧成功。capture_live.py 超时后仍会写出半条链的帧，
   下游只能报一个看不懂的 KeyError；这里直接非 0 退出并说清缺哪一帧。
3. 写出的 `meta.json` 里带图像的时间戳与 frame_id —— 帧与 TF 必须是同一时刻的量。
4. 关节角存一小段历史，写盘时挑**与图像时间戳最近**的那一条。臂在动时图像与
   `/joint_states` 各有几十到上百毫秒延迟（实测差过 140.5 ms），直接拿「最新的」
   会把这段时差算成手臂链的位姿误差。残差写进 `joint_states.json` 的 `gap_ms`。

用法：
  python3 precision_capture.py /tmp/frame-01
  python3 precision_capture.py /tmp/frame-01 --timeout 40

退出码：0 成功；2 超时/缺件（帧目录里不会有半成品）；3 图像编码不认识。
"""
import argparse
import collections
import json
import os
import sys
import time

sys.dont_write_bytecode = True

# base_link → 相机光心的完整链。缺任意一帧，base 系坐标就是错的（或者根本算不出来）。
CHAIN_KEYS = (
    "base_link:base_link_inertia", "base_link_inertia:shoulder_link",
    "shoulder_link:upper_arm_link", "upper_arm_link:forearm_link",
    "forearm_link:wrist_1_link", "wrist_1_link:wrist_2_link",
    "wrist_2_link:wrist_3_link", "wrist_3_link:flange", "flange:tool0",
    "tool0:camera_link", "camera_link:camera_color_frame",
    "camera_color_frame:camera_color_optical_frame",
)
TIP = "camera_color_optical_frame"

TOPICS = {
    "color": ("/camera/color/image_raw", "Image"),
    "depth": ("/camera/aligned_depth_to_color/image_raw", "Image"),
    "info": ("/camera/color/camera_info", "CameraInfo"),
    "js": ("/joint_states", "JointState"),
}


def stamp_ns(m):
    return int(m.header.stamp.sec) * 10 ** 9 + int(m.header.stamp.nanosec)


def joints_at(hist, stamp):
    """把关节角**插值到图像的时间戳**上，而不是取「最近的一条」。

    返回 (names, positions, 残差ms, 是否真插值)。
    图像与 /joint_states 都是周期量、关节角是光滑的，两点线性插值的误差在
    微弧度级；而直接取最近的一条，残差最多到半个周期，且**方向未知**。
    """
    rows = sorted((stamp_ns(m), m) for m in hist)
    if not rows:
        return None
    before = [r for r in rows if r[0] <= stamp]
    after = [r for r in rows if r[0] >= stamp]
    if before and after:
        (ta, a), (tb, b) = before[-1], after[0]
        names = list(a.name)
        if tb == ta:
            return names, [float(v) for v in a.position], 0.0, True
        w = (stamp - ta) / (tb - ta)
        pa = dict(zip(a.name, a.position))
        pb = dict(zip(b.name, b.position))
        pos = [pa[k] + w * (pb[k] - pa[k]) for k in names]
        return names, pos, min(stamp - ta, tb - stamp) / 1e6, True
    t, m = min(rows, key=lambda r: abs(r[0] - stamp))
    return list(m.name), [float(v) for v in m.position], abs(t - stamp) / 1e6, False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out", help="帧目录（会新建）")
    ap.add_argument("--timeout", type=float, default=30.0,
                    help="等话题的上限（秒）。到点还缺件就失败，不写半成品")
    ap.add_argument("--joints-only", action="store_true",
                    help="只取 /joint_states（发轨迹前算计划用），不碰相机与 TF")
    args = ap.parse_args()

    # 先确认文件系统能写，再花 30 s 去等话题 —— 免得等完才发现目录不可写。
    os.makedirs(args.out, exist_ok=True)
    probe = os.path.join(args.out, ".write_probe")
    with open(probe, "w") as f:
        f.write("ok")
    os.remove(probe)

    import numpy as np
    from PIL import Image
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image as RosImage, CameraInfo, JointState
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    node = Node("precision_capture")
    got = {}
    tf_acc = {"tf": {}, "tf_static": {}}
    chain = {}            # 12 键链，跨 /tf 与 /tf_static 合并
    chain_origin = {}
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    qos_static = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def keep(key):
        return lambda m: got.__setitem__(key, m)

    # 关节角留一小段历史：**臂在动时**图像与关节角各自有几十到上百毫秒的延迟，
    # 拿「保存那一刻最新的」关节角去配一张旧图，会把延迟直接算成手臂链的位姿误差
    # （实测两者差过 140 ms）。存历史、再挑**与图像时间戳最近**的那一条。
    js_hist = collections.deque(maxlen=200)

    def on_js(m):
        got["js"] = m
        js_hist.append(m)

    if not args.joints_only:
        node.create_subscription(RosImage, TOPICS["color"][0], keep("color"), qos)
        node.create_subscription(RosImage, TOPICS["depth"][0], keep("depth"), qos)
        node.create_subscription(CameraInfo, TOPICS["info"][0], keep("info"), qos)
    node.create_subscription(JointState, TOPICS["js"][0], on_js, qos)

    def acc(slot):
        def cb(m):
            d = tf_acc[slot]
            for t in m.transforms:
                key = "%s:%s" % (t.header.frame_id, t.child_frame_id)
                d[key] = t
                # 12 键的链**跨两个话题**：六个转动的关节在 /tf（动态），
                # flange/tool0、tool0:camera_link、相机内的固定帧在 /tf_static。
                # 只查 /tf_static 会永远缺那六个 —— 实测第一次跑就栽在这上面。
                chain[key] = t
                chain_origin[key] = slot
            got[slot] = d
        return cb

    if not args.joints_only:
        node.create_subscription(TFMessage, "/tf", acc("tf"), qos)
        node.create_subscription(TFMessage, "/tf_static", acc("tf_static"), qos_static)

    t0 = time.time()
    while time.time() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        if args.joints_only:
            if "js" in got:
                break
            continue
        have_msgs = all(k in got for k in ("color", "depth", "info", "js",
                                           "tf", "tf_static"))
        have_chain = all(k in chain for k in CHAIN_KEYS)
        if have_msgs and have_chain:
            # 收到还不够：要等 /joint_states 的历史把**图像时间戳夹住**，
            # 否则插值退化成「取最近一条」，残差就是那个未知的时差。
            # 相机 7.3 Hz、关节 100 Hz，通常 20 ms 内就夹住了。
            t1 = time.time()
            while time.time() - t1 < 2.0:
                lo = min(stamp_ns(m) for m in js_hist)
                hi = max(stamp_ns(m) for m in js_hist)
                if lo <= stamp_ns(got["color"]) <= hi:
                    break
                rclpy.spin_once(node, timeout_sec=0.05)
            break

    if args.joints_only:
        if "js" not in got:
            print("!! %.0f s 没收到 /joint_states —— 驱动起了吗？" % args.timeout,
                  file=sys.stderr)
            node.destroy_node()
            rclpy.shutdown()
            return 2
        js = got["js"]
        json.dump({"name": list(js.name),
                   "position": [float(v) for v in js.position],
                   "stamp_ns": stamp_ns(js)},
                  open(os.path.join(args.out, "joint_states.json"), "w"), indent=2)
        print("OK %s （只取关节角）" % args.out)
        print("  关节(deg)：%s" % {k: round(v * 57.2957795, 1)
                                  for k, v in zip(js.name, js.position)})
        node.destroy_node()
        rclpy.shutdown()
        return 0

    miss = [k for k in ("color", "depth", "info", "js", "tf", "tf_static")
            if k not in got]
    miss_chain = [k for k in CHAIN_KEYS if k not in chain]
    if miss or miss_chain:
        print("!! 抓帧失败：话题缺 %s；TF 链缺 %s" % (miss or "无", miss_chain or "无"),
              file=sys.stderr)
        if miss_chain:
            print("   ↑ 缺 TF 帧多半是 wrist_tf_publish.py 没起，或者 /tf_static 的"
                  "那一路发布者还没发 —— 臂这边不要动，重跑一次即可。", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 2

    def to_array(m):
        buf = np.frombuffer(m.data, dtype=np.uint8)
        if m.encoding in ("rgb8", "bgr8"):
            a = buf.reshape(m.height, m.step // 3, 3)[:, :m.width]
            return a[:, :, ::-1] if m.encoding == "bgr8" else a
        if m.encoding == "16UC1":
            return buf.view(np.uint16).reshape(m.height, m.step // 2)[:, :m.width]
        raise ValueError("未处理的 encoding %r" % m.encoding)

    try:
        color = to_array(got["color"])
        depth = to_array(got["depth"])
    except ValueError as e:
        print("!! %s" % e, file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 3

    Image.fromarray(color).save(os.path.join(args.out, "color.png"))
    Image.fromarray(depth).save(os.path.join(args.out, "depth.png"))

    info = got["info"]

    meta = {
        "camera_info": {
            "width": info.width, "height": info.height,
            "distortion_model": info.distortion_model,
            "D": list(info.d), "K": list(info.k),
            "R": list(info.r), "P": list(info.p),
            "frame_id": info.header.frame_id,
        },
        # depth 与 color 已对齐（话题名里就写着），所以共用一份 K。
        "color": {"width": got["color"].width, "height": got["color"].height,
                  "encoding": got["color"].encoding,
                  "is_bigendian": bool(got["color"].is_bigendian),
                  "step": got["color"].step,
                  "frame_id": got["color"].header.frame_id,
                  "stamp_ns": stamp_ns(got["color"])},
        "depth": {"width": got["depth"].width, "height": got["depth"].height,
                  "encoding": got["depth"].encoding,
                  "is_bigendian": bool(got["depth"].is_bigendian),
                  "step": got["depth"].step,
                  "frame_id": got["depth"].header.frame_id,
                  "stamp_ns": stamp_ns(got["depth"])},
        "chain_tip": TIP,
        "chain_keys": list(CHAIN_KEYS),
    }
    json.dump(meta, open(os.path.join(args.out, "meta.json"), "w"), indent=2)

    trans = {}
    for key, t in chain.items():
        trans[key] = {
            "translation": {"x": t.transform.translation.x,
                            "y": t.transform.translation.y,
                            "z": t.transform.translation.z},
            "rotation": {"x": t.transform.rotation.x,
                         "y": t.transform.rotation.y,
                         "z": t.transform.rotation.z,
                         "w": t.transform.rotation.w},
            "stamp_ns": stamp_ns(t),
            "frame_id": t.header.frame_id, "child_frame_id": t.child_frame_id,
        }
    json.dump({"ok": trans, "missing": [],
               "origin": {k: chain_origin.get(k) for k in trans}},
              open(os.path.join(args.out, "tf.json"), "w"), indent=2)

    # 关节角插值到**图像时间戳**上（见 joints_at 的说明）。残差 = 离最近的
    # 插值端点有多远；没夹住（图像早于/晚于整段历史）时退化成最近一条，并标 False。
    color_ns = stamp_ns(got["color"])
    names, pos, resid_ms, interp = joints_at(js_hist, color_ns)
    # **真正决定这帧能不能用**的不是上面那个残差，而是臂在窗口里有没有动：
    # 不动的话，图像/关节角/TF 谁早谁晚都无所谓。所以量 ±0.3 s 窗口内的
    # 关节峰峰值（弧度）与相邻样本最大间隔（判断上面那个残差是真时差还是采样空洞）。
    win = [m for m in js_hist if abs(stamp_ns(m) - color_ns) <= 3 * 10 ** 8]
    per = {}
    for m in win:
        for k, v in zip(m.name, m.position):
            per.setdefault(k, []).append(v)
    p2p = max((max(v) - min(v) for v in per.values()), default=None)
    st = sorted(stamp_ns(m) for m in win)
    gaps = [(b - a) / 1e6 for a, b in zip(st, st[1:])]
    max_gap = max(gaps) if gaps else None
    # 动态 TF（来自 /tf 的那几帧，实际就是六个转动关节）的时间戳与图像的差 ——
    # 相机位姿就是从这条链算的。/tf_static 的时间戳常是 0，不能一起比。
    dyn = [(k, t) for k, t in chain.items()
           if chain_origin.get(k) == "tf" and stamp_ns(t) > 0]
    tf_gaps = sorted((stamp_ns(t) - color_ns) / 1e6 for _, t in dyn
                     if stamp_ns(t) > 0)
    still = p2p is not None and p2p < 0.005      # ≈0.29°，对应指尖 <3 mm
    json.dump({"name": names,
               "position": pos,
               "stamp_ns": color_ns,
               "color_stamp_ns": color_ns,
               "gap_ms": resid_ms,
               "interpolated": interp,
               "samples": len(js_hist),
               "window_samples": len(win),
               "max_sample_gap_ms": max_gap,
               "joint_p2p_rad": p2p,
               "joint_p2p_deg": p2p * 57.2957795 if p2p is not None else None,
               "still": still,
               "tf_gap_ms": {"min": tf_gaps[0], "median": tf_gaps[len(tf_gaps) // 2],
                             "max": tf_gaps[-1]} if tf_gaps else None},
              open(os.path.join(args.out, "joint_states.json"), "w"), indent=2)

    print("OK %s" % args.out)
    print("  图像 %dx%d  深度 %dx%d  关节 %d 个  TF %d 帧"
          % (got["color"].width, got["color"].height,
             got["depth"].width, got["depth"].height,
             len(names), len(trans)))
    print("  关节角%s到图像时间戳：残差 %.1f ms（历史 %d 条，±0.3 s 内 %d 条，"
          "相邻最大间隔 %s ms）"
          % ("插值" if interp else "**没夹住，取最近一条**", resid_ms, len(js_hist),
             len(win), "%.1f" % max_gap if max_gap is not None else "—"))
    if p2p is not None:
        print("  抓帧窗口内关节峰峰 %.4f rad = %.3f° ⇒ %s"
              % (p2p, p2p * 57.2957795,
                 "臂静止，时间配准不成问题" if still
                 else "**臂在动**，这帧的位姿受时间配准误差影响"))
    if tf_gaps:
        print("  TF(动态)↔图像 时间差 min %+.0f / 中位 %+.0f / max %+.0f ms"
              % (tf_gaps[0], tf_gaps[len(tf_gaps) // 2], tf_gaps[-1]))
    print("  关节(deg)：%s" % {k: round(v * 57.2957795, 1)
                              for k, v in zip(names, pos)})

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

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
4. 关节角存一小段历史，写盘时**插值到图像时间戳**上。臂在动时图像与
   `/joint_states` 各有几十到上百毫秒延迟（实测差过 140.5 ms），直接拿「最新的」
   会把这段时差算成手臂链的位姿误差。残差写进 `joint_states.json` 的 `gap_ms`。
5. **常驻自旋 + 全链按时间戳配对**（2026-11-12 第 321 轮实测后改）。旧实现在写盘
   （每帧几百毫秒）期间完全不 spin，`wait_one()` 又靠旧状态立刻返回 ⇒ 订阅队列
   一次都没排空过：同一组 burst 里 TF↔图像时间差从 +42 ms 线性恶化到 −1793 ms，
   彩色与深度也常常差**一整帧**（0 / ±33.36 ms 各半）。现在：
     · 自旋在独立线程里常驻，回调只做 append（图像 QoS depth=1，只要最新）；
     · 每帧取「比上一帧新」的彩色，深度/内参/动态 TF 全部按**时间戳挑最近的**，
       差量写进 `meta_*.json`；配不上（> 40 ms 深度、> 250 ms TF）或动态 TF 流
       断了（最近的 `/tf` 已 > 500 ms 前）⇒ 这一帧判失败，不写半成品。
   `tf_*.json` 的 `origin` 字段仍标明每帧来自 `/tf` 还是 `/tf_static`。

用法：
  python3 precision_capture.py /tmp/frame-01
  python3 precision_capture.py /tmp/frame-01 --timeout 40
  python3 precision_capture.py /tmp/frame-01 --camera topcam
  python3 precision_capture.py /tmp/burst --camera wrist --burst 20

`--camera topcam` 抓的是**俯视第二路**（阶段 4）：话题 `/topcam/...`、链是纯静态的
`base_link→topcam_link→topcam_color_frame→topcam_color_optical_frame`、不要求
腕部那条链、也不要求 `/joint_states`。`/tf_static` 仍然全部 dump 下来（那条链在它上面）。

`--burst N` 连续抓 N 帧：第 i 帧写 `color_000.png / depth_000.png / meta_000.json /
tf_000.json`（**不是**覆盖同名文件），E4a 的 20 帧可见性判据就靠它。每次都会等
**一条新的**彩色帧，绝不把同一帧写 20 遍 —— 那样「20 帧」就是假的。

退出码：0 成功；2 超时/缺件（帧目录里不会有半成品）；3 图像编码不认识。
"""
import argparse
import collections
import json
import os
import sys
import threading
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

# ── 俯视第二路（阶段 4，E2）────────────────────────────────────────────
# 与腕部那条链的**根本差别**：俯视相机不挂在臂上，所以它的链是**纯静态**的，
# 不经过 tool0、也不经过任何转动的关节。
# ⚠️ 这里只列**驱动自己发**的那两段。`base_link → topcam_link` **不进就绪门**：
#    它只在标定出外参之后才存在（launch 里 `topcam_publish_tf:=true`），而第 1
#    轮标定之前它故意不发 —— 那个量正是要解出来的东西，拿旧值反喂就是自证。
#    把它写进就绪门 = 第 1 轮永远抓不到俯视帧（等到超时）。需要 base 系坐标的
#    下游（`graspnet_candidates.TOPCAM_CHAIN`）仍然要求它，缺了当场报错。
TOPCAM_CHAIN = (
    "topcam_link:topcam_color_frame",
    "topcam_color_frame:topcam_color_optical_frame",
)
TOPCAM_TIP = "topcam_color_optical_frame"

WRIST_TOPICS = {
    "color": ("/camera/color/image_raw", "Image"),
    "depth": ("/camera/aligned_depth_to_color/image_raw", "Image"),
    "info": ("/camera/color/camera_info", "CameraInfo"),
}
TOPCAM_TOPICS = {
    "color": ("/topcam/color/image_raw", "Image"),
    # align_depth.enable 打开后驱动发对齐深度（launch 里强制开）。
    "depth": ("/topcam/aligned_depth_to_color/image_raw", "Image"),
    "info": ("/topcam/color/camera_info", "CameraInfo"),
}
JOINT_STATE_TOPIC = ("/joint_states", "JointState")

#: 图像订阅的队列深度。**1**：只要最新一帧。实测（第 321 轮）旧值 10 会让
#: 「排空队列」变成处理一串旧帧；帧率 15 Hz 时积压 10 帧就是 0.67 s 的时差。
IMAGE_QOS_DEPTH = 1

#: 彩色与深度/内参按时间戳配对时允许的最大时差（毫秒）。相机一帧 66.7 ms，
#: 实测「取最新」的偏差恰好是 0 / ±33.357 ms —— 也就是**差了一帧**。
MATCH_TOL_MS = 40.0

#: 动态 TF 与图像时间戳之间的最大允许时差（毫秒）。实测抖动只有几毫秒，
#: 250 ms 远在之上；而第 321 轮那种积压是 1793 ms，会被这条挡住。
TF_MATCH_TOL_MS = 250.0

#: 动态 TF 流本身的鲜度上限（毫秒）：最近的 /tf 帧比现在旧这么多 ⇒ 流断了。
#: 第 320 轮跑后 pad burst 就栽在这里（整条链消失却先写了几帧）。
TF_STREAM_MAX_AGE_MS = 500.0

CAMERAS = {
    "wrist": {
        "topics": WRIST_TOPICS, "chain": CHAIN_KEYS, "tip": TIP,
        # 腕部帧要配臂的关节角/动态 TF —— 相机位姿是从它们算出来的。
        "need_js": True,
    },
    "topcam": {
        "topics": TOPCAM_TOPICS, "chain": TOPCAM_CHAIN, "tip": TOPCAM_TIP,
        # 俯视相机与臂无关：整条链都在 /tf_static 上，不要求 /joint_states。
        "need_js": False,
    },
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


def nearest(hist, stamp):
    """``hist`` 里时间戳离 ``stamp`` 最近的一条 → ``(|Δ|ns, stamp_ns, obj)``。

    ``hist`` 的元素是 ``(stamp_ns, obj)``。空表返回 None。
    """
    best = None
    for t, obj in hist:
        d = abs(t - stamp)
        if best is None or d < best[0]:
            best = (d, t, obj)
    return best


def match(hist, stamp, tol_ms, wait_s=0.2, sleep=time.sleep):
    """按时间戳配对：先等一个**完全相同**的戳，等不到再退到最近的一条。

    超过 ``tol_ms`` 视为配不上（返回 None）—— 宁可报错，也不拿另一帧的数据
    冒充这一帧的。2026-11-12 实测：旧实现取「最新」，color↔depth 的时差是
    0 / ±33.357 ms 各占一半，差的正好是**另一帧**（相机 66.7 ms 一帧）。
    """
    deadline = time.monotonic() + wait_s
    while True:
        hit = nearest(hist, stamp)
        if hit is not None and hit[0] == 0:
            return hit
        if time.monotonic() >= deadline:
            if hit is None:
                return None
            return hit if hit[0] <= tol_ms * 1e6 else None
        sleep(0.01)


def pick_chain(chain_keys, tf_static, tf_dyn, color_stamp, now_ns):
    """按图像时间戳取整条链：静态取最新，动态取**离图像最近**的那一帧。

    ``tf_static`` 是 ``{key: transform}``；``tf_dyn`` 是 ``{key: [(stamp, tf)]}``。

    返回 ``(trans, origin, deltas_ms, why)``：
      · 成功 ⇒ ``deltas_ms`` 是每个动态帧相对图像的时间差（可为空）；
      · 缺链 / 动态帧配不上 / 动态流断了 ⇒ ``trans is None``，``why`` 说明原因。
    """
    trans, origin, deltas, miss = {}, {}, [], []
    for key in chain_keys:
        hist = tf_dyn.get(key)
        if hist:
            d, t, tf = nearest(hist, color_stamp)
            if d > TF_MATCH_TOL_MS * 1e6:
                return None, None, None, (
                    "动态 TF 与图像配不上：%s 差 %.0f ms（上限 %.0f）"
                    % (key, (t - color_stamp) / 1e6, TF_MATCH_TOL_MS))
            trans[key] = tf
            origin[key] = "tf"
            deltas.append((t - color_stamp) / 1e6)
        elif key in tf_static:
            trans[key] = tf_static[key]
            origin[key] = "tf_static"
        else:
            miss.append(key)
    if miss:
        return None, None, None, "TF 链缺 %s" % miss
    # 动态 TF 流本身必须还活着（第 320 轮：流断了，旧帧还在缓冲里，照写不误）。
    ages = [(now_ns - hist[-1][0]) / 1e6
            for hist in (tf_dyn.get(k) for k in chain_keys) if hist]
    if ages and max(ages) > TF_STREAM_MAX_AGE_MS:
        return None, None, None, (
            "动态 TF 流断了：最近一帧已是 %.0f ms 前（上限 %.0f）"
            % (max(ages), TF_STREAM_MAX_AGE_MS))
    return trans, origin, sorted(deltas), None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out", help="帧目录（会新建）")
    ap.add_argument("--timeout", type=float, default=30.0,
                    help="等话题的上限（秒）。到点还缺件就失败，不写半成品")
    ap.add_argument("--camera", choices=sorted(CAMERAS), default="wrist",
                    help="wrist = /camera 腕部（默认，行为与旧版一致）；"
                         "topcam = /topcam 俯视第二路（阶段 4）")
    ap.add_argument("--burst", type=int, default=1,
                    help="连续抓几帧（默认 1）。>1 时文件名带 000 序号。"
                         "每帧都会等**一条新的**彩色消息")
    ap.add_argument("--interval", type=float, default=0.0,
                    help="burst 两帧之间至少间隔多少秒（默认 0 = 收到就抓）")
    ap.add_argument("--joints-only", action="store_true",
                    help="只取 /joint_states（发轨迹前算计划用），不碰相机与 TF")
    args = ap.parse_args()
    if args.burst < 1:
        print("!! --burst 至少是 1", file=sys.stderr)
        return 2

    cam = CAMERAS[args.camera]
    topics = cam["topics"]
    chain_keys = cam["chain"]
    tip = cam["tip"]

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

    # ---------------------------------------------------------------- 采集缓冲
    # 回调只把消息塞进缓冲，处理全在主线程 —— 但**自旋在独立线程里常驻**。
    #
    # 2026-11-12 第 321 轮实测：旧实现只在 `wait_one()` 里 spin_once，而 PNG 写盘
    # （1280×720 彩色 + 深度，每帧几百毫秒）期间**完全不 spin**，`wait_one()` 又
    # 靠旧状态立刻返回 ⇒ 订阅队列一次都没被排空过。同一组 burst 里 TF↔图像时间差
    # 从 +42 ms 线性恶化到 −1793 ms（每帧正好一个抓帧周期），队列里全是旧帧。
    # 臂静止时旧 TF 仍然是对的（所以没出过事），但「帧 / 关节 / TF 是同一时刻」
    # 这个前提已经破了。常驻自旋之后，缓冲里永远是新到的消息。
    color_q = collections.deque(maxlen=8)       # (stamp_ns, msg) 未消费的彩色帧
    depth_hist = collections.deque(maxlen=64)   # 供按**时间戳**配对（不是取最新）
    info_hist = collections.deque(maxlen=16)
    js_hist = collections.deque(maxlen=200)     # 关节角历史，插值到图像时间戳
    seen = set()

    def on_color(m):
        seen.add("color")
        color_q.append((stamp_ns(m), m))

    def on_depth(m):
        seen.add("depth")
        depth_hist.append((stamp_ns(m), m))

    def on_info(m):
        seen.add("info")
        info_hist.append((stamp_ns(m), m))

    def on_js(m):
        seen.add("js")
        js_hist.append(m)

    # 静态 / 动态 TF **分开存**：
    #   · `/tf_static` 上的帧不随时间变（实测本机脚本里整条腕部链只有那六个
    #     转动关节在 `/tf`，`tool0:camera_link` 也在 `/tf_static`），留最新一条就够；
    #   · `/tf` 上的必须**按图像时间戳挑**，否则臂在动时相机位姿用的是另一时刻
    #     的关节角 —— 旧实现取「最新的」，等于把延迟直接算成位姿误差。
    tf_static = {}                                   # key -> transform
    tf_dyn = collections.defaultdict(
        lambda: collections.deque(maxlen=120))       # key -> [(stamp_ns, tf)]

    def key_of(t):
        return "%s:%s" % (t.header.frame_id, t.child_frame_id)

    def acc_static(m):
        seen.add("tf_static")
        for t in m.transforms:
            tf_static[key_of(t)] = t

    def acc_dyn(m):
        seen.add("tf")
        for t in m.transforms:
            tf_dyn[key_of(t)].append((stamp_ns(t), t))

    qos_img = QoSProfile(depth=IMAGE_QOS_DEPTH,
                         reliability=ReliabilityPolicy.RELIABLE)
    qos_info = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    qos_static = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)

    if not args.joints_only:
        node.create_subscription(RosImage, topics["color"][0], on_color, qos_img)
        node.create_subscription(RosImage, topics["depth"][0], on_depth, qos_img)
        node.create_subscription(CameraInfo, topics["info"][0], on_info, qos_info)
    node.create_subscription(JointState, JOINT_STATE_TOPIC[0], on_js, qos_info)

    if not args.joints_only:
        node.create_subscription(TFMessage, "/tf", acc_dyn, qos_info)
        node.create_subscription(TFMessage, "/tf_static", acc_static, qos_static)

    # 自旋放到独立线程里常驻：主线程写盘时订阅照常被排空（见上面那段说明）。
    from rclpy.executors import SingleThreadedExecutor
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, name="capture-spin",
                                   daemon=True)
    spin_thread.start()

    def close_all():
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()

    def wait_for(pred, timeout):
        """等 pred() 为真（或超时）。自旋在别的线程，这里只睡觉。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if pred():
                return True
            time.sleep(0.02)
        return pred()

    if args.joints_only:
        if not wait_for(lambda: "js" in seen, args.timeout):
            print("!! %.0f s 没收到 /joint_states —— 驱动起了吗？" % args.timeout,
                  file=sys.stderr)
            close_all()
            return 2
        js = js_hist[-1]
        json.dump({"name": list(js.name),
                   "position": [float(v) for v in js.position],
                   "stamp_ns": stamp_ns(js)},
                  open(os.path.join(args.out, "joint_states.json"), "w"), indent=2)
        print("OK %s （只取关节角）" % args.out)
        print("  关节(deg)：%s" % {k: round(v * 57.2957795, 1)
                                  for k, v in zip(js.name, js.position)})
        close_all()
        return 0

    # 一次成功的抓帧需要哪些话题：俯视那条链整条在 /tf_static 上、与臂无关，
    # 所以**不要求** /joint_states 与动态 /tf（要求了就会在只起相机时白等 30 s）。
    needed = ["color", "depth", "info", "tf_static"] + (
        ["js", "tf"] if cam["need_js"] else [])

    def tf_snapshot(color_stamp):
        """``pick_chain`` 的现场版：把「现在」也传进去（流鲜度用它判）。"""
        return pick_chain(chain_keys, tf_static, tf_dyn, color_stamp,
                          time.time_ns())



    def to_array(m):
        buf = np.frombuffer(m.data, dtype=np.uint8)
        if m.encoding in ("rgb8", "bgr8"):
            a = buf.reshape(m.height, m.step // 3, 3)[:, :m.width]
            return a[:, :, ::-1] if m.encoding == "bgr8" else a
        if m.encoding == "16UC1":
            return buf.view(np.uint16).reshape(m.height, m.step // 2)[:, :m.width]
        raise ValueError("未处理的 encoding %r" % m.encoding)

    def _p(idx, name, ext):
        """文件名：单帧保持旧名（color.png），burst 时带三位序号（color_000.png）。

        单帧**不改名**是刻意的：仓库里已有几个脚本按 `color.png / meta.json`
        读帧目录，改名的代价是它们静默找不到文件。
        """
        if args.burst == 1:
            return os.path.join(args.out, name + ext)
        return os.path.join(args.out, "%s_%03d%s" % (name, idx, ext))

    def write_one(idx, fr):
        """把**这一帧的快照**写成文件。返回退出码。

        快照由主线程按时间戳配好再传进来（``fr``），这里不再读「全局最新」——
        旧实现读的是写盘那一刻的最新消息，与这张图像不是同一时刻。
        """
        color_m, depth_m, info = fr["color"], fr["depth"], fr["info"]
        color_ns = fr["color_stamp_ns"]
        try:
            color = to_array(color_m)
            depth = to_array(depth_m)
        except ValueError as e:
            print("!! %s" % e, file=sys.stderr)
            return 3

        Image.fromarray(color).save(_p(idx, "color", ".png"))
        Image.fromarray(depth).save(_p(idx, "depth", ".png"))

        meta = {
            "camera": args.camera,
            "camera_info": {
                "width": info.width, "height": info.height,
                "distortion_model": info.distortion_model,
                "D": list(info.d), "K": list(info.k),
                "R": list(info.r), "P": list(info.p),
                "frame_id": info.header.frame_id,
                "stamp_ns": stamp_ns(info),
                "stamp_delta_ms": fr["info_delta_ms"],
            },
            # depth 与 color 已对齐（话题名里就写着），所以共用一份 K。
            "color": {"width": color_m.width, "height": color_m.height,
                      "encoding": color_m.encoding,
                      "is_bigendian": bool(color_m.is_bigendian),
                      "step": color_m.step,
                      "frame_id": color_m.header.frame_id,
                      "stamp_ns": color_ns},
            "depth": {"width": depth_m.width, "height": depth_m.height,
                      "encoding": depth_m.encoding,
                      "is_bigendian": bool(depth_m.is_bigendian),
                      "step": depth_m.step,
                      "frame_id": depth_m.header.frame_id,
                      "stamp_ns": stamp_ns(depth_m),
                      # 与彩色时间戳的差。按时间戳配对后应恒为 0；不为 0 时
                      # 这里就是证据（旧实现中位数是 ±33.36 ms = 差一帧）。
                      "stamp_delta_ms": fr["depth_delta_ms"]},
            "chain_tip": tip,
            "chain_keys": list(chain_keys),
            # 动态 TF 相对图像的时间差（毫秒）。相机位姿就是拿它们算的。
            "tf_stamp_delta_ms": fr["tf_deltas"],
        }
        json.dump(meta, open(_p(idx, "meta", ".json"), "w"), indent=2)

        # ⚠️ 这里 dump 的是**链上所有**帧（静态 + 动态），不只是这次判定要用的。
        # 俯视那条链只有四段，但联合标定还要用驱动实测的 link→optical 链，
        # 少存一段就得重新抓帧。
        trans = {}
        for key, t in fr["trans"].items():
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
                   "origin": {k: fr["origin"].get(k) for k in trans}},
                  open(_p(idx, "tf", ".json"), "w"), indent=2)

        # 俯视相机与臂无关：可能根本没起驱动，那就没有 /joint_states。
        if "js" not in seen:
            print("  第 %d 帧：%s %dx%d（无 /joint_states，跳过关节记录）"
                  % (idx, args.camera, color_m.width, color_m.height))
            return 0

        # 关节角插值到**图像时间戳**上（见 joints_at 的说明）。残差 = 离最近的
        # 插值端点有多远；没夹住（图像早于/晚于整段历史）时退化成最近一条，并标 False。
        js = joints_at(js_hist, color_ns)
        if js is None:
            print("  第 %d 帧：关节历史为空，跳过关节记录" % idx)
            return 0
        names, pos, resid_ms, interp = js
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
        tf_gaps = fr["tf_deltas"]
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
                   "tf_gap_ms": {"min": tf_gaps[0],
                                 "median": tf_gaps[len(tf_gaps) // 2],
                                 "max": tf_gaps[-1]} if tf_gaps else None},
                  open(_p(idx, "joint_states", ".json"), "w"), indent=2)

        print("第 %d 帧 %s：图像 %dx%d  深度 %dx%d  关节 %d 个  TF %d 帧"
              % (idx, args.camera, color_m.width, color_m.height,
                 depth_m.width, depth_m.height, len(names), len(trans)))
        print("  深度/彩色 时间差 %+.1f ms，内参 %+.1f ms（按时间戳配对）"
              % (fr["depth_delta_ms"], fr["info_delta_ms"]))
        print("  关节角%s到图像时间戳：残差 %.1f ms（历史 %d 条，±0.3 s 内 %d 条，"
              "相邻最大间隔 %s ms）"
              % ("插值" if interp else "**没夹住，取最近一条**", resid_ms,
                 len(js_hist), len(win),
                 "%.1f" % max_gap if max_gap is not None else "—"))
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
        return 0

    last_stamp = 0            # 已经写过的最后一帧的彩色时间戳（burst 不许写同一帧两遍）
    for idx in range(args.burst):
        deadline = time.monotonic() + args.timeout
        fr, why = None, None
        while time.monotonic() < deadline:
            # 1) 一条**比上一帧新**的彩色帧
            while color_q and color_q[0][0] <= last_stamp:
                color_q.popleft()
            if not color_q:
                missing = [k for k in needed if k not in seen]
                why = ("话题缺 %s" % missing) if missing else \
                    "还没有新的彩色帧（上一帧 %d）" % last_stamp
                time.sleep(0.01)
                continue
            color_ns = color_q[0][0]
            # 2) 链上每帧都能按时间戳取到（静态取最新、动态取离图像最近的）
            trans, origin, deltas, why = tf_snapshot(color_ns)
            if trans is None:
                time.sleep(0.01)
                continue
            # 3) 腕部：关节历史要能把图像时间戳**夹住**（插值，不是取最近）
            if cam["need_js"]:
                if not js_hist:
                    why = "还没收到 /joint_states"
                    time.sleep(0.01)
                    continue
                lo = min(stamp_ns(m) for m in js_hist)
                hi = max(stamp_ns(m) for m in js_hist)
                if not lo <= color_ns <= hi:
                    why = ("关节历史还没夹住图像时间戳（图像 %d，历史 %d–%d）"
                           % (color_ns, lo, hi))
                    time.sleep(0.01)
                    continue
            # 4) 深度与内参：按时间戳配对，不拿「最新」冒充
            dhit = match(depth_hist, color_ns, MATCH_TOL_MS)
            ihit = match(info_hist, color_ns, 2 * MATCH_TOL_MS)
            if dhit is None or ihit is None:
                why = ("深度/内参与图像配不上：深度 %s，内参 %s"
                       % ("缺" if dhit is None else "差 %.0f ms" % (dhit[0] / 1e6),
                          "缺" if ihit is None else "差 %.0f ms" % (ihit[0] / 1e6)))
                time.sleep(0.01)
                continue
            # 全部就位 —— 从队列里真正取走这一帧，其余状态都是它的快照
            color_ns, color_msg = color_q.popleft()
            fr = {"color": color_msg, "depth": dhit[2],
                  "info": ihit[2], "color_stamp_ns": color_ns,
                  "depth_delta_ms": dhit[0] / 1e6, "info_delta_ms": ihit[0] / 1e6,
                  "trans": trans, "origin": origin, "tf_deltas": deltas}
            break

        if fr is None:
            print("!! 第 %d/%d 帧没等到（超时 %.0f s）：%s"
                  % (idx + 1, args.burst, args.timeout, why or "原因未知"),
                  file=sys.stderr)
            if cam["need_js"]:
                print("   ↑ 腕部链缺 TF 多半是 wrist_tf_publish.py 没起，或者"
                      " /tf_static 的那一路发布者还没发 —— 臂这边不要动，重跑一次即可。",
                      file=sys.stderr)
            else:
                print("   ↑ 俯视链缺 TF：先确认 topcam 驱动起来了、并且"
                      "（第 1 轮之后）topcam_publish_tf:=true 已发 base_link→topcam_link。",
                      file=sys.stderr)
            close_all()
            return 2
        rc = write_one(idx, fr)
        if rc:
            close_all()
            return rc
        last_stamp = fr["color_stamp_ns"]
        if idx + 1 < args.burst and args.interval > 0:
            time.sleep(args.interval)

    print("OK %s （%s，%d 帧）" % (args.out, args.camera, args.burst))
    close_all()
    return 0


if __name__ == "__main__":
    sys.exit(main())

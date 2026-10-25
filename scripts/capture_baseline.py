#!/usr/bin/env python3
"""精度基线采集：一次拍下**一帧的全部证据**。**只读** —— 不开设备、不发控制。

为什么要单独一份，而不是复用 `grab_frames.py`
==============================================
`grab_frames.py` 只存彩色图 + 内参，够做「真值帧」（俯视），但基线轮要能**事后
离线复算**每一次判断：

  · 腕部这帧的 `/detections` 与 `/targets_3d` 到底给了什么（分数、类别、3D 点）；
  · 那一刻的 TF（`base_link→tool0→camera_link`）是什么 —— 没有它，事后连「这块
    方块在 base 下应该在哪」都算不出来；
  · 对齐深度原样（16-bit PNG）—— 量化「读远 ~1.2%」要用原始读数，不能用截图；
  · 当次生效的俯视外参（从站点配置抄进 manifest）—— 外参**会无声过期**，
    报告里写「用的是哪一份」比写「应该是哪一份」有用。

⚠️ **不写半截数据集**：任一条必需话题/必需 TF 拿不到 ⇒ 退 2，**一个文件都不留**。
   半截数据最坏的地方不是缺，而是**看起来像有了**（评估脚本会把缺项当 0）。

⚠️ 本工具**不发任何控制**：不建 action client、不碰夹爪、不切控制器。臂的位置
   由人在示教器上摆；它只记录摆好之后的那一帧。

用法（板上；在动作前先 `source /opt/ros/humble/setup.bash`）::

    # 腕部一帧（4 个巡察位姿各拍一次，tag 自己起名）
    python3 scripts/capture_baseline.py --camera wrist --tag L1-P1 --note 布局1-西北角

    # 俯视真值帧（臂退到 HOME/视野外再拍）
    python3 scripts/capture_baseline.py --camera top --tag L1-truth

输出（默认 `~/benchmark/2026-10-baseline/`，**仓库外**）::

    manifest.json                        # 每帧一条，追加
    frames/<camera>/<tag>/
        color.png  depth.png  camera_info.json
        detections.json  targets_3d.json  tf.json  meta.json

退出码：0 = 存好；2 = 环境/话题/TF 不满足（**没写任何东西**）；3 = 目录已存在
（不覆盖已有证据，除非 `--force`）。
"""

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time

# ⚠️ 本仓约定：.pyc 的失效判据是 mtime+size
sys.dont_write_bytecode = True

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_OUT = os.path.expanduser("~/benchmark/2026-10-baseline")

#: 单层话题名（`pipeline.launch.py` 把真机的 `/camera/camera/...` 重映射成这个；
#: 名字不对的症状是「一直等不到消息」，看着像相机坏了）。做成可覆盖的。
COLOR = "/camera/color/image_raw"
DEPTH = "/camera/aligned_depth_to_color/image_raw"
INFO = "/camera/color/camera_info"
DETECTIONS = "/detections"
TARGETS = "/targets_3d"

#: 两种相机各自**必须**成立的 TF 链。最后一条是「相机在 base 下的位姿」——
#: 没有它，这一帧的 3D 点落不进基座系，离线评估无从谈起。
TF_PAIRS = {
    "wrist": ["base_link:tool0", "tool0:camera_link",
              "camera_link:camera_color_optical_frame",
              "base_link:camera_color_optical_frame"],
    "top": ["base_link:tool0", "base_link:camera_color_optical_frame",
            "camera_link:camera_color_optical_frame"],
}
REQUIRED_TF = {
    "wrist": "base_link:camera_color_optical_frame",
    "top": "base_link:camera_color_optical_frame",
}

SITE = os.path.expanduser("~/.config/ur5-vision-grasp/site.yaml")

OK = "✅"
BAD = "❌"
WARN = "⚠️ "


# ----------------------------------------------------------------------
# 纯函数（不碰 ROS，自检够得着）
# ----------------------------------------------------------------------
def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def file_entry(path):
    """一个文件的证据条目：哈希 + 字节数。名字用**相对帧目录**的。"""
    return {"sha256": sha256_file(path), "bytes": os.path.getsize(path)}


def stamp_ns(stamp):
    """ROS stamp → 纳秒整数（None 安全）。"""
    if stamp is None:
        return None
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def image_meta(msg):
    """`sensor_msgs/Image` 里事后复算需要的那些字段（**不存像素**）。"""
    return {"width": int(msg.width), "height": int(msg.height),
            "encoding": str(msg.encoding), "is_bigendian": bool(msg.is_bigendian),
            "step": int(msg.step), "frame_id": str(msg.header.frame_id),
            "stamp_ns": stamp_ns(msg.header.stamp)}


def header_meta(msg):
    """只有 header 的消息（检测/投影数组）也要留下时间戳与帧名。"""
    return {"frame_id": str(msg.header.frame_id),
            "stamp_ns": stamp_ns(msg.header.stamp)}


def camera_info_dict(msg):
    """内参 + 畸变，直接摊平成 JSON（下标含义与 `grab_frames.py` 一致）。

    ⚠️ 这里**必须**保留 ROS 原样（K/D/R/P 一维列表）：自己重排成 fx/fy/cx/cy
    的口径一旦写错（`cx` 拿成 `K[1]`）**不会报错**，只会让每一帧都偏同一个量。
    要 fx/fy/cx/cy 的地方在离线端按 `K[0]/K[4]/K[2]/K[5]` 取。
    """
    return {"width": int(msg.width), "height": int(msg.height),
            "distortion_model": str(msg.distortion_model),
            "D": [float(x) for x in msg.d],
            "K": [float(x) for x in msg.k],
            "R": [float(x) for x in msg.r],
            "P": [float(x) for x in msg.p],
            "frame_id": str(msg.header.frame_id),
            "stamp_ns": stamp_ns(msg.header.stamp)}


def tf_dict(tf_msg):
    """`TransformStamped` → JSON（t 米 / q 原样）。"""
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    return {"translation": {"x": float(t.x), "y": float(t.y), "z": float(t.z)},
            "rotation": {"x": float(q.x), "y": float(q.y),
                         "z": float(q.z), "w": float(q.w)},
            "stamp_ns": stamp_ns(tf_msg.header.stamp),
            "frame_id": str(tf_msg.header.frame_id),
            "child_frame_id": str(tf_msg.child_frame_id)}


def _best_hypothesis(det):
    """取分数最高的那条假设 → `{"score", "class_id"}`（没有假设时 None）。

    ⚠️ 字段路径是 `results[i].hypothesis.score` / `.hypothesis.class_id`，
    **不是** `results[i].score`。2026-10-25 实测：旧实现用
    `getattr(hyp, "score", 0.0)` 读顶层属性，真实消息里根本没有 —— 于是
    **每一框都静默记成 score=0.0、class_id=""**；而单元测试造的假消息恰好
    是扁平结构，测试一路绿。分数分布是这一轮要回答的核心问题之一，
    「写下来全是 0」的数据事后救不回来，所以这里**不给默认值兜底**：
    字段形状不对就当场抛，别写半截数据集。
    """
    best = None
    for hyp in det.results:
        score = float(hyp.hypothesis.score)
        class_id = str(hyp.hypothesis.class_id)
        if best is None or score > best["score"]:
            best = {"score": score, "class_id": class_id}
    return best


def detections_dict(msg):
    """`vision_msgs/Detection2DArray` → 每框一条（中心/尺寸/分数/类别）。"""
    out = []
    for det in msg.detections:
        box = det.bbox
        out.append({"cx": float(box.center.position.x),
                    "cy": float(box.center.position.y),
                    "theta": float(getattr(box.center, "theta", 0.0)),
                    "size_x": float(box.size_x), "size_y": float(box.size_y),
                    "best": _best_hypothesis(det)})
    return {"frame_id": str(msg.header.frame_id),
            "stamp_ns": stamp_ns(msg.header.stamp), "detections": out}


def targets_dict(msg):
    """`vision_msgs/Detection3DArray` → 每目标一条（位姿 + 尺寸 + 分数）。"""
    out = []
    for det in msg.detections:
        best = _best_hypothesis(det)
        p = det.bbox.center.position
        q = det.bbox.center.orientation
        out.append({"x": float(p.x), "y": float(p.y), "z": float(p.z),
                    "qx": float(q.x), "qy": float(q.y),
                    "qz": float(q.z), "qw": float(q.w),
                    "size_x": float(det.bbox.size.x),
                    "size_y": float(det.bbox.size.y),
                    "size_z": float(det.bbox.size.z),
                    "best": best})
    return {"frame_id": str(msg.header.frame_id),
            "stamp_ns": stamp_ns(msg.header.stamp), "targets": out}


def judge_topics(counts, required):
    """`{话题: 发布者数}` 够不够。→ `(ok, 缺的列表)`。**纯函数**。

    ⚠️ 判据是「有没有发布者」，不是「等消息超时了没有」—— 两者症状一样
    （都是等不到），但前者能当场指出**是哪一条**缺席。
    """
    missing = [t for t in required if counts.get(t, 0) <= 0]
    return (not missing), missing


def safe_tag(tag):
    """tag → 目录名。**不许**出现路径分隔符或 `..`（否则能写到帧目录外面）。"""
    bad = set('/\\\0')
    if not tag or any(c in bad for c in tag) or tag in (".", ".."):
        raise ValueError("tag 不能为空、不能含 / \\ 或 ..：%r" % tag)
    return tag


def load_site_extrinsics(path=SITE):
    """从站点配置里抄下**当次生效的**俯视外参。读不到就如实记 None。"""
    try:
        import yaml
    except ImportError:
        return {"path": path, "error": "本机没有 PyYAML"}
    if not os.path.exists(path):
        return {"path": path, "error": "站点配置不在"}
    try:
        with open(path, "r", encoding="utf-8") as fh:
            doc = yaml.safe_load(fh) or {}
    except Exception as exc:                             # noqa: BLE001
        return {"path": path, "error": "读不了：%s" % exc}
    top = (doc.get("topcam") or {}) if isinstance(doc, dict) else {}
    ext = top.get("extrinsics") if isinstance(top, dict) else None
    return {"path": path, "topcam_extrinsics": ext}


def tree_info(repo=REPO):
    """这棵树的身份：路径 / 分支 / SHA / 有没有未提交改动。取不到就 None。"""
    def git(*args):
        try:
            out = subprocess.run(["git"] + list(args), cwd=repo,
                                 capture_output=True, text=True, timeout=10)
            return out.stdout.strip() if out.returncode == 0 else None
        except Exception:                                # noqa: BLE001
            return None
    status = git("status", "--porcelain")
    return {"path": repo, "branch": git("rev-parse", "--abbrev-ref", "HEAD"),
            "sha": git("rev-parse", "HEAD"),
            "dirty": None if status is None else bool(status)}


def write_manifest(path, entry):
    """把一条记录**追加**进 manifest（JSON 数组）。原子替换，不留半截文件。

    ⚠️ 先读后写：manifest 是**唯一**把帧与「当时的环境」绑在一起的东西，
    写坏了就没法补。
    """
    rows = []
    if os.path.exists(path):
        with open(path, "r", encoding="utf-8") as fh:
            rows = json.load(fh)
        if not isinstance(rows, list):
            raise ValueError("%s 不是数组 —— 拒绝往里追（会毁掉已有记录）" % path)
    rows.append(entry)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(rows, fh, ensure_ascii=False, indent=2)
        fh.write("\n")
    os.replace(tmp, path)


# ----------------------------------------------------------------------
# 采一帧（要 ROS）
# ----------------------------------------------------------------------
def _require_ros():
    try:
        import rclpy                                    # noqa: F401
        from sensor_msgs.msg import Image               # noqa: F401
        from tf2_ros import Buffer, TransformListener   # noqa: F401
        from vision_msgs.msg import Detection2DArray    # noqa: F401
    except ImportError as exc:                          # noqa: BLE001
        print("%s 本机没有 ROS 2 Python 环境：%s" % (BAD, exc))
        print("   ⇒ 先 source /opt/ros/humble/setup.bash（板上还要 "
              "source ros2_ws/install/setup.bash）。")
        return False
    return True


def _wait_publishers(node, topics, timeout):
    """等到这些话题**都有发布者**（或超时）。→ `{topic: count}`。"""
    deadline = time.time() + timeout
    counts = {}
    while True:
        counts = {t: node.count_publishers(t) for t in topics}
        if all(v > 0 for v in counts.values()) or time.time() >= deadline:
            return counts
        time.sleep(0.2)


class Grabber:
    """订阅五路话题，每路记住**最新**一条；TF 用 Buffer 查。"""

    def __init__(self, topics):
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import Image, CameraInfo
        from vision_msgs.msg import Detection2DArray, Detection3DArray
        from tf2_ros import Buffer, TransformListener
        from rclpy.duration import Duration

        self.node = rclpy.create_node("capture_baseline")
        self.msgs = {}
        # 相机/图像走 best-effort（驱动的 QoS）；检测/投影是普通可靠发布。
        sensor_qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                                reliability=ReliabilityPolicy.BEST_EFFORT)
        self.node.create_subscription(Image, topics["color"],
                                      lambda m: self._put("color", m), sensor_qos)
        self.node.create_subscription(Image, topics["depth"],
                                      lambda m: self._put("depth", m), sensor_qos)
        self.node.create_subscription(CameraInfo, topics["info"],
                                      lambda m: self._put("info", m), sensor_qos)
        self.node.create_subscription(Detection2DArray, topics["detections"],
                                      lambda m: self._put("detections", m), 10)
        self.node.create_subscription(Detection3DArray, topics["targets"],
                                      lambda m: self._put("targets", m), 10)
        self.buf = Buffer(cache_time=Duration(seconds=10), node=self.node)
        self.listener = TransformListener(self.buf, self.node, spin_thread=False)
        self._Duration = Duration

    def _put(self, key, msg):
        self.msgs[key] = msg

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            import rclpy
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def wait_all(self, keys, timeout):
        """等到每一路都至少来一条。→ 缺的键列表。"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.spin_for(0.05)
            missing = [k for k in keys if k not in self.msgs]
            if not missing:
                return []
        return [k for k in keys if k not in self.msgs]

    def lookup(self, pair):
        """`"a:b"` → (tf_dict | None, 错误说明)。"""
        a, _, b = pair.partition(":")
        try:
            from rclpy.time import Time
            tf = self.buf.lookup_transform(a, b, Time(),
                                           self._Duration(seconds=0.5))
            return tf_dict(tf), None
        except Exception as exc:                        # noqa: BLE001
            return None, "%s: %s" % (type(exc).__name__, exc)


def _save_color(msg, path):
    import cv2
    import numpy as np
    enc = str(msg.encoding).lower()
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    if enc in ("rgb8", "bgr8"):
        img = arr.reshape(int(msg.height), int(msg.width), 3)
        if enc == "rgb8":                       # cv2 写盘按 BGR
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif enc in ("mono8",):
        img = arr.reshape(int(msg.height), int(msg.width))
    else:
        raise ValueError("不认识的彩色编码 %r（只支持 rgb8/bgr8/mono8）" % enc)
    if not cv2.imwrite(path, img):
        raise IOError("cv2.imwrite 失败：%s" % path)


def _save_depth(msg, png_path, npy_path):
    """→ 实际存下的文件名。16 位整数才写 PNG；其它编码原样存 .npy（不换算）。"""
    import cv2
    import numpy as np
    enc = str(msg.encoding).lower()
    dt = {"16uc1": np.uint16, "32fc1": np.float32,
          "mono16": np.uint16}.get(enc)
    if dt is None:
        raise ValueError("不认识的深度编码 %r" % enc)
    arr = np.frombuffer(msg.data, dtype=dt).reshape(int(msg.height),
                                                    int(msg.width))
    if dt is np.uint16:
        if not cv2.imwrite(png_path, arr):
            raise IOError("cv2.imwrite 失败：%s" % png_path)
        return os.path.basename(png_path)
    np.save(npy_path, arr)
    return os.path.basename(npy_path)


# ----------------------------------------------------------------------
def main(argv=None):
    ap = argparse.ArgumentParser(
        description="拍一帧精度基线证据（只读；不发控制）。",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", required=True, choices=("wrist", "top"))
    ap.add_argument("--tag", required=True, help="这一帧的名字（目录名）")
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--note", action="append", default=[],
                    help="自由备注（布局/位姿/谁摆的），可给多次")
    ap.add_argument("--color-topic", default=COLOR)
    ap.add_argument("--depth-topic", default=DEPTH)
    ap.add_argument("--info-topic", default=INFO)
    ap.add_argument("--detections-topic", default=DETECTIONS)
    ap.add_argument("--targets-topic", default=TARGETS)
    ap.add_argument("--timeout", type=float, default=15.0,
                    help="等话题/TF 的秒数（默认 %(default)s）")
    ap.add_argument("--require-tf", default=None,
                    help="必须成立的 TF（a:b）；默认按相机选")
    ap.add_argument("--skip-detections", action="store_true",
                    help="俯视真值帧用：只拍图，不等 /detections 与 /targets_3d")
    ap.add_argument("--force", action="store_true",
                    help="帧目录已存在也覆盖（默认拒绝：不覆盖已有证据）")
    args = ap.parse_args(argv)

    try:
        args.tag = safe_tag(args.tag)
    except ValueError as exc:
        print("%s %s" % (BAD, exc))
        return 2

    frame_dir = os.path.join(args.out, "frames", args.camera, args.tag)
    if os.path.exists(frame_dir) and not args.force:
        print("%s 帧目录已存在：%s" % (BAD, frame_dir))
        print("   ⇒ 不覆盖已有证据（要重拍加 --force，或换个 --tag）。")
        return 3

    topics = {"color": args.color_topic, "depth": args.depth_topic,
              "info": args.info_topic, "detections": args.detections_topic,
              "targets": args.targets_topic}
    need = ["color", "depth", "info"]
    if not args.skip_detections:
        need += ["detections", "targets"]

    if not _require_ros():
        return 2

    import rclpy
    rclpy.init()
    g = Grabber(topics)
    rc = 2
    tmp_dir = frame_dir + ".tmp"
    try:
        print("=" * 70)
        print("  精度基线：%s / %s" % (args.camera, args.tag))
        print("=" * 70)
        counts = _wait_publishers(g.node, [topics[k] for k in need], args.timeout)
        ok, missing = judge_topics(counts, [topics[k] for k in need])
        if not ok:
            print("%s 这些话题没有发布者（栈没起 / 话题名不对）：" % BAD)
            for t in missing:
                print("     · %s" % t)
            print("   ⇒ 一个文件都不写（半截数据集会被下游当成「有数据」）。")
            return 2

        # 等消息：先图像，再等检测/投影**不早于**图像那一拍
        missing = g.wait_all(["color", "depth", "info"], args.timeout)
        if missing:
            print("%s 等不到：%s（给了 %.0f 秒）"
                  % (BAD, "、".join(missing), args.timeout))
            return 2
        if not args.skip_detections:
            missing = g.wait_all(["detections", "targets"], args.timeout)
            if missing:
                print("%s 等不到：%s —— 检测器/投影节点起了吗？"
                      "（真值帧可用 --skip-detections）"
                      % (BAD, "、".join(missing)))
                return 2

        pairs = TF_PAIRS[args.camera]
        require = args.require_tf or REQUIRED_TF[args.camera]
        tfs, tf_errs = {}, {}
        deadline = time.time() + args.timeout
        while True:
            tfs, tf_errs = {}, {}
            for p in pairs:
                val, err = g.lookup(p)
                if val is None:
                    tf_errs[p] = err
                else:
                    tfs[p] = val
            if require in tfs or time.time() >= deadline:
                break
            g.spin_for(0.2)
        if require not in tfs:
            print("%s 必需 TF 查不到：`%s`" % (BAD, require))
            print("     底层异常：%s" % tf_errs.get(require))
            print("   ⇒ 腕部要 UR 驱动（base_link→tool0）+ wrist_tf_publish.py"
                  "（tool0→camera_link）+ RealSense 节点；俯视要站点外参启动器。")
            return 2

        # ---- 落盘：先写临时目录，全部成功再改名 ----
        os.makedirs(os.path.dirname(frame_dir), exist_ok=True)
        if os.path.exists(tmp_dir):
            shutil.rmtree(tmp_dir)
        os.makedirs(tmp_dir)

        files = {}
        _save_color(g.msgs["color"], os.path.join(tmp_dir, "color.png"))
        depth_name = _save_depth(g.msgs["depth"],
                                 os.path.join(tmp_dir, "depth.png"),
                                 os.path.join(tmp_dir, "depth.npy"))
        with open(os.path.join(tmp_dir, "camera_info.json"), "w",
                  encoding="utf-8") as fh:
            json.dump(camera_info_dict(g.msgs["info"]), fh,
                      ensure_ascii=False, indent=2)
        with open(os.path.join(tmp_dir, "tf.json"), "w", encoding="utf-8") as fh:
            json.dump({"ok": tfs, "missing": tf_errs}, fh,
                      ensure_ascii=False, indent=2)
        payload = {"camera_info": camera_info_dict(g.msgs["info"]),
                   "color": image_meta(g.msgs["color"]),
                   "depth": image_meta(g.msgs["depth"])}
        if not args.skip_detections:
            with open(os.path.join(tmp_dir, "detections.json"), "w",
                      encoding="utf-8") as fh:
                json.dump(detections_dict(g.msgs["detections"]), fh,
                          ensure_ascii=False, indent=2)
            with open(os.path.join(tmp_dir, "targets_3d.json"), "w",
                      encoding="utf-8") as fh:
                json.dump(targets_dict(g.msgs["targets"]), fh,
                          ensure_ascii=False, indent=2)
            payload["detections"] = header_meta(g.msgs["detections"])
            payload["targets"] = header_meta(g.msgs["targets"])
        with open(os.path.join(tmp_dir, "meta.json"), "w", encoding="utf-8") as fh:
            json.dump(payload, fh, ensure_ascii=False, indent=2)

        if os.path.exists(frame_dir):
            shutil.rmtree(frame_dir)
        os.replace(tmp_dir, frame_dir)

        for name in sorted(os.listdir(frame_dir)):
            files[name] = file_entry(os.path.join(frame_dir, name))

        rel = os.path.relpath(frame_dir, args.out)
        entry = {
            "schema": "ur5-vision-grasp/baseline-frame/1",
            "camera": args.camera,
            "tag": args.tag,
            "dir": rel,
            "captured_at_local": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "notes": args.note,
            # ⚠️ 板时钟与开发机不同步（实测慢 29.7 h）：这个时间**只作参考**，
            #    不要跨机比较时间戳；跨机对齐用 manifest 里的文件哈希。
            "clock_note": "板本机时间，不跨机比较",
            "ros": {"domain_id": os.environ.get("ROS_DOMAIN_ID"),
                    "distro": os.environ.get("ROS_DISTRO")},
            "tree": tree_info(),
            "topics": {k: topics[k] for k in need},
            "tf": {"ok": tfs, "missing": tf_errs, "required": require},
            "site": load_site_extrinsics(),
            "files": files,
        }
        write_manifest(os.path.join(args.out, "manifest.json"), entry)

        print("%s 存好了：%s" % (OK, frame_dir))
        print("   深度：%s" % depth_name)
        print("   TF 齐的：%s" % "、".join(sorted(tfs)))
        if tf_errs:
            print("%s TF 缺（不致命，已记进 tf.json）：%s"
                  % (WARN, "、".join(sorted(tf_errs))))
        if not args.skip_detections:
            d = detections_dict(g.msgs["detections"])["detections"]
            t = targets_dict(g.msgs["targets"])["targets"]
            print("   检测 %d 个 / 3D 候选 %d 个" % (len(d), len(t)))
        rc = 0
    finally:
        if os.path.isdir(tmp_dir):
            shutil.rmtree(tmp_dir, ignore_errors=True)   # 半截的现场不留
        try:
            g.node.destroy_node()
        except Exception:                                # noqa: BLE001
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:                                # noqa: BLE001
            pass
    return rc


if __name__ == "__main__":
    sys.exit(main())

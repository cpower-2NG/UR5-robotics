#!/usr/bin/env python3
"""仿真里抓一帧（只订阅，不发任何消息、不动臂）—— 与真机帧目录同一格式。

为什么单独一个脚本而不复用 precision_capture.py
--------------------------------------------
真机抓帧按**腕部 12 键链**（base_link→…→camera_color_optical_frame）判完整，
且相机话题固定为 ``/camera/*``。仿真的帧名与话题都不同（腕部
``wrist_camera_optical_frame`` / ``/wrist/*``，俯视 ``topcam_optical_frame`` /
``/camera/*``），直接复用会在「等 TF 链」那一步永远超时。

本脚本写出与 ``precision_capture.py`` **相同**的目录约定，下游
（``graspnet_candidates.py``、RNGNet 离线 runner）不需要任何改动：

    color.png   RGB 1280×720
    depth.png   16UC1 毫米（对齐到彩色）
    meta.json   camera_info.K 与两路时间戳
    tf.json     {"ok": {"base_link:camera_color_optical_frame": {...}}}
                —— 把所选相机的光心直接合成成**这个名字**写进去，
                   因为下游的 cam_pose 只认这条键；原始帧名记在 view.json。
    view.json   实际相机名 / 光心帧名 / 该帧 TF 时间戳

用法：
  python3 scripts/sim_capture_frame.py /tmp/sim-frame-top-01 --camera topcam
  python3 scripts/sim_capture_frame.py /tmp/sim-frame-wrist-01 --camera wrist
  python3 scripts/sim_capture_frame.py OUT --camera topcam --truth /tmp/grasp_scene_S0_0/scene_truth.json
"""
import argparse
import json
import os
import sys
import time

sys.dont_write_bytecode = True

# 相机名 → (彩色话题, 深度话题, 内参话题, 光心帧名)
CAMERAS = {
    "wrist": ("/wrist/color/image_raw", "/wrist/depth/image_raw",
              "/wrist/color/camera_info", "wrist_camera_optical_frame"),
    "topcam": ("/camera/color/image_raw", "/camera/depth/image_raw",
               "/camera/color/camera_info", "topcam_optical_frame"),
}
# 下游 cam_pose 认的固定键（与真机一致）
TIP_KEY = "camera_color_optical_frame"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out", help="帧目录（会新建）")
    ap.add_argument("--camera", choices=sorted(CAMERAS), default="topcam")
    ap.add_argument("--timeout", type=float, default=30.0)
    ap.add_argument("--truth", default=None,
                    help="scene_truth.json 路径；给了就拷进帧目录 truth.json")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    color_t, depth_t, info_t, optical = CAMERAS[args.camera]

    import numpy as np
    from PIL import Image
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image as RosImage
    import tf2_ros

    rclpy.init()
    node = Node("sim_capture_frame")
    buf = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
    tf2_ros.TransformListener(buf, node)
    got = {}
    qos = rclpy.qos.qos_profile_sensor_data

    node.create_subscription(
        RosImage, color_t, lambda m: got.__setitem__("color", m), qos)
    node.create_subscription(
        RosImage, depth_t, lambda m: got.__setitem__("depth", m), qos)
    node.create_subscription(
        CameraInfo, info_t, lambda m: got.__setitem__("info", m), qos)

    t0 = time.time()
    while time.time() - t0 < args.timeout and len(got) < 3:
        rclpy.spin_once(node, timeout_sec=0.2)
    miss = [k for k in ("color", "depth", "info") if k not in got]
    if miss:
        print("!! 抓帧失败：话题缺 %s（%s）" % (miss, args.camera),
              file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 2

    color_m, depth_m, info = got["color"], got["depth"], got["info"]
    # TF 取图像时间戳上的；拿不到就退到最新可用（仿真静止时两者等价，
    # 但把用了哪一种记进 view.json，别静默）。
    stamp = Time.from_msg(color_m.header.stamp)
    exact = True
    try:
        tf = buf.lookup_transform("base_link", optical, stamp)
    except Exception:
        exact = False
        tf = buf.lookup_transform("base_link", optical, Time())
    tr = tf.transform

    def to_array(m):
        buf_a = np.frombuffer(m.data, dtype=np.uint8)
        ch = 3 if m.encoding in ("rgb8", "bgr8") else 1
        a = buf_a.reshape(m.height, m.step // ch, 3)[:, :m.width]
        if m.encoding == "bgr8":
            return a[:, :, ::-1]
        return a

    color = to_array(color_m)
    if depth_m.encoding == "16UC1":
        d = np.frombuffer(depth_m.data, dtype=np.uint16)
        d = d.reshape(depth_m.height, depth_m.step // 2)[:, :depth_m.width]
    elif depth_m.encoding == "32FC1":
        d = np.frombuffer(depth_m.data, dtype=np.float32)
        d = d.reshape(depth_m.height, depth_m.step // 4)[:, :depth_m.width]
        d = np.clip(np.nan_to_num(d) * 1000.0, 0, 65535).astype(np.uint16)
    else:
        print("!! 未处理的深度编码 %r" % depth_m.encoding, file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 3

    Image.fromarray(color).save(os.path.join(args.out, "color.png"))
    Image.fromarray(d).save(os.path.join(args.out, "depth.png"))

    json.dump({
        "camera_info": {
            "width": info.width, "height": info.height,
            "distortion_model": info.distortion_model,
            "D": list(info.d), "K": list(info.k),
            "R": list(info.r), "P": list(info.p),
            "frame_id": info.header.frame_id,
        },
        "color": {"width": color_m.width, "height": color_m.height,
                  "encoding": color_m.encoding,
                  "frame_id": color_m.header.frame_id},
        "depth": {"width": depth_m.width, "height": depth_m.height,
                  "encoding": "16UC1", "unit": "mm",
                  "frame_id": depth_m.header.frame_id},
        "chain_tip": TIP_KEY,
        "sim_camera": args.camera,
    }, open(os.path.join(args.out, "meta.json"), "w"), indent=2)

    json.dump({"ok": {"base_link:%s" % TIP_KEY: {
        "translation": {"x": tr.translation.x, "y": tr.translation.y,
                        "z": tr.translation.z},
        "rotation": {"x": tr.rotation.x, "y": tr.rotation.y,
                     "z": tr.rotation.z, "w": tr.rotation.w},
    }}}, open(os.path.join(args.out, "tf.json"), "w"), indent=2)

    json.dump({"camera": args.camera, "optical_frame": optical,
               "tf_stamp_exact": exact,
               "color_stamp_ns": int(color_m.header.stamp.sec) * 10 ** 9
               + int(color_m.header.stamp.nanosec)},
              open(os.path.join(args.out, "view.json"), "w"), indent=2)

    if args.truth:
        import shutil
        shutil.copyfile(args.truth, os.path.join(args.out, "truth.json"))

    print("OK %s（%s，TF %s）" % (args.out, args.camera,
                                 "图像时间戳" if exact else "最新可用"))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
